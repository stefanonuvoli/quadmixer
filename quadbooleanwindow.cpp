#include <iostream>
#include <cstring>

#include <QFileDialog>
#include <QMessageBox>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

#include "quadbooleanwindow.h"

#include "meshtypes.h"

#include "quadboolean.h"


#define SAVEMESHES


QuadBooleanWindow::QuadBooleanWindow(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi (this);

    loadMesh(mesh1, "mesh1.obj");
    loadMesh(mesh2, "mesh4.obj");

    ui.glArea->setMesh1(&mesh1);
    ui.glArea->setMesh2(&mesh2);

    setTrackballOnMeshes();

    ui.glArea->updateGL();
}

std::string QuadBooleanWindow::chooseMeshFile()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Open Mesh"), QDir::currentPath(),
                                                    tr("Mesh (*.obj *.ply *.off)"));
    return filename.toStdString();
}

int QuadBooleanWindow::loadMesh(PolyMesh& mesh, const std::string& filename)
{
    int err=vcg::tri::io::Importer<PolyMesh>::Open(mesh, filename.c_str());
    if(err!=0){
        const char* errmsg=vcg::tri::io::Importer<PolyMesh>::ErrorMsg(err);
        QMessageBox::warning(this,tr("Error Loading Mesh"),QString(errmsg));
    }
    return err;
}

void QuadBooleanWindow::setTrackballOnMeshes()
{
    PolyMesh::CoordType center(0,0,0);
    PolyMesh::ScalarType maxDiag = 0;

    center += mesh1.bbox.Center();
    center += mesh2.bbox.Center();
    center /= 2;
    vcg::Point3f sceneCenter(static_cast<float>(center.X()),static_cast<float>(center.Y()),static_cast<float>(center.Z()));
    ui.glArea->setSceneCenter(sceneCenter);

    maxDiag = std::max(maxDiag, mesh1.bbox.Diag());
    maxDiag = std::max(maxDiag, mesh2.bbox.Diag());
    ui.glArea->setSceneRadius(static_cast<float>(maxDiag/2));
}


void QuadBooleanWindow::doTraceQuads() {
    //Clear data
    quadTracerLabel1.clear();
    quadTracerLabel2.clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    bool motorcycle = ui.motorcycleCheckBox->isChecked();

    //Trace quads following singularities
    QuadBoolean::internal::traceQuads(mesh1, quadTracerLabel1, motorcycle);
    QuadBoolean::internal::traceQuads(mesh2, quadTracerLabel2, motorcycle);

    std::cout << std::endl << " >> "
              << "Quad tracer: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    quadLayoutData1 = QuadBoolean::internal::getQuadLayoutData(mesh1, quadTracerLabel1);
    quadLayoutData2 = QuadBoolean::internal::getQuadLayoutData(mesh2, quadTracerLabel2);

    ui.glArea->setQuadLayout1(&quadLayoutData1);
    ui.glArea->setQuadLayout2(&quadLayoutData2);

    colorizeMesh(mesh1, quadTracerLabel1);
    colorizeMesh(mesh2, quadTracerLabel2);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh1, "res/mesh1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh2, "res/mesh2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::doComputeBooleans() {
    //Clear meshes
    triMesh1.Clear();
    triMesh2.Clear();
    boolean.Clear();
    birthQuad1.clear();
    birthQuad2.clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Triangulate
    QuadBoolean::internal::triangulateQuadMesh(mesh1, triMesh1, birthQuad1);
    QuadBoolean::internal::triangulateQuadMesh(mesh2, triMesh2, birthQuad2);

    std::cout << std::endl << " >> "
              << "Triangulation: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    QuadBoolean::Operation operation = QuadBoolean::Operation::UNION;
    if (ui.operationUnionRadio->isChecked()) {
        operation = QuadBoolean::Operation::UNION;
    }
    else if (ui.operationIntersectionRadio->isChecked()) {
        operation = QuadBoolean::Operation::INTERSECTION;
    }
    else if (ui.operationDifferenceRadio->isChecked()) {
        operation = QuadBoolean::Operation::DIFFERENCE;
    }

    start = chrono::steady_clock::now();

    //Boolean operation on trimeshes
    QuadBoolean::internal::computeBooleanOperation(
                triMesh1,
                triMesh2,
                operation,
                boolean,
                VA, VB, VR,
                FA, FB, FR,
                J);

    std::cout << std::endl << " >> "
              << "Boolean: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    start = chrono::steady_clock::now();

    intersectionCurves = QuadBoolean::internal::getIntersectionCurves(
                triMesh1, triMesh2,
                VA, VB, VR,
                FA, FB, FR,
                J);


    std::cout << std::endl << " >> "
              << "Intersection curves: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setBoolean(&boolean);

    ui.glArea->setIntersectionCurves(&intersectionCurves);


#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/trimesh1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/trimesh2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/boolean.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::doSmooth()
{
    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    int intersectionAVGNRing = ui.intersectionSmoothingNRingSpinBox->value();

    //Smooth along intersection curves
    QuadBoolean::internal::smoothAlongIntersectionCurves(
                boolean,
                VR,
                FR,
                intersectionCurves,
                intersectionSmoothingIterations,
                intersectionAVGNRing);

    std::cout << std::endl << " >> "
              << "Smooth along intersection curves: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;
}


void QuadBooleanWindow::doGetSurfaces() {
    preservedSurface.Clear();
    preservedSurfaceLabel.clear();
    preservedQuad1.clear();
    preservedQuad2.clear();
    preservedFaceLabel1.clear();
    preservedFaceLabel2.clear();
    newSurface.Clear();
    initialNewSurface.Clear();

    int minRectangleSide = ui.minRectangleSideSpinBox->value();
    bool mergeQuads = ui.mergeCheckBox->isChecked();
    bool deleteSmall = ui.deleteSmallCheckBox->isChecked();
    bool deleteNonConnected = ui.deleteNonConnectedCheckBox->isChecked();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Find preserved quads
    QuadBoolean::internal::findPreservedQuads(
                triMesh1, triMesh2,
                VA, VB, VR,
                FA, FB, FR,
                J,
                birthQuad1, birthQuad2,
                preservedQuad1, preservedQuad2);

    preservedFaceLabel1 = quadTracerLabel1;
    preservedFaceLabel2 = quadTracerLabel2;

    std::cout << std::endl << " >> "
              << "Find preserved quads: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;
    start = chrono::steady_clock::now();

    //Find affected patches
    std::unordered_set<int> affectedPatches1;
    std::unordered_set<int> affectedPatches2;

    QuadBoolean::internal::findAffectedPatches(mesh1, preservedQuad1, preservedFaceLabel1, affectedPatches1);
    QuadBoolean::internal::findAffectedPatches(mesh2, preservedQuad2, preservedFaceLabel2, affectedPatches2);

    //Maximum rectangles in the patches
    preservedFaceLabel1 = QuadBoolean::internal::splitQuadPatchesInMaximumRectangles(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1, minRectangleSide, true);
    preservedFaceLabel2 = QuadBoolean::internal::splitQuadPatchesInMaximumRectangles(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2, minRectangleSide, true);

    std::cout << std::endl << " >> "
              << "Max rectangle: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    //Merge rectangular patches
    if (mergeQuads) {
        start = chrono::steady_clock::now();

        int nMerged1 = QuadBoolean::internal::mergeQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
        int nMerged2 = QuadBoolean::internal::mergeQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Merge: "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
        std::cout << std::endl << " >> "
                  << "Merged -> mesh 1: " << nMerged1 << " / mesh 2: " << nMerged2 << std::endl;
    }

    if (deleteSmall) {
        start = chrono::steady_clock::now();

        //Delete small patches
        int nSmall1 = QuadBoolean::internal::deleteSmallQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
        int nSmall2 = QuadBoolean::internal::deleteSmallQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Small deleted -> mesh 1: " << nSmall1 << " / mesh 2: " << nSmall2 << " in "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
    }

    if (deleteNonConnected) {
        start = chrono::steady_clock::now();

        //Delete non-connected patches
        int nNonConnected1 = QuadBoolean::internal::deleteNonConnectedQuadPatches(mesh1, preservedFaceLabel1, preservedQuad1);
        int nNonConnected2 = QuadBoolean::internal::deleteNonConnectedQuadPatches(mesh2, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Non-connected deleted -> mesh 1: " << nNonConnected1 << " / mesh 2: " << nNonConnected2 << " in "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
    }



    start = chrono::steady_clock::now();

    //Get mesh of the preserved surface
    QuadBoolean::internal::getPreservedSurfaceMesh(
                mesh1, mesh2,
                preservedQuad1, preservedQuad2,
                preservedFaceLabel1, preservedFaceLabel2,
                preservedSurface, preservedSurfaceLabel);

    //New mesh (to be decomposed in patch)
    size_t nFirstFaces = triMesh1.face.size();
    QuadBoolean::internal::getNewSurfaceMesh(
                boolean,
                nFirstFaces,
                birthQuad1, birthQuad2,
                preservedQuad1, preservedQuad2,
                J,
                initialNewSurface);

    std::cout << std::endl << " >> "
              << "Get surfaces: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;



    quadLayoutDataPreserved1 = QuadBoolean::internal::getQuadLayoutData(mesh1, preservedFaceLabel1);
    quadLayoutDataPreserved2 = QuadBoolean::internal::getQuadLayoutData(mesh2, preservedFaceLabel2);

    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(newSurface, initialNewSurface);

    ui.glArea->setPreservedSurface(&preservedSurface);
    ui.glArea->setNewSurface(&newSurface);
    ui.glArea->setQuadLayoutPreserved1(&quadLayoutDataPreserved1);
    ui.glArea->setQuadLayoutPreserved2(&quadLayoutDataPreserved2);

    colorizeMesh(preservedSurface, preservedSurfaceLabel);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(newSurface, "res/newSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(preservedSurface, "res/preservedSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::doPatchDecomposition() {
    chrono::steady_clock::time_point start;

    newSurfacePartitions.clear();
    newSurfaceCorners.clear();
    newSurfaceLabel.clear();
    newSurface.Clear();
    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(newSurface, initialNewSurface);

    start = chrono::steady_clock::now();

    newSurfaceLabel = QuadBoolean::internal::getPatchDecomposition(newSurface, newSurfacePartitions, newSurfaceCorners);

    //TEST

//    newSurfaceLabel.resize(newSurface.face.size(), -1);
//    newSurface.Clear();
//    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(newSurface, preservedSurface);
//    std::vector<int> birthQuad = QuadBoolean::internal::splitQuadInTriangle(newSurface);
//    newSurfaceLabel.resize(newSurface.face.size(), -1);
//    for (size_t i = 0; i < newSurface.face.size(); i++) {
//        newSurfaceLabel[i] = preservedSurfaceLabel[birthQuad[i]];
//    }
//    vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(newSurface);
//    vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(newSurface);
//    vcg::tri::UpdateNormal<PolyMesh>::PerFaceNormalized(newSurface);
//    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(newSurface);
//    vcg::tri::UpdateTopology<PolyMesh>::FaceFace(newSurface);

    //-----------

    std::cout << std::endl << " >> "
              << "Decomposition: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    colorizeMesh(newSurface, newSurfaceLabel);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(newSurface, "res/decomposedNewSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    start = chrono::steady_clock::now();


    chartData = QuadBoolean::internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

    std::cout << std::endl << " >> "
              << "Get patches and sides: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setChartSides(&chartData);
}


void QuadBooleanWindow::doSolveILP() {
    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Solve ILP
    double alpha = ui.alphaSpinBox->value();

    ilpResult = QuadBoolean::internal::findBestSideSize(chartData, alpha);

    std::cout << std::endl << " >> "
              << "ILP: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setIlpResult(&ilpResult);
}

void QuadBooleanWindow::doQuadrangulate()
{
    chrono::steady_clock::time_point start;

    quadrangulatedSurface.Clear();
    quadrangulatedSurfaceLabel.clear();

    int chartSmoothingIterations = ui.chartSmoothingSpinBox->value();
    int meshSmoothingIterations = ui.meshSmoothingSpinBox->value();

    start = chrono::steady_clock::now();

    QuadBoolean::internal::quadrangulate(
                newSurface,
                chartData,
                ilpResult,
                chartSmoothingIterations,
                meshSmoothingIterations,
                quadrangulatedSurface,
                quadrangulatedSurfaceLabel);

    std::cout << std::endl << " >> "
              << "Quadrangulate new surface: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    colorizeMesh(quadrangulatedSurface, quadrangulatedSurfaceLabel);

    ui.glArea->setQuadrangulated(&quadrangulatedSurface);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(quadrangulatedSurface, "res/quadrangulatedSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::doGetResult()
{
    chrono::steady_clock::time_point start;

    PolyMesh coloredPreservedSurface;
    PolyMesh coloredquadrangulatedSurface;
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredPreservedSurface, preservedSurface);
    for (size_t i = 0; i < coloredPreservedSurface.face.size(); i++) {
        coloredPreservedSurface.face[i].C() = vcg::Color4b(100,200,100,255);
    }
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredquadrangulatedSurface, quadrangulatedSurface);
    for (size_t i = 0; i < coloredquadrangulatedSurface.face.size(); i++) {
        coloredquadrangulatedSurface.face[i].C() = vcg::Color4b(128,128,128,255);
    }

    //Clear data
    result.Clear();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    int resultAVGNRing = ui.resultSmoothingNRingSpinBox->value();

    start = chrono::steady_clock::now();

    QuadBoolean::internal::getResult(coloredPreservedSurface, coloredquadrangulatedSurface, result, resultSmoothingIterations, resultAVGNRing);

    std::cout << std::endl << " >> "
              << "Get result: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    start = chrono::steady_clock::now();

    bool motorcycle = ui.motorcycleCheckBox->isChecked();

    //Trace quads following singularities
    QuadBoolean::internal::traceQuads(result, quadTracerLabelResult, motorcycle);

    std::cout << std::endl << " >> "
              << "Quad tracer: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    quadLayoutDataResult = QuadBoolean::internal::getQuadLayoutData(result, quadTracerLabelResult);

    ui.glArea->setResult(&result);
    ui.glArea->setQuadLayoutResult(&quadLayoutDataResult);

//    colorizeMesh(result, quadTracerLabelResult);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, "res/result.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}


void QuadBooleanWindow::on_loadMeshesPushButton_clicked()
{

    std::string filename1 = chooseMeshFile();
    if(!filename1.empty()) {
        std::string filename2 = chooseMeshFile();
        if(!filename2.empty()) {
            ui.glArea->setMesh1(nullptr);
            ui.glArea->setMesh2(nullptr);
            ui.glArea->setQuadLayout1(nullptr);
            ui.glArea->setQuadLayout2(nullptr);
            ui.glArea->setBoolean(nullptr);
            ui.glArea->setIntersectionCurves(nullptr);
            ui.glArea->setPreservedSurface(nullptr);
            ui.glArea->setQuadLayoutPreserved1(nullptr);
            ui.glArea->setQuadLayoutPreserved2(nullptr);
            ui.glArea->setNewSurface(nullptr);
            ui.glArea->setChartSides(nullptr);
            ui.glArea->setIlpResult(nullptr);
            ui.glArea->setQuadrangulated(nullptr);
            ui.glArea->setQuadLayoutQuadrangulated(nullptr);
            ui.glArea->setResult(nullptr);
            ui.glArea->setQuadLayoutResult(nullptr);

            mesh1.Clear();
            mesh2.Clear();
            loadMesh(mesh1, filename1);
            loadMesh(mesh2, filename2);
            ui.glArea->setMesh1(&mesh1);
            ui.glArea->setMesh2(&mesh2);

            ui.showMesh1CheckBox->setChecked(true);
            ui.showMesh2CheckBox->setChecked(true);
            ui.showQuadLayout1CheckBox->setChecked(true);
            ui.showQuadLayout2CheckBox->setChecked(true);
            ui.showBooleanCheckBox->setChecked(false);
            ui.showIntersectionCurvesCheckBox->setChecked(false);
            ui.showIntersectionCurvesCheckBox->setChecked(false);
            ui.showPreservedSurfaceCheckBox->setChecked(false);
            ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
            ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
            ui.showNewSurfaceCheckBox->setChecked(false);
            ui.showQuadrangulatedCheckBox->setChecked(false);
            ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
            ui.showResultCheckBox->setChecked(false);
            ui.showResultLayoutCheckBox->setChecked(false);

            setTrackballOnMeshes();

            updateVisibility();
            ui.glArea->updateGL();
        }
    }
}


void QuadBooleanWindow::on_quadTracerPushButton_clicked()
{
    doTraceQuads();

    ui.showMesh1CheckBox->setChecked(true);
    ui.showMesh2CheckBox->setChecked(true);
    ui.showQuadLayout1CheckBox->setChecked(true);
    ui.showQuadLayout2CheckBox->setChecked(true);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_computeBooleanPushButton_clicked()
{
    doComputeBooleans();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(true);
    ui.showIntersectionCurvesCheckBox->setChecked(true);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_smoothPushButton_clicked()
{
    doSmooth();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(true);
    ui.showIntersectionCurvesCheckBox->setChecked(true);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_getSurfacesPushButton_clicked()
{
    doGetSurfaces();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_decompositionPushButton_clicked()
{
    doPatchDecomposition();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

}


void QuadBooleanWindow::on_ilpPushButton_clicked()
{
    doSolveILP();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showILPCheckBox->setChecked(true);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_quadrangulatePushButton_clicked()
{
    doQuadrangulate();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(true);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(true);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_getResultPushButton_clicked()
{
    doGetResult();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(true);
    ui.showResultLayoutCheckBox->setChecked(true);

    updateVisibility();
    ui.glArea->updateGL();


}

void QuadBooleanWindow::on_computeAllPushButton_clicked()
{
    doTraceQuads();
    doComputeBooleans();
    doSmooth();
    doGetSurfaces();
    doPatchDecomposition();
    doSolveILP();
    doQuadrangulate();
    doGetResult();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionCurvesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(true);
    ui.showResultLayoutCheckBox->setChecked(true);

    updateVisibility();
    ui.glArea->update();
}



void QuadBooleanWindow::on_showMesh1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setMesh1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showMesh2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setMesh2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showBooleanCheckBox_stateChanged(int arg1)
{
    ui.glArea->setBooleanVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showIntersectionCurvesCheckBox_stateChanged(int arg1)
{
    ui.glArea->setIntersectionCurvesVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showPreservedSurfaceCheckBox_stateChanged(int arg1)
{

    ui.glArea->setPreservedSurfaceVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showNewSurfaceCheckBox_stateChanged(int arg1)
{
    ui.glArea->setNewSurfaceVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}


void QuadBooleanWindow::on_showQuadLayout1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayout1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showQuadLayout2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayout2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showQuadLayoutPreserved1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutPreserved1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}


void QuadBooleanWindow::on_showQuadLayoutPreserved2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutPreserved2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showChartSidesCheckBox_stateChanged(int arg1)
{
    ui.glArea->setChartSidesVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showILPCheckBox_stateChanged(int arg1)
{
    ui.glArea->setILPVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showQuadrangulatedCheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadrangulatedVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showQuadrangulatedLayoutCheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutQuadrangulatedVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showResultCheckBox_stateChanged(int arg1)
{
    ui.glArea->setResultVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showResultLayoutCheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutResultVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}



void QuadBooleanWindow::on_resetTrackballButton_clicked()
{
    ui.glArea->resetTrackball();
}

void QuadBooleanWindow::updateVisibility()
{
    ui.glArea->setMesh1Visibility(ui.showMesh1CheckBox->isChecked());
    ui.glArea->setMesh2Visibility(ui.showMesh2CheckBox->isChecked());
    ui.glArea->setBooleanVisibility(ui.showBooleanCheckBox->isChecked());
    ui.glArea->setIntersectionCurvesVisibility(ui.showIntersectionCurvesCheckBox->isChecked());
    ui.glArea->setPreservedSurfaceVisibility(ui.showPreservedSurfaceCheckBox->isChecked());
    ui.glArea->setNewSurfaceVisibility(ui.showNewSurfaceCheckBox->isChecked());
    ui.glArea->setQuadLayout1Visibility(ui.showQuadLayout1CheckBox->isChecked());
    ui.glArea->setQuadLayout2Visibility(ui.showQuadLayout2CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved1Visibility(ui.showQuadLayoutPreserved1CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved2Visibility(ui.showQuadLayoutPreserved2CheckBox->isChecked());
    ui.glArea->setChartSidesVisibility(ui.showChartSidesCheckBox->isChecked());    
    ui.glArea->setILPVisibility(ui.showILPCheckBox->isChecked());
    ui.glArea->setQuadrangulatedVisibility(ui.showQuadrangulatedCheckBox->isChecked());
    ui.glArea->setQuadLayoutQuadrangulatedVisibility(ui.showQuadrangulatedLayoutCheckBox->isChecked());
    ui.glArea->setResultVisibility(ui.showResultCheckBox->isChecked());
    ui.glArea->setQuadLayoutResultVisibility(ui.showResultLayoutCheckBox->isChecked());

    ui.glArea->setMesh1Wireframe(ui.showWireframe->isChecked());
    ui.glArea->setMesh2Wireframe(ui.showWireframe->isChecked());
    ui.glArea->setBooleanWireframe(ui.showWireframe->isChecked());
    ui.glArea->setPreservedSurfaceWireframe(ui.showWireframe->isChecked());
    ui.glArea->setNewSurfaceWireframe(ui.showWireframe->isChecked());
    ui.glArea->setQuadrangulatedWireframe(ui.showWireframe->isChecked());
    ui.glArea->setResultWireframe(ui.showWireframe->isChecked());
}

template<class MeshType>
void QuadBooleanWindow::colorizeMesh(
        MeshType& mesh,
        const std::vector<int>& faceLabel)
{
    std::set<int> faceLabelSet;
    for (size_t i = 0; i < faceLabel.size(); i++)
        faceLabelSet.insert(faceLabel[i]);

    float subd = (float) 1 / (std::max(static_cast<size_t>(1), faceLabelSet.size() - 1));

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0) {
            vcg::Color4b color;
            color.SetHSVColor(subd * std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])), 1.0, 1.0);

//            color=vcg::Color4b::Scatter(faceLabel.size(),std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])));

            mesh.face[i].C() = color;
        }
    }
}

void QuadBooleanWindow::on_showWireframe_stateChanged(int arg1)
{
    ui.glArea->setMesh1Wireframe(ui.showWireframe->isChecked());
    ui.glArea->setMesh2Wireframe(ui.showWireframe->isChecked());
    ui.glArea->setBooleanWireframe(ui.showWireframe->isChecked());
    ui.glArea->setPreservedSurfaceWireframe(ui.showWireframe->isChecked());
    ui.glArea->setNewSurfaceWireframe(ui.showWireframe->isChecked());
    ui.glArea->setQuadrangulatedWireframe(ui.showWireframe->isChecked());
    ui.glArea->setResultWireframe(ui.showWireframe->isChecked());

    ui.glArea->updateGL();
}
