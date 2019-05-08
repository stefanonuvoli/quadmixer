#include <iostream>
#include <cstring>

#include <QFileDialog>
#include <QMessageBox>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include "quadbooleanwindow.h"

#include "meshtypes.h"

#include "quadboolean.h"

#ifndef NDEBUG
#define SAVEMESHES
#endif

QuadBooleanWindow::QuadBooleanWindow(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi (this);

    updateVisibility();
    ui.glArea->updateGL();
}

QuadBooleanWindow::~QuadBooleanWindow()
{
    for (PolyMesh* mesh : meshes) {
        if (mesh != nullptr) {
            delete mesh;
        }
    }
}

void QuadBooleanWindow::addMesh(const string &filename)
{
    PolyMesh* mesh = new PolyMesh();

    loadMesh(*mesh, filename, ui.moveCenterCheckBox->isChecked(), ui.scaleCheckBox->isChecked());

    addMesh(mesh);
}

void QuadBooleanWindow::addMesh(PolyMesh* mesh)
{
    if (!ui.keepColorsCheckBox->isChecked()) {
        for (size_t fId = 0; fId < mesh->face.size(); fId++) {
            mesh->face[fId].C() = vcg::Color4b(255,255,255,255);
        }
    }

    bool isQuadMesh = QuadBoolean::internal::isQuadMesh(*mesh);

    if (!isQuadMesh && !QuadBoolean::internal::isTriangleMesh(*mesh)) {
        mesh->Clear();
        delete mesh;

        QMessageBox::warning(this, QString("Error"), QString("Meshes must be either quad or triangle."));
        return;
    }

    size_t id = ui.glArea->addMesh(mesh);
    assert(id == meshes.size());

    this->meshes.push_back(mesh);
}

void QuadBooleanWindow::deleteMesh(PolyMesh* mesh)
{
    return deleteMesh(std::distance(meshes.begin(), std::find(meshes.begin(), meshes.end(), mesh)));
}


void QuadBooleanWindow::deleteMesh(const size_t& id)
{
    ui.glArea->deleteMesh(id);

    delete meshes[id];
    meshes[id] = nullptr;

    ui.glArea->updateGL();
}

std::string QuadBooleanWindow::chooseMeshFile()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Open Mesh"), QDir::currentPath(),
                                                    tr("Mesh (*.obj *.ply *.off)"));
    return filename.toStdString();
}

int QuadBooleanWindow::loadMesh(PolyMesh& mesh, const std::string& filename, const bool translateCenter, const bool scale)
{
    int err=vcg::tri::io::Importer<PolyMesh>::Open(mesh, filename.c_str());
    if(err!=0){
        const char* errmsg=vcg::tri::io::Importer<PolyMesh>::ErrorMsg(err);
        std::cout << "Error Loading Mesh: " << errmsg << std::endl;
    }

    if (translateCenter || scale)
        vcg::tri::UpdateBounding<PolyMesh>::Box(mesh);

    if (translateCenter) {
        PolyMesh::CoordType bbCenter = mesh.bbox.Center();
        for (int i = 0; i < mesh.vert.size(); i++) {
            mesh.vert[i].P() -= bbCenter;
        }
    }

    if (scale) {
        PolyMesh::ScalarType bbDiag = mesh.bbox.Diag();
        PolyMesh::ScalarType scaleFactor = 1.0 / bbDiag;
        for (int i = 0; i < mesh.vert.size(); i++) {
            mesh.vert[i].P().X() *= scaleFactor;
            mesh.vert[i].P().Y() *= scaleFactor;
            mesh.vert[i].P().Z() *= scaleFactor;
        }
    }

    return err;
}



void QuadBooleanWindow::on_loadButton_clicked()
{
    std::string filename = chooseMeshFile();
    if(!filename.empty()) {
        addMesh(filename);

        ui.glArea->resetSceneOnMeshes();
        ui.glArea->deselectTransformationMesh();

        updateVisibility();
        ui.glArea->updateGL();
    }
}

void QuadBooleanWindow::on_deleteAllButton_clicked()
{
    for (size_t id = 0; id < meshes.size(); id++) {
        if (meshes[id] != nullptr) {
            deleteMesh(id);
        }
    }
}

void QuadBooleanWindow::on_resetSceneButton_clicked()
{
    ui.glArea->resetSceneOnMeshes();
}


void QuadBooleanWindow::on_trackballCheckBox_stateChanged(int arg1)
{
    ui.glArea->setTrackballVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showWireframe_stateChanged(int arg1)
{
    ui.glArea->setWireframe(arg1 == Qt::Checked);

    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_showParametersCheckBox_stateChanged(int arg1)
{
    ui.parametersFrame->setVisible(arg1 == Qt::Checked);
}









void QuadBooleanWindow::doTraceQuads() {
    //Clear data
    quadTracerLabel1.clear();
    quadTracerLabel2.clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    isQuadMesh1 = QuadBoolean::internal::isQuadMesh(mesh1);
    isQuadMesh2 = QuadBoolean::internal::isQuadMesh(mesh2);

    bool motorcycle = ui.motorcycleCheckBox->isChecked();

    //Trace quads following singularities
    QuadBoolean::internal::traceQuads(mesh1, quadTracerLabel1, motorcycle);
    QuadBoolean::internal::traceQuads(mesh2, quadTracerLabel2, motorcycle);

    std::cout << std::endl << " >> "
              << "Quad tracer: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    quadLayoutData1 = QuadBoolean::internal::getQuadLayoutData(mesh1, isQuadMesh1, quadTracerLabel1);
    ui.glArea->setQuadLayout1(&quadLayoutData1);

    if (isQuadMesh1) {
        colorizeMesh(mesh1, quadTracerLabel1);
    }

    quadLayoutData2 = QuadBoolean::internal::getQuadLayoutData(mesh2, isQuadMesh2, quadTracerLabel2);
    ui.glArea->setQuadLayout2(&quadLayoutData2);

    if (isQuadMesh2) {
        colorizeMesh(mesh2, quadTracerLabel2);
    }

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
    QuadBoolean::internal::triangulateQuadMesh(mesh1, isQuadMesh1, triMesh1, birthQuad1);
    QuadBoolean::internal::triangulateQuadMesh(mesh2, isQuadMesh2, triMesh2, birthQuad2);

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
    booleanSmoothed.Clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    int intersectionAVGNRing = ui.intersectionSmoothingNRingSpinBox->value();
    double intersectionMaxB = ui.intersectionSmoothingMaxBBSpinBox->value();

    //Copy mesh
    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(booleanSmoothed, boolean);

    //Smooth along intersection curves
    QuadBoolean::internal::smoothAlongIntersectionCurves(
                booleanSmoothed,
                VR,
                FR,
                intersectionCurves,
                intersectionSmoothingIterations,
                intersectionAVGNRing,
                intersectionMaxB);

    std::cout << std::endl << " >> "
              << "Smooth along intersection curves: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setBoolean(&booleanSmoothed);
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

    int minRectangleArea = ui.minRectangleAreaSpinBox->value();
    int minPatchArea = ui.minPatchAreaSpinBox->value();
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
                isQuadMesh1, isQuadMesh2,
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
    preservedFaceLabel1 = QuadBoolean::internal::splitQuadPatchesInMaximumRectangles(mesh1, isQuadMesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1, minRectangleArea, true);
    preservedFaceLabel2 = QuadBoolean::internal::splitQuadPatchesInMaximumRectangles(mesh2, isQuadMesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2, minRectangleArea, true);

    std::cout << std::endl << " >> "
              << "Max rectangle: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    //Merge rectangular patches
    if (mergeQuads) {
        start = chrono::steady_clock::now();

        int nMerged1 = QuadBoolean::internal::mergeQuadPatches(mesh1, isQuadMesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
        int nMerged2 = QuadBoolean::internal::mergeQuadPatches(mesh2, isQuadMesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);

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
        int nSmall1 = QuadBoolean::internal::deleteSmallQuadPatches(mesh1, isQuadMesh1, affectedPatches1, minPatchArea, preservedFaceLabel1, preservedQuad1);
        int nSmall2 = QuadBoolean::internal::deleteSmallQuadPatches(mesh2, isQuadMesh2, affectedPatches2, minPatchArea, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Small deleted -> mesh 1: " << nSmall1 << " / mesh 2: " << nSmall2 << " in "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
    }

    if (deleteNonConnected) {
        start = chrono::steady_clock::now();

        //Delete non-connected patches
        int nNonConnected1 = QuadBoolean::internal::deleteNonConnectedQuadPatches(mesh1, isQuadMesh1, preservedFaceLabel1, preservedQuad1);
        int nNonConnected2 = QuadBoolean::internal::deleteNonConnectedQuadPatches(mesh2, isQuadMesh2, preservedFaceLabel2, preservedQuad2);

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
                booleanSmoothed,
                nFirstFaces,
                birthQuad1, birthQuad2,
                preservedQuad1, preservedQuad2,
                J,
                initialNewSurface);

    std::cout << std::endl << " >> "
              << "Get surfaces: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    quadLayoutDataPreserved1 = QuadBoolean::internal::getQuadLayoutData(mesh1, isQuadMesh1, preservedFaceLabel1);
    ui.glArea->setQuadLayoutPreserved1(&quadLayoutDataPreserved1);

    quadLayoutDataPreserved2 = QuadBoolean::internal::getQuadLayoutData(mesh2, isQuadMesh2, preservedFaceLabel2);
    ui.glArea->setQuadLayoutPreserved2(&quadLayoutDataPreserved2);


    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(newSurface, initialNewSurface);

    ui.glArea->setPreservedSurface(&preservedSurface);
    ui.glArea->setNewSurface(&newSurface);

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

    bool initialRemeshing = ui.initialRemeshingCheckBox->isChecked();
    bool edgeFactor = ui.edgeFactorSpinBox->value();
    bool reproject = ui.reprojectCheckBox->isChecked();
    bool splitConcaves = ui.splitConcavesCheckBox->isChecked();
    bool finalSmoothing = ui.finalSmoothingCheckBox->isChecked();

    start = chrono::steady_clock::now();

    newSurfaceLabel = QuadBoolean::internal::getPatchDecomposition(newSurface, newSurfacePartitions, newSurfaceCorners, initialRemeshing, edgeFactor, reproject, splitConcaves, finalSmoothing);

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
    double beta = ui.betaSpinBox->value();

    QuadBoolean::ILPMethod ilpMethod;
    if (ui.ilpMethodLSRadio->isChecked()) {
        ilpMethod = QuadBoolean::ILPMethod::LEASTSQUARES;
    }
    else if (ui.ilpMethodABSRadio->isChecked()) {
        ilpMethod = QuadBoolean::ILPMethod::ABS;
    }

    ilpResult = QuadBoolean::internal::findBestSideSize(newSurface, chartData, alpha, beta, ilpMethod);

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
    PolyMesh coloredQuadrangulatedSurface;
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredPreservedSurface, preservedSurface);
    for (size_t i = 0; i < coloredPreservedSurface.face.size(); i++) {
        coloredPreservedSurface.face[i].C() = vcg::Color4b(200,255,200,255);
    }
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredQuadrangulatedSurface, quadrangulatedSurface);
    for (size_t i = 0; i < coloredQuadrangulatedSurface.face.size(); i++) {
        coloredQuadrangulatedSurface.face[i].C() = vcg::Color4b(255,255,255,255);
    }

    //Clear data
    result.Clear();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    int resultAVGNRing = ui.resultSmoothingNRingSpinBox->value();

    int resultSmoothingLaplacianIterations = ui.resultSmoothingLaplacianSpinBox->value();
    int resultSmoothingLaplacianAVGNRing = ui.resultSmoothingLaplacianNRingSpinBox->value();

    start = chrono::steady_clock::now();

    QuadBoolean::internal::getResult(coloredPreservedSurface, coloredQuadrangulatedSurface, result, booleanSmoothed, resultSmoothingIterations, resultAVGNRing, resultSmoothingLaplacianIterations, resultSmoothingLaplacianAVGNRing);

    std::cout << std::endl << " >> "
              << "Get result: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, "res/result.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    ui.glArea->setResult(&result);
}

void QuadBooleanWindow::clearVisualizationData()
{
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

    updateVisibility();
    ui.glArea->update();
}



void QuadBooleanWindow::on_saveResultPushButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Open Mesh"),
                                                    QDir::currentPath(),
                                                    tr("Mesh (*.obj *.ply *.off)"));

    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, filename.toStdString().c_str(), vcg::tri::io::Mask::IOM_FACECOLOR);
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

    ui.glArea->setWireframe(ui.showWireframe->isChecked());

    ui.parametersFrame->setVisible(ui.showParametersCheckBox->isChecked());
    ui.debugFrame->setVisible(ui.glArea->debugMode);
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
//            color.SetHSVColor(subd * std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])), 1.0, 1.0);

            color=vcg::Color4b::Scatter(faceLabel.size(),std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])));

            mesh.face[i].C() = color;
        }
    }
}

void QuadBooleanWindow::on_debugModeButton_clicked()
{
    ui.glArea->deselectTransformationMesh();

    if (!ui.glArea->debugMode) {
        if (ui.glArea->targetMesh1 == nullptr || ui.glArea->targetMesh2 == nullptr) {
            QMessageBox::warning(this, QString("Error"), QString("Select two meshes to switch to the debug mode!"));
        }
        else {
            clearVisualizationData();

            //Copy mesh
            vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(mesh1, *ui.glArea->targetMesh1->mesh);
            vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(mesh2, *ui.glArea->targetMesh2->mesh);

            ui.glArea->setMesh1(&mesh1);
            ui.glArea->setMesh2(&mesh2);
            ui.glArea->debugMode = true;
        }
    }
    else {
        clearVisualizationData();

        mesh1.Clear();
        mesh2.Clear();

        ui.glArea->debugMode = false;
    }

    updateVisibility();
}

void QuadBooleanWindow::on_deleteButton_clicked()
{
    if (ui.glArea->targetMesh1 != nullptr) {
        if (ui.glArea->targetMesh2 != nullptr) {
            QMessageBox::warning(this, QString("Error"), QString("Please select only one mesh."));
        }
        else {
            deleteMesh(ui.glArea->targetMesh1->mesh);
        }
    }
    else {
        QMessageBox::warning(this, QString("Error"), QString("Please select the mesh to be deleted."));
    }
}

void QuadBooleanWindow::on_executePushButton_clicked()
{
    if (ui.glArea->targetMesh1 == nullptr || ui.glArea->targetMesh2 == nullptr) {
        QMessageBox::warning(this, QString("Error"), QString("Select two meshes to proceed!"));
    }
    else {
        PolyMesh* target1 = ui.glArea->targetMesh1->mesh;
        PolyMesh* target2 = ui.glArea->targetMesh2->mesh;
        PolyMesh* booleanComputed = new PolyMesh();

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

        QuadBoolean::quadBoolean(*ui.glArea->targetMesh1->mesh, *ui.glArea->targetMesh2->mesh, operation, *booleanComputed);

        deleteMesh(target1);
        deleteMesh(target2);
        addMesh(booleanComputed);

        ui.glArea->updateGL();
    }
}
