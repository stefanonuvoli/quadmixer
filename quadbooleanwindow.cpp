#include <iostream>
#include <cstring>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

#include "quadbooleanwindow.h"

#include "quadpatchtracer.h"
#include "quadilp.h"
#include "quadutils.h"
#include "quadconvert.h"
#include "quadbooleaninterface.h"
#include "quadpatterns.h"
#include "quadquadmapping.h"

#include <QFileDialog>
#include <QMessageBox>

#include <vcg/complex/algorithms/mesh_to_matrix.h>

#include <igl/writeOBJ.h>

#define SAVEMESHES


QuadBooleanWindow::QuadBooleanWindow(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi (this);

    loadMesh(mesh1, "mesh1.obj");
    loadMesh(mesh2, "mesh2.obj");

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


void QuadBooleanWindow::traceQuads() {
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Quad tracer
    updatePolymeshAttributes(mesh1);
    updatePolymeshAttributes(mesh2);

    QuadMeshTracer<PolyMesh> tracer1(mesh1);
    tracer1.MotorCycle = true;
    tracer1.TracePartitions();

    QuadMeshTracer<PolyMesh> tracer2(mesh2);
    tracer2.MotorCycle = true;
    tracer2.TracePartitions();

    tracerFaceLabel1 = tracer1.FacePatch;
    tracerFaceLabel2 = tracer2.FacePatch;

    std::cout << std::endl << " >> "
              << "Quad tracer: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setQuadLayout1(&quadData1);
    ui.glArea->setQuadLayout2(&quadData2);

    colorizeMesh(mesh1, tracerFaceLabel1);
    colorizeMesh(mesh2, tracerFaceLabel2);
}

void QuadBooleanWindow::computeBooleans() {
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Triangulate
    triMesh1.Clear();
    triMesh2.Clear();
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(triMesh1, mesh1);
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(triMesh2, mesh2);

    birthQuad1 = splitQuadInTriangle(triMesh1);
    birthQuad2 = splitQuadInTriangle(triMesh2);

    std::cout << std::endl << " >> "
              << "Triangulation: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    start = chrono::steady_clock::now();

    //Boolean operation on trimeshes
    vcg::tri::MeshToMatrix<PolyMesh>::GetTriMeshData(triMesh1, FA, VA);
    vcg::tri::MeshToMatrix<PolyMesh>::GetTriMeshData(triMesh2, FB, VB);

    trimeshUnion(VA,FA,VB,FB,VR,FR,J);

    //Result on VCG
    boolean.Clear();
    eigenToVCG(VR, FR, boolean);

    std::cout << std::endl << " >> "
              << "Boolean: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    ui.glArea->setBoolean(&boolean);
}


void QuadBooleanWindow::getPreservedQuads() {
    using namespace QuadBoolean;

    preservedSurface.Clear();
    newSurface.Clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    Eigen::Index nFirstFaces = FA.rows();

    preservedQuad1 = std::vector<bool>(mesh1.face.size(), false);
    preservedQuad2 = std::vector<bool>(mesh2.face.size(), false);

    computePreservedQuads(triMesh1, VA, FA, VR, FR, J, birthQuad1, 0, preservedQuad1);
    computePreservedQuads(triMesh2, VB, FB, VR, FR, J, birthQuad2, nFirstFaces, preservedQuad2);

    std::cout << std::endl << " >> "
              << "Find preserved quads: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    std::unordered_set<int> affectedPatches1;
    for (int i = 0; i < mesh1.face.size(); i++) {
        if (!preservedQuad1[i]) {
            affectedPatches1.insert(tracerFaceLabel1[i]);
        }
    }
    std::unordered_set<int> affectedPatches2;
    for (int i = 0; i < mesh2.face.size(); i++) {
        if (!preservedQuad2[i]) {
            affectedPatches2.insert(tracerFaceLabel2[i]);
        }
    }

    int minRectangleSide = ui.minRectangleSideSpinBox->value();

    start = chrono::steady_clock::now();

    //Maximum rectangles in the patches
    preservedFaceLabel1 = splitQuadPatchesInMaximumRectangles(mesh1, affectedPatches1, tracerFaceLabel1, preservedQuad1, minRectangleSide, true);
    preservedFaceLabel2 = splitQuadPatchesInMaximumRectangles(mesh2, affectedPatches2, tracerFaceLabel2, preservedQuad2, minRectangleSide, true);

    std::cout << std::endl << " >> "
              << "Max rectangle: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    bool mergeQuads = ui.mergeCheckBox->isChecked();
    bool deleteSmall = ui.deleteSmallCheckBox->isChecked();
    bool deleteNonConnected = ui.deleteNonConnectedCheckBox->isChecked();

    //Merge rectangular patches
    if (mergeQuads) {
        start = chrono::steady_clock::now();

        int nMerged1 = mergeQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
        int nMerged2 = mergeQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);

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
        int nSmall1 = deleteSmallQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
        int nSmall2 = deleteSmallQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Small deleted -> mesh 1: " << nSmall1 << " / mesh 2: " << nSmall2 << " in "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
    }

    if (deleteNonConnected) {
        start = chrono::steady_clock::now();

        //Delete non-connected patches
        int nNonConnected1 = deleteNonConnectedQuadPatches(mesh1, preservedFaceLabel1, preservedQuad1);
        int nNonConnected2 = deleteNonConnectedQuadPatches(mesh2, preservedFaceLabel2, preservedQuad2);

        std::cout << std::endl << " >> "
                  << "Non-connected deleted -> mesh 1: " << nNonConnected1 << " / mesh 2: " << nNonConnected2 << " in "
                  << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
                  << " ms" << std::endl;
    }



    start = chrono::steady_clock::now();

    //Get mesh of the preserved surface
    getPreservedSurfaceMesh(mesh1, mesh2, preservedQuad1, preservedQuad2, preservedFaceLabel1, preservedFaceLabel2, preservedSurface, preservedSurfaceLabel);

    //New mesh (to be decomposed in patch)
    getNewSurfaceMesh(boolean, triMesh1, triMesh2, preservedQuad1, preservedQuad2, J, newSurface);

    std::cout << std::endl << " >> "
              << "Get surfaces: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    start = chrono::steady_clock::now();

    quadDataPreserved1 = QuadBoolean::getQuadPatchesData(mesh1, preservedFaceLabel1);
    quadDataPreserved2 = QuadBoolean::getQuadPatchesData(mesh2, preservedFaceLabel2);

    std::cout << std::endl << " >> "
              << "Get quad data patches: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setPreservedSurface(&preservedSurface);
    ui.glArea->setNewSurface(&newSurface);
    ui.glArea->setQuadLayoutPreserved1(&quadDataPreserved1);
    ui.glArea->setQuadLayoutPreserved2(&quadDataPreserved2);

    colorizeMesh(preservedSurface, preservedSurfaceLabel);
}

void QuadBooleanWindow::getPatchDecomposition() {
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;


    start = chrono::steady_clock::now();

    newSurfaceLabel.resize(newSurface.face.size(), -1);

    //TODO DELETE
    newSurface.Clear();
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(newSurface, preservedSurface);
    std::vector<int> birthQuad = splitQuadInTriangle(newSurface);
    newSurfaceLabel.resize(newSurface.face.size(), -1);
    for (size_t i = 0; i < newSurface.face.size(); i++) {
        newSurfaceLabel[i] = preservedSurfaceLabel[birthQuad[i]];
    }
    vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(newSurface);
    vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(newSurface);
    vcg::tri::UpdateNormal<PolyMesh>::PerFaceNormalized(newSurface);
    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(newSurface);
    vcg::tri::UpdateTopology<PolyMesh>::FaceFace(newSurface);

    std::cout << std::endl << " >> "
              << "Decomposition: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    start = chrono::steady_clock::now();


    chartData = getCharts(newSurface, newSurfaceLabel);

    std::cout << std::endl << " >> "
              << "Get patches and sides: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    ui.glArea->setChartSides(&chartData);

    colorizeMesh(newSurface, newSurfaceLabel);
}


void QuadBooleanWindow::solveILP() {
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    //Solve ILP
    double alpha = ui.alphaSpinBox->value();
    ilpResult = solveChartSideILP(chartData, alpha);

    std::cout << std::endl << " >> "
              << "ILP: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setIlpResult(&ilpResult);
}

void QuadBooleanWindow::quadrangulateNewSurface()
{
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;



    start = chrono::steady_clock::now();

    quadrangulatedNewSurface.Clear();


    for (size_t i = 0; i < chartData.charts.size(); i++) {
        const Chart& chart = chartData.charts[i];

        if (chart.faces.size() == 0)
            continue;

        //Input mesh
        Eigen::MatrixXd chartV;
        Eigen::MatrixXi chartF;
        vcg::tri::UpdateFlags<PolyMesh>::FaceClearS(newSurface);
        vcg::tri::UpdateFlags<PolyMesh>::VertexClearS(newSurface);
        for (const size_t& fId : chart.faces) {
            newSurface.face[fId].SetS();
            for (int k = 0; k < newSurface.face[fId].VN(); k++) {
                newSurface.face[fId].V(k)->SetS();
            }
        }
        std::vector<int> vMap, fMap;
        VCGToEigenSelected(newSurface, chartV, chartF, vMap, fMap, 3);


        const std::vector<ChartSide>& chartSides = chart.chartSides;

        assert(chartSides.size() >= 3 && chartSides.size() <= 6);

        //Input subdivisions
        Eigen::VectorXi l(chartSides.size());

        std::vector<double> sideLengths(chartSides.size());
        std::vector<std::vector<size_t>> sides(chartSides.size());

        for (size_t i = 0; i < chartSides.size(); i++) {
            size_t targetSize = 0;
            for (const size_t& subSideId : chartSides[i].subsides) {
                targetSize += ilpResult[subSideId];
            }

            l(static_cast<int>(i)) = targetSize;
            sideLengths[i] = chartSides[i].length;

            sides[i].resize(chartSides[i].vertices.size());

            for (size_t j = 0; j < chartSides[i].vertices.size(); j++) {
                size_t vId = chartSides[i].vertices[j];
                assert(vMap[vId] >= 0);
                sides[i][j] = vMap[vId];
            }
        }

        //Pattern quadrangulation
        Eigen::MatrixXd patchV;
        Eigen::MatrixXi patchF;
        std::vector<size_t> patchBorders;
        std::vector<size_t> patchCorners;
        computePattern(l, patchV, patchF, patchBorders, patchCorners);

        std::vector<std::vector<size_t>> patchSides = getPatchSides(patchBorders, patchCorners, l);

        assert(chartSides.size() == patchCorners.size());
        assert(chartSides.size() == patchSides.size());

        Eigen::MatrixXd uvMap;
        Eigen::MatrixXd quadrangulationV;
        Eigen::MatrixXi quadrangulationF;
        computeQuadrangulation(chartV, chartF, patchV, patchF, sides, sideLengths, patchSides, uvMap, quadrangulationV, quadrangulationF);

        assert(chartV.rows() == uvMap.rows());

//        Eigen::MatrixXd uvMesh(uvMap.rows(), 3);
//        for (int i = 0; i < uvMap.rows(); i++) {
//            uvMesh(i, 0) = uvMap(i, 0);
//            uvMesh(i, 1) = uvMap(i, 1);
//            uvMesh(i, 2) = 0;
//        }

        PolyMesh quadrangulatedChartMesh;
        eigenToVCG(quadrangulationV, quadrangulationF, quadrangulatedChartMesh, 4);
        vcg::tri::io::ExporterOBJ<PolyMesh>::Save(quadrangulatedChartMesh, "results/chart.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
        vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(quadrangulatedNewSurface, quadrangulatedChartMesh, false);
//        vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(quadrangulatedNewSurface);
//        vcg::tri::Clean<PolyMesh>::MergeCloseVertex(quadrangulatedNewSurface, 0.00001);
//        vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(quadrangulatedNewSurface);
    }

    std::cout << std::endl << " >> "
              << "Quadrangulate new surface: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setQuadrangulated(&quadrangulatedNewSurface);
}

void QuadBooleanWindow::getResult()
{
    using namespace QuadBoolean;

    chrono::steady_clock::time_point start;


    start = chrono::steady_clock::now();


    //TODO HERE

    std::cout << std::endl << " >> "
              << "Get result: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    //    colorizeMesh(result, newSurfaceLabel);
}


void QuadBooleanWindow::on_loadMeshesPushButton_clicked()
{

    std::string filename1 = chooseMeshFile();
    if(!filename1.empty()) {
        std::string filename2 = chooseMeshFile();
        if(!filename2.empty()) {
            ui.glArea->setMesh1(nullptr);
            ui.glArea->setMesh2(nullptr);
            mesh1.Clear();
            mesh2.Clear();
            loadMesh(mesh1, filename1);
            loadMesh(mesh2, filename2);

            ui.showMesh1CheckBox->setChecked(true);
            ui.showMesh2CheckBox->setChecked(true);
            ui.showQuadLayout1CheckBox->setChecked(true);
            ui.showQuadLayout2CheckBox->setChecked(true);
            ui.showBooleanCheckBox->setChecked(false);
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
    traceQuads();

    quadData1 = QuadBoolean::getQuadPatchesData(mesh1, tracerFaceLabel1);
    quadData2 = QuadBoolean::getQuadPatchesData(mesh2, tracerFaceLabel2);

    ui.showMesh1CheckBox->setChecked(true);
    ui.showMesh2CheckBox->setChecked(true);
    ui.showQuadLayout1CheckBox->setChecked(true);
    ui.showQuadLayout2CheckBox->setChecked(true);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh1, "results/mesh1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh2, "results/mesh2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

}

void QuadBooleanWindow::on_computeBooleanPushButton_clicked()
{
    computeBooleans();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(boolean, "results/boolean.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(true);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_getPreservedQuadsPushButton_clicked()
{
    getPreservedQuads();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(newSurface, "results/newSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(preservedSurface, "results/preservedSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::on_decompositionPushButton_clicked()
{
    getPatchDecomposition();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(newSurface, "results/decomposedNewSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}


void QuadBooleanWindow::on_ilpPushButton_clicked()
{
    solveILP();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadBooleanWindow::on_quadrangulatePushButton_clicked()
{
    quadrangulateNewSurface();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(true);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(true);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(quadrangulatedNewSurface, "results/quadrangulatedNewSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadBooleanWindow::on_getResultPushButton_clicked()
{
    getResult();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(true);
    ui.showResultLayoutCheckBox->setChecked(true);

    updateVisibility();
    ui.glArea->updateGL();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, "results/result.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

}

void QuadBooleanWindow::on_computeAllPushButton_clicked()
{
    traceQuads();
    computeBooleans();
    getPreservedQuads();
    getPatchDecomposition();
    solveILP();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showQuadrangulatedCheckBox->setChecked(false);
    ui.showQuadrangulatedLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
    ui.showResultLayoutCheckBox->setChecked(false);

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
    ui.glArea->setPreservedSurfaceVisibility(ui.showPreservedSurfaceCheckBox->isChecked());
    ui.glArea->setNewSurfaceVisibility(ui.showNewSurfaceCheckBox->isChecked());
    ui.glArea->setQuadLayout1Visibility(ui.showQuadLayout1CheckBox->isChecked());
    ui.glArea->setQuadLayout2Visibility(ui.showQuadLayout2CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved1Visibility(ui.showQuadLayoutPreserved1CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved2Visibility(ui.showQuadLayoutPreserved2CheckBox->isChecked());
    ui.glArea->setChartSidesVisibility(ui.showChartSidesCheckBox->isChecked());
    ui.glArea->setQuadrangulatedVisibility(ui.showQuadrangulatedCheckBox->isChecked());
    ui.glArea->setQuadLayoutQuadrangulatedVisibility(ui.showQuadrangulatedLayoutCheckBox->isChecked());
    ui.glArea->setResultVisibility(ui.showResultCheckBox->isChecked());
    ui.glArea->setQuadLayoutResultVisibility(ui.showResultLayoutCheckBox->isChecked());
}
