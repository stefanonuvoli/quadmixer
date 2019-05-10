#include <iostream>
#include <cstring>

#include "quadmixerwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include <quadboolean/includes/quadlibiglbooleaninterface.h>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include "src/includes/envelope_generator.h"

#ifndef NDEBUG
#define SAVEMESHES
#endif

QuadMixerWindow::QuadMixerWindow(QWidget* parent) : QMainWindow(parent)
{
    ui.setupUi (this);

    updateVisibility();
    ui.glArea->updateGL();

    setLastOperation(nullptr, nullptr, nullptr, nullptr);
}

QuadMixerWindow::~QuadMixerWindow()
{
    for (PolyMesh* mesh : meshes) {
        hideMesh(mesh);
        if (mesh != nullptr) {
            delete mesh;
        }
    }
}

void QuadMixerWindow::booleanOperation()
{
    PolyMesh* target1 = ui.glArea->targetMesh1->mesh;
    PolyMesh* target2 = ui.glArea->targetMesh2->mesh;
    PolyMesh* booleanResult = new PolyMesh();

    QuadBoolean::Parameters parameters = getParametersFromUI();
    QuadBoolean::Operation operation = getOperationFromUI();

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(*target1, "res/booleanoperation1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(*target2, "res/booleanoperation2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    QuadBoolean::quadBoolean(*target1, *target2, operation, *booleanResult, parameters);

    hideMesh(target1);
    hideMesh(target2);
    addMesh(booleanResult);

    setLastOperation(target1, target2, booleanResult, nullptr);

    ui.glArea->updateGL();
}

void QuadMixerWindow::detachOperationSelect()
{    
    ui.glArea->setDetachMode(true);

    ui.glArea->updateGL();
}

void QuadMixerWindow::detachOperation()
{
    ui.glArea->deselectTransformationMesh();

    std::vector<PolyMesh::CoordType> points = ui.glArea->targetMesh1->pickedPoints;

    PolyMesh* target1 = ui.glArea->targetMesh1->mesh;
    PolyMesh* detachResult1 = new PolyMesh();
    PolyMesh* detachResult2 = new PolyMesh();

    PolyMesh targetCopy;
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(targetCopy, *target1);
    QuadBoolean::internal::splitQuadInTriangle(targetCopy);

    TriangleMesh targetTriangulated;
    vcg::tri::Append<TriangleMesh, PolyMesh>::Mesh(targetTriangulated, targetCopy);

    TriangleMesh detachedTriangleMesh1;
    TriangleMesh detachedTriangleMesh2;
    bool result = EnvelopeGenerator<TriangleMesh>::GenerateEnvelope(targetTriangulated, points, detachedTriangleMesh1, detachedTriangleMesh2, 2, 1, true, 0.02);

    if (result) {
        PolyMesh detachedPolyMesh1;
        PolyMesh detachedPolyMesh2;
        vcg::tri::Append<PolyMesh, TriangleMesh>::Mesh(detachedPolyMesh1, detachedTriangleMesh1);
        vcg::tri::Append<PolyMesh, TriangleMesh>::Mesh(detachedPolyMesh2, detachedTriangleMesh2);

    #ifdef SAVEMESHES
        vcg::tri::io::ExporterOBJ<PolyMesh>::Save(*target1, "res/detachedinput.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
        vcg::tri::io::ExporterOBJ<PolyMesh>::Save(detachedPolyMesh1, "res/detached1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
        vcg::tri::io::ExporterOBJ<PolyMesh>::Save(detachedPolyMesh2, "res/detached2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    #endif

        QuadBoolean::Parameters parameters = getParametersFromUI();
        parameters.intersectionSmoothingMaxBB = 1;
        parameters.intersectionSmoothingNRing = 1;
        parameters.intersectionSmoothingIterations = 2;
        parameters.resultSmoothingLaplacianNRing = 1;
        parameters.resultSmoothingLaplacianIterations = 2;
        QuadBoolean::quadBoolean(*target1, detachedPolyMesh1, QuadBoolean::Operation::INTERSECTION, *detachResult1, parameters);
        QuadBoolean::quadBoolean(*target1, detachedPolyMesh2, QuadBoolean::Operation::INTERSECTION, *detachResult2, parameters);

        addMesh(detachResult1);
        addMesh(detachResult2);

        hideMesh(target1);

        setLastOperation(target1, nullptr, detachResult1, detachResult2);
    }
    else {
        QMessageBox::warning(this, QString("Error"), QString("Detach points too close."));
    }

    ui.glArea->setDetachMode(true);

    ui.glArea->updateGL();
}

int QuadMixerWindow::addMesh(const string &filename)
{
    PolyMesh* mesh = new PolyMesh();

    loadMesh(*mesh, filename, ui.moveCenterCheckBox->isChecked(), ui.scaleCheckBox->isChecked());

    return addMesh(mesh);
}

int QuadMixerWindow::addMesh(PolyMesh* mesh)
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
        return -1;
    }

    size_t id = ui.glArea->addMesh(mesh);
    assert(id == meshes.size());

    this->meshes.push_back(mesh);

    return id;
}

void QuadMixerWindow::hideMesh(PolyMesh* mesh)
{
    return hideMesh(std::distance(meshes.begin(), std::find(meshes.begin(), meshes.end(), mesh)));
}


void QuadMixerWindow::hideMesh(const size_t& id)
{
    ui.glArea->removeMesh(id);
    meshes[id] = nullptr;
    ui.glArea->updateGL();
}

void QuadMixerWindow::setLastOperation(
        PolyMesh* target1,
        PolyMesh* target2,
        PolyMesh* result1,
        PolyMesh* result2)
{
    if (lastTarget1 != nullptr) {
        delete lastTarget1;
        lastTarget1 = nullptr;
    }
    if (lastTarget2 != nullptr) {
        delete lastTarget2;
        lastTarget2 = nullptr;
    }
    if (lastResult1 != nullptr) {
        lastResult1 = nullptr;
    }
    if (lastResult2 != nullptr) {
        lastResult2 = nullptr;
    }
    lastTarget1 = target1;
    lastTarget2 = target2;
    lastResult1 = result1;
    lastResult2 = result2;
}

void QuadMixerWindow::undoLastOperation()
{
    if (lastTarget1 != nullptr) {
        addMesh(lastTarget1);
        lastTarget1 = nullptr;
    }
    if (lastTarget2 != nullptr) {
        addMesh(lastTarget2);
        lastTarget2 = nullptr;
    }
    if (lastResult1 != nullptr) {
        hideMesh(lastResult1);
        delete lastResult1;
        lastResult1 = nullptr;
    }
    if (lastResult2 != nullptr) {
        hideMesh(lastResult2);
        delete lastResult2;
        lastResult2 = nullptr;
    }
}

QuadBoolean::Parameters QuadMixerWindow::getParametersFromUI()
{
    QuadBoolean::Parameters parameters;

    bool motorcycle = ui.motorcycleCheckBox->isChecked();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    int intersectionSmoothingNRing = ui.intersectionSmoothingNRingSpinBox->value();
    double intersectionSmoothingMaxBB = ui.intersectionSmoothingMaxBBSpinBox->value();

    int minRectangleArea = ui.minRectangleAreaSpinBox->value();
    int minPatchArea = ui.minPatchAreaSpinBox->value();
    bool mergeQuads = ui.mergeCheckBox->isChecked();
    bool deleteSmall = ui.deleteSmallCheckBox->isChecked();
    bool deleteNonConnected = ui.deleteNonConnectedCheckBox->isChecked();

    bool initialRemeshing = ui.initialRemeshingCheckBox->isChecked();
    double initialRemeshingEdgeFactor = ui.edgeFactorSpinBox->value();
    bool reproject = ui.reprojectCheckBox->isChecked();
    bool splitConcaves = ui.splitConcavesCheckBox->isChecked();
    bool finalSmoothing = ui.finalSmoothingCheckBox->isChecked();

    double alpha = ui.alphaSpinBox->value();
    double beta = ui.betaSpinBox->value();

    int chartSmoothingIterations = ui.chartSmoothingSpinBox->value();
    int quadrangulationSmoothingIterations = ui.quadrangulationSmoothingSpinBox->value();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    int resultSmoothingNRing = ui.resultSmoothingNRingSpinBox->value();

    int resultSmoothingLaplacianIterations = ui.resultSmoothingLaplacianSpinBox->value();
    int resultSmoothingLaplacianNRing = ui.resultSmoothingLaplacianNRingSpinBox->value();

    parameters.motorcycle = motorcycle;
    parameters.intersectionSmoothingIterations = intersectionSmoothingIterations;
    parameters.intersectionSmoothingNRing = intersectionSmoothingNRing;
    parameters.intersectionSmoothingMaxBB = intersectionSmoothingMaxBB;
    parameters.minRectangleArea = minRectangleArea;
    parameters.minPatchArea = minPatchArea;
    parameters.mergeQuads = mergeQuads;
    parameters.deleteSmall = deleteSmall;
    parameters.deleteNonConnected = deleteNonConnected;
    parameters.initialRemeshing = initialRemeshing;
    parameters.initialRemeshingEdgeFactor = initialRemeshingEdgeFactor;
    parameters.reproject = reproject;
    parameters.splitConcaves = splitConcaves;
    parameters.finalSmoothing = finalSmoothing;
    parameters.alpha = alpha;
    parameters.beta = beta;
    parameters.chartSmoothingIterations = chartSmoothingIterations;
    parameters.quadrangulationSmoothingIterations = quadrangulationSmoothingIterations;
    parameters.resultSmoothingIterations = resultSmoothingIterations;
    parameters.resultSmoothingNRing = resultSmoothingNRing;
    parameters.resultSmoothingLaplacianIterations = resultSmoothingLaplacianIterations;
    parameters.resultSmoothingLaplacianNRing = resultSmoothingLaplacianNRing;

    return parameters;
}

QuadBoolean::Operation QuadMixerWindow::getOperationFromUI()
{
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

    return operation;
}


std::string QuadMixerWindow::chooseMeshFile()
{

    std::cout << std::string(QDir::currentPath().toStdString() + "/../../QuadMixer/dataset") << std::endl;
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Open Mesh"), QDir::currentPath() + "/../../QuadMixer/dataset",
                                                    tr("Mesh (*.obj *.ply *.off)"));
    return filename.toStdString();
}

int QuadMixerWindow::loadMesh(PolyMesh& mesh, const std::string& filename, const bool translateCenter, const bool scale)
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

void QuadMixerWindow::updateVisibility()
{
    ui.glArea->setMesh1Visibility(ui.showMesh1CheckBox->isChecked());
    ui.glArea->setMesh2Visibility(ui.showMesh2CheckBox->isChecked());
    ui.glArea->setBooleanVisibility(ui.showBooleanCheckBox->isChecked());
    ui.glArea->setIntersectionVerticesVisibility(ui.showIntersectionVerticesCheckBox->isChecked());
    ui.glArea->setPreservedSurfaceVisibility(ui.showPreservedSurfaceCheckBox->isChecked());
    ui.glArea->setNewSurfaceVisibility(ui.showNewSurfaceCheckBox->isChecked());
    ui.glArea->setQuadLayout1Visibility(ui.showQuadLayout1CheckBox->isChecked());
    ui.glArea->setQuadLayout2Visibility(ui.showQuadLayout2CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved1Visibility(ui.showQuadLayoutPreserved1CheckBox->isChecked());
    ui.glArea->setQuadLayoutPreserved2Visibility(ui.showQuadLayoutPreserved2CheckBox->isChecked());
    ui.glArea->setChartSidesVisibility(ui.showChartSidesCheckBox->isChecked());
    ui.glArea->setILPVisibility(ui.showILPCheckBox->isChecked());
    ui.glArea->setQuadrangulationVisibility(ui.showQuadrangulationCheckBox->isChecked());
    ui.glArea->setQuadLayoutQuadrangulationVisibility(ui.showQuadrangulationLayoutCheckBox->isChecked());
    ui.glArea->setResultVisibility(ui.showResultCheckBox->isChecked());

    ui.glArea->setWireframe(ui.showWireframe->isChecked());

    ui.parametersFrame->setVisible(ui.showParametersCheckBox->isChecked());
    ui.debugFrame->setVisible(ui.glArea->debugMode);
}


void QuadMixerWindow::on_loadMeshButton_clicked()
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
void QuadMixerWindow::on_deleteMeshButton_clicked()
{
    if (ui.glArea->targetMesh1 != nullptr) {
        if (ui.glArea->targetMesh2 != nullptr) {
            QMessageBox::warning(this, QString("Error"), QString("Please select a single mesh to be deleted.."));
        }
        else {
            if (ui.glArea->targetMesh1->mesh == lastResult1 || ui.glArea->targetMesh1->mesh == lastResult2) {
                setLastOperation(nullptr, nullptr, nullptr, nullptr);
            }

            PolyMesh* mesh = ui.glArea->targetMesh1->mesh;
            hideMesh(ui.glArea->targetMesh1->mesh);
            delete mesh;
        }
    }
    else {
        QMessageBox::warning(this, QString("Error"), QString("Please select the mesh to be deleted."));
    }
}

void QuadMixerWindow::on_deleteAllButton_clicked()
{
    setLastOperation(nullptr, nullptr, nullptr, nullptr);
    for (size_t id = 0; id < meshes.size(); id++) {
        if (meshes[id] != nullptr) {
            hideMesh(id);
            delete meshes[id];
        }
    }
    ui.glArea->setDetachMode(false);
}


void QuadMixerWindow::on_saveMeshButton_clicked()
{
    if (ui.glArea->targetMesh1 != nullptr) {
        if (ui.glArea->targetMesh2 != nullptr) {
            QMessageBox::warning(this, QString("Error"), QString("Please select a single mesh to save."));
        }
        else {
            QString filename = QFileDialog::getSaveFileName(this,
                                                            tr("Save Mesh"),
                                                            QDir::currentPath() + "/../../QuadMixer/dataset",
                                                            tr("Mesh (*.obj *.ply *.off)"));

            vcg::tri::io::ExporterOBJ<PolyMesh>::Save(*ui.glArea->targetMesh1->mesh, filename.toStdString().c_str(), vcg::tri::io::Mask::IOM_FACECOLOR);
        }
    }
    else {
        QMessageBox::warning(this, QString("Error"), QString("Please select a mesh to save."));
    }
}

void QuadMixerWindow::on_booleanOperationButton_clicked()
{
    if (ui.glArea->targetMesh1 == nullptr || ui.glArea->targetMesh2 == nullptr) {
        QMessageBox::warning(this, QString("Error"), QString("Please select two meshes for the boolean operation."));
    }
    else {
        booleanOperation();
    }
}

void QuadMixerWindow::on_detachButton_clicked()
{
    if (ui.glArea->getDetachMode()) {
        if (ui.glArea->targetMesh1->pickedPoints.size() < 2) {
            QMessageBox::warning(this, QString("Error"), QString("Please select at least two vertices in the mesh for the detach operation."));
        }
        else if (ui.glArea->targetMesh1->pickedPoints.size() % 2 != 0) {
            QMessageBox::warning(this, QString("Error"), QString("Please select an even number of points."));
        }
        else {
            detachOperation();
        }
    }
    else {
        if (ui.glArea->targetMesh1 != nullptr) {
            if (ui.glArea->targetMesh2 != nullptr) {
                QMessageBox::warning(this, QString("Error"), QString("Please select a single mesh for the detach operation."));
            }
            else {
                detachOperationSelect();
            }
        }
        else {
            QMessageBox::warning(this, QString("Error"), QString("Please select a mesh for the detach operation."));
        }
    }
}

void QuadMixerWindow::on_abortDetachButton_clicked()
{    
    ui.glArea->setDetachMode(false);
    if (ui.glArea->targetMesh1 != nullptr)

    ui.glArea->updateGL();
}

void QuadMixerWindow::on_undoButton_clicked()
{
    if (lastTarget1 == nullptr && lastTarget1 == nullptr && lastResult1 == nullptr &&  lastResult2 == nullptr) {
        QMessageBox::warning(this, QString("Error"), QString("There is no operation to undo."));
    }
    else {
        undoLastOperation();
    }
}

void QuadMixerWindow::on_resetSceneButton_clicked()
{
    ui.glArea->resetSceneOnMeshes();
}


void QuadMixerWindow::on_trackballCheckBox_stateChanged(int arg1)
{
    ui.glArea->setTrackballVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showWireframe_stateChanged(int arg1)
{
    ui.glArea->setWireframe(arg1 == Qt::Checked);

    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showParametersCheckBox_stateChanged(int arg1)
{
    ui.parametersFrame->setVisible(arg1 == Qt::Checked);
}

void QuadMixerWindow::on_debugModeButton_clicked()
{
    ui.glArea->deselectTransformationMesh();

    if (!ui.glArea->debugMode) {
        if (ui.glArea->targetMesh1 == nullptr || ui.glArea->targetMesh2 == nullptr) {
            QMessageBox::warning(this, QString("Error"), QString("Select two meshes to switch to the debug mode."));
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







void QuadMixerWindow::doTraceQuads() {
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

void QuadMixerWindow::doComputeBooleans() {
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

    intersectionVertices = QuadBoolean::internal::getIntersectionVertices(
                VA, VB, VR,
                FA, FB, FR,
                J);


    std::cout << std::endl << " >> "
              << "Intersection vertices: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setBoolean(&boolean);

    ui.glArea->setIntersectionVertices(&intersectionVertices);


#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/trimesh1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/trimesh2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(boolean, "res/boolean.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadMixerWindow::doSmooth()
{
    booleanSmoothed.Clear();

    chrono::steady_clock::time_point start;

    start = chrono::steady_clock::now();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    int intersectionNRing = ui.intersectionSmoothingNRingSpinBox->value();
    double intersectionSmoothingMaxBB = ui.intersectionSmoothingMaxBBSpinBox->value();

    //Copy mesh
    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(booleanSmoothed, boolean);

    //Smooth along intersection vertices
    QuadBoolean::internal::smoothAlongIntersectionVertices(
                booleanSmoothed,
                VR,
                FR,
                intersectionVertices,
                intersectionSmoothingIterations,
                intersectionNRing,
                intersectionSmoothingMaxBB);

    vcg::tri::UpdateNormal<TriangleMesh>::PerFaceNormalized(booleanSmoothed);
    vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalized(booleanSmoothed);
    vcg::tri::UpdateBounding<TriangleMesh>::Box(booleanSmoothed);
    vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalizedPerFace(booleanSmoothed);

    std::cout << std::endl << " >> "
              << "Smooth along intersection vertices: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setBoolean(&booleanSmoothed);
}


void QuadMixerWindow::doGetSurfaces() {
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
                mesh1, mesh2,
                booleanSmoothed,
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
    QuadBoolean::internal::getNewSurfaceMesh(
                booleanSmoothed,
                mesh1, mesh2,
                preservedQuad1, preservedQuad2,
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

void QuadMixerWindow::doPatchDecomposition() {
    chrono::steady_clock::time_point start;

    newSurfacePartitions.clear();
    newSurfaceCorners.clear();
    newSurfaceLabel.clear();
    newSurface.Clear();
    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(newSurface, initialNewSurface);

    bool initialRemeshing = ui.initialRemeshingCheckBox->isChecked();
    double initialRemeshingEdgeFactor = ui.edgeFactorSpinBox->value();
    bool reproject = ui.reprojectCheckBox->isChecked();
    bool splitConcaves = ui.splitConcavesCheckBox->isChecked();
    bool finalSmoothing = ui.finalSmoothingCheckBox->isChecked();

    start = chrono::steady_clock::now();

    newSurfaceLabel = QuadBoolean::internal::getPatchDecomposition(newSurface, newSurfacePartitions, newSurfaceCorners, initialRemeshing, initialRemeshingEdgeFactor, reproject, splitConcaves, finalSmoothing);

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


void QuadMixerWindow::doSolveILP() {
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

void QuadMixerWindow::doQuadrangulate()
{
    chrono::steady_clock::time_point start;

    quadrangulation.Clear();
    quadrangulationLabel.clear();

    int chartSmoothingIterations = ui.chartSmoothingSpinBox->value();
    int quadrangulationSmoothingIterations = ui.quadrangulationSmoothingSpinBox->value();

    start = chrono::steady_clock::now();

    QuadBoolean::internal::quadrangulate(
                newSurface,
                chartData,
                ilpResult,
                chartSmoothingIterations,
                quadrangulationSmoothingIterations,
                quadrangulation,
                quadrangulationLabel);

    std::cout << std::endl << " >> "
              << "Quadrangulate new surface: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    colorizeMesh(quadrangulation, quadrangulationLabel);

    ui.glArea->setQuadrangulation(&quadrangulation);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(quadrangulation, "res/quadrangulation.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}

void QuadMixerWindow::doGetResult()
{
    chrono::steady_clock::time_point start;

    PolyMesh coloredPreservedSurface;
    PolyMesh coloredQuadrangulation;
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredPreservedSurface, preservedSurface);
    for (size_t i = 0; i < coloredPreservedSurface.face.size(); i++) {
        coloredPreservedSurface.face[i].C() = vcg::Color4b(200,255,200,255);
    }
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(coloredQuadrangulation, quadrangulation);
    for (size_t i = 0; i < coloredQuadrangulation.face.size(); i++) {
        coloredQuadrangulation.face[i].C() = vcg::Color4b(255,255,255,255);
    }

    //Clear data
    result.Clear();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    int resultSmoothingNRing = ui.resultSmoothingNRingSpinBox->value();

    int resultSmoothingLaplacianIterations = ui.resultSmoothingLaplacianSpinBox->value();
    int resultSmoothingLaplacianNRing = ui.resultSmoothingLaplacianNRingSpinBox->value();

    start = chrono::steady_clock::now();

    QuadBoolean::internal::getResult(coloredPreservedSurface, coloredQuadrangulation, result, booleanSmoothed, resultSmoothingIterations, resultSmoothingNRing, resultSmoothingLaplacianIterations, resultSmoothingLaplacianNRing);

    std::cout << std::endl << " >> "
              << "Get result: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, "res/result.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    ui.glArea->setResult(&result);
}

void QuadMixerWindow::clearVisualizationData()
{
    ui.glArea->setMesh1(nullptr);
    ui.glArea->setMesh2(nullptr);
    ui.glArea->setQuadLayout1(nullptr);
    ui.glArea->setQuadLayout2(nullptr);
    ui.glArea->setBoolean(nullptr);
    ui.glArea->setIntersectionVertices(nullptr);
    ui.glArea->setPreservedSurface(nullptr);
    ui.glArea->setQuadLayoutPreserved1(nullptr);
    ui.glArea->setQuadLayoutPreserved2(nullptr);
    ui.glArea->setNewSurface(nullptr);
    ui.glArea->setChartSides(nullptr);
    ui.glArea->setIlpResult(nullptr);
    ui.glArea->setQuadrangulation(nullptr);
    ui.glArea->setQuadLayoutQuadrangulation(nullptr);
    ui.glArea->setResult(nullptr);

    ui.showMesh1CheckBox->setChecked(true);
    ui.showMesh2CheckBox->setChecked(true);
    ui.showQuadLayout1CheckBox->setChecked(true);
    ui.showQuadLayout2CheckBox->setChecked(true);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);
}


void QuadMixerWindow::on_quadTracerButton_clicked()
{
    doTraceQuads();

    ui.showMesh1CheckBox->setChecked(true);
    ui.showMesh2CheckBox->setChecked(true);
    ui.showQuadLayout1CheckBox->setChecked(true);
    ui.showQuadLayout2CheckBox->setChecked(true);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_computeBooleanButton_clicked()
{
    doComputeBooleans();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(true);
    ui.showIntersectionVerticesCheckBox->setChecked(true);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_smoothButton_clicked()
{
    doSmooth();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(true);
    ui.showIntersectionVerticesCheckBox->setChecked(true);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_getSurfacesButton_clicked()
{
    doGetSurfaces();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_decompositionButton_clicked()
{
    doPatchDecomposition();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();

}


void QuadMixerWindow::on_ilpButton_clicked()
{
    doSolveILP();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(true);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(true);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(true);
    ui.showNewSurfaceCheckBox->setChecked(true);
    ui.showChartSidesCheckBox->setChecked(true);
    ui.showILPCheckBox->setChecked(true);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_quadrangulateButton_clicked()
{
    doQuadrangulate();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(true);
    ui.showQuadrangulationLayoutCheckBox->setChecked(true);
    ui.showResultCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_getResultButton_clicked()
{
    doGetResult();

    ui.showMesh1CheckBox->setChecked(false);
    ui.showMesh2CheckBox->setChecked(false);
    ui.showQuadLayout1CheckBox->setChecked(false);
    ui.showQuadLayout2CheckBox->setChecked(false);
    ui.showBooleanCheckBox->setChecked(false);
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(true);

    updateVisibility();
    ui.glArea->updateGL();


}

void QuadMixerWindow::on_computeAllButton_clicked()
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
    ui.showIntersectionVerticesCheckBox->setChecked(false);
    ui.showPreservedSurfaceCheckBox->setChecked(false);
    ui.showQuadLayoutPreserved1CheckBox->setChecked(false);
    ui.showQuadLayoutPreserved2CheckBox->setChecked(false);
    ui.showNewSurfaceCheckBox->setChecked(false);
    ui.showChartSidesCheckBox->setChecked(false);
    ui.showILPCheckBox->setChecked(false);
    ui.showQuadrangulationCheckBox->setChecked(false);
    ui.showQuadrangulationLayoutCheckBox->setChecked(false);
    ui.showResultCheckBox->setChecked(true);

    updateVisibility();
    ui.glArea->update();
}



void QuadMixerWindow::on_saveResultButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(
                this,
                tr("Save result"),
                QDir::currentPath() + "/../../QuadMixer/dataset",
                tr("Mesh (*.obj *.ply *.off)"));

    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, filename.toStdString().c_str(), vcg::tri::io::Mask::IOM_FACECOLOR);
}



void QuadMixerWindow::on_showMesh1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setMesh1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showMesh2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setMesh2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showBooleanCheckBox_stateChanged(int arg1)
{
    ui.glArea->setBooleanVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showIntersectionVerticesCheckBox_stateChanged(int arg1)
{
    ui.glArea->setIntersectionVerticesVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showPreservedSurfaceCheckBox_stateChanged(int arg1)
{

    ui.glArea->setPreservedSurfaceVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showNewSurfaceCheckBox_stateChanged(int arg1)
{
    ui.glArea->setNewSurfaceVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}


void QuadMixerWindow::on_showQuadLayout1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayout1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showQuadLayout2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayout2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showQuadLayoutPreserved1CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutPreserved1Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}


void QuadMixerWindow::on_showQuadLayoutPreserved2CheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutPreserved2Visibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showChartSidesCheckBox_stateChanged(int arg1)
{
    ui.glArea->setChartSidesVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showILPCheckBox_stateChanged(int arg1)
{
    ui.glArea->setILPVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showQuadrangulationCheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadrangulationVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showQuadrangulationLayoutCheckBox_stateChanged(int arg1)
{
    ui.glArea->setQuadLayoutQuadrangulationVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_showResultCheckBox_stateChanged(int arg1)
{
    ui.glArea->setResultVisibility(arg1 == Qt::Checked);
    ui.glArea->updateGL();
}


template<class MeshType>
void QuadMixerWindow::colorizeMesh(
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

