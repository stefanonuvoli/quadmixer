#include <iostream>
#include <cstring>
#include <random>

#include "quadmixerwindow.h"

#include <QFileDialog>
#include <QMessageBox>

#include "../quadmixer/includes/envelope_generator.h"

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

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


    chrono::steady_clock::time_point start;

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    QuadBoolean::quadBoolean(*target1, *target2, operation, *booleanResult, parameters);

    std::cout << " >> "
              << "Computed in: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    hideMesh(target1);
    hideMesh(target2);
    addMesh(booleanResult);

    setLastOperation(target1, target2, booleanResult, nullptr);
}

void QuadMixerWindow::detachOperationSelect()
{    
    ui.glArea->setDetachMode(true);
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
    QuadRetopology::internal::splitFacesInTriangles(targetCopy);

    TriangleMesh targetTriangulated;
    vcg::tri::Append<TriangleMesh, PolyMesh>::Mesh(targetTriangulated, targetCopy);

    TriangleMesh detachedTriangleMesh1;
    TriangleMesh detachedTriangleMesh2;
    bool result = EnvelopeGenerator<TriangleMesh>::GenerateEnvelope(targetTriangulated, points, detachedTriangleMesh1, detachedTriangleMesh2, 5, 1, true, 0.02);

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
        parameters.maxBB = 0.005;
        parameters.intersectionSmoothingNRing = 1;
        parameters.intersectionSmoothingIterations = 3;
        parameters.resultSmoothingLaplacianNRing = 2;
        parameters.resultSmoothingLaplacianIterations = 5;
        parameters.preservePolygons1 = true;
        parameters.preservePolygons2 = false;
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

    ui.glArea->setDetachMode(false);
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

            if (mesh->face[fId].VN() != 4) {
                mesh->face[fId].C() = vcg::Color4b(255,255,200,255);
            }
        }
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
    int id1 = -1;
    int id2 = -1;
    if (lastTarget1 != nullptr) {
        id1 = addMesh(lastTarget1);
        lastTarget1 = nullptr;
    }
    if (lastTarget2 != nullptr) {
        id2 = addMesh(lastTarget2);
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
    if (id1 >= 0) {
        ui.glArea->selectTargetMesh(id1);
    }
    if (id2 >= 0) {
        ui.glArea->selectTargetMesh(id2);
    }
}

QuadBoolean::Parameters QuadMixerWindow::getParametersFromUI()
{
    QuadBoolean::Parameters parameters;

    bool motorcycle = ui.motorcycleCheckBox->isChecked();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    double intersectionSmoothingNRing = ui.intersectionSmoothingNRingSpinBox->value();
    double maxBB = ui.maxBBSpinBox->value();
    bool preservePolygons1 = ui.preservePolygons1CheckBox->isChecked();
    bool preservePolygons2 = ui.preservePolygons2CheckBox->isChecked();

    bool patchRetraction = ui.patchRetractionCheckBox->isChecked();
    double patchRetractionNRing = ui.patchRetractionNRingSpinBox->value();

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
    bool polychordSolver = ui.polychordSolverCheckBox->isChecked();
    bool splitSolver = ui.splitSolverCheckBox->isChecked();

    QuadRetopology::ILPMethod ilpMethod;
    if (ui.ilpMethodLSRadio->isChecked()) {
        ilpMethod = QuadRetopology::ILPMethod::LEASTSQUARES;
    }
    else if (ui.ilpMethodABSRadio->isChecked()) {
        ilpMethod = QuadRetopology::ILPMethod::ABS;
    }
    else {
        ilpMethod = QuadRetopology::ILPMethod::LEASTSQUARES;
    }
    double alpha = ui.alphaSpinBox->value();

    int chartSmoothingIterations = ui.chartSmoothingSpinBox->value();
    int quadrangulationFixedSmoothingIterations = ui.quadrangulationFixedSmoothingSpinBox->value();
    int quadrangulationNonFixedSmoothingIterations = ui.quadrangulationNonFixedSmoothingSpinBox->value();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    double resultSmoothingNRing = ui.resultSmoothingNRingSpinBox->value();

    int resultSmoothingLaplacianIterations = ui.resultSmoothingLaplacianSpinBox->value();
    double resultSmoothingLaplacianNRing = ui.resultSmoothingLaplacianNRingSpinBox->value();


    parameters.motorcycle = motorcycle;
    parameters.intersectionSmoothingIterations = intersectionSmoothingIterations;
    parameters.intersectionSmoothingNRing = intersectionSmoothingNRing;
    parameters.maxBB = maxBB;
    parameters.preservePolygons1 = preservePolygons1;
    parameters.preservePolygons2 = preservePolygons2;
    parameters.patchRetraction = patchRetraction;
    parameters.patchRetractionNRing = patchRetractionNRing;
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
    parameters.polychordSolver = polychordSolver;
    parameters.splitSolver = splitSolver;
    parameters.ilpMethod = ilpMethod;
    parameters.alpha = alpha;
    parameters.chartSmoothingIterations = chartSmoothingIterations;
    parameters.quadrangulationFixedSmoothingIterations = quadrangulationFixedSmoothingIterations;
    parameters.quadrangulationNonFixedSmoothingIterations = quadrangulationFixedSmoothingIterations;
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
    ui.glArea->setOriginVerticesVisibility(ui.showOriginVerticesCheckBox->isChecked());

    ui.glArea->setWireframe(ui.showWireframe->isChecked());
    ui.glArea->setWireframeSize(ui.wireframeSlider->value());

    ui.parametersFrame->setVisible(ui.showParametersCheckBox->isChecked());
    ui.debugFrame->setVisible(ui.glArea->debugMode);
}


void QuadMixerWindow::on_loadMeshButton_clicked()
{
    std::string filename = chooseMeshFile();
    if(!filename.empty()) {
        addMesh(filename);

        ui.glArea->deselectTransformationMesh();

        updateVisibility();
        ui.glArea->updateGL();
    }
}
void QuadMixerWindow::on_deleteMeshButton_clicked()
{
    if (!ui.glArea->debugMode) {
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
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_deleteAllButton_clicked()
{
    if (ui.glArea->debugMode) {
        clearVisualizationData();

        mesh1.Clear();
        mesh2.Clear();

        ui.glArea->debugMode = false;
    }

    setLastOperation(nullptr, nullptr, nullptr, nullptr);
    for (size_t id = 0; id < meshes.size(); id++) {
        if (meshes[id] != nullptr) {
            hideMesh(id);
            delete meshes[id];
        }
    }
    ui.glArea->setDetachMode(false);

    ui.glArea->deselectTransformationMesh();
    ui.glArea->updateGL();
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
        ui.glArea->updateGL();
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
    ui.glArea->updateGL();
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
        ui.glArea->updateGL();
    }
}

void QuadMixerWindow::on_resetSceneButton_clicked()
{
    ui.glArea->resetSceneOnMeshes();
    ui.glArea->updateGL();
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

void QuadMixerWindow::on_wireframeSlider_valueChanged(int value)
{
    ui.glArea->setWireframeSize(value);
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

            colorMeshByComboBox(mesh1, ui.colorMeshesComboBox->currentIndex(), isPreserved1);
            colorMeshByComboBox(mesh2, ui.colorMeshesComboBox->currentIndex(), isPreserved2);
            colorResultByComboBox(result, ui.colorResultComboBox->currentIndex());

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
    ui.glArea->updateGL();
}



void QuadMixerWindow::doComputeBooleans() {

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh1, "res/mesh1.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh2, "res/mesh2.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    //Clear meshes
    trimesh1.Clear();
    trimesh2.Clear();
    boolean.Clear();
    birthFace1.clear();
    birthFace2.clear();

    chrono::steady_clock::time_point start;

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Triangulate
    QuadBoolean::internal::triangulateMesh(mesh1, trimesh1, birthFace1);
    QuadBoolean::internal::triangulateMesh(mesh2, trimesh2, birthFace2);

    std::cout << " >> "
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

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Boolean operation on trimeshes
    QuadBoolean::internal::computeBooleanOperation(
                trimesh1,
                trimesh2,
                operation,
                boolean,
                VA, VB, VR,
                FA, FB, FR,
                J);

    std::cout << " >> "
              << "Boolean: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    std::cout << std::endl;
    start = chrono::steady_clock::now();

    intersectionVertices = QuadBoolean::internal::getIntersectionVertices(
                VA, VB, VR,
                FA, FB, FR,
                J);

    birthTriangle = QuadBoolean::internal::getBirthTriangles(FA, FR, J);


    std::cout << " >> "
              << "Intersection vertices and birth triangles: "
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
    smoothedVertices.clear();

    chrono::steady_clock::time_point start;

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    int intersectionSmoothingIterations = ui.intersectionSmoothingSpinBox->value();
    double intersectionNRing = ui.intersectionSmoothingNRingSpinBox->value();
    double maxBB = ui.maxBBSpinBox->value();

    //Copy mesh
    vcg::tri::Append<TriangleMesh, TriangleMesh>::Mesh(booleanSmoothed, boolean);

    //Smooth along intersection vertices
    QuadBoolean::internal::smoothAlongIntersectionVertices(
                booleanSmoothed,
                intersectionVertices,
                intersectionSmoothingIterations,
                intersectionNRing,
                maxBB,
                smoothedVertices);

    vcg::tri::UpdateNormal<TriangleMesh>::PerFaceNormalized(booleanSmoothed);
    vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalized(booleanSmoothed);
    vcg::tri::UpdateBounding<TriangleMesh>::Box(booleanSmoothed);
    
    vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalizedPerFace(booleanSmoothed);

    std::cout << " >> "
              << "Smooth along intersection vertices: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setBoolean(&booleanSmoothed);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(booleanSmoothed, "res/booleanSmoothed.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
}


void QuadMixerWindow::doGetSurfaces() {


    //To make it deterministic (remeshing)
    if (!preservedSurface.IsEmpty()) {
        doSmooth();
    }

    //Clear data
    quadTracerLabel1.clear();
    quadTracerLabel2.clear();

    isPreserved1.clear();
    isPreserved2.clear();

    preservedSurface.Clear();
    preservedSurfaceLabel.clear();
    meshLabel1.clear();
    meshLabel2.clear();

    preservedBirthVertexInfo.clear();
    preservedBirthVertexInfo.clear();

    isNewSurface.clear();
    newSurface.Clear();


    bool motorcycle = ui.motorcycleCheckBox->isChecked();
    bool patchRetraction = ui.patchRetractionCheckBox->isChecked();
    double patchRetractionNRing = ui.patchRetractionNRingSpinBox->value();
    int minRectangleArea = ui.minRectangleAreaSpinBox->value();
    int minPatchArea = ui.minPatchAreaSpinBox->value();
    bool mergeQuads = ui.mergeCheckBox->isChecked();
    bool deleteSmall = ui.deleteSmallCheckBox->isChecked();
    bool deleteNonConnected = ui.deleteNonConnectedCheckBox->isChecked();
    double maxBB = ui.maxBBSpinBox->value();
    bool preservePolygons1 = ui.preservePolygons1CheckBox->isChecked();
    bool preservePolygons2 = ui.preservePolygons2CheckBox->isChecked();

    chrono::steady_clock::time_point start;

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    QuadBoolean::internal::getSurfaces(
            mesh1,
            mesh2,
            booleanSmoothed,
            birthTriangle,
            birthFace1,
            birthFace2,
            intersectionVertices,
            smoothedVertices,
            motorcycle,
            patchRetraction,
            patchRetractionNRing,
            minRectangleArea,
            minPatchArea,
            mergeQuads,
            deleteSmall,
            deleteNonConnected,
            maxBB,
            preservePolygons1,
            preservePolygons2,
            meshLabel1,
            meshLabel2,
            isPreserved1,
            isPreserved2,
            isNewSurface,
            preservedSurfaceLabel,
            preservedBirthVertexInfo,
            preservedBirthVertexInfo,
            preservedSurface,
            newSurface);

    std::cout << " >> "
              << "Get surfaces: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


    //Trace quads following singularities
    QuadBoolean::internal::tracePatchLayout(mesh1, quadTracerLabel1, motorcycle);
    QuadBoolean::internal::tracePatchLayout(mesh2, quadTracerLabel2, motorcycle);

    quadLayoutData1 = QuadBoolean::internal::getQuadLayoutData(mesh1, quadTracerLabel1);
    ui.glArea->setQuadLayout1(&quadLayoutData1);

    quadLayoutData2 = QuadBoolean::internal::getQuadLayoutData(mesh2, quadTracerLabel2);
    ui.glArea->setQuadLayout2(&quadLayoutData2);


    //Quad data layout of the preserved
    quadLayoutDataPreserved1 = QuadBoolean::internal::getQuadLayoutData(mesh1, meshLabel1);
    ui.glArea->setQuadLayoutPreserved1(&quadLayoutDataPreserved1);

    quadLayoutDataPreserved2 = QuadBoolean::internal::getQuadLayoutData(mesh2, meshLabel2);
    ui.glArea->setQuadLayoutPreserved2(&quadLayoutDataPreserved2);


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

    //To make it deterministic (remeshing)
    if (!newSurfaceLabel.empty()) {
        doGetSurfaces();
    }

    newSurfacePartitions.clear();
    newSurfaceCorners.clear();
    newSurfaceLabel.clear();

    bool initialRemeshing = ui.initialRemeshingCheckBox->isChecked();
    double initialRemeshingEdgeFactor = ui.edgeFactorSpinBox->value();
    bool reproject = ui.reprojectCheckBox->isChecked();
    bool splitConcaves = ui.splitConcavesCheckBox->isChecked();
    bool finalSmoothing = ui.finalSmoothingCheckBox->isChecked();
    bool polychordSolver = ui.polychordSolverCheckBox->isChecked();
    bool splitSolver = ui.splitSolverCheckBox->isChecked();


    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Make ILP feasible
    QuadBoolean::internal::makeILPFeasible(preservedSurface, newSurface, polychordSolver, splitSolver);

    std::cout << " >> "
              << "Make ILP feasible: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;


#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(newSurface, "res/feasibilityNewSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(preservedSurface, "res/feasibilityPreservedSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    newSurfaceLabel = QuadRetopology::patchDecomposition(newSurface, preservedSurface, newSurfacePartitions, newSurfaceCorners, initialRemeshing, initialRemeshingEdgeFactor, reproject, splitConcaves, finalSmoothing);

    //-----------

    std::cout << " >> "
              << "Decomposition: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    colorizeMesh(newSurface, newSurfaceLabel);

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<TriangleMesh>::Save(newSurface, "res/decomposedNewSurface.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif


    std::cout << std::endl;
    start = chrono::steady_clock::now();


    chartData = chartData = QuadRetopology::computeChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

    std::cout << " >> "
              << "Get patches and sides: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setChartSides(&chartData);
}


void QuadMixerWindow::doSolveILP() {
    chrono::steady_clock::time_point start;

    //Solve ILP
    double alpha = ui.alphaSpinBox->value();

    QuadRetopology::ILPMethod ilpMethod;
    if (ui.ilpMethodLSRadio->isChecked()) {
        ilpMethod = QuadRetopology::ILPMethod::LEASTSQUARES;
    }
    else if (ui.ilpMethodABSRadio->isChecked()) {
        ilpMethod = QuadRetopology::ILPMethod::ABS;
    }


    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Select the subsides to fix
    ilpResults.resize(chartData.subsides.size(), ILP_FIND_SUBDIVISION);
    for (size_t subsideId = 0; subsideId < chartData.subsides.size(); ++subsideId) {
        QuadRetopology::ChartSubside& subside = chartData.subsides[subsideId];
        if (subside.isOnBorder) {
            fixedPositionSubsides.push_back(subsideId);
            ilpResults[subsideId] = subside.size;
        }
    }

    //Get chart length
    std::vector<double> chartEdgeLength = QuadRetopology::computeChartEdgeLength(chartData, 5, ilpResults, 0.7);

    //Solve ILP to find best side size
    double gap;

    std::vector<float> callbackTimeLimit = { 2.00, 3.000, 5.0, 7.0, 8.0, 10.0, 15.0, 20.0 };
    std::vector<float> callbackGapLimit = { 0.001, 0.005, 0.01, 0.05, 0.10, 0.15, 0.20, 0.300 };

    //Solve ILP to find best side size
    QuadRetopology::findSubdivisions(
            chartData,
            chartEdgeLength,
            ilpMethod,                              //method
            alpha,                                  //alpha
            true,                                   //isometry
            true,                                   //regularityForQuadrilaterals
            false,                                  //regularityForNonQuadrilaterals
            0.9,                                    //regularityNonQuadrilateralWeight
            false,                                  //alignSingularities
            0.1,                                    //alignSingularitiesWeight
            0,                                      //repeatLosingConstraintsIterations
            false,                                  //repeatLosingConstraintsQuads
            false,                                  //repeatLosingConstraintsNonQuads
            false,                                  //repeatLosingConstraintsAlign
            false,                                  //feasibilityFix
            true,                                   //hardParityConstraint
            21,                                     //timeLimit
            0.0,                                    //gapLimit
            callbackTimeLimit,                      //callbackTimeLimit
            callbackGapLimit,                       //callbackGapLimit
            0.3,                                    //minimumGap
            gap,
            ilpResults);

    std::cout << " >> "
              << "ILP: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

    ui.glArea->setIlpResult(&ilpResults);
}

void QuadMixerWindow::doQuadrangulate()
{
    chrono::steady_clock::time_point start;

    quadrangulation.Clear();
    quadrangulationLabel.clear();

    int chartSmoothingIterations = ui.chartSmoothingSpinBox->value();
    int quadrangulationFixedSmoothingIterations = ui.quadrangulationFixedSmoothingSpinBox->value();
    int quadrangulationNonFixedSmoothingIterations = ui.quadrangulationNonFixedSmoothingSpinBox->value();

    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Quadrangulate,
    QuadRetopology::quadrangulate(
                newSurface,
                chartData,
                fixedPositionSubsides,
                ilpResults,
                chartSmoothingIterations,
                quadrangulationFixedSmoothingIterations,
                quadrangulationNonFixedSmoothingIterations,
                true,
                quadrangulation,
                quadrangulationLabel,
                quadrangulationPartitions,
                quadrangulationCorners);

    std::cout << " >> "
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

    //Clear data
    result.Clear();

    int resultSmoothingIterations = ui.resultSmoothingSpinBox->value();
    double resultSmoothingNRing = ui.resultSmoothingNRingSpinBox->value();

    int resultSmoothingLaplacianIterations = ui.resultSmoothingLaplacianSpinBox->value();
    double resultSmoothingLaplacianNRing = ui.resultSmoothingLaplacianNRingSpinBox->value();


    std::cout << std::endl;
    start = chrono::steady_clock::now();

    //Get the result
    QuadRetopology::computeResult(
                preservedSurface, quadrangulation,
                result, boolean,
                false,
                resultSmoothingIterations,
                resultSmoothingNRing,
                resultSmoothingLaplacianIterations,
                resultSmoothingLaplacianNRing,
                resultPreservedVertexMap, resultPreservedFaceMap);

    std::cout << " >> "
              << "Get result: "
              << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count()
              << " ms" << std::endl;

#ifdef SAVEMESHES
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(result, "res/result.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif
    for (size_t i = 0; i < result.face.size(); i++) {
        result.face[i].C() = vcg::Color4b(255,255,255,255);
    }

    colorResultByComboBox(result, ui.colorResultComboBox->currentIndex());

    ui.glArea->setResult(&result);
    ui.glArea->setOriginVertices(&sourceInfo.newVertices);
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
    ui.glArea->setOriginVertices(nullptr);

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
    ui.showOriginVerticesCheckBox->setChecked(false);
    ui.colorMeshesComboBox->setCurrentIndex(0);
    ui.colorResultComboBox->setCurrentIndex(0);
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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

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
    ui.showOriginVerticesCheckBox->setChecked(false);

    updateVisibility();
    ui.glArea->updateGL();


}

void QuadMixerWindow::on_computeAllButton_clicked()
{
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
    ui.showOriginVerticesCheckBox->setChecked(false);

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

void QuadMixerWindow::on_showOriginVerticesCheckBox_stateChanged(int arg1)
{
    ui.glArea->setOriginVerticesVisibility(arg1 == Qt::Checked);
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


template<class MeshType>
void QuadMixerWindow::colorMeshByComboBox(MeshType& mesh, int index, const std::vector<std::pair<bool, bool>>& preservedFace) {
    if (mesh.face.size() == 0)
        return;

    QuadBoolean::internal::QuadMeshTracer<MeshType> quadTracer(mesh);
    switch (index) {
    case 1:
        for (size_t i = 0; i < mesh.face.size(); i++) {
            if (preservedFace.empty() || !preservedFace[i].first)
                mesh.face[i].C() = vcg::Color4b(255,255,255,255);
            else
                mesh.face[i].C() = vcg::Color4b(200,255,200,255);
        }
        break;
    case 2:
        quadTracer.MotorCycle=true;
        quadTracer.updatePolymeshAttributes();
        quadTracer.TracePartitions();
        colorizeMesh(mesh, quadTracer.FacePatch);
        break;
    case 3:
        quadTracer.MotorCycle=false;
        quadTracer.updatePolymeshAttributes();
        quadTracer.TracePartitions();
        colorizeMesh(mesh, quadTracer.FacePatch);
        break;
    case 4:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QAngle);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;
    case 5:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QPlanar);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;
    case 6:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QTemplate);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;
    case 7:
        if (&mesh1 == &mesh) {
            for (size_t i = 0; i < mesh.face.size(); i++) {
                vcg::glColor(vcg::Color4b(200,255,200,255));
            }
        }
        else {
            for (size_t i = 0; i < mesh.face.size(); i++) {
                vcg::glColor(vcg::Color4b(200,200,255,255));
            }
        }
        break;

    default:
        for (size_t i = 0; i < mesh.face.size(); i++) {
            mesh.face[i].C() = vcg::Color4b(255,255,255,255);
        }
    }
}

template<class MeshType>
void QuadMixerWindow::colorResultByComboBox(MeshType& mesh, int index) {
    if (mesh.face.size() == 0)
        return;

    QuadBoolean::internal::QuadMeshTracer<MeshType> quadTracer(mesh);
    switch (index) {
    case 1:
        for (const std::pair<size_t, QuadBoolean::OriginEntity> oldFaceEntry : sourceInfo.oldFacesMap) {
            size_t faceId = oldFaceEntry.first;
            const QuadBoolean::OriginEntity& originEntity = oldFaceEntry.second;
            if (originEntity.meshId == 1) {
                mesh.face[faceId].C() = vcg::Color4b(200,255,200,255);
            }
            else if (originEntity.meshId == 2) {
                mesh.face[faceId].C() = vcg::Color4b(200,200,255,255);
            }
            else {
                assert(originEntity.meshId == 0);
                mesh.face[faceId].C() = vcg::Color4b(255,200,200,255);
            }
        }
        for (size_t& newFaceId : sourceInfo.newFaces) {
            mesh.face[newFaceId].C() = vcg::Color4b(255,255,255,255);
        }
        break;
    case 2:
        quadTracer.MotorCycle=true;
        quadTracer.updatePolymeshAttributes();
        quadTracer.TracePartitions();
        colorizeMesh(mesh, quadTracer.FacePatch);
        break;
    case 3:
        quadTracer.MotorCycle=false;
        quadTracer.updatePolymeshAttributes();
        quadTracer.TracePartitions();
        colorizeMesh(mesh, quadTracer.FacePatch);
        break;
    case 4:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QAngle);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;
    case 5:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QPlanar);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;
    case 6:
        vcg::PolygonalAlgorithm<MeshType>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QTemplate);
        vcg::tri::UpdateColor<MeshType>::UpdateColor::PerFaceQualityRamp(mesh, 1, 0);
        break;

    default:
        for (size_t i = 0; i < mesh.face.size(); i++) {
            mesh.face[i].C() = vcg::Color4b(255,255,255,255);
        }
    }
}

void QuadMixerWindow::on_colorResultComboBox_currentIndexChanged(int index)
{
    colorResultByComboBox(result, index);
    ui.glArea->updateGL();
}

void QuadMixerWindow::on_colorMeshesComboBox_currentIndexChanged(int index)
{
    colorMeshByComboBox(mesh1, index, isPreserved1);
    colorMeshByComboBox(mesh2, index, isPreserved2);
    ui.glArea->updateGL();
}


void QuadMixerWindow::on_saveMesh1Button_clicked()
{
    QString filename = QFileDialog::getSaveFileName(
                this,
                tr("Save result"),
                QDir::currentPath() + "/../../QuadMixer/dataset",
                tr("Mesh (*.obj *.ply *.off)"));

    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh1, filename.toStdString().c_str(), vcg::tri::io::Mask::IOM_FACECOLOR);
}

void QuadMixerWindow::on_saveMesh2Button_clicked()
{
    QString filename = QFileDialog::getSaveFileName(
                this,
                tr("Save result"),
                QDir::currentPath() + "/../../QuadMixer/dataset",
                tr("Mesh (*.obj *.ply *.off)"));

    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh2, filename.toStdString().c_str(), vcg::tri::io::Mask::IOM_FACECOLOR);
}

void QuadMixerWindow::on_autoRotateButton_clicked()
{
    ui.glArea->autoRotate();
}

void QuadMixerWindow::on_continuityTestButton_clicked()
{
    double dimY = ui.glArea->targetMesh1->mesh->bbox.DimY() - ui.glArea->targetMesh1->mesh->bbox.DimY()*1.3/3;
    continuityLength = dimY;
    continuityOffset = 0;

    QTimer::singleShot(0, this, SLOT(continuityTest()));
}


void QuadMixerWindow::saveScreenshot(const std::string& filename) {
    int windowWidth = ui.glArea->width();
    int windowHeight = ui.glArea->height();
    const int numberOfPixels = windowWidth * windowHeight * 3;
    unsigned char pixels[numberOfPixels];

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, windowWidth, windowHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);

    FILE *outputFile = fopen(filename.c_str(), "w");
    short header[] = {0, 2, 0, 0, 0, 0, (short) windowWidth, (short) windowHeight, 24};

    fwrite(&header, sizeof(header), 1, outputFile);
    fwrite(pixels, numberOfPixels, 1, outputFile);
    fclose(outputFile);
}

void QuadMixerWindow::continuityTest()
{
    if (continuityOffset > continuityLength || ui.glArea->targetMesh1 == nullptr || ui.glArea->targetMesh2 == nullptr) {
        undoLastOperation();
        return;
    }

    PolyMesh* target2 = ui.glArea->targetMesh2->mesh;
    PolyMesh* target1 = ui.glArea->targetMesh1->mesh;

    double continuityStep = continuityLength/40;
    vcg::Point3d tr(0, -continuityStep, 0);

    QuadBoolean::Parameters parameters = getParametersFromUI();
    QuadBoolean::Operation operation = getOperationFromUI();

    std::vector<vcg::Point3d> translateVectors;

//    for (double i = 0; i <= 0; i += 0.5) {
//        for (double j = -3; j <= 3; j += 0.5) {
//            for (double k = 0; k <= 0; k += 0.5) {
//                translateVectors.push_back(continuityStep*vcg::Point3d(i, j, k));
//            }
//        }
//    }
    translateVectors.push_back(vcg::Point3d(0,0,0));
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> rand(0,1);
    for (double i = 0; i < 20; i++) {
        translateVectors.push_back(vcg::Point3d(rand(rng), rand(rng), rand(rng))/1000);
    }

    static int nResult = 0;

    PolyMesh exactResult;    

    for (size_t i = 0; i < target2->vert.size(); i++) {
        target2->vert[i].P() += tr;
    }

    QuadBoolean::quadBoolean(*target1, *target2, operation, exactResult, parameters);


    PolyMesh* drawMeshExact = new PolyMesh();
    vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(*drawMeshExact, exactResult);

    hideMesh(target1);
    hideMesh(target2);
    addMesh(drawMeshExact);

    setLastOperation(target1, target2, drawMeshExact, nullptr);

    ui.glArea->updateGL();

    std::string filenameExact("img/exactresult_" + std::to_string(nResult) + ".tga");
    saveScreenshot(filenameExact);

    undoLastOperation();

    vcg::tri::UpdateNormal<PolyMesh>::PerFaceNormalized(exactResult);
    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(exactResult);
    vcg::tri::UpdateBounding<PolyMesh>::Box(exactResult);
    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(exactResult);

    vcg::tri::UpdateBounding<PolyMesh>::Box(exactResult);
    typename PolyMesh::ScalarType maxD=exactResult.bbox.Diag();
    typename PolyMesh::ScalarType minD=0;

    vcg::GridStaticPtr<typename PolyMesh::FaceType,typename PolyMesh::FaceType::ScalarType> Grid;
    Grid.Set(exactResult.face.begin(),exactResult.face.end());

    PolyMesh bestResult;
    double bestScore = std::numeric_limits<double>::max();
    int nImage = 0;
    for (const vcg::Point3d& vec : translateVectors) {
        PolyMesh tmpTarget1, tmpTarget2, tmpResult;
        vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(tmpTarget1, *target1);
        vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(tmpTarget2, *target2);

        for (size_t i = 0; i < tmpTarget2.vert.size(); i++) {
            tmpTarget2.vert[i].P() += vec;
        }

        QuadBoolean::quadBoolean(tmpTarget1, tmpTarget2, operation, tmpResult, parameters);


        double qualityScore = 0;
        vcg::PolygonalAlgorithm<PolyMesh>::UpdateQuality(tmpResult, vcg::PolygonalAlgorithm<PolyMesh>::QTemplate);
        for (size_t i = 0; i < tmpResult.face.size(); i++) {
            qualityScore += tmpResult.face[i].Q();
        }
        qualityScore /= tmpResult.face.size();

        double singScore = 0;
        vcg::tri::UpdateQuality<PolyMesh>::VertexValence(tmpResult);
        for (size_t i = 0; i < tmpResult.vert.size(); i++) {
            singScore += std::fabs(4.0 - tmpResult.vert[i].Q());
        }
        singScore /= tmpResult.vert.size();

//        //Reproject
//        for (size_t i=0;i<tmpResult.vert.size();i++)
//        {
//            typename PolyMesh::CoordType closestPT;
//            typename PolyMesh::FaceType *f=
//                    vcg::tri::GetClosestFaceBase<PolyMesh>(
//                        exactResult,
//                        Grid,
//                        tmpResult.vert[i].P(),
//                        maxD,minD,
//                        closestPT);

//            tmpResult.vert[i].P()=closestPT;
//        }

        double score = 0.3*qualityScore + 0.7*singScore;
        if (score < bestScore) {
            bestScore = score;
            bestResult.Clear();
            vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(bestResult, tmpResult);
        }

        PolyMesh* drawMeshFrame = new PolyMesh();
        vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(*drawMeshFrame, tmpResult);


        hideMesh(target1);
        hideMesh(target2);
        addMesh(drawMeshFrame);

        setLastOperation(target1, target2, drawMeshFrame, nullptr);

        ui.glArea->updateGL();

        std::string filename("img/frame_" + std::to_string(nResult) + "_attempt_" + std::to_string(nImage++) + ".tga");
        saveScreenshot(filename);

        undoLastOperation();
    }


//    //Reproject
//    for (size_t i=0;i<bestResult.vert.size();i++)
//    {
//        typename PolyMesh::CoordType closestPT;
//        typename PolyMesh::FaceType *f=
//                vcg::tri::GetClosestFaceBase<PolyMesh>(
//                    exactResult,
//                    Grid,
//                    bestResult.vert[i].P(),
//                    maxD,minD,
//                    closestPT);

//        bestResult.vert[i].P()=closestPT;
//    }

    PolyMesh* drawMeshResult = new PolyMesh();
    vcg::tri::Append<PolyMesh,PolyMesh>::Mesh(*drawMeshResult, bestResult);

    hideMesh(target1);
    hideMesh(target2);
    addMesh(drawMeshResult);

    setLastOperation(target1, target2, drawMeshResult, nullptr);

    ui.glArea->updateGL();

    std::string filename("img/result_" + std::to_string(nResult++) + ".tga");
    saveScreenshot(filename);

    undoLastOperation();

    continuityOffset += continuityStep;
    QTimer::singleShot(0, this, SLOT(continuityTestMetric()));
}


void QuadMixerWindow::on_generalRetractionSpinBox_valueChanged(double arg1)
{
    ui.patchRetractionNRingSpinBox->setValue(arg1);
    ui.intersectionSmoothingNRingSpinBox->setValue(arg1);
}

void QuadMixerWindow::on_generalMaxBBSpinBox_valueChanged(double arg1)
{
    ui.maxBBSpinBox->setValue(arg1);
}

void QuadMixerWindow::on_generalSmallPatchSpinBox_valueChanged(int arg1)
{
    ui.minPatchAreaSpinBox->setValue(arg1);

}

void QuadMixerWindow::on_generalAlphaSpinBox_valueChanged(double arg1)
{
    ui.alphaSpinBox->setValue(arg1);
}

void QuadMixerWindow::on_generalSmoothingSpinBox_valueChanged(int arg1)
{
    ui.resultSmoothingSpinBox->setValue(arg1 * 2);
    ui.resultSmoothingLaplacianSpinBox->setValue(arg1);
}

void QuadMixerWindow::on_generalSmoothingPropagationSpinBox_valueChanged(double arg1)
{
    ui.resultSmoothingNRingSpinBox->setValue(arg1);
    ui.resultSmoothingLaplacianNRingSpinBox->setValue(arg1);
}

void QuadMixerWindow::on_generalLSRadio_toggled(bool checked)
{
    ui.ilpMethodLSRadio->setChecked(checked);
}

void QuadMixerWindow::on_generalABSRadio_toggled(bool checked)
{
    ui.ilpMethodABSRadio->setChecked(checked);
}
