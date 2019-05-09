#ifndef QUADBOOLEANWINDOW_H
#define QUADBOOLEANWINDOW_H

#include "ui_quadbooleanwindow.h"

#include "quadboolean.h"
#include "meshtypes.h"

class QuadBooleanWindow : public QMainWindow
{
Q_OBJECT 

private:

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::TriangleMesh TriangleMesh;
    typedef QuadBoolean::internal::QuadLayoutData<PolyMesh> QuadLayoutData;
    typedef QuadBoolean::internal::ChartData TriangleChartData;


public:

    QuadBooleanWindow(QWidget * parent = nullptr);
    ~QuadBooleanWindow();

    void booleanOperation();
    void detachOperationSelect();
    void detachOperation();

    void addMesh(const std::string& filename);
    void addMesh(PolyMesh *mesh);
    void hideMesh(PolyMesh* mesh);
    void hideMesh(const size_t& id);

    void setLastOperation(
            PolyMesh* target1,
            PolyMesh* target2,
            PolyMesh* result1,
            PolyMesh* result2);
    void undoLastOperation();

private:

    Ui::mainWindow ui;
    std::vector<PolyMesh*> meshes;

    PolyMesh* lastTarget1;
    PolyMesh* lastTarget2;
    PolyMesh* lastResult1;
    PolyMesh* lastResult2;

    QuadBoolean::Parameters getParametersFromUI();
    QuadBoolean::Operation getOperationFromUI();

    std::string chooseMeshFile();
    int loadMesh(PolyMesh& mesh, const std::string& filename, const bool translateCenter, const bool scale);

    void updateVisibility();


private slots:

    void on_loadMeshButton_clicked();

    void on_booleanOperationButton_clicked();
    void on_detachButton_clicked();
    void on_undoButton_clicked();

    void on_deleteMeshButton_clicked();
    void on_deleteAllButton_clicked();

    void on_resetSceneButton_clicked();
    void on_trackballCheckBox_stateChanged(int arg1);
    void on_showWireframe_stateChanged(int arg1);
    void on_showParametersCheckBox_stateChanged(int arg1);    
    void on_debugModeButton_clicked();

private:

    void doTraceQuads();
    void doComputeBooleans();
    void doSmooth();
    void doGetSurfaces();
    void doPatchDecomposition();
    void doSolveILP();
    void doQuadrangulate();
    void doGetResult();

    void clearVisualizationData();

private slots:

    void on_quadTracerButton_clicked();
    void on_computeBooleanButton_clicked();
    void on_smoothButton_clicked();
    void on_getSurfacesButton_clicked();
    void on_decompositionButton_clicked();
    void on_ilpButton_clicked();
    void on_computeAllButton_clicked();
    void on_quadrangulateButton_clicked();
    void on_getResultButton_clicked();
    void on_saveResultButton_clicked();

    void on_showMesh1CheckBox_stateChanged(int arg1);
    void on_showMesh2CheckBox_stateChanged(int arg1);
    void on_showBooleanCheckBox_stateChanged(int arg1);
    void on_showIntersectionCurvesCheckBox_stateChanged(int arg1);
    void on_showPreservedSurfaceCheckBox_stateChanged(int arg1);
    void on_showNewSurfaceCheckBox_stateChanged(int arg1);
    void on_showQuadLayoutPreserved1CheckBox_stateChanged(int arg1);
    void on_showQuadLayoutPreserved2CheckBox_stateChanged(int arg1);
    void on_showQuadLayout1CheckBox_stateChanged(int arg1);
    void on_showQuadLayout2CheckBox_stateChanged(int arg1);
    void on_showChartSidesCheckBox_stateChanged(int arg1);
    void on_showILPCheckBox_stateChanged(int arg1);
    void on_showQuadrangulationCheckBox_stateChanged(int arg1);
    void on_showQuadrangulationLayoutCheckBox_stateChanged(int arg1);
    void on_showResultCheckBox_stateChanged(int arg1);


private:

    PolyMesh mesh1;
    PolyMesh mesh2;

    bool isQuadMesh1;
    bool isQuadMesh2;

    std::vector<int> quadTracerLabel1;
    std::vector<int> quadTracerLabel2;

    TriangleMesh triMesh1, triMesh2, boolean;
    Eigen::MatrixXd VA, VB, VR;
    Eigen::MatrixXi FA, FB, FR;
    Eigen::VectorXi J;
    std::vector<int> birthQuad1;
    std::vector<int> birthQuad2;

    std::vector<std::vector<size_t>> intersectionCurves;

    TriangleMesh booleanSmoothed;

    QuadLayoutData quadLayoutData1;
    QuadLayoutData quadLayoutData2;

    std::vector<bool> preservedQuad1;
    std::vector<bool> preservedQuad2;

    std::vector<int> preservedFaceLabel1;
    std::vector<int> preservedFaceLabel2;
    QuadLayoutData quadLayoutDataPreserved1;
    QuadLayoutData quadLayoutDataPreserved2;

    PolyMesh preservedSurface;
    std::vector<int> preservedSurfaceLabel;

    TriangleMesh initialNewSurface;

    TriangleMesh newSurface;
    std::vector<int> newSurfaceLabel;

    std::vector<std::vector<size_t>> newSurfacePartitions;
    std::vector<std::vector<size_t>> newSurfaceCorners;
    TriangleChartData newSurfaceChartData;

    QuadBoolean::internal::ChartData chartData;

    std::vector<int> ilpResult;

    PolyMesh quadrangulation;
    std::vector<int> quadrangulationLabel;
    QuadLayoutData quadLayoutDataQuadrangulatedSurface;

    PolyMesh result;


    template<class MeshType>
    void colorizeMesh(
            MeshType& mesh,
            const std::vector<int>& faceLabel);
};

#endif //QUADBOOLEANWINDOW_H
