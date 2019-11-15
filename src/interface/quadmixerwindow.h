#ifndef QUADBOOLEANWINDOW_H
#define QUADBOOLEANWINDOW_H

#include "ui_quadmixerwindow.h"

#include <Eigen/Core>

#include <quadboolean/quadboolean.h>

class QuadMixerWindow : public QMainWindow
{
Q_OBJECT 

private:

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::TriangleMesh TriangleMesh;
    typedef QuadBoolean::internal::QuadLayoutData<PolyMesh> QuadLayoutData;
    typedef QuadBoolean::internal::ChartData TriangleChartData;


public:

    QuadMixerWindow(QWidget * parent = nullptr);
    ~QuadMixerWindow();

    void booleanOperation();
    void detachOperationSelect();
    void detachOperation();

    int addMesh(const std::string& filename);
    int addMesh(PolyMesh *mesh);
    void hideMesh(PolyMesh* mesh);
    void hideMesh(const size_t& id);

    void setLastOperation(
            PolyMesh* target1,
            PolyMesh* target2,
            PolyMesh* result1,
            PolyMesh* result2);
    void undoLastOperation();

    void saveScreenshot(const std::string &filename);
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
    void on_deleteMeshButton_clicked();
    void on_deleteAllButton_clicked();
    void on_saveMeshButton_clicked();

    void on_booleanOperationButton_clicked();
    void on_detachButton_clicked();
    void on_abortDetachButton_clicked();
    void on_undoButton_clicked();

    void on_resetSceneButton_clicked();
    void on_trackballCheckBox_stateChanged(int arg1);
    void on_showWireframe_stateChanged(int arg1);
    void on_wireframeSlider_valueChanged(int value);
    void on_showParametersCheckBox_stateChanged(int arg1);    
    void on_debugModeButton_clicked();

private:

    void doComputeBooleans();
    void doSmooth();
    void doGetSurfaces();
    void doPatchDecomposition();
    void doSolveILP();
    void doQuadrangulate();
    void doGetResult();

    void clearVisualizationData();

private slots:

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
    void on_showIntersectionVerticesCheckBox_stateChanged(int arg1);
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
    void on_showOriginVerticesCheckBox_stateChanged(int arg1);

    void on_colorResultComboBox_currentIndexChanged(int index);

    void on_colorMeshesComboBox_currentIndexChanged(int index);


    void on_saveMesh1Button_clicked();

    void on_saveMesh2Button_clicked();

    void on_autoRotateButton_clicked();

    void on_continuityTestButton_clicked();
    void continuityTest();

    void on_generalRetractionSpinBox_valueChanged(double arg1);

    void on_generalMaxBBSpinBox_valueChanged(double arg1);

    void on_generalSmallPatchSpinBox_valueChanged(int arg1);

    void on_generalAlphaSpinBox_valueChanged(double arg1);

    void on_generalSmoothingSpinBox_valueChanged(int arg1);

    void on_generalSmoothingPropagationSpinBox_valueChanged(double arg1);

    void on_generalLSRadio_toggled(bool checked);
    void on_generalABSRadio_toggled(bool checked);

private:

    PolyMesh mesh1;
    PolyMesh mesh2;

    bool isQuadMesh1;
    bool isQuadMesh2;

    std::vector<int> quadTracerLabel1;
    std::vector<int> quadTracerLabel2;

    TriangleMesh trimesh1, trimesh2, boolean;
    Eigen::MatrixXd VA, VB, VR;
    Eigen::MatrixXi FA, FB, FR;
    Eigen::VectorXi J;

    std::vector<std::pair<size_t, size_t>> birthTriangle;
    std::vector<int> birthFace1;
    std::vector<int> birthFace2;

    std::vector<size_t> intersectionVertices;
    std::vector<size_t> smoothedVertices;

    TriangleMesh booleanSmoothed;

    QuadLayoutData quadLayoutData1;
    QuadLayoutData quadLayoutData2;

    std::vector<std::pair<bool, bool>> isPreserved1;
    std::vector<std::pair<bool, bool>> isPreserved2;
    std::vector<bool> isNewSurface;

    std::vector<int> meshLabel1;
    std::vector<int> meshLabel2;
    QuadLayoutData quadLayoutDataPreserved1;
    QuadLayoutData quadLayoutDataPreserved2;

    PolyMesh preservedSurface;
    std::vector<int> preservedSurfaceLabel;
    std::unordered_map<size_t, size_t> preservedFacesMap;
    std::unordered_map<size_t, size_t> preservedVerticesMap;

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
    QuadBoolean::SourceInfo sourceInfo;

    double continuityLength;
    double continuityOffset;

    template<class MeshType>
    void colorizeMesh(
            MeshType& mesh,
            const std::vector<int>& faceLabel);
    template<class MeshType>
    void colorResultByComboBox(MeshType& mesh, int index);
    template<class MeshType>
    void colorMeshByComboBox(MeshType& mesh, int index, const std::vector<std::pair<bool, bool>>& preservedFace);
};

#endif //QUADBOOLEANWINDOW_H
