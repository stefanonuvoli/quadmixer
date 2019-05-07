#ifndef QUADBOOLEANWINDOW_H
#define QUADBOOLEANWINDOW_H


#include "meshtypes.h"
#include "ui_quadbooleanwindow.h"

#include "quadboolean/quadlayoutdata.h"
#include "quadboolean/quadcharts.h"

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

    std::string chooseMeshFile();
    int loadMesh(PolyMesh& mesh, const std::string& filename, bool scaleAndTranslateOnCenter);

    void setTrackballOnMeshes();

    void doTraceQuads();
    void doComputeBooleans();
    void doSmooth();
    void doGetSurfaces();
    void doPatchDecomposition();
    void doSolveILP();
    void doQuadrangulate();
    void doGetResult();

private slots:

    void on_loadMeshesPushButton_clicked();
    void on_quadTracerPushButton_clicked();
    void on_computeBooleanPushButton_clicked();    
    void on_smoothPushButton_clicked();
    void on_getSurfacesPushButton_clicked();
    void on_decompositionPushButton_clicked();
    void on_ilpPushButton_clicked();
    void on_computeAllPushButton_clicked();
    void on_quadrangulatePushButton_clicked();
    void on_getResultPushButton_clicked();
    void on_saveResultPushButton_clicked();

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
    void on_showQuadrangulatedCheckBox_stateChanged(int arg1);
    void on_showQuadrangulatedLayoutCheckBox_stateChanged(int arg1);
    void on_showResultCheckBox_stateChanged(int arg1);

    void on_resetTrackballButton_clicked();
    void on_trackSceneButton_clicked();
    void on_track1Button_clicked();
    void on_track2Button_clicked();
    void on_showWireframe_stateChanged(int arg1);

private:

    Ui::mainWindow ui;

    PolyMesh mesh1;
    PolyMesh mesh2;
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

    PolyMesh quadrangulatedSurface;
    std::vector<int> quadrangulatedSurfaceLabel;
    QuadLayoutData quadLayoutDataQuadrangulatedSurface;

    PolyMesh result;


    void updateVisibility();

    template<class MeshType>
    void colorizeMesh(
            MeshType& mesh,
            const std::vector<int>& faceLabel);
};

#endif //QUADBOOLEANWINDOW_H
