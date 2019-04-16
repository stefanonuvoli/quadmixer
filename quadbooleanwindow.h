#ifndef QUADBOOLEANWINDOW_H
#define QUADBOOLEANWINDOW_H


#include "quadcommontypes.h"
#include "ui_quadbooleanwindow.h"

#include "quadpatches.h"
#include "quadcharts.h"

class QuadBooleanWindow : public QMainWindow
{
Q_OBJECT 

private:

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::QuadData QuadData;
    typedef QuadBoolean::ChartData TriangleChartData;

public:

    QuadBooleanWindow(QWidget * parent = nullptr);

    std::string chooseMeshFile();
    int loadMesh(PolyMesh& mesh, const std::string& filename);

    void setTrackballOnMeshes();

    void traceQuads();
    void computeBooleans();
    void getPreservedQuads();
    void solveILP();
    void getPatchDecomposition();
    void quadrangulateNewSurface();
    void getResult();

    void updateVisibility();

private slots:

    void on_loadMeshesPushButton_clicked();
    void on_quadTracerPushButton_clicked();
    void on_computeBooleanPushButton_clicked();
    void on_getPreservedQuadsPushButton_clicked();
    void on_decompositionPushButton_clicked();
    void on_ilpPushButton_clicked();
    void on_computeAllPushButton_clicked();
    void on_quadrangulatePushButton_clicked();
    void on_getResultPushButton_clicked();

    void on_showMesh1CheckBox_stateChanged(int arg1);
    void on_showMesh2CheckBox_stateChanged(int arg1);
    void on_showBooleanCheckBox_stateChanged(int arg1);
    void on_showPreservedSurfaceCheckBox_stateChanged(int arg1);
    void on_showNewSurfaceCheckBox_stateChanged(int arg1);
    void on_showQuadLayoutPreserved1CheckBox_stateChanged(int arg1);
    void on_showQuadLayoutPreserved2CheckBox_stateChanged(int arg1);
    void on_showQuadLayout1CheckBox_stateChanged(int arg1);
    void on_showQuadLayout2CheckBox_stateChanged(int arg1);
    void on_showChartSidesCheckBox_stateChanged(int arg1);
    void on_showQuadrangulatedCheckBox_stateChanged(int arg1);
    void on_showQuadrangulatedLayoutCheckBox_stateChanged(int arg1);
    void on_showResultCheckBox_stateChanged(int arg1);
    void on_showResultLayoutCheckBox_stateChanged(int arg1);

    void on_resetTrackballButton_clicked();

private:

    Ui::mainWindow ui;

    PolyMesh mesh1;
    PolyMesh mesh2;
    std::vector<int> tracerFaceLabel1;
    std::vector<int> tracerFaceLabel2;

    QuadData quadData1;
    QuadData quadData2;

    PolyMesh triMesh1, triMesh2, boolean;
    Eigen::MatrixXd VA, VB, VR;
    Eigen::MatrixXi FA, FB, FR;
    Eigen::VectorXi J;
    std::vector<int> birthQuad1;
    std::vector<int> birthQuad2;

    std::vector<bool> preservedQuad1;
    std::vector<bool> preservedQuad2;

    std::vector<int> preservedFaceLabel1;
    std::vector<int> preservedFaceLabel2;
    QuadData quadDataPreserved1;
    QuadData quadDataPreserved2;

    PolyMesh preservedSurface;
    std::vector<int> preservedSurfaceLabel;

    PolyMesh newSurface;
    std::vector<int> newSurfaceLabel;
    TriangleChartData newSurfaceChartData;

    QuadBoolean::ChartData chartData;

    std::vector<int> ilpResult;

    PolyMesh quadrangulatedNewSurface;
    QuadData quadDataQuadrangulated;

    PolyMesh result;
    QuadData quadDataResult;
};

#endif //QUADBOOLEANWINDOW_H
