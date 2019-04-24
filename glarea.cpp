#include "glarea.h"

#include <QMessageBox>
#include <QCheckBox>
#include <QKeyEvent>
#include <QKeyEvent>
#include <QWheelEvent>

#include <wrap/qt/trackball.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

GLArea::GLArea(QWidget* parent) : QGLWidget (parent)
{
    this->sceneCenter = vcg::Point3f(0,0,0);
    this->sceneRadius = 0;
}


void GLArea::setMesh1(PolyMesh* mesh1)
{
    initMeshWrapper(this->glWrapMesh1, mesh1);
    this->glWrapQuadLayout1.mesh = mesh1;
    this->glWrapQuadLayoutPreserved1.mesh = mesh1;
}

void GLArea::setMesh2(PolyMesh* mesh2){

    initMeshWrapper(this->glWrapMesh2, mesh2);
    this->glWrapQuadLayout2.mesh = mesh2;
    this->glWrapQuadLayoutPreserved2.mesh = mesh2;
}

void GLArea::setQuadLayout1(QuadLayoutData* quadLayoutData1)
{
    initQuadLayoutWrapper(this->glWrapQuadLayout1, quadLayoutData1);
}

void GLArea::setQuadLayout2(QuadLayoutData* quadLayoutData2)
{
    initQuadLayoutWrapper(this->glWrapQuadLayout2, quadLayoutData2);
}

void GLArea::setBoolean(PolyMesh* boolean)
{
    initMeshWrapper(this->glWrapBoolean, boolean);
}

void GLArea::setPreservedSurface(PolyMesh* preservedSurface)
{
    initMeshWrapper(this->glWrapPreservedSurface, preservedSurface);
//    this->glWrapQuadLayoutDataPreserved.mesh = preservedSurface;
}

void GLArea::setQuadLayoutPreserved1(QuadLayoutData* quadLayoutDataPreserved1)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutPreserved1, quadLayoutDataPreserved1);
}

void GLArea::setQuadLayoutPreserved2(QuadLayoutData* quadLayoutDataPreserved2)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutPreserved2, quadLayoutDataPreserved2);
}

void GLArea::setNewSurface(PolyMesh* newSurface)
{
    initMeshWrapper(this->glWrapNewSurface, newSurface);
    glWrapChartSides.mesh = newSurface;
}

void GLArea::setChartSides(ChartData* chartData)
{
    initChartSidesWrapper(this->glWrapChartSides, chartData);
}

void GLArea::setIlpResult(std::vector<int>* ilpResult)
{
    this->glWrapChartSides.ilpResult = ilpResult;
}

void GLArea::setQuadrangulated(PolyMesh* quadrangulated)
{
    initMeshWrapper(this->glWrapQuadrangulated, quadrangulated);
}

void GLArea::setQuadLayoutQuadrangulated(QuadLayoutData* quadLayoutDataQuadrangulated)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutQuadrangulated, quadLayoutDataQuadrangulated);
}

void GLArea::setResult(PolyMesh* result)
{
    initMeshWrapper(this->glWrapResult, result);
}

void GLArea::setQuadLayoutResult(QuadLayoutData* quadLayoutDataResult)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutResult, quadLayoutDataResult);
}




void GLArea::setMesh1Visibility(bool visible)
{
    glWrapMesh1.visible = visible;
}

void GLArea::setMesh2Visibility(bool visible)
{
    glWrapMesh2.visible = visible;
}

void GLArea::setQuadLayout1Visibility(bool visible)
{
    glWrapQuadLayout1.visible = visible;
}

void GLArea::setQuadLayout2Visibility(bool visible)
{
    glWrapQuadLayout2.visible = visible;
}

void GLArea::setBooleanVisibility(bool visible)
{
    glWrapBoolean.visible = visible;
}

void GLArea::setPreservedSurfaceVisibility(bool visible)
{
    glWrapPreservedSurface.visible = visible;
}

void GLArea::setQuadLayoutPreserved1Visibility(bool visible)
{
    glWrapQuadLayoutPreserved1.visible = visible;
}

void GLArea::setQuadLayoutPreserved2Visibility(bool visible)
{
    glWrapQuadLayoutPreserved2.visible = visible;
}

void GLArea::setNewSurfaceVisibility(bool visible)
{
    glWrapNewSurface.visible = visible;
}


void GLArea::setChartSidesVisibility(bool visible)
{
    glWrapChartSides.visible = visible;
}

void GLArea::setQuadrangulatedVisibility(bool visible)
{
    glWrapQuadrangulated.visible = visible;
}

void GLArea::setQuadLayoutQuadrangulatedVisibility(bool visible)
{
    glWrapQuadLayoutQuadrangulated.visible = visible;
}

void GLArea::setResultVisibility(bool visible)
{
    glWrapResult.visible = visible;
}

void GLArea::setQuadLayoutResultVisibility(bool visible)
{
    glWrapQuadLayoutResult.visible = visible;
}





void GLArea::resetTrackball()
{
    trackball.Reset();
    updateGL();
}

float GLArea::getSceneRadius() const
{
    return sceneRadius;
}

void GLArea::setSceneRadius(float value)
{
    sceneRadius = value;
}

vcg::Point3f GLArea::getSceneCenter() const
{
    return sceneCenter;
}

void GLArea::setSceneCenter(const vcg::Point3f &value)
{
    sceneCenter = value;
}




void GLArea::initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh) {
    assert(mesh != nullptr);

    //PolygonalAlgorithm<MyQuadMesh>::UpdateFaceNormals(*this);
    vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormalByFitting(*mesh);
    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(*mesh);

    vcg::tri::UpdateBounding<PolyMesh>::Box(*mesh);
    vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(*mesh);
    vcg::tri::UpdateNormal<PolyMesh>::PerFaceNormalized(*mesh);

    glWrap.mesh = mesh;
}

void GLArea::initQuadLayoutWrapper(GLQuadLayoutWrap<PolyMesh> &glWrap, QuadLayoutData* quadLayoutData)
{
    assert(quadLayoutData != nullptr);
    glWrap.quadLayoutData = quadLayoutData;
}

void GLArea::initChartSidesWrapper(GLChartSidesWrap<PolyMesh> &glWrap, ChartData *chartData)
{
    assert(chartData != nullptr);
    glWrap.chartData = chartData;
}


void GLArea::initializeGL()
{
    glClearColor(0.8f, 0.8f, 0.8f, 0);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void GLArea::resizeGL (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    initializeGL();
}

void GLArea::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, static_cast<double>(GLArea::width())/GLArea::height(), 0.1, 1000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    trackball.center = sceneCenter;
    if (sceneRadius > 0) {
        gluLookAt(sceneCenter.X(),  sceneCenter.Y(),    sceneCenter.Z() + sceneRadius*5,
                  sceneCenter.X(),  sceneCenter.Y(),    sceneCenter.Z(),
                  0,                sceneRadius*5,      0);

        trackball.radius = sceneRadius;
    }
    else {
        gluLookAt(0,0,5,   0,0,0,   0,5,0);

        trackball.radius = 1;
    }

    trackball.GetView();
    trackball.Apply();
    glPushMatrix();

    glWrapMesh1.GLDraw();
    glWrapMesh2.GLDraw();
    glWrapBoolean.GLDraw();
    glWrapPreservedSurface.GLDraw();
    glWrapNewSurface.GLDraw();
    glWrapQuadLayout1.GLDraw();
    glWrapQuadLayout2.GLDraw();
    glWrapQuadLayoutPreserved1.GLDraw();
    glWrapQuadLayoutPreserved2.GLDraw();
    glWrapChartSides.GLDraw();
    glWrapQuadrangulated.GLDraw();
    glWrapQuadLayoutQuadrangulated.GLDraw();
    glWrapResult.GLDraw();
    glWrapQuadLayoutResult.GLDraw();

    glPopMatrix();
    trackball.DrawPostApply();
}

void GLArea::keyReleaseEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control)
        trackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)
        trackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt)
        trackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));
    updateGL ();
}

void GLArea::keyPressEvent (QKeyEvent * e)
{
    e->ignore ();
    if (e->key () == Qt::Key_Control)
        trackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
    if (e->key () == Qt::Key_Shift)
        trackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
    if (e->key () == Qt::Key_Alt)
        trackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));
    updateGL ();
}

void GLArea::mousePressEvent (QMouseEvent * e)
{
    e->accept ();
    setFocus ();
    trackball.MouseDown (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
    updateGL ();
}

void GLArea::mouseMoveEvent (QMouseEvent * e)
{
    if (e->buttons ()) {
        trackball.MouseMove (QT2VCG_X(this,e), QT2VCG_Y(this,e));
        updateGL ();
    }
}

void GLArea::mouseReleaseEvent (QMouseEvent * e)
{
    trackball.MouseUp (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
    updateGL ();
}

void GLArea::wheelEvent (QWheelEvent * e)
{
    const int WHEEL_STEP = 120;
    trackball.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));
    updateGL ();
}

