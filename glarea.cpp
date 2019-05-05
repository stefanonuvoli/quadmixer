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
    selectedMesh = nullptr;
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

void GLArea::setBoolean(TriangleMesh* boolean)
{
    initMeshWrapper(this->glWrapBoolean, boolean);
    glWrapIntersectionCurves.mesh = boolean;
}

void GLArea::setIntersectionCurves(std::vector<std::vector<size_t>>* intersectionCurves)
{
    glWrapIntersectionCurves.edges = intersectionCurves;
}

void GLArea::setPreservedSurface(PolyMesh* preservedSurface)
{
    initMeshWrapper(this->glWrapPreservedSurface, preservedSurface);
}

void GLArea::setQuadLayoutPreserved1(QuadLayoutData* quadLayoutDataPreserved1)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutPreserved1, quadLayoutDataPreserved1);
}

void GLArea::setQuadLayoutPreserved2(QuadLayoutData* quadLayoutDataPreserved2)
{
    initQuadLayoutWrapper(this->glWrapQuadLayoutPreserved2, quadLayoutDataPreserved2);
}

void GLArea::setNewSurface(TriangleMesh* newSurface)
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

void GLArea::setIntersectionCurvesVisibility(bool visible)
{
    glWrapIntersectionCurves.visible = visible;
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

void GLArea::setILPVisibility(bool visible)
{
    glWrapChartSides.ilpVisible = visible;
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



void GLArea::setMesh1Wireframe(bool wireframe)
{
    glWrapMesh1.wireframe = wireframe;
}

void GLArea::setMesh2Wireframe(bool wireframe)
{
    glWrapMesh2.wireframe = wireframe;
}

void GLArea::setBooleanWireframe(bool wireframe)
{
    glWrapBoolean.wireframe = wireframe;
}

void GLArea::setPreservedSurfaceWireframe(bool wireframe)
{
    glWrapPreservedSurface.wireframe = wireframe;
}

void GLArea::setNewSurfaceWireframe(bool wireframe)
{
    glWrapNewSurface.wireframe = wireframe;
}

void GLArea::setQuadrangulatedWireframe(bool wireframe)
{
    glWrapQuadrangulated.wireframe = wireframe;
}

void GLArea::setResultWireframe(bool wireframe)
{
    glWrapResult.wireframe = wireframe;
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


void GLArea::resetTrackball()
{
    sceneTrackball.Reset();
    updateGL();
}


void GLArea::setSceneOnMesh()
{
    if (glWrapMesh1.mesh != nullptr && glWrapMesh2.mesh != nullptr) {
        vcg::tri::UpdateBounding<PolyMesh>::Box(*glWrapMesh1.mesh);
        vcg::tri::UpdateBounding<PolyMesh>::Box(*glWrapMesh2.mesh);

        PolyMesh::CoordType center(0,0,0);
        PolyMesh::ScalarType maxDiag = 0;

        center += glWrapMesh1.mesh->bbox.Center();
        center += glWrapMesh2.mesh->bbox.Center();
        center /= 2;
        vcg::Point3f sceneCenter(static_cast<float>(center.X()),static_cast<float>(center.Y()),static_cast<float>(center.Z()));
        setSceneCenter(sceneCenter);

        maxDiag = std::max(maxDiag, glWrapMesh1.mesh->bbox.Diag());
        maxDiag = std::max(maxDiag, glWrapMesh2.mesh->bbox.Diag());
        setSceneRadius(static_cast<float>(maxDiag/2));
    }
    selectedMesh = nullptr;
}

void GLArea::selectAndTrackScene() {
    selectedMeshVertices.clear();

    setSceneOnMesh();

    updateGL();
}

void GLArea::selectAndTrackMesh1()
{
    if (glWrapMesh1.mesh != nullptr) {
        selectAndTrackMesh(glWrapMesh1.mesh);
    }
    updateGL();
}

void GLArea::selectAndTrackMesh2()
{
    if (glWrapMesh2.mesh != nullptr) {
        selectAndTrackMesh(glWrapMesh2.mesh);
    }
}

void GLArea::selectAndTrackMesh(PolyMesh* mesh)
{
    vcg::tri::UpdateBounding<PolyMesh>::Box(*mesh);

    PolyMesh::CoordType center(0,0,0);
    center = mesh->bbox.Center();

    selectedMesh = mesh;
    selectedMeshVertices = mesh->vert;
    selectedMeshInitialCenter = center;

    PolyMesh::ScalarType maxDiag = 0;

    vcg::Point3f sceneCenter(0,0,0);
    selectedTrackball.center = sceneCenter;

    maxDiag = std::max(maxDiag, mesh->bbox.Diag());
    selectedTrackball.radius = static_cast<float>(maxDiag/2);

    selectedTrackball.Reset();

    updateGL();
}

void GLArea::applySelectedMeshTransformation()
{
    if (selectedMesh != nullptr) {
        selectedTrackball.GetView();

        vcg::Similarityf sim = selectedTrackball.track;

        float scaleFactor = 1/sim.sca;
        vcg::Point3f tra = sim.tra;
        vcg::Matrix33f rot;

        vcg::Quaternionf quat = sim.rot;
        quat.ToMatrix(rot);

        for (size_t i = 0; i < selectedMesh->vert.size(); i++) {
            vcg::Point3f initialPoint(
                static_cast<float>(selectedMeshVertices[i].P().X()),
                static_cast<float>(selectedMeshVertices[i].P().Y()),
                static_cast<float>(selectedMeshVertices[i].P().Z()));

            vcg::Point3f point = initialPoint;
            point -= vcg::Point3f(selectedMeshInitialCenter.X(), selectedMeshInitialCenter.Y(), selectedMeshInitialCenter.Z());
            point = rot * point;
            point += vcg::Point3f(selectedMeshInitialCenter.X(), selectedMeshInitialCenter.Y(), selectedMeshInitialCenter.Z());
            point *= scaleFactor;
            point += tra;

            selectedMesh->vert[i].P() = vcg::Point3d(
                static_cast<double>(point.X()),
                static_cast<double>(point.Y()),
                static_cast<double>(point.Z())
            );
        }

        vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormals(*selectedMesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(*selectedMesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(*selectedMesh);
    }
}


void GLArea::initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh) {
    if (mesh != nullptr) {
//        vcg::PolygonalAlgorithm<MeshType>::UpdateFaceNormalByFitting(*mesh);
        vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormals(*mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(*mesh);

        vcg::tri::UpdateBounding<PolyMesh>::Box(*mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(*mesh);
    }
    glWrap.mesh = mesh;
}

void GLArea::initMeshWrapper(GLPolyWrap<TriangleMesh>& glWrap, TriangleMesh* mesh) {
    if (mesh != nullptr) {
        vcg::tri::UpdateNormal<TriangleMesh>::PerFaceNormalized(*mesh);
        vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalized(*mesh);

        vcg::tri::UpdateBounding<TriangleMesh>::Box(*mesh);
        vcg::tri::UpdateNormal<TriangleMesh>::PerVertexNormalizedPerFace(*mesh);
    }
    glWrap.mesh = mesh;
}

template<class MeshType>
void GLArea::initQuadLayoutWrapper(GLQuadLayoutWrap<MeshType> &glWrap, QuadLayoutData* quadLayoutData)
{
    glWrap.quadLayoutData = quadLayoutData;
}

template<class MeshType>
void GLArea::initChartSidesWrapper(GLChartSidesWrap<MeshType> &glWrap, ChartData *chartData)
{
    glWrap.chartData = chartData;
}


void GLArea::initializeGL()
{
    glClearColor(0.9f, 0.9f, 0.9f, 0);
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

    sceneTrackball.center = sceneCenter;
    if (sceneRadius > 0) {
        gluLookAt(sceneCenter.X(),  sceneCenter.Y(),    sceneCenter.Z() + sceneRadius*5,
                  sceneCenter.X(),  sceneCenter.Y(),    sceneCenter.Z(),
                  0,                sceneRadius*5,      0);

        sceneTrackball.radius = sceneRadius;
    }
    else {
        gluLookAt(0,0,5,   0,0,0,   0,5,0);

        sceneTrackball.radius = 1;
    }


    sceneTrackball.GetView();
    sceneTrackball.Apply();

    glPushMatrix();

    glWrapMesh1.GLDraw();
    glWrapMesh2.GLDraw();
    glWrapBoolean.GLDraw();
    glWrapIntersectionCurves.GLDraw();
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

    glPopMatrix();

    if (selectedMesh == nullptr) {
        sceneTrackball.DrawPostApply();
    }
}

void GLArea::keyReleaseEvent (QKeyEvent * e)
{
    e->ignore ();

    if (selectedMesh == nullptr) {
        if (e->key () == Qt::Key_Control)
            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
        if (e->key () == Qt::Key_Shift)
            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
        if (e->key () == Qt::Key_Alt)
            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));

        applySelectedMeshTransformation();
    }
    else {
        if (e->key () == Qt::Key_Control)
            selectedTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
        if (e->key () == Qt::Key_Shift)
            selectedTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
        if (e->key () == Qt::Key_Alt)
            selectedTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::keyPressEvent (QKeyEvent * e)
{
    e->ignore ();

    if (selectedMesh == nullptr) {
        if (e->key () == Qt::Key_Control)
            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
        if (e->key () == Qt::Key_Shift)
            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
        if (e->key () == Qt::Key_Alt)
            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));
    }
    else {
        if (e->key () == Qt::Key_Control)
            selectedTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
        if (e->key () == Qt::Key_Shift)
            selectedTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
        if (e->key () == Qt::Key_Alt)
            selectedTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::mousePressEvent (QMouseEvent * e)
{
    e->accept ();

    if (selectedMesh == nullptr) {
        setFocus ();
        sceneTrackball.MouseDown (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
    }
    else {
        selectedTrackball.MouseDown (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::mouseMoveEvent (QMouseEvent * e)
{
    if (e->buttons ()) {

        if (selectedMesh == nullptr) {
            sceneTrackball.MouseMove (QT2VCG_X(this,e), QT2VCG_Y(this,e));
        }
        else {
            selectedTrackball.MouseMove (QT2VCG_X(this,e), QT2VCG_Y(this,e));

            applySelectedMeshTransformation();
        }

        updateGL ();
    }
}

void GLArea::mouseReleaseEvent (QMouseEvent * e)
{
    if (selectedMesh == nullptr) {
        sceneTrackball.MouseUp (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
    }
    else {
        selectedTrackball.MouseUp (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::wheelEvent (QWheelEvent * e)
{
    const int WHEEL_STEP = 500;
    if (selectedMesh == nullptr) {
        sceneTrackball.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));
    }
    else {
        const int WHEEL_STEP = 1000;
        selectedTrackball.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

