#include "glarea.h"

#include <QMessageBox>
#include <QCheckBox>
#include <QKeyEvent>
#include <QKeyEvent>
#include <QWheelEvent>

#include <wrap/qt/trackball.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

#include <wrap/gl/picking.h>

GLArea::GLArea(QWidget* parent) : QGLWidget (parent)
{
    this->sceneCenter = vcg::Point3f(0,0,0);
    this->sceneRadius = 0;
    this->sceneTrackballVisible = true;
    this->transformationMesh = nullptr;
    this->targetMesh1 = nullptr;
    this->targetMesh2 = nullptr;
    this->debugMode = false;
}

size_t GLArea::addMesh(GLArea::PolyMesh *mesh)
{
    deselectTransformationMesh();

    size_t id = glWrapMeshes.size();
    glWrapMeshes.push_back(GLPolyWrap<PolyMesh>());
    initMeshWrapper(glWrapMeshes[id], mesh);
    glWrapMeshes[id].name = id;

    return id;
}

void GLArea::deleteMesh(const size_t& id)
{
    deselectTransformationMesh();
    deselectTargetMesh();

    glWrapMeshes[id].mesh = nullptr;
}

void GLArea::resetSceneOnMeshes()
{
    const double maxDouble = std::numeric_limits<double>::max();
    PolyMesh::CoordType center(0,0,0);
    float radius = 0;


    PolyMesh::CoordType maxBBPoint(-maxDouble,-maxDouble,-maxDouble);
    PolyMesh::CoordType minBBPoint(maxDouble,maxDouble,maxDouble);

    if (debugMode) {
        if (glWrapMesh1.mesh != nullptr && glWrapMesh2.mesh != nullptr) {
            vcg::tri::UpdateBounding<PolyMesh>::Box(*glWrapMesh1.mesh);
            vcg::tri::UpdateBounding<PolyMesh>::Box(*glWrapMesh2.mesh);

            center += glWrapMesh1.mesh->bbox.Center();
            center += glWrapMesh2.mesh->bbox.Center();
            center /= 2;

            vcg::Point3d pMin1 = glWrapMesh1.mesh->bbox.min;
            vcg::Point3d pMax1 = glWrapMesh2.mesh->bbox.max;
            maxBBPoint.X() = std::max(maxBBPoint.X(), pMax1.X());
            maxBBPoint.Y() = std::max(maxBBPoint.Y(), pMax1.Y());
            maxBBPoint.Z() = std::max(maxBBPoint.Z(), pMax1.Z());
            minBBPoint.X() = std::min(minBBPoint.X(), pMin1.X());
            minBBPoint.Y() = std::min(minBBPoint.Y(), pMin1.Y());
            minBBPoint.Z() = std::min(minBBPoint.Z(), pMin1.Z());


            vcg::Point3d pMin2 = glWrapMesh1.mesh->bbox.min;
            vcg::Point3d pMax2 = glWrapMesh2.mesh->bbox.max;
            maxBBPoint.X() = std::max(maxBBPoint.X(), pMax2.X());
            maxBBPoint.Y() = std::max(maxBBPoint.Y(), pMax2.Y());
            maxBBPoint.Z() = std::max(maxBBPoint.Z(), pMax2.Z());
            minBBPoint.X() = std::min(minBBPoint.X(), pMin2.X());
            minBBPoint.Y() = std::min(minBBPoint.Y(), pMin2.Y());
            minBBPoint.Z() = std::min(minBBPoint.Z(), pMin2.Z());

            radius = static_cast<float>((maxBBPoint - minBBPoint).Norm()/2);
        }
    }
    else {
        int n = 0;

        for (GLPolyWrap<PolyMesh>& glWrapMesh : glWrapMeshes) {
            if (glWrapMesh.mesh != nullptr) {
                vcg::tri::UpdateBounding<PolyMesh>::Box(*glWrapMesh.mesh);

                center += glWrapMesh.mesh->bbox.Center();

                vcg::Point3d pMin = glWrapMesh.mesh->bbox.min;
                vcg::Point3d pMax = glWrapMesh.mesh->bbox.max;
                maxBBPoint.X() = std::max(maxBBPoint.X(), pMax.X());
                maxBBPoint.Y() = std::max(maxBBPoint.Y(), pMax.Y());
                maxBBPoint.Z() = std::max(maxBBPoint.Z(), pMax.Z());
                minBBPoint.X() = std::min(minBBPoint.X(), pMin.X());
                minBBPoint.Y() = std::min(minBBPoint.Y(), pMin.Y());
                minBBPoint.Z() = std::min(minBBPoint.Z(), pMin.Z());

                n++;
            }
        }

        if (n > 0) {
            center /= n;
            radius = static_cast<float>((maxBBPoint - minBBPoint).Norm()/2);
        }
    }

    sceneCenter = vcg::Point3f(static_cast<float>(center.X()),static_cast<float>(center.Y()),static_cast<float>(center.Z()));
    sceneRadius = radius*0.8f;

    sceneTrackball.Reset();
    updateGL();
}

static void drawMesh(std::vector<GLPolyWrap<QuadBoolean::PolyMesh>>::value_type& meshWrap)
{
    meshWrap.GLDraw(false);
}


void GLArea::manageRightClick(const int& x, const int& y)
{
    deselectTransformationMesh();

    std::vector<GLPolyWrap<PolyMesh>*> picked;
    vcg::Pick(x, y, glWrapMeshes, picked, &drawMesh);

    if (picked.size() > 0) {
        selectTransformationMesh(picked[0]);
    }
}

void GLArea::manageDoubleClick(const int &x, const int &y)
{

    std::vector<GLPolyWrap<PolyMesh>*> picked;
    vcg::Pick(x, y, glWrapMeshes, picked, &drawMesh);

    if (picked.size() > 0) {
        selectTargetMesh(picked[0]);
    }
    else {
        deselectTargetMesh();
    }
}

void GLArea::selectTargetMesh(GLPolyWrap<PolyMesh>* meshWrap)
{
    if (targetMesh1 == nullptr) {
        this->targetMesh1 = meshWrap;
        this->targetMesh1->target1 = true;
    }
    else if (targetMesh2 == nullptr && targetMesh1 != meshWrap) {
        this->targetMesh2 = meshWrap;
        this->targetMesh2->target2 = true;
    }
    else {
        deselectTargetMesh();

        selectTargetMesh(meshWrap);
    }

    updateGL();
}

void GLArea::deselectTargetMesh() {
    if (targetMesh1 != nullptr) {
        this->targetMesh1->target1 = false;
        this->targetMesh1 = nullptr;
    }
    if (targetMesh2 != nullptr) {
        this->targetMesh2->target2 = false;
        this->targetMesh2 = nullptr;
    }
}

void GLArea::selectTransformationMesh(GLPolyWrap<PolyMesh>* meshWrap)
{
    vcg::tri::UpdateBounding<PolyMesh>::Box(*meshWrap->mesh);

    PolyMesh::CoordType center(0,0,0);
    center = meshWrap->mesh->bbox.Center();

    this->transformationMesh = meshWrap;
    this->transformationMesh->transformation = true;
    this->transformationMeshVertices = meshWrap->mesh->vert;
    this->transformationMeshCenter = center;

    PolyMesh::ScalarType maxDiag = 0;

    vcg::Point3f centerF(center.X(),center.Y(),center.Z());
    transformationMeshTrackball.center = centerF;

    maxDiag = std::max(maxDiag, meshWrap->mesh->bbox.Diag());
    transformationMeshTrackball.radius = static_cast<float>(maxDiag/2);

    transformationMeshTrackball.Reset();

    updateGL();
}

void GLArea::deselectTransformationMesh() {
    if (transformationMesh != nullptr) {
        this->transformationMeshVertices.clear();
        this->transformationMeshCenter.SetZero();
        this->transformationMesh->transformation = false;
        this->transformationMesh = nullptr;

        transformationMeshTrackball.Reset();
    }
}

void GLArea::applySelectedMeshTransformation()
{
    if (this->transformationMesh->mesh != nullptr) {
        transformationMeshTrackball.GetView();

        vcg::Similarityf sim = transformationMeshTrackball.track;

        float scaleFactor = 1/sim.sca;
        vcg::Point3f tra = sim.tra;
        vcg::Matrix33f rot;

        vcg::Quaternionf quat = sim.rot;
        quat.ToMatrix(rot);

        for (size_t i = 0; i < this->transformationMesh->mesh->vert.size(); i++) {
            vcg::Point3f initialPoint(
                static_cast<float>(this->transformationMeshVertices[i].P().X()),
                static_cast<float>(this->transformationMeshVertices[i].P().Y()),
                static_cast<float>(this->transformationMeshVertices[i].P().Z()));

            vcg::Point3f point = initialPoint;
            vcg::Point3f translateCenter(
                        transformationMeshCenter.X(),
                        transformationMeshCenter.Y(),
                        transformationMeshCenter.Z());
            point -= translateCenter;
            point = rot * point;
            point += translateCenter;
            point += rot * tra;
            point *= scaleFactor;

            this->transformationMesh->mesh->vert[i].P() = vcg::Point3d(
                static_cast<double>(point.X()),
                static_cast<double>(point.Y()),
                static_cast<double>(point.Z())
            );
        }

        vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormals(*this->transformationMesh->mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(*this->transformationMesh->mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(*this->transformationMesh->mesh);
    }
}


void GLArea::setTrackballVisibility(bool visible)
{
    this->sceneTrackballVisible = visible;
}

void GLArea::initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh) {
    if (mesh != nullptr) {
        //        vcg::PolygonalAlgorithm<MeshType>::UpdateFaceNormalByFitting(*mesh);
        vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormalByFitting(*mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(*mesh);

        vcg::tri::UpdateBounding<PolyMesh>::Box(*mesh);
        vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(*mesh);
    }
    glWrap.mesh = mesh;
}



void GLArea::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 0);
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

    if (!debugMode) {
        for (GLPolyWrap<PolyMesh>& glWrap : glWrapMeshes) {
            glWrap.GLDraw(wireframe);
        }
    }
    else {
        glWrapMesh1.GLDraw(wireframe);
        glWrapMesh2.GLDraw(wireframe);
        glWrapBoolean.GLDraw(wireframe);
        glWrapIntersectionCurves.GLDraw();
        glWrapPreservedSurface.GLDraw(wireframe);
        glWrapNewSurface.GLDraw(wireframe);
        glWrapQuadLayout1.GLDraw();
        glWrapQuadLayout2.GLDraw();
        glWrapQuadLayoutPreserved1.GLDraw();
        glWrapQuadLayoutPreserved2.GLDraw();
        glWrapChartSides.GLDraw();
        glWrapQuadrangulated.GLDraw(wireframe);
        glWrapQuadLayoutQuadrangulated.GLDraw();
        glWrapResult.GLDraw(wireframe);
    }

    glPopMatrix();

    if (this->sceneTrackballVisible) {
        sceneTrackball.DrawPostApply();
    }
}

void GLArea::keyReleaseEvent (QKeyEvent * e)
{
    e->ignore ();

    if (this->transformationMesh == nullptr) {
        if (e->key () == Qt::Key_Control)
            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
//        if (e->key () == Qt::Key_Shift)
//            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
//        if (e->key () == Qt::Key_Alt)
//            sceneTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));
    }
    else {
        if (e->key () == Qt::Key_Control)
            transformationMeshTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ControlModifier));
//        if (e->key () == Qt::Key_Shift)
//            selectedTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
//        if (e->key () == Qt::Key_Alt)
//            selectedTrackball.ButtonUp (QT2VCG (Qt::NoButton, Qt::AltModifier));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::keyPressEvent (QKeyEvent * e)
{
    e->ignore ();

    if (this->transformationMesh == nullptr) {
        if (e->key () == Qt::Key_Control)
            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
//        if (e->key () == Qt::Key_Shift)
//            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
//        if (e->key () == Qt::Key_Alt)
//            sceneTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));
    }
    else {
        if (e->key () == Qt::Key_Control)
            transformationMeshTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ControlModifier));
//        if (e->key () == Qt::Key_Shift)
//            selectedTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::ShiftModifier));
//        if (e->key () == Qt::Key_Alt)
//            selectedTrackball.ButtonDown (QT2VCG (Qt::NoButton, Qt::AltModifier));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::mousePressEvent (QMouseEvent * e)
{
    e->accept();

    if (e->button() == Qt::RightButton)
    {
        manageRightClick(e->x(), height() - e->y());
    }
    else if (e->button() == Qt::LeftButton) {
        if (this->transformationMesh == nullptr) {
            setFocus ();
            sceneTrackball.MouseDown (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
        }
        else {
            transformationMeshTrackball.MouseDown (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));

            applySelectedMeshTransformation();
        }
    }
    updateGL ();
}

void GLArea::mouseMoveEvent (QMouseEvent * e)
{
    if (e->buttons ()) {

        if (this->transformationMesh == nullptr) {
            sceneTrackball.MouseMove (QT2VCG_X(this,e), QT2VCG_Y(this,e));
        }
        else {
            transformationMeshTrackball.MouseMove (QT2VCG_X(this,e), QT2VCG_Y(this,e));

            applySelectedMeshTransformation();
        }

        updateGL ();
    }
}

void GLArea::mouseReleaseEvent (QMouseEvent * e)
{
    if (this->transformationMesh == nullptr) {
        sceneTrackball.MouseUp (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));
    }
    else {
        transformationMeshTrackball.MouseUp (QT2VCG_X(this,e), QT2VCG_Y(this,e), QT2VCG (e->button (), e->modifiers ()));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::wheelEvent (QWheelEvent * e)
{
    const int WHEEL_STEP = 500;
    if (this->transformationMesh == nullptr) {
        sceneTrackball.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));
    }
    else {
        const int WHEEL_STEP = 1000;
        transformationMeshTrackball.MouseWheel (e->delta () / float (WHEEL_STEP), QTWheel2VCG (e->modifiers ()));

        applySelectedMeshTransformation();
    }

    updateGL ();
}

void GLArea::mouseDoubleClickEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
    {
        manageDoubleClick(e->x(), height() - e->y());
    }
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

void GLArea::setWireframe(bool visible)
{
    this->wireframe = visible;
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

