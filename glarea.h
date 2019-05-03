#ifndef GLAREA_H
#define GLAREA_H

#include "meshtypes.h"

#include <GL/glew.h>
#include <QGLWidget>

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/create/platonic.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/gui/trackball.h>

#include "globjects/glpolywrap.h"
#include "globjects/glquadlayoutwrap.h"
#include "globjects/glchartsideswrap.h"
#include "globjects/gledgeswrap.h"


class GLArea : public QGLWidget
{
    Q_OBJECT

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::TriangleMesh TriangleMesh;
    typedef QuadBoolean::internal::QuadLayoutData<PolyMesh> QuadLayoutData;
    typedef QuadBoolean::internal::ChartData ChartData;

public:

    GLArea(QWidget* parent = nullptr);

    void setMesh1(PolyMesh* mesh);
    void setMesh2(PolyMesh* mesh);
    void setQuadLayout1(QuadLayoutData* quadLayoutData1);
    void setQuadLayout2(QuadLayoutData* quadLayoutData2);
    void setBoolean(TriangleMesh* boolean);
    void setIntersectionCurves(std::vector<std::vector<size_t>>* intersectionCurves);
    void setPreservedSurface(PolyMesh* boolean);
    void setQuadLayoutPreserved1(QuadLayoutData* quadLayoutData2);
    void setQuadLayoutPreserved2(QuadLayoutData* quadLayoutData2);
    void setNewSurface(TriangleMesh* boolean);
    void setChartSides(ChartData* chartData);
    void setIlpResult(std::vector<int>* ilpResult);
    void setQuadrangulated(PolyMesh* quadrangulatedNewSurface);
    void setQuadLayoutQuadrangulated(QuadLayoutData* quadLayoutDataQuadrangulated);
    void setResult(PolyMesh* result);
    void setQuadLayoutResult(QuadLayoutData* quadLayoutDataResult);

    void setMesh1Visibility(bool visible);
    void setMesh2Visibility(bool visible);
    void setQuadLayout1Visibility(bool visible);
    void setQuadLayout2Visibility(bool visible);
    void setQuadLayoutPreserved1Visibility(bool visible);
    void setQuadLayoutPreserved2Visibility(bool visible);
    void setBooleanVisibility(bool visible);
    void setIntersectionCurvesVisibility(bool visible);
    void setPreservedSurfaceVisibility(bool visible);
    void setNewSurfaceVisibility(bool visible);
    void setChartSidesVisibility(bool visible);
    void setILPVisibility(bool visible);
    void setQuadrangulatedVisibility(bool visible);
    void setQuadLayoutQuadrangulatedVisibility(bool visible);
    void setResultVisibility(bool visible);
    void setQuadLayoutResultVisibility(bool visible);

    void setMesh1Wireframe(bool wireframe);
    void setMesh2Wireframe(bool wireframe);
    void setBooleanWireframe(bool wireframe);
    void setPreservedSurfaceWireframe(bool wireframe);
    void setNewSurfaceWireframe(bool wireframe);
    void setQuadrangulatedWireframe(bool wireframe);
    void setResultWireframe(bool wireframe);

    vcg::Point3f getSceneCenter() const;
    void setSceneCenter(const vcg::Point3f &value);
    float getSceneRadius() const;
    void setSceneRadius(float value);

    void resetTrackball();
    void setSceneOnMesh();
    void setTrackballOnSelected();
    void selectAndTrackScene();
    void selectAndTrackMesh1();
    void selectAndTrackMesh2();
    void selectAndTrackMesh(PolyMesh* mesh);
    void applySelectedMeshTransformation();

signals:
    void setStatusBar(QString message);

protected:

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void keyReleaseEvent(QKeyEvent* e);
    void keyPressEvent(QKeyEvent* e);
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void mouseReleaseEvent(QMouseEvent* e);
    void wheelEvent(QWheelEvent* e);

private:

    GLPolyWrap<PolyMesh> glWrapMesh1;
    GLPolyWrap<PolyMesh> glWrapMesh2;
    GLPolyWrap<TriangleMesh> glWrapBoolean;
    GLEdgesWrap<TriangleMesh> glWrapIntersectionCurves;
    GLPolyWrap<PolyMesh> glWrapPreservedSurface;
    GLPolyWrap<TriangleMesh> glWrapNewSurface;
    GLPolyWrap<PolyMesh> glWrapQuadrangulated;
    GLPolyWrap<PolyMesh> glWrapResult;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout2;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved2;
    GLChartSidesWrap<TriangleMesh> glWrapChartSides;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutQuadrangulated;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutResult;

    vcg::Trackball sceneTrackball;
    vcg::Trackball selectedTrackball;
    vcg::Point3f sceneCenter;
    float sceneRadius;

    PolyMesh* selectedMesh;
    std::vector<PolyMesh::VertexType> selectedMeshVertices;
    vcg::Point3d selectedMeshInitialCenter;

    void initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh);
    void initMeshWrapper(GLPolyWrap<TriangleMesh>& glWrap, TriangleMesh* mesh);
    template<class MeshType>
    void initQuadLayoutWrapper(GLQuadLayoutWrap<MeshType>& glWrap, QuadLayoutData* quadLayoutData);
    template<class MeshType>
    void initChartSidesWrapper(GLChartSidesWrap<MeshType>& glWrap, ChartData* chartData);
};

#endif //GLAREA_H
