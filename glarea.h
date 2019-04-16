#ifndef GLAREA_H
#define GLAREA_H

#include "quadcommontypes.h"

/// Opengl related imports
#include <GL/glew.h>
#include <QGLWidget>

/// vcg imports
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/create/platonic.h>

/// wrapper imports
#include <wrap/io_trimesh/import.h>
#include <wrap/gui/trackball.h>

#include "glpolywrap.h"
#include "glquadlayoutwrap.h"
#include "glchartsideswrap.h"

class GLArea : public QGLWidget
{
    Q_OBJECT

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::QuadData QuadData;
    typedef QuadBoolean::ChartData ChartData;

public:

    GLArea(QWidget* parent = nullptr);

    void setMesh1(PolyMesh* mesh);
    void setMesh2(PolyMesh* mesh);
    void setQuadLayout1(QuadData* quadData1);
    void setQuadLayout2(QuadData* quadData2);
    void setBoolean(PolyMesh* boolean);
    void setPreservedSurface(PolyMesh* boolean);
    void setQuadLayoutPreserved1(QuadData* quadData2);
    void setQuadLayoutPreserved2(QuadData* quadData2);
    void setNewSurface(PolyMesh* boolean);
    void setChartSides(ChartData* chartData);
    void setIlpResult(std::vector<int>* ilpResult);
    void setQuadrangulatedSurface(PolyMesh* quadrangulatedNewSurface);
    void setQuadLayoutQuadrangulated(QuadData* quadDataQuadrangulated);
    void setResult(PolyMesh* result);
    void setQuadLayoutResult(QuadData* quadDataResult);

    void setMesh1Visibility(bool visible);
    void setMesh2Visibility(bool visible);
    void setQuadLayout1Visibility(bool visible);
    void setQuadLayout2Visibility(bool visible);
    void setQuadLayoutPreserved1Visibility(bool visible);
    void setQuadLayoutPreserved2Visibility(bool visible);
    void setBooleanVisibility(bool visible);
    void setPreservedSurfaceVisibility(bool visible);
    void setNewSurfaceVisibility(bool visible);
    void setChartSidesVisibility(bool visible);
    void setQuadrangulatedVisibility(bool visible);
    void setQuadLayoutQuadrangulatedVisibility(bool visible);
    void setResultVisibility(bool visible);
    void setQuadLayoutResultVisibility(bool visible);

    void resetTrackball();

    vcg::Point3f getSceneCenter() const;
    void setSceneCenter(const vcg::Point3f &value);
    float getSceneRadius() const;
    void setSceneRadius(float value);

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
    GLPolyWrap<PolyMesh> glWrapBoolean;
    GLPolyWrap<PolyMesh> glWrapPreservedSurface;
    GLPolyWrap<PolyMesh> glWrapNewSurface;
    GLPolyWrap<PolyMesh> glWrapQuadrangulatedSurface;
    GLPolyWrap<PolyMesh> glWrapResult;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout2;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved2;
    GLChartSidesWrap<PolyMesh> glWrapChartSides;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutQuadrangulated;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutResult;

    vcg::Trackball trackball;
    vcg::Point3f sceneCenter;
    float sceneRadius;

    void initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh);
    void initQuadLayoutWrapper(GLQuadLayoutWrap<PolyMesh>& glWrap, QuadData* quadData);
    void initChartSidesWrapper(GLChartSidesWrap<PolyMesh>& glWrap, ChartData* quadData);
};

#endif //GLAREA_H
