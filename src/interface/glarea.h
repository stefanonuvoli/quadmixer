/* Copyright(C) 2019


The authors of

QuadMixer: Layout Preserving Blending of Quadrilateral Meshes
SIGGRAPH Asia 2019


All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
****************************************************************************/

#ifndef GLAREA_H
#define GLAREA_H

#include <GL/glew.h>
#include <QGLWidget>
#include <QTimer>

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/create/platonic.h>

#include <wrap/io_trimesh/import.h>
#include <wrap/gui/trackball.h>

#include "globjects/glpolywrap.h"
#include "globjects/glquadlayoutwrap.h"
#include "globjects/glchartsideswrap.h"
#include "globjects/glverticeswrap.h"
#include "globjects/glsegmentswrap.h"

#include <quadmixer/quadboolean.h>

#define ROTATION_TIME 5000
#define ROTATION_ITERATIONS 500

class GLArea : public QGLWidget
{
    Q_OBJECT

    typedef QuadBoolean::PolyMesh PolyMesh;
    typedef QuadBoolean::TriangleMesh TriangleMesh;
    typedef QuadBoolean::internal::QuadLayoutData<PolyMesh> QuadLayoutData;
    typedef QuadRetopology::ChartData ChartData;

public:

    GLArea(QWidget* parent = nullptr);

    size_t addMesh(PolyMesh* mesh);
    void removeMesh(const size_t& id);

    void manageRightClick(const int& x, const int& y);
    void manageDoubleClick(const int& x, const int& y);

    void resetSceneOnMeshes();

    void selectTargetMesh(GLPolyWrap<PolyMesh>* meshWrap);
    void selectTargetMesh(size_t id);
    void deselectTargetMeshes();
    void selectTransformationMesh(GLPolyWrap<PolyMesh>* meshWrap);
    void deselectTransformationMesh();
    void applySelectedMeshTransformation();

    void setTrackballVisibility(bool visible);
    void setWireframe(bool visible);
    void setWireframeSize(int size);

    bool getDetachMode() const;
    void setDetachMode(bool value);

    void autoRotate();

    vcg::Point3f sceneCenter;
    float sceneRadius;

    bool debugMode;

    bool wireframe;
    int wireframeSize;

    GLPolyWrap<PolyMesh>* targetMesh1;
    GLPolyWrap<PolyMesh>* targetMesh2;

    GLSegmentsWrap<PolyMesh> envelopeWrap;

private:
    QTimer rotationTimer;
    int rotationIteration;
    double rotationAngle;
    double currentAngle;

    vcg::Trackball sceneTrackball;
    bool sceneTrackballVisible;

    GLPolyWrap<PolyMesh>* transformationMesh;
    std::vector<PolyMesh::VertexType> transformationMeshVertices;
    vcg::Point3d transformationMeshCenter;
    vcg::Trackball transformationMeshTrackball;

    void initMeshWrapper(GLPolyWrap<PolyMesh>& glWrap, PolyMesh* mesh);

    bool detachMode;

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
    void mouseDoubleClickEvent(QMouseEvent* e);

private slots:
    void updateRotate();





public:

    void setMesh1(PolyMesh* mesh);
    void setMesh2(PolyMesh* mesh);
    void setQuadLayout1(QuadLayoutData* quadLayoutData1);
    void setQuadLayout2(QuadLayoutData* quadLayoutData2);
    void setBoolean(TriangleMesh* boolean);
    void setIntersectionVertices(std::vector<size_t>* intersectionCurves);
    void setPreservedSurface(PolyMesh* boolean);
    void setQuadLayoutPreserved1(QuadLayoutData* quadLayoutData2);
    void setQuadLayoutPreserved2(QuadLayoutData* quadLayoutData2);
    void setNewSurface(TriangleMesh* boolean);
    void setChartSides(ChartData* chartData);
    void setIlpResult(std::vector<int>* ilpResult);
    void setQuadrangulation(PolyMesh* quadrangulation);
    void setQuadLayoutQuadrangulation(QuadLayoutData* quadLayoutQuadrangulation);
    void setResult(PolyMesh* result);
    void setOriginVertices(std::vector<size_t>* OriginVertices);

    void setMesh1Visibility(bool visible);
    void setMesh2Visibility(bool visible);
    void setQuadLayout1Visibility(bool visible);
    void setQuadLayout2Visibility(bool visible);
    void setQuadLayoutPreserved1Visibility(bool visible);
    void setQuadLayoutPreserved2Visibility(bool visible);
    void setBooleanVisibility(bool visible);
    void setIntersectionVerticesVisibility(bool visible);
    void setPreservedSurfaceVisibility(bool visible);
    void setNewSurfaceVisibility(bool visible);
    void setChartSidesVisibility(bool visible);
    void setILPVisibility(bool visible);
    void setQuadrangulationVisibility(bool visible);
    void setQuadLayoutQuadrangulationVisibility(bool visible);
    void setResultVisibility(bool visible);
    void setOriginVerticesVisibility(bool visible);

    void resetSceneOnDebugMeshes();



private:
    std::vector<GLPolyWrap<PolyMesh>> glWrapMeshes;

    GLPolyWrap<PolyMesh> glWrapMesh1;
    GLPolyWrap<PolyMesh> glWrapMesh2;
    GLPolyWrap<TriangleMesh> glWrapBoolean;
    GLVerticesWrap<TriangleMesh> glWrapIntersectionVertices;
    GLPolyWrap<PolyMesh> glWrapPreservedSurface;
    GLPolyWrap<TriangleMesh> glWrapNewSurface;
    GLPolyWrap<PolyMesh> glWrapQuadrangulation;
    GLPolyWrap<PolyMesh> glWrapResult;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayout2;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved1;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutPreserved2;
    GLChartSidesWrap<TriangleMesh> glWrapChartSides;
    GLQuadLayoutWrap<PolyMesh> glWrapQuadLayoutQuadrangulation;
    GLVerticesWrap<PolyMesh> glWrapOriginVertices;


    void initMeshWrapper(GLPolyWrap<TriangleMesh>& glWrap, TriangleMesh* mesh);
    template<class MeshType>
    void initQuadLayoutWrapper(GLQuadLayoutWrap<MeshType>& glWrap, QuadLayoutData* quadLayoutData);
    template<class MeshType>
    void initChartSidesWrapper(GLChartSidesWrap<MeshType>& glWrap, ChartData* chartData);
};

#endif //GLAREA_H
