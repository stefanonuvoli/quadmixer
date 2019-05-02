#include "glchartsideswrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

#include "gldrawtext.h"

template<class MeshType>
GLChartSidesWrap<MeshType>::GLChartSidesWrap()
{
    this->mesh = nullptr;
    this->chartData = nullptr;
    this->ilpResult = nullptr;
    this->visible = true;
    this->ilpVisible = true;
}

template<class MeshType>
void GLChartSidesWrap<MeshType>::GLDraw()
{
    if (this->mesh != nullptr && this->chartData != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.999999);
        glLineWidth(3);
        glDisable(GL_LIGHTING);

        for (int sId = 0; sId < chartData->subSides.size(); sId++) {
            const QuadBoolean::internal::ChartSubSide& side = chartData->subSides[sId];
            for (int i = 0; i < side.vertices.size()-1; i++) {
                vcg::glColor(vcg::Color4b(80,80,80,255));
                glBegin(GL_LINES);
                vcg::glVertex(mesh->vert[side.vertices[i]].P());
                vcg::glVertex(mesh->vert[side.vertices[i+1]].P());
                glEnd();
            }

            assert(side.vertices.size()>1);

            typename MeshType::VertexType& firstV = mesh->vert[side.vertices.at((side.vertices.size()-1)/2+1)];
            typename MeshType::VertexType& endV = mesh->vert[side.vertices.at((side.vertices.size()-1)/2)];
            typename MeshType::CoordType centerV = (endV.P() + firstV.P())/2;

            std::string sideInfo = std::to_string(side.size);
            if (this->ilpResult != nullptr && this->ilpVisible) {
                sideInfo += " -> " + std::to_string((*ilpResult)[sId]);
            }

            vcg::glColor(vcg::Color4b(128,128,128,255));
            drawTextGL(centerV.X(),
                       centerV.Y(),
                       centerV.Z(),
                       sideInfo.c_str());
        }

        glPopAttrib();
    }
}
