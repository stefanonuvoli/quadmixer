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
}

template<class MeshType>
void GLChartSidesWrap<MeshType>::GLDraw()
{
    if (this->mesh != nullptr && this->chartData != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.999999);
        glLineWidth(4);

        for (int sId = 0; sId < chartData->sides.size(); sId++) {
            const QuadBoolean::ChartSide& side = chartData->sides[sId];
            for (int i = 0; i < side.vertices.size()-1; i++) {
                vcg::glColor(vcg::Color4b(20,20,20,255));
                glBegin(GL_LINES);
                vcg::glVertex(side.vertices[i]->P());
                vcg::glVertex(side.vertices[i+1]->P());
                glEnd();
            }

            typename MeshType::VertexType* centerV = side.vertices.at(side.vertices.size()/2);

            std::string sideInfo = std::to_string(side.size);
            if (this->ilpResult != nullptr) {
                sideInfo += " -> " + std::to_string((*ilpResult)[sId]);
            }

            drawTextGL(centerV->P().X(),
                       centerV->P().Y(),
                       centerV->P().Z(),
                       sideInfo.c_str());
        }

        glPopAttrib();
    }
}
