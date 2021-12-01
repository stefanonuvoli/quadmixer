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

        glDepthRange(0.0,0.99999);
        glDisable(GL_LIGHTING);

        for (int sId = 0; sId < chartData->subsides.size(); sId++) {
            const QuadRetopology::ChartSubside& subside = chartData->subsides[sId];

            for (int i = 0; i < subside.vertices.size()-1; i++) {
                glLineWidth(6);
                vcg::glColor(vcg::Color4b(50,50,50,255));
                glBegin(GL_LINES);
                vcg::glVertex(mesh->vert[subside.vertices[i]].P());
                vcg::glVertex(mesh->vert[subside.vertices[i+1]].P());
                glEnd();
            }

            glPointSize(4);
            vcg::glColor(vcg::Color4b(80,160,80,255));
            glBegin(GL_POINTS);
            vcg::glVertex(mesh->vert[subside.vertices[0]].P());
            vcg::glVertex(mesh->vert[subside.vertices[subside.vertices.size()-1]].P());
            glEnd();


            assert(subside.vertices.size()>1);

            typename MeshType::VertexType& firstV = mesh->vert[subside.vertices.at((subside.vertices.size()-1)/2+1)];
            typename MeshType::VertexType& endV = mesh->vert[subside.vertices.at((subside.vertices.size()-1)/2)];
            typename MeshType::CoordType centerV = (endV.P() + firstV.P())/2;

            if (this->ilpResult != nullptr && this->ilpVisible) {
                std::string sideInfo = std::to_string((*ilpResult)[sId]);

                vcg::glColor(vcg::Color4b(0,0,0,255));
                drawTextGL(centerV.X(),
                           centerV.Y(),
                           centerV.Z(),
                           sideInfo.c_str(),
                           GLUT_BITMAP_TIMES_ROMAN_24);
            }
        }

        glPopAttrib();
    }
}
