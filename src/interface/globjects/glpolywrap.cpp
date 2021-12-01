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

#include "glpolywrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

#include "gldrawtext.h"

template<class MeshType>
GLPolyWrap<MeshType>::GLPolyWrap()
{
    this->mesh = nullptr;
    this->visible = true;
    this->transformation = false;
    this->target1 = false;
    this->target2 = false;
    this->name = -1;
}

template<class MeshType>
void GLPolyWrap<MeshType>::GLDraw(bool wireframe, int wireframeSize)
{
    if (mesh != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthFunc(GL_LESS);
        glDepthRange(0.0001,1);

        glEnable(GL_LIGHTING);

        glDisable(GL_CULL_FACE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);


        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glShadeModel(GL_FLAT);
//        glShadeModel(GL_SMOOTH);

        //vcg::glColor(vcg::Color4b(200,200,200,255));

        std::vector<typename MeshType::FaceType>& face = mesh->face;

        for(unsigned int i=0; i < face.size(); i++)
        {
            if (transformation) {
                vcg::glColor(vcg::Color4b(255,200,30,255));
            }
            else if (target1) {
                vcg::glColor(vcg::Color4b(200,255,200,255));
            }
            else if (target2) {
                vcg::glColor(vcg::Color4b(200,200,255,255));
            }
            else {
                vcg::glColor(face[i].C());
            }

            if(face[i].IsD())  continue;

            glBegin(GL_POLYGON);

//            for(int j=0; j<face[i].VN(); j++) {
//                vcg::glNormal(face[i].V(j)->N() );
//                vcg::glVertex(face[i].V(j)->P() );
//            }
            vcg::glNormal(face[i].N());
            for(int j=0; j<face[i].VN(); j++)
                vcg::glVertex(face[i].V(j)->P() );

            glEnd();
        }

        if (wireframe) {
            glDepthRange(0.0,0.9999);
            glDepthFunc(GL_LEQUAL);

            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

            glLineWidth(wireframeSize);

            for(unsigned int i=0; i<face.size(); i++)
            {
                if(face[i].IsD())  continue;
                int size=face[i].VN();
                for(int j=0; j<face[i].VN(); j++)
                {
                    vcg::glColor(vcg::Color4b(0,0,0,255));

                    glBegin(GL_LINES);
                    vcg::glVertex( face[i].V(j)->P() );
                    vcg::glVertex( face[i].V((j+1)%size)->P() );
                    glEnd();
                }
            }
            glDepthFunc(GL_LESS);
        }

//        if (transformation) {
//            vcg::glColor(vcg::Color4b(255,165,0,255));
//            drawTextGL(mesh->bbox.max.X(),
//                       mesh->bbox.max.Y(),
//                       mesh->bbox.max.Z(),
//                       "T",
//                       GLUT_BITMAP_TIMES_ROMAN_24);
//        }
//        if (target1) {
//            vcg::glColor(vcg::Color4b(100,200,100,255));
//            drawTextGL(mesh->bbox.min.X(),
//                       mesh->bbox.min.Y(),
//                       mesh->bbox.min.Z(),
//                       "A",
//                       GLUT_BITMAP_TIMES_ROMAN_24);
//        }
//        if (target2) {
//            vcg::glColor(vcg::Color4b(100,100,200,255));
//            drawTextGL(mesh->bbox.min.X(),
//                       mesh->bbox.min.Y(),
//                       mesh->bbox.min.Z(),
//                       "B",
//                       GLUT_BITMAP_TIMES_ROMAN_24);
//        }

        int n = 0;

        glDepthRange(0.0, 0.9999);
        glDepthFunc(GL_LEQUAL);

        glDisable(GL_LIGHTING);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glPointSize(8);

        glBegin(GL_POINTS);
        for (const typename MeshType::CoordType& point : pickedPoints) {
            if (n % 2 == 0) {
                vcg::glColor(vcg::Color4b(255,0,0,255));
            }
            else {
                vcg::glColor(vcg::Color4b(0,0,255,255));
            }
            vcg::glVertex(point);

            n++;
        }

        glDepthFunc(GL_LESS);


        glEnd();

        glPopAttrib();
    }

}
