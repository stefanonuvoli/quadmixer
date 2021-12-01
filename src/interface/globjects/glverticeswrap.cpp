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

#include "glverticeswrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

template<class MeshType>
GLVerticesWrap<MeshType>::GLVerticesWrap()
{
    this->mesh = nullptr;
    this->vertices = nullptr;
    this->visible = true;
}

template<class MeshType>
void GLVerticesWrap<MeshType>::GLDraw()
{
    if (this->mesh != nullptr && this->vertices != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.99999);
        glPointSize(10);
        glDisable(GL_LIGHTING);

        glBegin(GL_POINTS);
        for (const size_t& vId : *vertices) {
            vcg::glColor(vcg::Color4b(0,255,0,255));
            vcg::glVertex(mesh->vert[vId].P());
        }
        glEnd();

        //glEnd();
        glPopAttrib();
    }
}
