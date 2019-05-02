#include "gledgeswrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

template<class MeshType>
GLEdgesWrap<MeshType>::GLEdgesWrap()
{
    this->mesh = nullptr;
    this->edges = nullptr;
    this->visible = true;
}

template<class MeshType>
void GLEdgesWrap<MeshType>::GLDraw()
{
    if (this->mesh != nullptr && this->edges != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.999999);
        glLineWidth(4);
        glDisable(GL_LIGHTING);

        for (const std::vector<size_t>& edge : *edges) {
            for (size_t i = 0; i < edge.size() - 1; i++) {
                vcg::glColor(vcg::Color4b(0,0,200,255));
                glBegin(GL_LINES);
                vcg::glVertex(mesh->vert[edge[i]].P());
                vcg::glVertex(mesh->vert[edge[i+1]].P());
                glEnd();
            }
        }

        //glEnd();
        glPopAttrib();
    }
}
