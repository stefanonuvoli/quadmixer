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

        glDepthRange(0.0,0.999999);
        glPointSize(6);
        glDisable(GL_LIGHTING);

        glBegin(GL_POINTS);
        for (const size_t& vId : *vertices) {
            vcg::glColor(vcg::Color4b(0,0,200,255));
            vcg::glVertex(mesh->vert[vId].P());
        }
        glEnd();

        //glEnd();
        glPopAttrib();
    }
}
