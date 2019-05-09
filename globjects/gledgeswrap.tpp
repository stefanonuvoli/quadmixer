#include "gledgeswrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

template<class MeshType>
GLSegmentsWrap<MeshType>::GLSegmentsWrap()
{
    this->visible = true;
}

template<class MeshType>
void GLSegmentsWrap<MeshType>::GLDraw()
{
    if (this->segments.size() > 0 && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.999999);
        glLineWidth(6);
        glDisable(GL_LIGHTING);

        for (const std::pair<typename MeshType::CoordType, typename MeshType::CoordType>& edge : segments) {
            vcg::glColor(vcg::Color4b(200,200,0,255));
            glBegin(GL_LINES);
            vcg::glVertex(edge.first);
            vcg::glVertex(edge.second);
            glEnd();
        }

        //glEnd();
        glPopAttrib();
    }
}
