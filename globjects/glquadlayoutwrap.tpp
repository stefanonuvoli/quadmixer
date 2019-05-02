#include "glquadlayoutwrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

template<class MeshType>
GLQuadLayoutWrap<MeshType>::GLQuadLayoutWrap()
{
    this->mesh = nullptr;
    this->quadLayoutData = nullptr;
    this->visible = true;
}

template<class MeshType>
void GLQuadLayoutWrap<MeshType>::GLDraw()
{
    if (this->mesh != nullptr && this->quadLayoutData != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.0,0.999999);
        glLineWidth(3);
        glDisable(GL_LIGHTING);

        for (const int& pId : quadLayoutData->labels) {
            const QuadBoolean::internal::QuadLayoutPatch<MeshType>& patch = quadLayoutData->quadPatches[pId];

            const vcg::face::Pos<typename MeshType::FaceType>& startPos = patch.startPos;
            vcg::face::Pos<typename MeshType::FaceType> pos;

            const size_t& sizeX = patch.sizeX;
            const size_t& sizeY = patch.sizeY;

            assert(!startPos.IsNull());

            pos.Set(startPos.F(), startPos.E(), startPos.V());
            for (size_t i = 0; i < 4; i++) {
                for (size_t j = 0; j < (i%2 == 0 ? sizeX : sizeY); j++) {
                    vcg::glColor(vcg::Color4b(80,80,80,255));

                    glBegin(GL_LINES);
                    pos.FlipV();
                    vcg::glVertex(pos.V()->P());
                    pos.FlipV();
                    vcg::glVertex(pos.V()->P());
                    glEnd();

                    if (j < (i%2 == 0 ? sizeX-1 : sizeY-1)) {
                        //Go forward
                        pos.FlipE();
                        pos.FlipF();
                        pos.FlipE();
                        pos.FlipV();
                    }
                }

                //Turn left and go forward
                pos.FlipE();
                pos.FlipV();
            }
        }
        //glEnd();
        glPopAttrib();
    }
}
