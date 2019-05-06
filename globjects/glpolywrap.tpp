#include "glpolywrap.h"

#include <GL/gl.h>

#include <vcg/complex/complex.h>
#include <wrap/gl/space.h>

template<class MeshType>
GLPolyWrap<MeshType>::GLPolyWrap()
{
    this->mesh = nullptr;
    this->visible = true;
    this->wireframe = true;
    this->selected = false;
}

template<class MeshType>
void GLPolyWrap<MeshType>::GLDraw()
{
    if (mesh != nullptr && this->visible) {
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glDepthRange(0.000001,1);

        glEnable(GL_LIGHTING);

        glDisable(GL_CULL_FACE);
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

        //vcg::glColor(vcg::Color4b(200,200,200,255));

        std::vector<typename MeshType::FaceType>& face = mesh->face;

        for(unsigned int i=0; i < face.size(); i++)
        {
            if (selected)
                vcg::glColor(vcg::Color4b(255,100,100,255));
            else
                vcg::glColor(face[i].C());

            if(face[i].IsD())  continue;

            glBegin(GL_POLYGON);

            vcg::glNormal(face[i].N());
            for(int j=0; j<face[i].VN(); j++)
                vcg::glVertex(face[i].V(j)->P() );

            glEnd();
        }

        if (wireframe) {
            glDepthRange(0.0,0.999999);
            glLineWidth(1);
            glDisable(GL_LIGHTING);
            for(unsigned int i=0; i<face.size(); i++)
            {
                if(face[i].IsD())  continue;
                int size=face[i].VN();
                for(int j=0; j<face[i].VN(); j++)
                {
                    vcg::glColor(vcg::Color4b(100,100,100,255));

                    glBegin(GL_LINES);
                    vcg::glVertex( face[i].V(j)->P() );
                    vcg::glVertex( face[i].V((j+1)%size)->P() );
                    glEnd();
                }
            }
        }

        //glEnd();
        glPopAttrib();
    }

}
