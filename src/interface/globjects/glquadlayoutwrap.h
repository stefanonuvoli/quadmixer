#ifndef GLQUADLAYOUTWRAP_H
#define GLQUADLAYOUTWRAP_H

#include <quadboolean/includes/quadlayoutdata.h>

template<class MeshType>
class GLQuadLayoutWrap
{
public:
    GLQuadLayoutWrap();

    void GLDraw();

    MeshType* mesh;
    QuadBoolean::internal::QuadLayoutData<MeshType>* quadLayoutData;

    bool visible;
};

#include "glquadlayoutwrap.cpp"

#endif // GLQUADLAYOUTWRAP_H
