#ifndef GLQUADLAYOUTWRAP_H
#define GLQUADLAYOUTWRAP_H

#include "quadpatches.h"

template<class MeshType>
class GLQuadLayoutWrap
{
public:
    GLQuadLayoutWrap();

    void GLDraw();

    MeshType* mesh;
    QuadBoolean::QuadData* quadData;

    bool visible;
};

#include "glquadlayoutwrap.tpp"

#endif // GLQUADLAYOUTWRAP_H
