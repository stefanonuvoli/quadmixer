#ifndef GLPOLYWRAP_H
#define GLPOLYWRAP_H


template<class MeshType>
class GLPolyWrap
{
public:
    GLPolyWrap();

    void GLDraw();

    MeshType* mesh;
    bool visible;

    bool wireframe;
};

#include "glpolywrap.tpp"

#endif // GLPOLYWRAP_H
