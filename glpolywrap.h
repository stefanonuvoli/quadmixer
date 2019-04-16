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
};

#include "glpolywrap.tpp"

#endif // GLPOLYWRAP_H
