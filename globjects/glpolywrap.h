#ifndef GLPOLYWRAP_H
#define GLPOLYWRAP_H


template<class MeshType>
class GLPolyWrap
{
public:
    GLPolyWrap();

    void GLDraw(bool wireframe);

    MeshType* mesh;
    bool visible;

    bool wireframe;

    bool transformation;
    bool target1;
    bool target2;

    int name;
};

#include "glpolywrap.tpp"

#endif // GLPOLYWRAP_H
