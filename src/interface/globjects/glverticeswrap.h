#ifndef GLPOINTSWRAP
#define GLPOINTSWRAP

#include <vector>

template<class MeshType>
class GLVerticesWrap
{
public:
    GLVerticesWrap();

    void GLDraw();

    MeshType* mesh;
    std::vector<size_t>* vertices;

    bool visible;
};

#include "glverticeswrap.tpp"

#endif // GLPOINTSWRAP
