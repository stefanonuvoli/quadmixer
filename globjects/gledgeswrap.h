#ifndef GLEDGESWRAP_H
#define GLEDGESWRAP_H

#include <vector>

template<class MeshType>
class GLEdgesWrap
{
public:
    GLEdgesWrap();

    void GLDraw();

    MeshType* mesh;
    std::vector<std::vector<size_t>>* edges;

    bool visible;
};

#include "gledgeswrap.tpp"

#endif // GLEDGESWRAP_H
