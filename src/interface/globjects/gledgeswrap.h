#ifndef GLEDGESWRAP_H
#define GLEDGESWRAP_H

#include <vector>

template<class MeshType>
class GLSegmentsWrap
{
public:
    GLSegmentsWrap();

    void GLDraw();

    std::vector<std::pair<typename MeshType::CoordType, typename MeshType::CoordType>> segments;

    bool visible;
};

#include "gledgeswrap.tpp"

#endif // GLEDGESWRAP_H
