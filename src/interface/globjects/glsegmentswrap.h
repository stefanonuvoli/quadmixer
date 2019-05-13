#ifndef GLSEGMENTSWRAP_H
#define GLSEGMENTSWRAP_H

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

#include "glsegmentswrap.tpp"

#endif // GLSEGMENTSWRAP_H
