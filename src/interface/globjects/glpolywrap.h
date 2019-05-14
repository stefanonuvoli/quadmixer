#ifndef GLPOLYWRAP_H
#define GLPOLYWRAP_H

#include <vector>

template<class MeshType>
class GLPolyWrap
{
public:
    GLPolyWrap();

    void GLDraw(bool wireframe = true, int wireframeSize = 1);

    MeshType* mesh;
    bool visible;

    bool wireframe;

    bool transformation;
    bool target1;
    bool target2;

    std::vector<typename MeshType::CoordType> pickedPoints;

    int name;
};

#include "glpolywrap.tpp"

#endif // GLPOLYWRAP_H
