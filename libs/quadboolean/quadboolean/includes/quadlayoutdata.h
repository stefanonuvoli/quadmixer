#ifndef QUADLAYOUTDATA_H
#define QUADLAYOUTDATA_H

#include <vector>
#include <set>

#include <Eigen/Core>

#include <vcg/complex/complex.h>

#define MINIMUMSIDELENGTHDIFFERENCE 3
#define MINIMUMSIDEMINLENGTH 2
#define MINIMUMSIDEMAXLENGTH 5

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
struct QuadLayoutPatch {
    std::vector<size_t> faces;
    vcg::face::Pos<typename PolyMeshType::FaceType> startPos;
    size_t sizeX;
    size_t sizeY;
};
template<class PolyMeshType>
struct QuadLayoutData {
    std::set<int> labels;
    std::vector<QuadLayoutPatch<PolyMeshType>> quadPatches;
};


template<class PolyMeshType>
QuadLayoutData<PolyMeshType> getQuadLayoutData(
        PolyMeshType& mesh,        
        const bool isQuadMesh,
        const std::vector<int>& faceLabel);

}
}

#include "quadlayoutdata.cpp"

#endif // QUADLAYOUTDATA_H
