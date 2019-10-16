#ifndef QUADPRESERVED_H
#define QUADPRESERVED_H

#include <vector>
#include <unordered_set>

#include <Eigen/Core>

#include "quadlayoutdata.h"

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType, class TriangleMeshType>
void computePreservedQuadForMesh(
        PolyMeshType& mesh,
        TriangleMeshType& triResult,
        const bool isQuadMesh,
        const double minDistance,
        std::vector<bool>& preservedQuad);

template<class PolyMeshType>
std::vector<int> splitQuadPatchesInMaximumRectangles(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad,
        const int minimumRectangleArea,
        const bool recursive = true);

template<class PolyMeshType>
int mergeQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedQuad);

template<class PolyMeshType>
int deleteNonConnectedQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);

template<class PolyMeshType>
int deleteSmallQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        const std::unordered_set<int>& affectedPatches,
        const int minPatchArea,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);

}
}

#include "quadpreserved.cpp"


#endif // QUADPRESERVED_H
