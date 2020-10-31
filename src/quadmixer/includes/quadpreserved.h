#ifndef QUADPRESERVED_H
#define QUADPRESERVED_H

#include <vector>
#include <unordered_set>
#include <unordered_map>

#include <Eigen/Core>

#include "quadlayoutdata.h"

namespace QuadBoolean {
namespace internal {


template<class PolyMeshType, class TriangleMeshType>
void findPreservedFaces(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        TriangleMeshType& boolean,
        const std::vector<size_t>& intersectionVertices,
        const std::vector<size_t>& smoothedVertices,
        const bool patchRetraction,
        const double patchRetractionNRing,
        const double maxBB,
        const bool preservePolygons1,
        const bool preservePolygons2,
        const std::vector<std::pair<size_t, size_t>>& birthTriangle,
        const std::vector<int>& birthFace1,
        const std::vector<int>& birthFace2,
        std::vector<std::pair<bool, bool>>& isPreserved1,
        std::vector<std::pair<bool, bool>>& isPreserved2,
        std::vector<bool>& isNewSurface);

template<class PolyMeshType>
void findAffectedPatches(
        PolyMeshType& mesh,
        const std::vector<std::pair<bool, bool>>& preservedFace,
        const std::vector<int>& faceLabel,
        std::unordered_set<int>& affectedPatches);

template<class PolyMeshType>
void getPreservedSurfaceMesh(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<std::pair<bool, bool>>& isPreserved1,
        const std::vector<std::pair<bool, bool>>& isPreserved2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMeshType& preservedSurface,
        std::vector<int>& newFaceLabel,
        std::vector<std::pair<int, int>>& preservedBirthVertexInfo,
        std::vector<std::pair<int, int>>& preservedBirthFaceInfo);

template<class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& boolean,
        const std::vector<std::pair<size_t, size_t>>& birthTriangle,
        const std::vector<int>& birthFace1,
        const std::vector<int>& birthFace2,
        const std::vector<std::pair<bool, bool>>& isPreserved1,
        const std::vector<std::pair<bool, bool>>& isPreserved2,
        std::vector<bool>& isNewSurface,
        TriangleMeshType& newSurface);

template<class PolyMeshType>
std::vector<int> splitPatchesInMaximumRectangles(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<std::pair<bool, bool>>& isPreserved,
        const int minimumRectangleArea,
        const bool recursive = true);

template<class PolyMeshType>
int mergePatches(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        const std::vector<std::pair<bool, bool>>& isPreserved);

template<class PolyMeshType>
int deleteNonConnectedPatches(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        std::vector<std::pair<bool, bool>>& isPreserved);

template<class PolyMeshType>
int deleteSmallPatches(
        PolyMeshType& mesh,
        const std::unordered_set<int>& affectedPatches,
        const int minPatchArea,
        std::vector<int>& faceLabel,
        std::vector<std::pair<bool, bool>>& isPreserved);

}
}

#include "quadpreserved.cpp"


#endif // QUADPRESERVED_H
