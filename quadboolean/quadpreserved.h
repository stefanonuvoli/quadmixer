#ifndef QUADPRESERVED_H
#define QUADPRESERVED_H

#include <vector>
#include <unordered_set>

#include <Eigen/Core>

namespace QuadBoolean {
namespace internal {


template<class PolyMeshType>
void computePreservedQuadForMesh(
        PolyMeshType& triMesh,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J,
        const std::vector<int>& birthQuad,
        const size_t offset,
        std::vector<bool>& preservedQuad);

template<class PolyMeshType>
std::vector<int> splitQuadPatchesInMaximumRectangles(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad,
        const int minimumRectangleArea,
        const bool recursive = true);

template<class PolyMeshType>
int mergeQuadPatches(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedQuad);

template<class PolyMeshType>
int deleteNonConnectedQuadPatches(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);

template<class PolyMeshType>
int deleteSmallQuadPatches(
        PolyMeshType& mesh,
        const std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);

}
}

#include "quadpreserved.tpp"


#endif // QUADPRESERVED_H
