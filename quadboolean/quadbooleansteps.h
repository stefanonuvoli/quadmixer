#ifndef QUADBOOLEANSTEPS_H
#define QUADBOOLEANSTEPS_H

#include <vector>

#include <Eigen/Core>

#include <unordered_set>

#include "quadboolean/quadcharts.h"
#include "quadboolean/quadbooleanoperation.h"

namespace QuadBoolean {

namespace internal {

template<class PolyMeshType>
void traceQuads(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        bool motorcycle = true);

template<class PolyMeshType, class TriangleMeshType>
void triangulateQuadMesh(
        PolyMeshType& mesh,
        TriangleMeshType& triMesh,
        std::vector<int>& birthQuad);

template<class TriangleMeshType>
void computeBooleanOperation(
        TriangleMeshType& triMesh1,
        TriangleMeshType& triMesh2,
        const QuadBoolean::Operation& operation,
        TriangleMeshType& result,
        Eigen::MatrixXd& VA,
        Eigen::MatrixXd& VB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FA,
        Eigen::MatrixXi& FB,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

template<class TriangleMeshType>
void findPreservedQuads(
        TriangleMeshType& triMesh1,
        TriangleMeshType& triMesh2,
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FB,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J,
        const std::vector<int>& birthQuad1,
        const std::vector<int>& birthQuad2,
        std::vector<bool>& preservedQuad1,
        std::vector<bool>& preservedQuad2);

template<class PolyMeshType>
void findAffectedPatches(
        PolyMeshType& mesh,
        const std::vector<bool>& preservedQuad,
        const std::vector<int>& faceLabel,
        std::unordered_set<int>& affectedPatches);

std::vector<int> findBestSideSize(
        const ChartData& chartData,
        const double& alpha);

template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        const ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int meshSmoothingIterations,
        PolyMeshType& quadrangulatedNewSurface,
        std::vector<int>& quadrangulatedNewSurfaceLabel);

template<class TriangleMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners);

}
}

#include "quadbooleansteps.tpp"

#endif // QUADBOOLEANSTEPS_H
