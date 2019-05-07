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
std::vector<std::vector<size_t>> getIntersectionCurves(
        TriangleMeshType& triMesh1,
        TriangleMeshType& triMesh2,
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FB,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J);

template<class TriangleMeshType>
void smoothAlongIntersectionCurves(
        TriangleMeshType& boolean,
        const std::vector<std::vector<size_t>>& intersectionCurves,
        const int intersectionSmoothingInterations);

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

template<class PolyMeshType>
void getPreservedSurfaceMesh(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMeshType& preservedSurface,
        std::vector<int>& newFaceLabel);

template<class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& triResult,
        const size_t& nFirstFaces,
        const std::vector<int>& birthQuad1,
        const std::vector<int>& birthQuad2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const Eigen::VectorXi& J,
        TriangleMeshType& newSurface);

template<class TriangleMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners,
        const bool initialRemeshing,
        const double edgeFactor,
        const bool reproject,
        const bool splitConcaves,
        const bool finalSmoothing);

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


template<class PolyMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        const double meshSmoothingIterations);

}
}

#include "quadbooleansteps.tpp"

#endif // QUADBOOLEANSTEPS_H
