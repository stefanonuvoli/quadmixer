#ifndef QUADBOOLEANSTEPS_H
#define QUADBOOLEANSTEPS_H

#include <vector>

#include <Eigen/Core>

#include <unordered_set>

#include "quadcharts.h"
#include "quadbooleanoperation.h"
#include "quadilp.h"

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
        const bool isQuadMesh,
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

std::vector<size_t> getIntersectionVertices(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FB,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J);

template<class TriangleMeshType>
void smoothAlongIntersectionVertices(
        TriangleMeshType& boolean,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        const std::vector<size_t>& intersectionVertices,
        const int intersectionSmoothingInterations,
        const int avgNRing,
        const double maxBB);

template<class PolyMeshType, class TriangleMeshType>
void findPreservedQuads(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        TriangleMeshType& triResult,
        const bool isQuadMesh1,
        const bool isQuadMesh2,
        const std::vector<size_t>& intersectionVertices,
        const bool patchRetraction,
        const int patchRetractionNRing,
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

template<class PolyMeshType, class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& triResult,
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        TriangleMeshType& newSurface);

template<class PolyMeshType, class TriangleMeshType>
bool makeILPFeasible(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface);

template<class TriangleMeshType, class PolyMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        PolyMeshType& preservedSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners,
        const bool initialRemeshing,
        const double edgeFactor,
        const bool reproject,
        const bool splitConcaves,
        const bool finalSmoothing);

template<class TriangleMeshType>
std::vector<int> findBestSideSize(
        TriangleMeshType& newSurface,
        ChartData& chartData,
        const double alpha,
        const double beta,
        const ILPMethod& method);

template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        const ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int quadrangulationSmoothingIterations,
        PolyMeshType& quadrangulatedNewSurface,
        std::vector<int>& quadrangulatedNewSurfaceLabel);


template<class PolyMeshType, class TriangleMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        TriangleMeshType& targetBoolean,
        const int resultSmoothingIterations,
        const int resultSmoothingAvgNRing,
        const int resultSmoothingLaplacianIterations,
        const int resultSmoothingLaplacianAvgNRing,
        std::vector<size_t>& preservedFacesId,
        std::vector<size_t>& newFacesIds);

}
}

#include "quadbooleansteps.tpp"

#endif // QUADBOOLEANSTEPS_H
