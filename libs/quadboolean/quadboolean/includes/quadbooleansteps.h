#ifndef QUADBOOLEANSTEPS_H
#define QUADBOOLEANSTEPS_H

#include <vector>

#include <Eigen/Core>

#include <unordered_set>

#include "quadcharts.h"
#include "quadbooleancommon.h"
#include "quadilp.h"

namespace QuadBoolean {

namespace internal {

template<class PolyMeshType>
void tracePatchLayout(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        bool motorcycle = true);

template<class PolyMeshType, class TriangleMeshType>
void triangulateMesh(
        PolyMeshType& mesh,
        TriangleMeshType& trimesh,
        std::vector<int>& birthFace);

template<class TriangleMeshType>
void computeBooleanOperation(
        TriangleMeshType& trimesh1,
        TriangleMeshType& trimesh2,
        const QuadBoolean::Operation& operation,
        TriangleMeshType& result,
        Eigen::MatrixXd& VA,
        Eigen::MatrixXd& VB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FA,
        Eigen::MatrixXi& FB,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

inline std::vector<std::pair<size_t, size_t>> getBirthTriangles(
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J);

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
        const std::vector<size_t>& intersectionVertices,
        const int intersectionSmoothingInterations,
        const double NRing,
        const double maxBB,
        std::vector<size_t>& smoothedVertices);

template<class PolyMeshType, class TriangleMeshType>
void getSurfaces(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        TriangleMeshType& boolean,
        const std::vector<std::pair<size_t, size_t>>& birthTriangle,
        const std::vector<int>& birthFace1,
        const std::vector<int>& birthFace2,
        const std::vector<size_t>& intersectionVertices,
        const std::vector<size_t>& smoothedVertices,
        const bool motorcycle,
        const bool patchRetraction,
        const double patchRetractionNRing,
        const int minRectangleArea,
        const int minPatchArea,
        const bool mergeQuads,
        const bool deleteSmall,
        const bool deleteNonConnected,
        const double maxBB,
        const bool preservePolygons1,
        const bool preservePolygons2,
        std::vector<int>& faceLabel1,
        std::vector<int>& faceLabel2,
        std::vector<bool>& isPreserved1,
        std::vector<bool>& isPreserved2,
        std::vector<bool>& isNewSurface,
        std::vector<int>& preservedSurfaceLabel,
        std::unordered_map<size_t, size_t>& preservedFacesMap,
        std::unordered_map<size_t, size_t>& preservedVerticesMap,
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface);

template<class PolyMeshType, class TriangleMeshType>
bool makeILPFeasible(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver);

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
std::vector<int> findSubdivisions(
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
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        TriangleMeshType& targetBoolean,
        const int resultSmoothingIterations,
        const double resultSmoothingNRing,
        const int resultSmoothingLaplacianIterations,
        const double resultSmoothingLaplacianRing,
        const std::unordered_map<size_t, size_t>& preservedFacesMap,
        const std::unordered_map<size_t, size_t>& preservedVerticesMap,
        SourceInfo& sourceInfo);

}
}

#include "quadbooleansteps.cpp"

#endif // QUADBOOLEANSTEPS_H
