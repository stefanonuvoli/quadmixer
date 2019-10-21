#include "quadboolean.h"

namespace QuadBoolean {



template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        SourceInfo& info)
{
    Parameters defaultParam;

    return quadBoolean(
                mesh1,
                mesh2,
                operation,
                result,
                defaultParam,
                info);
}

template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const Parameters& parameters,
        SourceInfo& info)
{
     std::vector<int> quadTracerLabel1;
     std::vector<int> quadTracerLabel2;

     TriangleMeshType trimesh1, trimesh2, boolean;
     Eigen::MatrixXd VA, VB, VR;
     Eigen::MatrixXi FA, FB, FR;
     Eigen::VectorXi J;

     std::vector<size_t> intersectionVertices;

     std::vector<std::pair<size_t, size_t>> birthTriangle;
     std::vector<int> birthFace1;
     std::vector<int> birthFace2;

     std::vector<bool> isPreserved1;
     std::vector<bool> isPreserved2;
     std::vector<bool> isNewSurface;

     std::unordered_set<int> affectedPatches1;
     std::unordered_set<int> affectedPatches2;

     std::vector<int> preservedFaceLabel1;
     std::vector<int> preservedFaceLabel2;

     PolyMeshType preservedSurface;
     std::vector<int> preservedSurfaceLabel;
     std::unordered_map<size_t, size_t> preservedFacesMap;
     std::unordered_map<size_t, size_t> preservedVerticesMap;

     TriangleMeshType newSurface;
     std::vector<int> newSurfaceLabel;

     internal::ChartData chartData;

     std::vector<int> ilpResult;

     PolyMeshType quadrangulation;
     std::vector<int> quadrangulationLabel;

     //Triangulate
     internal::triangulateMesh(mesh1, trimesh1, birthFace1);
     internal::triangulateMesh(mesh2, trimesh2, birthFace2);

     //Compute boolean operation
     internal::computeBooleanOperation(
                 trimesh1,
                 trimesh2,
                 operation,
                 boolean,
                 VA, VB, VR,
                 FA, FB, FR,
                 J);

     //Get intersection curves
     intersectionVertices =
             QuadBoolean::internal::getIntersectionVertices(
                 VA, VB, VR,
                 FA, FB, FR,
                 J);

     //Get birth triangles
     birthTriangle = QuadBoolean::internal::getBirthTriangles(FA, FR, J);


     //Smooth along intersection curves
     QuadBoolean::internal::smoothAlongIntersectionVertices(
                 boolean,
                 intersectionVertices,
                 parameters.intersectionSmoothingIterations,
                 parameters.intersectionSmoothingNRing,
                 parameters.maxBB);

     //Get preserved and new surface
     QuadBoolean::internal::getSurfaces(
             mesh1,
             mesh2,
             trimesh1,
             trimesh2,
             boolean,
             birthTriangle,
             birthFace1,
             birthFace2,
             intersectionVertices,
             parameters.motorcycle,
             parameters.patchRetraction,
             parameters.patchRetractionNRing,
             parameters.minRectangleArea,
             parameters.minPatchArea,
             parameters.mergeQuads,
             parameters.deleteSmall,
             parameters.deleteNonConnected,
             parameters.maxBB,
             parameters.preserveNonQuads,
             preservedFaceLabel1,
             preservedFaceLabel2,
             isPreserved1,
             isPreserved2,
             isNewSurface,
             preservedSurfaceLabel,
             preservedFacesMap,
             preservedVerticesMap,
             preservedSurface,
             newSurface);

     //Make ILP feasible
     internal::makeILPFeasible(preservedSurface, newSurface, parameters.onlyQuads);

     //Get patch decomposition of the new surface
     std::vector<std::vector<size_t>> newSurfacePartitions;
     std::vector<std::vector<size_t>> newSurfaceCorners;
     newSurfaceLabel = internal::getPatchDecomposition(newSurface, preservedSurface, newSurfacePartitions, newSurfaceCorners, parameters.initialRemeshing, parameters.initialRemeshingEdgeFactor, parameters.reproject, parameters.splitConcaves, parameters.finalSmoothing);

     //Get chart data
     chartData = internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

     //Solve ILP to find best side size
     ilpResult = internal::findSubdivisions(
                 newSurface,
                 chartData,
                 parameters.alpha,
                 parameters.beta,
                 parameters.ilpMethod);

     //Quadrangulate
     internal::quadrangulate(
                 newSurface,
                 chartData,
                 ilpResult,
                 parameters.chartSmoothingIterations,
                 parameters.quadrangulationSmoothingIterations,
                 quadrangulation,
                 quadrangulationLabel);

     //Get the result
     QuadBoolean::internal::getResult(
                 mesh1, mesh2, preservedSurface, quadrangulation,
                 result, boolean,
                 parameters.resultSmoothingIterations,
                 parameters.resultSmoothingNRing,
                 parameters.resultSmoothingLaplacianIterations,
                 parameters.resultSmoothingLaplacianNRing,
                 preservedFacesMap, preservedVerticesMap, info);

}

}
