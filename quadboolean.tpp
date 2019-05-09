#include "quadboolean.h"


namespace QuadBoolean {

inline Parameters::Parameters()
{
    motorcycle = DEFAULTMOTORCYCLE;
    intersectionSmoothingIterations = DEFAULTINTERSECTIONSMOOTHINGITERATIONS;
    intersectionSmoothingNRing = DEFAULTINTERSECTIONSMOOTHINGNRING;
    intersectionSmoothingMaxBB = DEFAULTINTERSECTIONSMOOTHINGMAXBB;
    minRectangleArea = DEFAULTMINRECTANGLEAREA;
    minPatchArea = DEFAULTMINPATCHAREA;
    mergeQuads = DEFAULTMERGEQUADS;
    deleteSmall = DEFAULTDELETESMALL;
    deleteNonConnected = DEFAULTDELETENONCONNECTED;
    ilpMethod = DEFAULTILPMETHOD;
    alpha = DEFAULTALPHA;
    beta = DEFAULTBETA;
    initialRemeshing = DEFAULTINITIALREMESHING;
    initialRemeshingEdgeFactor = DEFAULTEDGEFACTOR;
    reproject = DEFAULTREPROJECT;
    splitConcaves = DEFAULTSPLITCONCAVES;
    finalSmoothing = DEFAULTFINALSMOOTHING;
    chartSmoothingIterations = DEFAULTCHARTSMOOTHINGITERATIONS;
    quadrangulationSmoothingIterations = DEFAULTMESHSMOOTHINGITERATIONS;
    resultSmoothingIterations = DEFAULTRESULTSMOOTHINGITERATIONS;
    resultSmoothingNRing = DEFAULTRESULTSMOOTHINGNRING;
    resultSmoothingLaplacianIterations = DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS;
    resultSmoothingLaplacianNRing = DEFAULTRESULTSMOOTHINGLAPLACIANNRING;
}


template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result)
{
    Parameters defaultParam;

    return quadBoolean(
                mesh1,
                mesh2,
                operation,
                result,
                defaultParam);
}

template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const Parameters& parameters)
{
     std::vector<int> quadTracerLabel1;
     std::vector<int> quadTracerLabel2;

     TriangleMeshType triMesh1, triMesh2, boolean;
     Eigen::MatrixXd VA, VB, VR;
     Eigen::MatrixXi FA, FB, FR;
     Eigen::VectorXi J;

     std::vector<size_t> intersectionVertices;

     std::vector<int> birthQuad1;
     std::vector<int> birthQuad2;

     std::vector<bool> preservedQuad1;
     std::vector<bool> preservedQuad2;

     std::unordered_set<int> affectedPatches1;
     std::unordered_set<int> affectedPatches2;

     std::vector<int> preservedFaceLabel1;
     std::vector<int> preservedFaceLabel2;

     PolyMesh preservedSurface;
     std::vector<int> preservedSurfaceLabel;

     TriangleMeshType newSurface;
     std::vector<int> newSurfaceLabel;

     internal::ChartData chartData;

     std::vector<int> ilpResult;

     PolyMesh quadrangulation;
     std::vector<int> quadrangulationLabel;

     bool isTriangleMesh1 = internal::isTriangleMesh(mesh1);
     bool isTriangleMesh2 = internal::isTriangleMesh(mesh2);
     bool isQuadMesh1 = internal::isQuadMesh(mesh1);
     bool isQuadMesh2 = internal::isQuadMesh(mesh2);

     if (!isQuadMesh1 && !isTriangleMesh1) {
         std::cout << "Impossible to compute: you can only use quad or triangle meshes." << std::endl;
         return;
     }

     if (!isQuadMesh2 && !isTriangleMesh2) {
         std::cout << "Impossible to compute: you can only use quad or triangle meshes." << std::endl;
         return;
     }

     //Trace quads following singularities
     internal::traceQuads(mesh1, quadTracerLabel1, parameters.motorcycle);
     internal::traceQuads(mesh2, quadTracerLabel2, parameters.motorcycle);

     //Triangulate
     internal::triangulateQuadMesh(mesh1, isQuadMesh1, triMesh1, birthQuad1);
     internal::triangulateQuadMesh(mesh2, isQuadMesh2, triMesh2, birthQuad2);

     //Compute boolean operation
     internal::computeBooleanOperation(
                 triMesh1,
                 triMesh2,
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


     //Smooth along intersection curves
     QuadBoolean::internal::smoothAlongIntersectionVertices(
                 boolean,
                 VR,
                 FR,
                 intersectionVertices,
                 parameters.intersectionSmoothingIterations,
                 parameters.intersectionSmoothingNRing,
                 parameters.intersectionSmoothingMaxBB);

     //Face labels
     preservedFaceLabel1 = quadTracerLabel1;
     preservedFaceLabel2 = quadTracerLabel2;

     //Find preserved quads
     internal::findPreservedQuads(
                 mesh1, mesh2,
                 boolean,
                 isQuadMesh1, isQuadMesh2,
                 preservedQuad1, preservedQuad2);

     //Find affected patches
     internal::findAffectedPatches(mesh1, preservedQuad1, preservedFaceLabel1, affectedPatches1);
     internal::findAffectedPatches(mesh2, preservedQuad2, preservedFaceLabel2, affectedPatches2);

     //Maximum rectangles in the patches
     preservedFaceLabel1 = internal::splitQuadPatchesInMaximumRectangles(mesh1, isQuadMesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1, parameters.minRectangleArea, true);
     preservedFaceLabel2 = internal::splitQuadPatchesInMaximumRectangles(mesh2, isQuadMesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2, parameters.minRectangleArea, true);

     //Merge rectangular patches
     if (parameters.mergeQuads) {
         int nMerged1 = internal::mergeQuadPatches(mesh1, isQuadMesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
         int nMerged2 = internal::mergeQuadPatches(mesh2, isQuadMesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);
     }
     if (parameters.deleteSmall) {
         //Delete small patches
         int nSmall1 = internal::deleteSmallQuadPatches(mesh1, isQuadMesh1, affectedPatches1, parameters.minPatchArea, preservedFaceLabel1, preservedQuad1);
         int nSmall2 = internal::deleteSmallQuadPatches(mesh2, isQuadMesh2, affectedPatches2, parameters.minPatchArea, preservedFaceLabel2, preservedQuad2);
     }
     if (parameters.deleteNonConnected) {
         //Delete non-connected patches
         int nNonConnected1 = internal::deleteNonConnectedQuadPatches(mesh1, isQuadMesh1, preservedFaceLabel1, preservedQuad1);
         int nNonConnected2 = internal::deleteNonConnectedQuadPatches(mesh2, isQuadMesh2, preservedFaceLabel2, preservedQuad2);
     }

     //Get mesh of the preserved surface
     internal::getPreservedSurfaceMesh(
                 mesh1, mesh2,
                 preservedQuad1, preservedQuad2,
                 preservedFaceLabel1, preservedFaceLabel2,
                 preservedSurface, preservedSurfaceLabel);


     //New mesh (to be decomposed in patch)
     internal::getNewSurfaceMesh(
                 boolean,
                 mesh1,
                 mesh2,
                 preservedQuad1,
                 preservedQuad2,
                 newSurface);


     //Get patch decomposition of the new surface
     std::vector<std::vector<size_t>> newSurfacePartitions;
     std::vector<std::vector<size_t>> newSurfaceCorners;
     newSurfaceLabel = internal::getPatchDecomposition(newSurface, newSurfacePartitions, newSurfaceCorners, parameters.initialRemeshing, parameters.initialRemeshingEdgeFactor, parameters.reproject, parameters.splitConcaves, parameters.finalSmoothing);

     //Get chart data
     chartData = internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

     //Solve ILP to find best side size
     ilpResult = internal::findBestSideSize(newSurface, chartData, parameters.alpha, parameters.beta, parameters.ilpMethod);

     //Quadrangulate
     internal::quadrangulate(
                 newSurface,
                 chartData,
                 ilpResult,
                 parameters.chartSmoothingIterations,
                 parameters.quadrangulationSmoothingIterations,
                 quadrangulation,
                 quadrangulationLabel);

     vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormalByFitting(quadrangulation);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(quadrangulation);
     vcg::tri::UpdateBounding<PolyMesh>::Box(quadrangulation);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(quadrangulation);

     vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormalByFitting(preservedSurface);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(preservedSurface);
     vcg::tri::UpdateBounding<PolyMesh>::Box(preservedSurface);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(preservedSurface);

     //Get the result
     result.Clear();
     QuadBoolean::internal::getResult(preservedSurface, quadrangulation, result, boolean, parameters.resultSmoothingIterations, parameters.resultSmoothingNRing, parameters.resultSmoothingLaplacianIterations, parameters.resultSmoothingLaplacianNRing);



     vcg::PolygonalAlgorithm<PolyMesh>::UpdateFaceNormalByFitting(result);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalized(result);
     vcg::tri::UpdateBounding<PolyMesh>::Box(result);
     vcg::tri::UpdateNormal<PolyMesh>::PerVertexNormalizedPerFace(result);
}

}
