#include "quadboolean.h"


namespace QuadBoolean {

Parameters::Parameters()
{
    motorcycle = DEFAULTMOTORCYCLE;
    intersectionSmoothingIterations = DEFAULTINTERSECTIONSMOOTHINGITERATIONS;
    intersectionSmoothingAVGNRing = DEFAULTINTERSECTIONSMOOTHINGAVGNRING;
    minRectangleArea = DEFAULTMINRECTANGLEAREA;
    mergeQuads = DEFAULTMERGEQUADS;
    deleteSmall = DEFAULTDELETESMALL;
    deleteNonConnected = DEFAULTDELETENONCONNECTED;
    alpha = DEFAULTALPHA;
    initialRemeshing = DEFAULTINITIALREMESHING;
    edgeFactor = DEFAULTEDGEFACTOR;
    reproject = DEFAULTREPROJECT;
    splitConcaves = DEFAULTSPLITCONCAVES;
    finalSmoothing = DEFAULTFINALSMOOTHING;
    chartSmoothingIterations = DEFAULTCHARTSMOOTHINGITERATIONS;
    meshSmoothingIterations = DEFAULTMESHSMOOTHINGITERATIONS;
    resultSmoothingIterations = DEFAULTRESULTSMOOTHINGITERATIONS;
    resultSmoothingAVGNRing = DEFAULTRESULTSMOOTHINGAVGNRING;
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

     std::vector<std::vector<size_t>> intersectionCurves;

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

     PolyMesh quadrangulatedNewSurface;
     std::vector<int> quadrangulatedNewSurfaceLabel;

     //Trace quads
     internal::traceQuads(mesh1, quadTracerLabel1, parameters.motorcycle);
     internal::traceQuads(mesh2, quadTracerLabel2, parameters.motorcycle);

     //Triangulate
     internal::triangulateQuadMesh(mesh1, triMesh1, birthQuad1);
     internal::triangulateQuadMesh(mesh2, triMesh2, birthQuad2);

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
     intersectionCurves =
             QuadBoolean::internal::getIntersectionCurves(
                 triMesh1, triMesh2,
                 VA, VB, VR,
                 FA, FB, FR,
                 J);


     //Smooth along intersection curves
     QuadBoolean::internal::smoothAlongIntersectionCurves(
                 boolean,
                 VR,
                 FR,
                 intersectionCurves,
                 parameters.intersectionSmoothingIterations,
                 parameters.intersectionSmoothingAVGNRing);

     //Face labels
     preservedFaceLabel1 = quadTracerLabel1;
     preservedFaceLabel2 = quadTracerLabel2;

     //Find preserved quads
     internal::findPreservedQuads(
                 triMesh1, triMesh2,
                 VA, VB, VR,
                 FA, FB, FR,
                 J,
                 birthQuad1, birthQuad2,
                 preservedQuad1, preservedQuad2);

     //Find affected patches
     internal::findAffectedPatches(mesh1, preservedQuad1, preservedFaceLabel1, affectedPatches1);
     internal::findAffectedPatches(mesh2, preservedQuad2, preservedFaceLabel2, affectedPatches2);

     //Maximum rectangles in the patches
     preservedFaceLabel1 = internal::splitQuadPatchesInMaximumRectangles(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1, parameters.minRectangleArea, true);
     preservedFaceLabel2 = internal::splitQuadPatchesInMaximumRectangles(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2, parameters.minRectangleArea, true);

     //Merge rectangular patches
     if (parameters.mergeQuads) {
         int nMerged1 = internal::mergeQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
         int nMerged2 = internal::mergeQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);
     }
     if (parameters.deleteSmall) {
         //Delete small patches
         int nSmall1 = internal::deleteSmallQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
         int nSmall2 = internal::deleteSmallQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);
     }
     if (parameters.deleteNonConnected) {
         //Delete non-connected patches
         int nNonConnected1 = internal::deleteNonConnectedQuadPatches(mesh1, preservedFaceLabel1, preservedQuad1);
         int nNonConnected2 = internal::deleteNonConnectedQuadPatches(mesh2, preservedFaceLabel2, preservedQuad2);
     }

     //Get mesh of the preserved surface
     internal::getPreservedSurfaceMesh(
                 mesh1, mesh2,
                 preservedQuad1, preservedQuad2,
                 preservedFaceLabel1, preservedFaceLabel2,
                 preservedSurface, preservedSurfaceLabel);


     //New mesh (to be decomposed in patch)
     size_t nFirstFaces = triMesh1.face.size();

     internal::getNewSurfaceMesh(
                 boolean,
                 nFirstFaces,
                 birthQuad1, birthQuad2,
                 preservedQuad1, preservedQuad2,
                 J,
                 newSurface);


     //Get patch decomposition of the new surface     
     std::vector<std::vector<size_t>> newSurfacePartitions;
     std::vector<std::vector<size_t>> newSurfaceCorners;
     newSurfaceLabel = internal::getPatchDecomposition(newSurface, newSurfacePartitions, newSurfaceCorners, parameters.initialRemeshing, parameters.edgeFactor, parameters.reproject, parameters.splitConcaves, parameters.finalSmoothing);

     //Get chart data
     chartData = internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

     //Solve ILP to find best side size
     ilpResult = internal::findBestSideSize(chartData, parameters.alpha);

     //Quadrangulate
     internal::quadrangulate(
                 newSurface,
                 chartData,
                 ilpResult,
                 parameters.chartSmoothingIterations,
                 parameters.meshSmoothingIterations,
                 quadrangulatedNewSurface,
                 quadrangulatedNewSurfaceLabel);

     //Get the result
     result.Clear();
     QuadBoolean::internal::getResult(preservedSurface, quadrangulatedNewSurface, result, boolean, parameters.resultSmoothingIterations, parameters.resultSmoothingAVGNRing);
}

}
