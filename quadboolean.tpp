#include "quadboolean.h"


namespace QuadBoolean {

template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result)
{
    return quadBoolean(
                mesh1,
                mesh2,
                operation,
                result,
                DEFAULTMOTORCYCLE,
                DEFAULTMINRECTANGLESIDE,
                DEFAULTMERGEQUADS,
                DEFAULTDELETESMALL,
                DEFAULTDELETENONCONNECTED,
                DEFAULTALPHA,
                DEFAULTCHARTSMOOTHINGITERATION,
                DEFAULTMESHSMOOTHINGITERATION);
}

template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const bool motorcycle,
        const size_t minRectangleSide,
        const bool mergeQuads,
        const bool deleteSmall,
        const bool deleteNonConnected,
        const double alpha,
        int chartSmoothingIterations,
        int meshSmoothingIterations)
{
     std::vector<int> quadTracerLabel1;
     std::vector<int> quadTracerLabel2;

     TriangleMeshType triMesh1, triMesh2, boolean;
     Eigen::MatrixXd VA, VB, VR;
     Eigen::MatrixXi FA, FB, FR;
     Eigen::VectorXi J;

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

     PolyMesh newSurface;
     std::vector<int> newSurfaceLabel;

     internal::ChartData chartData;

     std::vector<int> ilpResult;

     PolyMesh quadrangulatedNewSurface;
     std::vector<int> quadrangulatedNewSurfaceLabel;

     //Trace quads
     internal::traceQuads(mesh1, quadTracerLabel1, motorcycle);
     internal::traceQuads(mesh2, quadTracerLabel2, motorcycle);

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
     preservedFaceLabel1 = internal::splitQuadPatchesInMaximumRectangles(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1, minRectangleSide, true);
     preservedFaceLabel2 = internal::splitQuadPatchesInMaximumRectangles(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2, minRectangleSide, true);

     //Merge rectangular patches
     if (mergeQuads) {
         int nMerged1 = mergeQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
         int nMerged2 = mergeQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);
     }
     if (deleteSmall) {
         //Delete small patches
         int nSmall1 = deleteSmallQuadPatches(mesh1, affectedPatches1, preservedFaceLabel1, preservedQuad1);
         int nSmall2 = deleteSmallQuadPatches(mesh2, affectedPatches2, preservedFaceLabel2, preservedQuad2);
     }
     if (deleteNonConnected) {
         //Delete non-connected patches
         int nNonConnected1 = deleteNonConnectedQuadPatches(mesh1, preservedFaceLabel1, preservedQuad1);
         int nNonConnected2 = deleteNonConnectedQuadPatches(mesh2, preservedFaceLabel2, preservedQuad2);
     }

     //Get mesh of the preserved surface
     internal::getPreservedSurfaceMesh(
                 mesh1, mesh2,
                 preservedQuad1, preservedQuad2,
                 preservedFaceLabel1, preservedFaceLabel2,
                 preservedSurface, preservedSurfaceLabel);


     //New mesh (to be decomposed in patch)
     size_t nFirstFaces = triMesh1.face.size();
     QuadBoolean::internal::getNewSurfaceMesh(
                 boolean,
                 nFirstFaces,
                 birthQuad1, birthQuad2,
                 preservedQuad1, preservedQuad2,
                 J,
                 newSurface);


     //Get chart data
     chartData = internal::getCharts(newSurface, newSurfaceLabel);

     //Solve ILP to find best side size
     ilpResult = internal::findBestSideSize(chartData, alpha);

     //Quadrangulate
     internal::quadrangulate(
                 newSurface,
                 chartData,
                 ilpResult,
                 chartSmoothingIterations,
                 meshSmoothingIterations,
                 quadrangulatedNewSurface,
                 quadrangulatedNewSurfaceLabel);
}

}
