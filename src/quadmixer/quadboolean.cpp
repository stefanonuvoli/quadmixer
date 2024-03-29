/* Copyright(C) 2019


The authors of

QuadMixer: Layout Preserving Blending of Quadrilateral Meshes
SIGGRAPH Asia 2019


All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
****************************************************************************/

#include "quadboolean.h"

#include <quadretopology/quadretopology.h>

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
        SourceInfo& sourceInfo)
{
     std::vector<int> quadTracerLabel1;
     std::vector<int> quadTracerLabel2;

     TriangleMeshType trimesh1, trimesh2, boolean;
     Eigen::MatrixXd VA, VB, VR;
     Eigen::MatrixXi FA, FB, FR;
     Eigen::VectorXi J;

     std::vector<std::pair<size_t, size_t>> birthTriangle;
     std::vector<int> birthFace1;
     std::vector<int> birthFace2;

     std::vector<size_t> intersectionVertices;
     std::vector<size_t> smoothedVertices;

     std::vector<std::pair<bool, bool>> isPreserved1;
     std::vector<std::pair<bool, bool>> isPreserved2;
     std::vector<bool> isNewSurface;

     std::unordered_set<int> affectedPatches1;
     std::unordered_set<int> affectedPatches2;

     std::vector<int> preservedFaceLabel1;
     std::vector<int> preservedFaceLabel2;

     PolyMeshType preservedSurface;
     std::vector<int> preservedSurfaceLabel;
     std::vector<std::pair<int, int>> preservedBirthVertexInfo;
     std::vector<std::pair<int, int>> preservedBirthFaceInfo;

     TriangleMeshType newSurface;
     std::vector<int> newSurfaceLabel;
     std::vector<std::vector<size_t>> newSurfacePartitions;
     std::vector<std::vector<size_t>> newSurfaceCorners;

     QuadRetopology::ChartData chartData;

     std::vector<int> ilpResults;
     double gap;

     PolyMeshType quadrangulation;
     std::vector<std::vector<size_t>> quadrangulationPartitions;
     std::vector<std::vector<size_t>> quadrangulationCorners;
     std::vector<int> quadrangulationLabel;

     std::vector<int> resultPreservedVertexMap;
     std::vector<int> resultPreservedFaceMap;

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
                 parameters.maxBB,
                 smoothedVertices);

     //Get preserved and new surface
     QuadBoolean::internal::getSurfaces(
             mesh1,
             mesh2,
             boolean,
             birthTriangle,
             birthFace1,
             birthFace2,
             intersectionVertices,
             smoothedVertices,
             parameters.motorcycle,
             parameters.patchRetraction,
             parameters.patchRetractionNRing,
             parameters.minRectangleArea,
             parameters.minPatchArea,
             parameters.mergeQuads,
             parameters.deleteSmall,
             parameters.deleteNonConnected,
             parameters.maxBB,
             parameters.preservePolygons1,
             parameters.preservePolygons2,
             preservedFaceLabel1,
             preservedFaceLabel2,
             isPreserved1,
             isPreserved2,
             isNewSurface,
             preservedSurfaceLabel,
             preservedBirthVertexInfo,
             preservedBirthFaceInfo,
             preservedSurface,
             newSurface);

     //Make ILP feasible
     internal::makeILPFeasible(preservedSurface, newSurface, parameters.polychordSolver, parameters.splitSolver);

     //Get patch decomposition of the new surface
     newSurfaceLabel = QuadRetopology::patchDecomposition(newSurface, preservedSurface, newSurfacePartitions, newSurfaceCorners, true, 1.0, true, false, true);

     //Get chart data
     chartData = QuadRetopology::computeChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

     //Select the subsides to fix
     ilpResults.resize(chartData.subsides.size(), ILP_FIND_SUBDIVISION);
     std::vector<size_t> fixedPositionSubsides;
     for (size_t subsideId = 0; subsideId < chartData.subsides.size(); ++subsideId) {
         QuadRetopology::ChartSubside& subside = chartData.subsides[subsideId];
         if (subside.isOnBorder) {
             fixedPositionSubsides.push_back(subsideId);
             ilpResults[subsideId] = subside.size;
         }
     }

     //Get chart length
     std::vector<double> chartEdgeLength = QuadRetopology::computeChartEdgeLength(chartData, 5, ilpResults, 0.7);

     std::vector<float> callbackTimeLimit = { 2.00, 3.000, 5.0, 7.0, 8.0, 10.0, 15.0, 20.0 };
     std::vector<float> callbackGapLimit = { 0.001, 0.005, 0.01, 0.05, 0.10, 0.15, 0.20, 0.300 };

     //Solve ILP to find best side size
     QuadRetopology::findSubdivisions(
             chartData,
             chartEdgeLength,
             parameters.ilpMethod,                   //method
             parameters.alpha,                       //alpha
             true,                                   //isometry
             true,                                   //regularityForQuadrilaterals
             false,                                  //regularityForNonQuadrilaterals
             0.9,                                    //regularityNonQuadrilateralWeight
             false,                                  //alignSingularities
             0.1,                                    //alignSingularitiesWeight
             0,                                      //repeatLosingConstraintsIterations
             false,                                  //repeatLosingConstraintsQuads
             false,                                  //repeatLosingConstraintsNonQuads
             false,                                  //repeatLosingConstraintsAlign
             false,                                  //feasibilityFix
             true,                                   //hardParityConstraint
             21,                                     //timeLimit
             0.0,                                    //gapLimit
             callbackTimeLimit,                      //callbackTimeLimit
             callbackGapLimit,                       //callbackGapLimit
             0.3,                                    //minimumGap
             gap,
             ilpResults);

     //Quadrangulate,
     QuadRetopology::quadrangulate(
                 newSurface,
                 chartData,
                 fixedPositionSubsides,
                 ilpResults,
                 parameters.chartSmoothingIterations,
                 parameters.quadrangulationFixedSmoothingIterations,
                 parameters.quadrangulationNonFixedSmoothingIterations,
                 true,
                 quadrangulation,
                 quadrangulationLabel,
                 quadrangulationPartitions,
                 quadrangulationCorners);

     //Get the result
     QuadRetopology::computeResult(
                 preservedSurface, quadrangulation,
                 result, boolean,
                 false,
                 parameters.resultSmoothingIterations,
                 parameters.resultSmoothingNRing,
                 parameters.resultSmoothingLaplacianIterations,
                 parameters.resultSmoothingLaplacianNRing,
                 resultPreservedVertexMap, resultPreservedFaceMap);


     //Old and new vertices
     sourceInfo.oldVerticesMap.clear();
     sourceInfo.newVertices.clear();
     for (size_t i = 0; i < result.vert.size(); i++) {
         if (!result.vert[i].IsD()) {
             if (result.vert[i].Q() >= 0) {
                 int preservedVertexId = resultPreservedVertexMap[i];
                 int originMesh = preservedBirthVertexInfo[preservedVertexId].first;
                 int currentVertexId = preservedBirthVertexInfo[preservedVertexId].second;
                 if (originMesh >= 0) {
                     sourceInfo.oldVerticesMap.insert(std::make_pair(i, OriginEntity(1, static_cast<size_t>(currentVertexId))));
                 }
                 else {
                     sourceInfo.newVertices.push_back(i);
                 }
             }
         }
     }

     //Old and new faces
     sourceInfo.oldFacesMap.clear();
     sourceInfo.newFaces.clear();
     for (size_t i = 0; i < result.face.size(); i++) {
         if (!result.face[i].IsD()) {
             if (result.face[i].Q() >= 0) {
                 int preservedFaceId = resultPreservedFaceMap[i];
                 int originMesh = preservedBirthFaceInfo[preservedFaceId].first;
                 int currentFaceId = preservedBirthFaceInfo[preservedFaceId].second;
                 if (originMesh >= 0) {
                     sourceInfo.oldFacesMap.insert(std::make_pair(i, OriginEntity(1, static_cast<size_t>(currentFaceId))));
                 }
                 else {
                     sourceInfo.newFaces.push_back(i);
                 }
             }
         }
     }



}

}
