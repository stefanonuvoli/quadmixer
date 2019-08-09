#include "quadbooleansteps.h"

#include "quadpatchtracer.h"
#include "quadutils.h"
#include "quadpreserved.h"
#include "quadconvert.h"
#include "quadpatterns.h"
#include "quadquadmapping.h"
#include "quadlibiglbooleaninterface.h"
#include "even_pairing.h"

#include <map>

#include <vcg/complex/algorithms/polygonal_algorithms.h>

#define USE_NEW_DECOMPOSER

#ifdef USE_NEW_DECOMPOSER
#include "patch_assembler.h"
#else
#include "patch_decomposer.h"
#endif

#ifndef NDEBUG
#include <igl/writeOBJ.h>
#endif

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
void traceQuads(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        bool motorcycle)
{
    if (mesh.face.size() == 0)
        return;

    //Compute tracer
    QuadMeshTracer<PolyMeshType> tracer(mesh);
    tracer.updatePolymeshAttributes();
    tracer.MotorCycle = motorcycle;
    tracer.TracePartitions();

    //Setting resulting labels
    faceLabel = tracer.FacePatch;
}

template<class PolyMeshType, class TriangleMeshType>
void triangulateQuadMesh(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        TriangleMeshType& triMesh,
        std::vector<int>& birthQuad)
{
    //Copy
    PolyMeshType tmpPoly;
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpPoly, mesh);

    //Triangulate and get birth data
    if (isQuadMesh) {
        birthQuad = splitQuadInTriangle(tmpPoly);
    }
    else {
        birthQuad.resize(mesh.face.size(), -1);
        for (int i = 0; i < mesh.face.size(); i++)
            birthQuad[i] = i;
    }

    vcg::tri::Append<TriangleMeshType, PolyMeshType>::Mesh(triMesh, tmpPoly);
}

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
        Eigen::VectorXi& J)
{
    //Get trimeshes
    vcg::tri::MeshToMatrix<TriangleMeshType>::GetTriMeshData(triMesh1, FA, VA);
    vcg::tri::MeshToMatrix<TriangleMeshType>::GetTriMeshData(triMesh2, FB, VB);

    //Boolean operation on trimeshes
    if (operation == QuadBoolean::Operation::UNION)
        trimeshUnion(VA,FA,VB,FB,VR,FR,J);
    else if (operation == QuadBoolean::Operation::INTERSECTION)
        trimeshIntersection(VA,FA,VB,FB,VR,FR,J);
    else if (operation == QuadBoolean::Operation::DIFFERENCE)
        trimeshDifference(VA,FA,VB,FB,VR,FR,J);
    else
        return;

    //Result on VCG
    eigenToVCG(VR, FR, result);
}

inline std::vector<size_t> getIntersectionVertices(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FB,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J)
{
    std::vector<size_t> intersectionVertices;

    int nFirstFaces = FA.rows();

    std::set<int> vertexSet1;
    std::set<int> vertexSet2;
    std::vector<std::vector<size_t>> vertexMap(VR.rows(), std::vector<size_t>());


    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstFaces) {
            bool isNew = false;

            for (int j = 0; j < 3; j++) {
                if (VA(FA(birthFace, j)) != VR(FR(i, j))) {
                    isNew = true;
                }
            }

            if (isNew) {
                for (int j = 0; j < 3; j++) {
                    vertexSet1.insert(FR(i, j));
                }
            }
        }
        else {
            birthFace -= nFirstFaces;
            bool isNew = false;

            for (int j = 0; j < 3; j++) {
                if (VB(FB(birthFace, j)) != VR(FR(i, j))) {
                    isNew = true;
                }
            }

            if (isNew) {
                for (int j = 0; j < 3; j++) {
                    vertexSet2.insert(FR(i, j));
                }
            }
        }
    }

    std::set<int> vertexSet;

    std::set_intersection(vertexSet1.begin(), vertexSet1.end(), vertexSet2.begin(), vertexSet2.end(), std::back_inserter(intersectionVertices));

    return intersectionVertices;
}

template<class TriangleMeshType>
void smoothAlongIntersectionVertices(
        TriangleMeshType& boolean,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        const std::vector<size_t>& intersectionVertices,
        const int intersectionSmoothingInterations,
        const double avgNRing,
        const double maxBB)
{
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(boolean);

    typename TriangleMeshType::ScalarType maxDistance = std::min(averageEdgeLength(boolean) * avgNRing, boolean.bbox.Diag()*maxBB);

    vcg::tri::UpdateSelection<TriangleMeshType>::VertexClear(boolean);

    if (intersectionVertices.size() == 0)
        return;

    for (const size_t& vId : intersectionVertices) {
        boolean.vert[vId].SetS();
    }

    LaplacianGeodesic(boolean, intersectionSmoothingInterations, maxDistance, 0.8);

    std::vector<int> vMap;
    std::vector<int> fMap;
    QuadBoolean::internal::VCGToEigen(boolean, VR, FR, vMap, fMap);
}


template<class PolyMeshType, class TriangleMeshType>
void findPreservedQuads(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        TriangleMeshType& boolean,
        const bool isQuadMesh1,
        const bool isQuadMesh2,
        const std::vector<size_t>& intersectionVertices,
        const bool patchRetraction,
        const double patchRetractionNRing,
        const double maxBB,
        std::vector<bool>& preservedQuad1,
        std::vector<bool>& preservedQuad2)
{

    typename TriangleMeshType::ScalarType maxDistance = 0;

    for (size_t i = 0; i < boolean.vert.size(); i++) {
        boolean.vert[i].Q() = 0;
    }

    if (patchRetraction) {
        std::vector<typename TriangleMeshType::VertexPointer> seedVec;
        for (const size_t& vId : intersectionVertices) {
            seedVec.push_back(&boolean.vert[vId]);
        }
        vcg::tri::EuclideanDistance<TriangleMeshType> ed;
        vcg::tri::UpdateTopology<TriangleMeshType>::VertexFace(boolean);
        vcg::tri::Geodesic<TriangleMeshType>::Compute(boolean, seedVec, ed);

        maxDistance = std::min(averageEdgeLength(boolean) * patchRetractionNRing, boolean.bbox.Diag()*maxBB);
    }

    computePreservedQuadForMesh(mesh1, boolean, isQuadMesh1, maxDistance, preservedQuad1);
    computePreservedQuadForMesh(mesh2, boolean, isQuadMesh2, maxDistance, preservedQuad2);
}

template<class PolyMeshType>
void findAffectedPatches(
        PolyMeshType& mesh,
        const std::vector<bool>& preservedQuad,
        const std::vector<int>& faceLabel,
        std::unordered_set<int>& affectedPatches)
{
    //Find affected patches
    for (int i = 0; i < mesh.face.size(); i++) {
        if (!preservedQuad[i]) {
            affectedPatches.insert(faceLabel[i]);
        }
    }
}

template<class PolyMeshType>
void getPreservedSurfaceMesh(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMeshType& preservedSurface,
        std::vector<int>& newFaceLabel)
{
    int maxLabel1 = 0;
    for (const int& l : faceLabel1) {
        maxLabel1 = std::max(maxLabel1, l);
    }

    //Select the face which are remaining
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(mesh1);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(mesh2);
    for (size_t i = 0; i < mesh1.face.size(); i++) {
        if (preservedQuad1[i]) {
            mesh1.face[i].SetS();
            mesh1.face[i].Q() = faceLabel1[i];
        }
    }
    for (size_t i = 0; i < mesh2.face.size(); i++) {
        if (preservedQuad2[i]) {
            mesh2.face[i].SetS();
            mesh2.face[i].Q() = maxLabel1 + faceLabel2[i];
        }
    }

    //Create result
    PolyMeshType tmpMesh;
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpMesh, mesh1, true);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpMesh, mesh2, true);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(tmpMesh);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateFace(tmpMesh);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(tmpMesh);

    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(preservedSurface, tmpMesh);

    newFaceLabel.resize(preservedSurface.face.size(), -1);
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        newFaceLabel[i] = static_cast<int>(preservedSurface.face[i].Q());
    }

    vcg::tri::Allocator<PolyMeshType>::CompactEveryVector(preservedSurface);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(preservedSurface);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(preservedSurface);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(preservedSurface);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalizedPerFace(preservedSurface);
}


template<class PolyMeshType, class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& triResult,
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        TriangleMeshType& newSurface)
{
    vcg::tri::UpdateTopology<TriangleMeshType>::FaceFace(triResult);

    //Selected the new surface triangle faces
    std::vector<bool> isNewSurface(triResult.face.size(), true);

    std::set<std::set<typename PolyMeshType::CoordType>> quadSet;

    for (size_t i = 0; i < mesh1.face.size(); i++) {
        if (preservedQuad1[i]) {
            std::set<typename PolyMeshType::CoordType> coordSet;
            for (int j = 0; j < mesh1.face[i].VN(); j++)
                coordSet.insert(mesh1.face[i].V(j)->P());

            quadSet.insert(coordSet);
        }
    }
    for (size_t i = 0; i < mesh2.face.size(); i++) {
        if (preservedQuad2[i]) {
            std::set<typename PolyMeshType::CoordType> coordSet;
            for (int j = 0; j < mesh2.face[i].VN(); j++)
                coordSet.insert(mesh2.face[i].V(j)->P());

            quadSet.insert(coordSet);
        }
    }

    for (size_t i = 0; i < triResult.face.size(); i++) {
        if (isNewSurface[i]) {
            std::set<typename TriangleMeshType::CoordType> coordSet;

            for (int k = 0; k < triResult.face[i].VN(); k++) {
                coordSet.insert(triResult.face[i].V(k)->P());
            }

            for (int k = 0; k < triResult.face[i].VN() && isNewSurface[i]; k++) {
                typename TriangleMeshType::FacePointer fp = triResult.face[i].FFp(k);

                if (fp == &triResult.face[i])
                    continue;

                std::set<typename PolyMeshType::CoordType> coordSetComplete = coordSet;

                int otherFaceEdge = triResult.face[i].FFi(k);
                int oppositeVert = (otherFaceEdge + 2) % 3;

                coordSetComplete.insert(fp->V(oppositeVert)->P());

                typename std::set<std::set<typename PolyMeshType::CoordType>>::iterator findIt = quadSet.find(coordSetComplete);
                if (findIt != quadSet.end()) {
                    quadSet.erase(findIt);

                    size_t adjFaceId = vcg::tri::Index(triResult, fp);

                    isNewSurface[adjFaceId] = false;
                    isNewSurface[i] = false;
                }
            }
        }
    }

    vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearS(triResult);
    for (size_t i = 0; i < triResult.face.size(); i++) {
        if (isNewSurface[i]) {
            triResult.face[i].SetS();
        }
    }
    vcg::tri::Append<TriangleMeshType, TriangleMeshType>::Mesh(newSurface, triResult, true);
    vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateVertex(newSurface);
    vcg::tri::Clean<TriangleMeshType>::RemoveUnreferencedVertex(newSurface);
}

template<class PolyMeshType, class TriangleMeshType>
bool makeILPFeasible(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface)
{
    internal::EvenPairing<TriangleMeshType, PolyMeshType> evenPairing(newSurface, preservedSurface);

    typename internal::EvenPairing<TriangleMeshType, PolyMeshType>::ResultType result;

    result = evenPairing.SolvePairing(true);

    if (result == internal::EvenPairing<TriangleMeshType, PolyMeshType>::Solved) {
        std::cout << "Even pairing solved!" << std::endl;
        return true;
    }
    if (result == internal::EvenPairing<TriangleMeshType, PolyMeshType>::AlreadyOk) {
        std::cout << "Even pairing was already okay!" << std::endl;
        return true;
    }
    else if (result == internal::EvenPairing<TriangleMeshType, PolyMeshType>::NonConsistent) {
        std::cout << "Error: even pairing was not consistent!" << std::endl;
        return false;
    }
    else if (result == internal::EvenPairing<TriangleMeshType, PolyMeshType>::NonSolved) {
        std::cout << "Error: even pairing was not solved!" << std::endl;
        return false;
    }
    else {
        std::cout << "Error: undefined return value by even pairing!" << std::endl;
        return false;
    }
}

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
        const bool finalSmoothing)
{
    if (newSurface.face.size() <= 0)
        return std::vector<int>();

#ifdef USE_NEW_DECOMPOSER
    std::vector<std::vector<std::vector<std::pair<size_t,size_t>>>> sides;
    PatchAssembler<TriangleMeshType, PolyMeshType> patchAssembler(newSurface, preservedSurface);
    typename PatchAssembler<TriangleMeshType, PolyMeshType>::Parameters parameters;
    parameters.InitialRemesh = initialRemeshing;
    parameters.EdgeSizeFactor = edgeFactor;
    parameters.FinalSmooth = finalSmoothing;
    parameters.SplitAllConcave = splitConcaves;
    parameters.Reproject = reproject;
    patchAssembler.BatchProcess(partitions, corners, sides, parameters);
#else
    PatchDecomposer<TriangleMeshType> decomposer(newSurface);
    typename PatchDecomposer<TriangleMeshType>::Parameters parameters;
    decomposer.SetParam(parameters);
    decomposer.BatchProcess(partitions, corners);
    vcg::tri::Allocator<TriangleMesh>::CompactEveryVector(newSurface);
#endif

    std::vector<int> newSurfaceLabel(newSurface.face.size(), -1);
    for (size_t pId = 0; pId < partitions.size(); pId++) {
        for (const size_t& fId : partitions[pId]) {
            assert(newSurfaceLabel[fId] == -1);
            newSurfaceLabel[fId] = static_cast<int>(pId);
        }
    }

    return newSurfaceLabel;
}

//template<class PolyMeshType, class TriangleMeshType>
//std::vector<int> findBestSideSize(
//        PolyMeshType& mesh1,
//        PolyMeshType& mesh2,
//        TriangleMeshType& trimesh1,
//        TriangleMeshType& trimesh2,
//        std::vector<int>& birthQuad1,
//        std::vector<int>& birthQuad2,
//        std::vector<int>& faceLabel1,
//        std::vector<int>& faceLabel2,
//        TriangleMeshType& newSurface,
//        std::vector<int>& newSurfaceLabel,
//        std::vector<std::vector<size_t>>& newSurfaceCorners,
//        std::vector<bool>& preservedQuad1,
//        std::vector<bool>& preservedQuad2,
//        ChartData& chartData,
//        const double alpha,
//        const double beta,
//        const ILPMethod& method)
//{
//    if (chartData.charts.size() <= 0)
//        return std::vector<int>();

//    //Solve ILP to find the best patches
//    std::vector<int> ilpResult = solveChartSideILP(newSurface, chartData, alpha, beta, method);

//    bool feasible = true;
////    do {
//        std::vector<size_t> changedBorders;

//        for (size_t i = 0; i < chartData.charts.size(); i++) {
//            const Chart& chart = chartData.charts[i];

//            if (chart.faces.size() > 0) {
//                int sizeSum = 0;
//                for (const size_t& subSideId : chart.chartSubSides) {
//                    sizeSum += ilpResult[subSideId];
//                }

//                if (sizeSum % 2 == 1) {
//                    std::cout << "Error! Not even, chart: " << i << std::endl;
//                    for (const size_t& subSideId : chart.chartSubSides) {
//                        ilpResult[subSideId] = -1;
//                    }
//                    feasible = false;
//                }
//            }
//        }

//        if (!feasible) {
//            size_t startLabel = chartData.charts.size();
//            size_t currentLabel = chartData.charts.size();

//            QuadLayoutData<PolyMeshType> quadLayoutData1 = QuadBoolean::internal::getQuadLayoutData(mesh1, isQuadMesh(mesh1), faceLabel1);
//            QuadLayoutData<PolyMeshType> quadLayoutData2 = QuadBoolean::internal::getQuadLayoutData(mesh2, isQuadMesh(mesh2), faceLabel2);

//            std::vector<std::vector<typename PolyMeshType::CoordType>> newSurfaceCornerPoints;

//            std::vector<bool> isQuadPatchRemoved1(quadLayoutData1.quadPatches.size(), false);
//            std::vector<bool> isQuadPatchRemoved2(quadLayoutData2.quadPatches.size(), false);

//            std::vector<bool> visited(chartData.charts.size(), false);
//            for (const size_t& nonFeasibleChartId : changedBorders) {
////                Chart& nonFeasibleChart = chartData.charts[nonFeasibleChartId];
////                for (size_t id : nonFeasibleChart.faces) {
////                    newSurface.face[id].C() = vcg::Color4b(0,0,255,255);
////                }

//                std::set<typename TriangleMeshType::CoordType> borderPoints;

////                for (size_t i = 0; i < chart.chartSides.size(); i++) {
////                    const ChartSide& chartSide = chart.chartSides[i];

////                    for (std::vector<size_t>::const_iterator it = chartSide.vertices.begin(); it !=  chartSide.vertices.end(); it++) {
////                        borderPoints.insert(newSurface.vert[*it].P());
////                    }
////                }

//                std::stack<size_t> stack;
//                stack.push(nonFeasibleChartId);

//                while (!stack.empty()) {
//                    size_t currentChartId = stack.top();
//                    stack.pop();

//                    if (!visited[currentChartId]) {
//                        Chart& currentChart = chartData.charts[currentChartId];

//                        for (size_t i = 0; i < currentChart.chartSubSides.size(); i++) {
//                            const ChartSubSide& chartSubSide = chartData.subSides[currentChart.chartSubSides[i]];

//                            if (chartSubSide.isOnBorder) {
//                                for (size_t j = 0; j < chartSubSide.vertices.size(); j++) {
//                                    borderPoints.insert(newSurface.vert[chartSubSide.vertices[j]].P());
//                                }
//                            }
//                        }

//                        for (size_t& adjChart : currentChart.adjacentCharts) {
//                            stack.push(adjChart);
//                        }

//                        visited[currentChartId] = true;
//                    }

//                }

//                internal::fixNonFeasibleChart(
//                            mesh1, trimesh1, birthQuad1, faceLabel1, quadLayoutData1,
//                            newSurface, newSurfaceLabel, newSurfaceCornerPoints,
//                            preservedQuad1, borderPoints, currentLabel, isQuadPatchRemoved1);

//                internal::fixNonFeasibleChart(
//                            mesh2, trimesh2, birthQuad2, faceLabel2, quadLayoutData2,
//                            newSurface, newSurfaceLabel, newSurfaceCornerPoints,
//                            preservedQuad2, borderPoints, currentLabel, isQuadPatchRemoved2);

//                //Clean
//                vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateVertex(newSurface);
//                vcg::tri::Clean<TriangleMeshType>::RemoveUnreferencedVertex(newSurface);
//                vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateFace(newSurface);
//            }

//            for (int i = 0; i < currentLabel - startLabel; i++) {
//                assert(newSurfaceCornerPoints[i].size() == 4);
//                std::vector<size_t> newCornersId(4, -1);

//                for (int j = 0; j < 4; j++) {
//                    newCornersId[j] = -1;
//                    typename TriangleMeshType::CoordType& coord = newSurfaceCornerPoints[i][j];
//                    for (size_t vId = 0; vId < newSurface.vert.size(); vId++) {
//                        if (!newSurface.vert[vId].IsD() && newSurface.vert[vId].P() == coord) {
//                            newCornersId[j] = vId;
//                        }
//                    }
//                    assert(newCornersId[j] >= 0);
//                }

//                newSurfaceCorners.push_back(newCornersId);
//            }

//#ifndef NDEBUG
//    vcg::tri::io::ExporterOBJ<TriangleMeshType>::Save(newSurface, "res/newSurfaceILPRedo.obj", vcg::tri::io::Mask::IOM_FACECOLOR | vcg::tri::io::Mask::IOM_VERTCOLOR);
//#endif

//            //Get chart data
//            chartData = internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

//            //Solve ILP
//            solveChartSideILP(newSurface, chartData, alpha, beta, method);
//        }
////    }
////    while (!feasible);

//    return ilpResult;
//}

//template<class PolyMeshType, class TriangleMeshType>
//std::vector<int> findBestSideSize(
//        PolyMeshType& mesh1,
//        PolyMeshType& mesh2,
//        TriangleMeshType& trimesh1,
//        TriangleMeshType& trimesh2,
//        std::vector<int>& birthQuad1,
//        std::vector<int>& birthQuad2,
//        std::vector<int>& faceLabel1,
//        std::vector<int>& faceLabel2,
//        TriangleMeshType& newSurface,
//        std::vector<int>& newSurfaceLabel,
//        std::vector<std::vector<size_t>>& newSurfaceCorners,
//        std::vector<bool>& preservedQuad1,
//        std::vector<bool>& preservedQuad2,
//        ChartData& chartData,
//        const double alpha,
//        const double beta,
//        const ILPMethod& method)
//{
//    if (chartData.charts.size() <= 0)
//        return std::vector<int>();

//    //Solve ILP to find the best patches
//    const double timeLimit = 5.0;
//    const double gapLimit = 30.0;

//    double gap;
//    ILPStatus status;

//    std::vector<int> ilpResult = solveChartSideILPFixedBorders(newSurface, chartData, alpha, beta, method, true, timeLimit, gap, status);

//    if (status == ILPStatus::SOLUTIONFOUND && gap < gapLimit) {
//        std::cout << "Solution found! Gap: " << gap << std::endl;
//        return ilpResult;
//    }
//    else if (status == ILPStatus::SOLUTIONWRONG || gap >= gapLimit) {
//        std::vector<int> ilpResult = solveChartSideILPFixedBorders(newSurface, chartData, alpha, beta, ILPMethod::ABS, false, timeLimit*4, gap, status);

//        std::cout << "Solution found (avoid regularity term)! Gap: " << gap << std::endl;
//        return ilpResult;
//    }
//    else {
//        assert(status == ILPStatus::INFEASIBLE);

//        std::vector<int> ilpResult = solveChartSideILPFreeBorders(newSurface, chartData, alpha, beta, ILPMethod::ABS, false, timeLimit*4, gap, status);
//        assert(status == ILPStatus::SOLUTIONFOUND);

//        std::vector<size_t> nonFeasibleCharts;

//        size_t startLabel = chartData.charts.size();
//        size_t currentLabel = chartData.charts.size();

//        QuadLayoutData<PolyMeshType> quadLayoutData1 = QuadBoolean::internal::getQuadLayoutData(mesh1, isQuadMesh(mesh1), faceLabel1);
//        QuadLayoutData<PolyMeshType> quadLayoutData2 = QuadBoolean::internal::getQuadLayoutData(mesh2, isQuadMesh(mesh2), faceLabel2);

//        std::vector<bool> quadPatchToAdd1(quadLayoutData1.quadPatches.size(), false);
//        std::vector<bool> quadPatchToAdd2(quadLayoutData2.quadPatches.size(), false);

//        for (size_t i = 0; i < chartData.charts.size(); i++) {
//            const Chart& chart = chartData.charts[i];

//            if (chart.faces.size() > 0) {
//                std::set<typename TriangleMeshType::CoordType> targetBorderPoints;
//                std::vector<std::vector<typename PolyMeshType::CoordType>> newSurfaceCornerPoints;

//                int sizeSum = 0;
//                for (const size_t& subSideId : chart.chartSubSides) {
//                    const ChartSubSide& subSide = chartData.subSides[subSideId];
//                    sizeSum += ilpResult[subSideId];

//                    if (subSide.isOnBorder && subSide.size != ilpResult[subSideId]) {
//                        std::cout << "Border different in chart: " << i << " / subside: " << subSideId << std::endl;
//                        for (size_t j = 0; j < subSide.vertices.size(); j++) {
//                            targetBorderPoints.insert(newSurface.vert[subSide.vertices[j]].P());
//                        }
//#ifndef NDEBUG
//                        for (size_t id : chart.faces) {
//                            newSurface.face[id].C() = vcg::Color4b(0,0,255,255);
//                        }
//#endif
//                    }
//                }
//                assert(sizeSum % 2 == 0);

//                if(targetBorderPoints.size() > 0) {
//                    internal::fixNonFeasibleChart(
//                                mesh1, trimesh1, birthQuad1, faceLabel1, quadLayoutData1,
//                                newSurface, newSurfaceLabel, newSurfaceCornerPoints,
//                                preservedQuad1, targetBorderPoints, currentLabel, quadPatchToAdd1);

//                    internal::fixNonFeasibleChart(
//                                mesh2, trimesh2, birthQuad2, faceLabel2, quadLayoutData2,
//                                newSurface, newSurfaceLabel, newSurfaceCornerPoints,
//                                preservedQuad2, targetBorderPoints, currentLabel, quadPatchToAdd2);

//                    //Clean
//                    vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateVertex(newSurface);
//                    vcg::tri::Clean<TriangleMeshType>::RemoveUnreferencedVertex(newSurface);
//                    vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateFace(newSurface);

//                    for (int i = 0; i < currentLabel - startLabel; i++) {
//                        assert(newSurfaceCornerPoints[i].size() == 4);
//                        std::vector<size_t> newCornersId(4, -1);

//                        for (int j = 0; j < 4; j++) {
//                            newCornersId[j] = -1;
//                            typename TriangleMeshType::CoordType& coord = newSurfaceCornerPoints[i][j];
//                            for (size_t vId = 0; vId < newSurface.vert.size(); vId++) {
//                                if (!newSurface.vert[vId].IsD() && newSurface.vert[vId].P() == coord) {
//                                    newCornersId[j] = vId;
//                                }
//                            }
//                            assert(newCornersId[j] >= 0);
//                        }

//                        newSurfaceCorners.push_back(newCornersId);
//                    }
//                }
//            }

//#ifndef NDEBUG
//            vcg::tri::io::ExporterOBJ<TriangleMeshType>::Save(newSurface, "res/newSurfaceILPRedo.obj", vcg::tri::io::Mask::IOM_FACECOLOR | vcg::tri::io::Mask::IOM_VERTCOLOR);
//#endif

//            //Get chart data
//            chartData = internal::getPatchDecompositionChartData(newSurface, newSurfaceLabel, newSurfaceCorners);

//            //Solve ILP
//            return findBestSideSize(
//                        mesh1,
//                        mesh2,
//                        trimesh1,
//                        trimesh2,
//                        birthQuad1,
//                        birthQuad2,
//                        faceLabel1,
//                        faceLabel2,
//                        newSurface,
//                        newSurfaceLabel,
//                        newSurfaceCorners,
//                        preservedQuad1,
//                        preservedQuad2,
//                        chartData,
//                        alpha,
//                        beta,
//                        method);
//        }
//    }
//}



template<class TriangleMeshType>
std::vector<int> findBestSideSize(
        TriangleMeshType& newSurface,
        ChartData& chartData,
        const double alpha,
        const double beta,
        const ILPMethod& method)
{
    if (chartData.charts.size() <= 0)
        return std::vector<int>();

#ifndef NDEBUG
    const double timeLimit = 30.0;
#else
    const double timeLimit = 15.0;
#endif
    const double gapLimit = 0.3;

    double gap;
    ILPStatus status;

    //Solve ILP to find the best patches
    std::vector<int> ilpResult = solveChartSideILPFixedBorders(newSurface, chartData, alpha, beta, method, true, timeLimit, gap, status);

    if (status == ILPStatus::SOLUTIONFOUND && gap < gapLimit) {
        std::cout << "Solution found! Gap: " << gap << std::endl;
    }
    else {
        if (status == ILPStatus::INFEASIBLE) {
            std::cout << "Error! Model was infeasible or time limit exceeded!" << std::endl;
        }

        ilpResult = solveChartSideILPFixedBorders(newSurface, chartData, alpha, beta, ILPMethod::ABS, true, timeLimit*2, gap, status);

        if (status == ILPStatus::SOLUTIONFOUND) {
            std::cout << "Solution found (ABS)! Gap: " << gap << std::endl;
        }
        else {
            ilpResult = solveChartSideILPFixedBorders(newSurface, chartData, alpha, beta, ILPMethod::ABS, false, timeLimit*4, gap, status);
            std::cout << "Solution found? (ABS without regularity)! Gap: " << gap << std::endl;
        }
    }

    return ilpResult;
}


template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        const ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int quadrangulationSmoothingIterations,
        PolyMeshType& quadrangulation,
        std::vector<int>& quadrangulatedNewSurfaceLabel)
{
    if (newSurface.face.size() <= 0)
        return;

    std::vector<std::vector<size_t>> vertexSubsideMap(chartData.subSides.size());
    std::vector<int> cornerVertices(newSurface.vert.size(), -1);

    //Fill fixed vertices (subsides corners)
    for (const ChartSubSide& subside : chartData.subSides) {
        size_t vStart = subside.vertices[0];
        size_t vEnd = subside.vertices[subside.vertices.size() - 1];

        if (cornerVertices[vStart] == -1) {
            cornerVertices[vStart] = quadrangulation.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulation,
                        newSurface.vert.at(vStart).P());
        }

        if (cornerVertices[vEnd] == -1) {
            cornerVertices[vEnd] = quadrangulation.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulation,
                        newSurface.vert.at(vEnd).P());
        }
    }

    //Fill subside map for fixed borders
    std::set<size_t> finalMeshBorders;
    for (size_t i = 0; i < chartData.subSides.size(); i++) {
        const ChartSubSide& subside = chartData.subSides[i];
        if (subside.isOnBorder) {
            for (size_t k = 0; k < subside.vertices.size(); k++) {
                const size_t& vId = subside.vertices.at(k);

                size_t newVertexId;

                if (cornerVertices[vId] == -1) {
                    assert(k > 0 && k < subside.vertices.size() - 1);

                    newVertexId = quadrangulation.vert.size();
                    vcg::tri::Allocator<PolyMeshType>::AddVertex(
                                quadrangulation,
                                newSurface.vert.at(vId).P());
                }
                else {
                    newVertexId = cornerVertices[vId];
                    assert(newVertexId >= 0);
                }

                finalMeshBorders.insert(newVertexId);

                vertexSubsideMap[i].push_back(newVertexId);
            }
        }
    }


    //For each chart
    for (size_t cId = 0; cId < chartData.charts.size(); cId++) {
        const Chart& chart = chartData.charts[cId];

        if (chart.faces.size() == 0)
            continue;

        const std::vector<ChartSide>& chartSides = chart.chartSides;
        if (chartSides.size() < 3 || chartSides.size() > 6) {
            std::cout << "Chart " << cId << " with corners less than 3 or greater than 6!" << std::endl;
            continue;
        }

        bool ilpSolvedForAll = true;
        for (size_t sId : chart.chartSubSides) {
            if (ilpResult[sId] < 0)
                ilpSolvedForAll = false;
        }

        if (!ilpSolvedForAll) {
            std::cout << "Chart " << cId << " not computed. ILP was not solved." << std::endl;
            continue;
        }

        //Input mesh
        Eigen::MatrixXd chartV;
        Eigen::MatrixXi chartF;
        vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearS(newSurface);
        vcg::tri::UpdateFlags<TriangleMeshType>::VertexClearS(newSurface);
        for (const size_t& fId : chart.faces) {
            newSurface.face[fId].SetS();
            for (int k = 0; k < newSurface.face[fId].VN(); k++) {
                newSurface.face[fId].V(k)->SetS();
            }
        }
        std::vector<int> vMap, fMap;
        VCGToEigen(newSurface, chartV, chartF, vMap, fMap, true, 3);

        //Input subdivisions
        Eigen::VectorXi l(chartSides.size());

        std::vector<double> chartSideLength(chartSides.size());
        std::vector<std::vector<size_t>> chartEigenSides(chartSides.size());

        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& chartSide = chartSides[i];

            size_t targetSize = 0;
            for (const size_t& subSideId : chartSides[i].subsides) {
                targetSize += ilpResult[subSideId];

                if (ilpResult[subSideId] < 0) {
                    std::cout << "Warning: ILP not valid" << std::endl;
                    return;
                }
            }

            l(static_cast<int>(i)) = targetSize;

            chartSideLength[i] = chartSide.length;

            chartEigenSides[i].resize(chartSide.vertices.size());

            for (size_t j = 0; j < chartSide.vertices.size(); j++) {
                size_t vId = chartSide.vertices[j];
                assert(vMap[vId] >= 0);
                chartEigenSides[i][j] = vMap[vId];
            }
        }

        //Pattern quadrangulation
        Eigen::MatrixXd patchV;
        Eigen::MatrixXi patchF;
        std::vector<size_t> patchBorders;
        std::vector<size_t> patchCorners;
        QuadBoolean::internal::computePattern(l, patchV, patchF, patchBorders, patchCorners);    


#ifndef NDEBUG
        igl::writeOBJ(std::string("res/") + std::to_string(cId) + std::string("_patch.obj"), patchV, patchF);
#endif

        std::vector<std::vector<size_t>> patchEigenSides = getPatchSides(patchV, patchF, patchBorders, patchCorners, l);

        assert(chartSides.size() == patchCorners.size());
        assert(chartSides.size() == patchEigenSides.size());

#ifndef NDEBUG
        if (patchV.rows() == 3)
            igl::writeOBJ(std::string("res/") + std::to_string(cId) + std::string("_chart.obj"), chartV, chartF);
#endif

        //Compute quadrangulation
        Eigen::MatrixXd uvMapV;
        Eigen::MatrixXi uvMapF;
        Eigen::MatrixXd quadrangulationV;
        Eigen::MatrixXi quadrangulationF;
        QuadBoolean::internal::computeQuadrangulation(chartV, chartF, patchV, patchF, chartEigenSides, chartSideLength, patchEigenSides, uvMapV, uvMapF, quadrangulationV, quadrangulationF);

#ifndef NDEBUG
        Eigen::MatrixXd uvMesh(uvMapV.rows(), 3);
        for (int i = 0; i < uvMapV.rows(); i++) {
            uvMesh(i, 0) = uvMapV(i, 0);
            uvMesh(i, 1) = uvMapV(i, 1);
            uvMesh(i, 2) = 0;
        }

        std::string uvFile = std::string("res/") + std::to_string(cId) + std::string("_uv.obj");
        igl::writeOBJ(uvFile, uvMesh, uvMapF);
#endif
        assert(chartV.rows() == uvMapV.rows());

        //Get polymesh
        PolyMeshType quadrangulatedChartMesh;
        eigenToVCG(quadrangulationV, quadrangulationF, quadrangulatedChartMesh, 4);

#ifndef NDEBUG
        igl::writeOBJ(std::string("res/") + std::to_string(cId) + std::string("_quadrangulation.obj"), quadrangulationV, quadrangulationF);
#endif

        //Smoothing
        if (chartSmoothingIterations > 0) {
            vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulatedChartMesh);
            for (size_t vId : patchBorders) {
                quadrangulatedChartMesh.vert[vId].ClearS();
            }
            vcg::PolygonalAlgorithm<PolyMeshType>::LaplacianReproject(quadrangulatedChartMesh, chartSmoothingIterations, 0.5, true);
        }

        std::vector<int> currentVertexMap(quadrangulatedChartMesh.vert.size(), -1);

        //Map subsides on the vertices of the current mesh (create if necessary)
        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& side = chartSides[i];
            const std::vector<size_t>& patchSide = patchEigenSides[i];

            size_t currentPatchSideVertex = 0;

            for (size_t j = 0; j < side.subsides.size(); j++) {
                const size_t& subSideId = side.subsides[j];
                const bool& reversed = side.reversedSubside[j];
                const ChartSubSide& subside = chartData.subSides[subSideId];

                //Create new vertices of the subsides
                if (vertexSubsideMap[subSideId].empty()) {
                    assert(!subside.isOnBorder);

                    //Get fixed corners of the subside
                    size_t vStart = subside.vertices[0];
                    size_t vEnd = subside.vertices[subside.vertices.size() - 1];
                    assert(cornerVertices[vStart] >= 0 && cornerVertices[vEnd] >= 0);

                    currentVertexMap[patchSide[currentPatchSideVertex]] = cornerVertices[vStart];
                    currentVertexMap[patchSide[currentPatchSideVertex + ilpResult[subSideId]]] = cornerVertices[vEnd];

                    for (int k = 0; k <= ilpResult[subSideId]; k++) {
                        size_t patchSideVId = patchSide[currentPatchSideVertex];

                        if (currentVertexMap[patchSideVId] == -1) {
                            assert(k > 0 && k < ilpResult[subSideId]);

                            //Add new vertex
                            size_t newVertexId = quadrangulation.vert.size();

                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulation, coord);

                            currentVertexMap[patchSideVId] = newVertexId;

                            vertexSubsideMap[subSideId].push_back(newVertexId);
                        }
                        else {
                            //Use the existing vertex
                            int existingVertexId = currentVertexMap[patchSideVId];
                            assert(existingVertexId >= 0);
                            vertexSubsideMap[subSideId].push_back(existingVertexId);
                        }

                        currentPatchSideVertex++;
                    }

                    if (reversed) {
                        std::reverse(vertexSubsideMap[subSideId].begin(), vertexSubsideMap[subSideId].end());
                    }
                }
                //Set the existing vertices
                else {
                    assert(vertexSubsideMap[subSideId].size() == ilpResult[subSideId] + 1);

                    for (int k = 0; k <= ilpResult[subSideId]; k++) {
                        int patchSideVId = patchSide[currentPatchSideVertex];

                        size_t subSideVertexIndex = reversed ? ilpResult[subSideId] - k : k;

                        currentVertexMap[patchSideVId] = vertexSubsideMap[subSideId][subSideVertexIndex];

                        size_t existingVertexId = currentVertexMap[patchSideVId];

                        //If it is not a corner or if it is not on border
                        if (!subside.isOnBorder && k > 0 && k < ilpResult[subSideId]) {
                            //Average
                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            quadrangulation.vert.at(existingVertexId).P() =
                                    (coord + quadrangulation.vert.at(existingVertexId).P())/2;
                        }

                        currentPatchSideVertex++;
                    }
                }

                currentPatchSideVertex--;
            }

            assert(currentPatchSideVertex+1 == patchSide.size());
        }

        //Internal vertices
        for (size_t i = 0; i < quadrangulatedChartMesh.vert.size(); i++) {
            if (currentVertexMap[i] == -1) {
                size_t newId = quadrangulation.vert.size();

                const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[i].P();
                vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulation, coord);

                currentVertexMap[i] = newId;
            }
        }

        //Set faces
        for (size_t i = 0; i < quadrangulatedChartMesh.face.size(); i++) {
            assert(quadrangulatedChartMesh.face[i].VN() == 4);

            size_t newFaceId = quadrangulation.face.size();

            vcg::tri::Allocator<PolyMeshType>::AddFaces(quadrangulation, 1);

            quadrangulation.face[newFaceId].Alloc(quadrangulatedChartMesh.face[i].VN());
            for (int j = 0; j < quadrangulatedChartMesh.face[i].VN(); j++) {
                int vId = currentVertexMap[vcg::tri::Index(quadrangulatedChartMesh, quadrangulatedChartMesh.face[i].V(j))];
                assert(vId >= 0);

                quadrangulation.face[newFaceId].V(j) = &quadrangulation.vert[vId];
            }
            quadrangulatedNewSurfaceLabel.push_back(chart.label);
        }
    }

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulation);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);
    OrientFaces<PolyMeshType>::AutoOrientFaces(quadrangulation);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);

    vcg::GridStaticPtr<typename TriangleMeshType::FaceType,typename TriangleMeshType::FaceType::ScalarType> Grid;
    Grid.Set(newSurface.face.begin(),newSurface.face.end());

    //Reproject
    vcg::tri::UpdateBounding<PolyMeshType>::Box(quadrangulation);
    typename TriangleMeshType::ScalarType maxD=quadrangulation.bbox.Diag();
    typename TriangleMeshType::ScalarType minD=0;

    for (size_t i=0;i<quadrangulation.vert.size();i++)
    {
        typename TriangleMeshType::CoordType closestPT;
        typename TriangleMeshType::FaceType *f=
                vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                    newSurface,
                    Grid,
                    quadrangulation.vert[i].P(),
                    maxD,minD,
                    closestPT);

        quadrangulation.vert[i].P()=closestPT;
    }

    if (quadrangulationSmoothingIterations > 0) {
        vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulation);
        for (const size_t& borderVertexId : finalMeshBorders) {
            quadrangulation.vert[borderVertexId].ClearS();
        }
        vcg::PolygonalAlgorithm<PolyMeshType>::template LaplacianReproject<TriangleMeshType>(quadrangulation, newSurface, quadrangulationSmoothingIterations, 0.7, 0.7, true);
    }

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(quadrangulation);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(quadrangulation);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(quadrangulation);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalizedPerFace(quadrangulation);
}


template<class PolyMeshType, class TriangleMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        TriangleMeshType& targetBoolean,
        const int resultSmoothingIterations,
        const double resultSmoothingAvgNRing,
        const int resultSmoothingLaplacianIterations,
        const double resultSmoothingLaplacianAvgNRing,
        std::vector<size_t>& preservedFacesIds,
        std::vector<size_t>& newFacesIds)
{
    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(quadrangulatedNewSurface);
    for (typename PolyMeshType::FaceIterator fIt = quadrangulatedNewSurface.face.begin(); fIt != quadrangulatedNewSurface.face.end(); fIt++) {
        for (int k = 0; k < fIt->VN(); k++) {
            fIt->V(k)->SetV();
        }
    }

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(preservedSurface);
    for (typename PolyMeshType::FaceIterator fIt = preservedSurface.face.begin(); fIt != preservedSurface.face.end(); fIt++) {
        fIt->SetV();
        bool borderFace = false;
        for (int k = 0; k < fIt->VN(); k++) {
            if (vcg::face::IsBorder(*fIt, k)) {
                borderFace = true;
            }
        }
        if (borderFace) {
            for (int k = 0; k < fIt->VN(); k++) {
                fIt->V(k)->SetV();
            }
        }
    }

    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(result);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, preservedSurface);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, quadrangulatedNewSurface);

    std::vector<size_t> smoothingVertices;
    for (size_t i = 0; i < result.vert.size(); i++) {
        if (result.vert[i].IsV() && !result.vert[i].IsD()) {
            smoothingVertices.push_back(i);
        }
    }

    for (size_t i = 0; i < result.face.size(); i++) {
        if (result.face[i].IsV() && !result.face[i].IsD()) {
            preservedFacesIds.push_back(i);
        }
        else {
            newFacesIds.push_back(i);
        }
    }

    vcg::tri::Clean<PolyMeshType>::MergeCloseVertex(result, 0.0000001);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(result);

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    OrientFaces<PolyMeshType>::AutoOrientFaces(result);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);

#ifndef NDEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(result, "res/resultbeforereproj.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(targetBoolean);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalized(targetBoolean);
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(targetBoolean);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalizedPerFace(targetBoolean);

    if (result.face.size() == 0)
        return;

    vcg::GridStaticPtr<typename TriangleMeshType::FaceType,typename TriangleMeshType::FaceType::ScalarType> Grid;
    Grid.Set(targetBoolean.face.begin(),targetBoolean.face.end());

    //Reproject
    vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
    typename TriangleMeshType::ScalarType maxD=result.bbox.Diag();
    typename TriangleMeshType::ScalarType minD=0;

    for (size_t i=0;i<result.vert.size();i++)
    {
        typename TriangleMeshType::CoordType closestPT;
        typename TriangleMeshType::FaceType *f=
                vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                    targetBoolean,
                    Grid,
                    result.vert[i].P(),
                    maxD,minD,
                    closestPT);

        result.vert[i].P()=closestPT;
    }

    for (int it = 0; it < resultSmoothingIterations; it++) {
        typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingAvgNRing;

        //Smoothing
        vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(result);
        for (size_t& vId : smoothingVertices) {
            result.vert[vId].SetS();
        }

        LaplacianGeodesic(result, 1, maxDistance, 0.7);

        //Reproject
        vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
        typename TriangleMeshType::ScalarType maxD=result.bbox.Diag();
        typename TriangleMeshType::ScalarType minD=0;

        for (size_t i=0;i<result.vert.size();i++)
        {
            typename TriangleMeshType::CoordType closestPT;
            typename TriangleMeshType::FaceType *f=
                    vcg::tri::GetClosestFaceBase<TriangleMeshType>(
                        targetBoolean,
                        Grid,
                        result.vert[i].P(),
                        maxD,minD,
                        closestPT);

            result.vert[i].P()=closestPT;
        }
    }

    typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingLaplacianAvgNRing;

    vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(result);
    for (size_t& vId : smoothingVertices) {
        result.vert[vId].SetS();
    }

    LaplacianGeodesic(result, resultSmoothingLaplacianIterations, maxDistance, 0.8);


    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(result);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalizedPerFace(result);
}



}
}
