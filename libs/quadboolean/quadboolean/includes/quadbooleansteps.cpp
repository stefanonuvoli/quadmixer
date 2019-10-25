#include "quadbooleansteps.h"

#include "quadpatchtracer.h"
#include "quadutils.h"
#include "quadpreserved.h"
#include "quadconvert.h"
#include "quadpatterns.h"
#include "quadquadmapping.h"
#include "quadlibiglbooleaninterface.h"
#include "quadfeasibility.h"
#include "patch_assembler.h"

#include <map>

#include <vcg/complex/algorithms/polygonal_algorithms.h>

#ifndef NDEBUG
#include <igl/writeOBJ.h>
#endif

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
void tracePatchLayout(
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
void triangulateMesh(
        PolyMeshType& mesh,
        TriangleMeshType& trimesh,
        std::vector<int>& birthFace)
{
    //Copy
    PolyMeshType tmpPoly;
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpPoly, mesh);

    //Triangulate and get birth data
    birthFace = splitFacesInTriangles(tmpPoly);

    vcg::tri::Append<TriangleMeshType, PolyMeshType>::Mesh(trimesh, tmpPoly);
}

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
        Eigen::VectorXi& J)
{
    //Get trimeshes
    vcg::tri::MeshToMatrix<TriangleMeshType>::GetTriMeshData(trimesh1, FA, VA);
    vcg::tri::MeshToMatrix<TriangleMeshType>::GetTriMeshData(trimesh2, FB, VB);

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

inline std::vector<std::pair<size_t, size_t>> getBirthTriangles(
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J)
{
    std::vector<std::pair<size_t, size_t>> birthTriangle(FR.rows());

    int nFirstFaces = FA.rows();

    for (int i = 0; i < J.rows(); i++) {
        assert(J[i] >= 0);

        int birthFace = J[i];
        int mesh = 1;

        //If the birth face is in the second mesh
        if (birthFace >= nFirstFaces) {
            birthFace -= nFirstFaces;
            mesh = 2;
        }

        birthTriangle[i] = std::make_pair(mesh, birthFace);
    }

    return birthTriangle;
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
        const std::vector<size_t>& intersectionVertices,
        const int intersectionSmoothingInterations,
        const double NRing,
        const double maxBB,
        std::vector<size_t>& smoothedVertices)
{
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(boolean);

    typename TriangleMeshType::ScalarType maxDistance = std::min(averageEdgeLength(boolean) * NRing, boolean.bbox.Diag()*maxBB);

    vcg::tri::UpdateSelection<TriangleMeshType>::VertexClear(boolean);

    if (intersectionVertices.size() == 0)
        return;

    for (const size_t& vId : intersectionVertices) {
        boolean.vert[vId].SetS();
    }

    LaplacianGeodesic(boolean, intersectionSmoothingInterations, maxDistance, 0.8, smoothedVertices);
}

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
        std::vector<std::pair<bool, bool>>& isPreserved1,
        std::vector<std::pair<bool, bool>>& isPreserved2,
        std::vector<bool>& isNewSurface,
        std::vector<int>& preservedSurfaceLabel,
        std::unordered_map<size_t, size_t>& preservedFacesMap,
        std::unordered_map<size_t, size_t>& preservedVerticesMap,
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface)
{
    //Trace quads following singularities
    QuadBoolean::internal::tracePatchLayout(mesh1, faceLabel1, motorcycle);
    QuadBoolean::internal::tracePatchLayout(mesh2, faceLabel2, motorcycle);

    //Find preserved quads
    QuadBoolean::internal::findPreservedFaces(
                mesh1, mesh2,
                boolean,
                intersectionVertices,
                smoothedVertices,
                patchRetraction,
                patchRetractionNRing,
                maxBB,
                preservePolygons1,
                preservePolygons2,
                birthTriangle,
                birthFace1, birthFace2,
                isPreserved1, isPreserved2,
                isNewSurface);

    //Find affected patches
    std::unordered_set<int> affectedPatches1;
    std::unordered_set<int> affectedPatches2;

    QuadBoolean::internal::findAffectedPatches(mesh1, isPreserved1, faceLabel1, affectedPatches1);
    QuadBoolean::internal::findAffectedPatches(mesh2, isPreserved2, faceLabel2, affectedPatches2);

    //Maximum rectangles in the patches
    faceLabel1 = QuadBoolean::internal::splitPatchesInMaximumRectangles(mesh1, affectedPatches1, faceLabel1, isPreserved1, minRectangleArea, true);
    faceLabel2 = QuadBoolean::internal::splitPatchesInMaximumRectangles(mesh2, affectedPatches2, faceLabel2, isPreserved2, minRectangleArea, true);


    //Merge rectangular patches
    if (mergeQuads) {
        QuadBoolean::internal::mergePatches(mesh1, affectedPatches1, faceLabel1, isPreserved1);
        QuadBoolean::internal::mergePatches(mesh2, affectedPatches2, faceLabel2, isPreserved2);
    }

    if (deleteSmall) {
        //Delete small patches
        QuadBoolean::internal::deleteSmallPatches(mesh1, affectedPatches1, minPatchArea, faceLabel1, isPreserved1);
        QuadBoolean::internal::deleteSmallPatches(mesh2, affectedPatches2, minPatchArea, faceLabel2, isPreserved2);
    }

    if (deleteNonConnected) {
        //Delete non-connected patches
        QuadBoolean::internal::deleteNonConnectedPatches(mesh1, faceLabel1, isPreserved1);
        QuadBoolean::internal::deleteNonConnectedPatches(mesh2, faceLabel2, isPreserved2);
    }

    //Get mesh of the preserved surface
    QuadBoolean::internal::getPreservedSurfaceMesh(
                mesh1, mesh2,
                isPreserved1, isPreserved2,
                faceLabel1, faceLabel2,
                preservedSurface, preservedSurfaceLabel,
                preservedFacesMap, preservedVerticesMap);

    //New mesh (to be decomposed in patch)
    QuadBoolean::internal::getNewSurfaceMesh(
                boolean,
                birthTriangle,
                birthFace1,
                birthFace2,
                isPreserved1,
                isPreserved2,
                isNewSurface,
                newSurface);
}

template<class PolyMeshType, class TriangleMeshType>
bool makeILPFeasible(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver)
{
    FeasibilityResult result = QuadBoolean::internal::solveFeasibility(
                preservedSurface,
                newSurface,
                polychordSolver,
                splitSolver);

    if (result == AlreadyOk) {
            std::cout << "Feasibility was already okay!" << std::endl;
            return true;
    }
    else if (result == SolvedQuadOnly) {
        std::cout << "Feasibility solved with only quads!" << std::endl;
        return true;
    }
    else if (result == SolvedQuadDominant) {
        std::cout << "Feasibility solved with quad dominant!" << std::endl;
        if (polychordSolver) {
            std::cout << "Warning: it has been impossible to solve with polychord splits!" << std::endl;
        }
        return true;
    }
    else if (result == NonSolved) {
        if (polychordSolver || splitSolver) {
            std::cout << "Error: feasibility not solved!" << std::endl;
        }
        else {
            std::cout << "Feasibility non solved" << std::endl;
        }
        return false;
    }
    else if (result == NonConsistant) {
        std::cout << "Error: the model was not consistant. The border vertices were different!" << std::endl;
        return false;
    }
    else {
        std::cout << "Error: undefined return value by feasibility solver!" << std::endl;
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

    std::vector<std::vector<std::vector<std::pair<size_t,size_t>>>> sides;
    PatchAssembler<TriangleMeshType, PolyMeshType> patchAssembler(newSurface, preservedSurface);
    typename PatchAssembler<TriangleMeshType, PolyMeshType>::Parameters parameters;
    parameters.InitialRemesh = initialRemeshing;
    parameters.EdgeSizeFactor = edgeFactor;
    parameters.FinalSmooth = finalSmoothing;
    parameters.SplitAllConcave = splitConcaves;
    parameters.Reproject = reproject;
    patchAssembler.BatchProcess(partitions, corners, sides, parameters);


    std::vector<int> newSurfaceLabel(newSurface.face.size(), -1);
    for (size_t pId = 0; pId < partitions.size(); pId++) {
        for (const size_t& fId : partitions[pId]) {
            assert(newSurfaceLabel[fId] == -1);
            newSurfaceLabel[fId] = static_cast<int>(pId);
        }
    }

    return newSurfaceLabel;
}


template<class TriangleMeshType>
std::vector<int> findSubdivisions(
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
    const double timeLimit = 5.0;
#endif
    const double gapLimit = 0.3;

    double gap;
    ILPStatus status;

    //Solve ILP to find the best patches
    std::vector<int> ilpResult = solveILP(newSurface, chartData, alpha, beta, method, true, timeLimit, gap, status);

    if (status == ILPStatus::SOLUTIONFOUND && gap < gapLimit) {
        std::cout << "Solution found! Gap: " << gap << std::endl;
    }
    else {
        if (status == ILPStatus::INFEASIBLE) {
            std::cout << "Error! Model was infeasible or time limit exceeded!" << std::endl;
        }
        else {
            ilpResult = solveILP(newSurface, chartData, alpha, beta, ILPMethod::ABS, true, timeLimit*2, gap, status);

            if (status == ILPStatus::SOLUTIONFOUND) {
                std::cout << "Solution found (ABS)! Gap: " << gap << std::endl;
            }
            else {
                ilpResult = solveILP(newSurface, chartData, alpha, beta, ILPMethod::ABS, false, timeLimit*4, gap, status);
                std::cout << "Solution found? (ABS without regularity)! Gap: " << gap << std::endl;
            }
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
    if (ilpResult.size() == 0)
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
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(quadrangulation);

    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(newSurface);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalized(newSurface);
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(newSurface);

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
}


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
        const double resultSmoothingLaplacianNRing,
        const std::unordered_map<size_t, size_t>& preservedFacesMap,
        const std::unordered_map<size_t, size_t>& preservedVerticesMap,
        SourceInfo& sourceInfo)
{
    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(quadrangulatedNewSurface);
    for (size_t i = 0; i < quadrangulatedNewSurface.face.size(); i++) {
        if (!quadrangulatedNewSurface.face[i].IsD()) {
            quadrangulatedNewSurface.face[i].Q() = -1;
        }
    }

    std::vector<bool> smoothingVerticesInPreserved(preservedSurface.vert.size(), false);

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearV(preservedSurface);
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        if (!preservedSurface.face[i].IsD()) {
            preservedSurface.face[i].Q() = i;

            for (int k = 0; k < preservedSurface.face[i].VN(); k++) {
                if (vcg::face::IsBorder(preservedSurface.face[i], k)) {
                    smoothingVerticesInPreserved[vcg::tri::Index(preservedSurface, preservedSurface.face[i].V0(k))] = true;
                    smoothingVerticesInPreserved[vcg::tri::Index(preservedSurface, preservedSurface.face[i].V1(k))] = true;
                }
            }
        }
    }

    for (size_t i = 0; i < preservedSurface.vert.size(); i++) {
        if (!preservedSurface.vert[i].IsD()) {
            preservedSurface.vert[i].Q() = i;
        }
    }
    for (size_t i = 0; i < quadrangulatedNewSurface.vert.size(); i++) {
        if (!quadrangulatedNewSurface.vert[i].IsD()) {
            quadrangulatedNewSurface.vert[i].Q() = -1;
        }
    }

    //Create result
    PolyMeshType tmpMesh;
    result.Clear();
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, preservedSurface);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, quadrangulatedNewSurface);

    vcg::tri::Clean<PolyMeshType>::MergeCloseVertex(result, 0.0000001);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(result);

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(result);

    int numNonManifoldFaces = vcg::tri::Clean<PolyMeshType>::RemoveNonManifoldFace(result);
    if (numNonManifoldFaces > 0) {
        std::cout << "Removed " << numNonManifoldFaces << " non-manifold faces." << std::endl;
    }

    int numHoles = vcg::tri::Hole<PolyMeshType>::template EarCuttingFill<vcg::tri::TrivialEar<PolyMeshType>>(result, result.face.size(), false);
    if (numHoles > 0) {
        std::cout << "Removed " << numHoles << " holes." << std::endl;
    }

    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateFace(result);
    vcg::tri::Allocator<PolyMeshType>::CompactEveryVector(result);

    //    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpMesh, result);
    //    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, tmpMesh);

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    OrientFaces<PolyMeshType>::AutoOrientFaces(result);
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(result);


    //Get smoothing vertices
    std::vector<size_t> smoothingVertices;
    for (size_t i = 0; i < result.vert.size(); i++) {
        if (!result.vert[i].IsD()) {
            if (result.vert[i].Q() == -1 || smoothingVerticesInPreserved[static_cast<size_t>(result.vert[i].Q())]) {
                smoothingVertices.push_back(i);
            }
        }
    }

    //Old and new faces
    sourceInfo.oldFacesMap.clear();
    sourceInfo.newFaces.clear();
    for (size_t i = 0; i < result.face.size(); i++) {
        if (!result.face[i].IsD()) {
            if (result.face[i].Q() >= 0) {
                int currentFaceId = -1;
                std::unordered_map<size_t, size_t>::const_iterator it = preservedFacesMap.find(static_cast<size_t>(result.face[i].Q()));
                if (it != preservedFacesMap.end())
                    currentFaceId = static_cast<int>(it->second);

                if (currentFaceId < 0) {
                    sourceInfo.oldFacesMap.insert(std::make_pair(i, OriginEntity(0, 0)));
                }
                else if (currentFaceId < mesh1.face.size()) {
                    sourceInfo.oldFacesMap.insert(std::make_pair(i, OriginEntity(1, static_cast<size_t>(currentFaceId))));
                }
                else {
                    sourceInfo.oldFacesMap.insert(std::make_pair(i, OriginEntity(2, currentFaceId - mesh1.face.size())));
                }
            }
            else {
                sourceInfo.newFaces.push_back(i);
            }
        }
    }

    //Old and new vertices
    sourceInfo.oldVerticesMap.clear();
    sourceInfo.newVertices.clear();
    for (size_t i = 0; i < result.vert.size(); i++) {
        if (!result.vert[i].IsD()) {
            if (result.vert[i].Q() >= 0) {
                int currentVertId = -1;
                std::unordered_map<size_t, size_t>::const_iterator it = preservedVerticesMap.find(static_cast<size_t>(result.vert[i].Q()));
                if (it != preservedVerticesMap.end())
                    currentVertId = static_cast<int>(it->second);

                if (currentVertId < 0) { //It shouldn't happen
                    sourceInfo.oldVerticesMap.insert(std::make_pair(i, OriginEntity(0, 0)));
                }
                else if (currentVertId < mesh1.face.size()) {
                    sourceInfo.oldVerticesMap.insert(std::make_pair(i, OriginEntity(1, static_cast<size_t>(currentVertId))));
                }
                else {
                    sourceInfo.oldVerticesMap.insert(std::make_pair(i, OriginEntity(2, currentVertId - mesh1.vert.size())));
                }
            }
            else {
                sourceInfo.newVertices.push_back(i);
            }
        }
    }

#ifndef NDEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(result, "res/resultbeforereproj.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(targetBoolean);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerVertexNormalized(targetBoolean);
    vcg::tri::UpdateBounding<TriangleMeshType>::Box(targetBoolean);

    if (result.face.size() == 0)
        return;

    vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(result);
    vcg::tri::UpdateSelection<PolyMeshType>::FaceClear(result);
    for (const size_t& vId : smoothingVertices) {
        result.vert[vId].SetS();
    }

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
        typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingNRing;

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

    typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * resultSmoothingLaplacianNRing;

    LaplacianGeodesic(result, resultSmoothingLaplacianIterations, maxDistance, 0.8);

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(result);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(result);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(result);
}



}
}
