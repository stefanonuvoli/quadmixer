#include "quadbooleansteps.h"

#include "quadpatchtracer.h"
#include "quadutils.h"
#include "quadpreserved.h"
#include "quadilp.h"
#include "quadconvert.h"
#include "quadpatterns.h"
#include "quadquadmapping.h"
#include "quadlibiglbooleaninterface.h"

#include <vcg/complex/algorithms/polygonal_algorithms.h>
#include <vcg/complex/algorithms/mesh_to_matrix.h>


namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
void traceQuads(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        bool motorcycle)
{
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
        TriangleMeshType& triMesh,
        std::vector<int>& birthQuad)
{
    //Copy
    vcg::tri::Append<TriangleMeshType, PolyMeshType>::Mesh(triMesh, mesh);

    //Triangulate and get birth data
    birthQuad = splitQuadInTriangle(triMesh);
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
    vcg::tri::MeshToMatrix<PolyMesh>::GetTriMeshData(triMesh1, FA, VA);
    vcg::tri::MeshToMatrix<PolyMesh>::GetTriMeshData(triMesh2, FB, VB);

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
        std::vector<bool>& preservedQuad2)
{
    //Get preserved quads
    size_t nFirstFaces = triMesh1.face.size();

    computePreservedQuadForMesh(triMesh1, VA, FA, VR, FR, J, birthQuad1, 0, preservedQuad1);
    computePreservedQuadForMesh(triMesh2, VB, FB, VR, FR, J, birthQuad2, nFirstFaces, preservedQuad2);
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

std::vector<int> findBestSideSize(
        const ChartData& chartData,
        const double& alpha)
{
    //Solve ILP to find the best patches
    return solveChartSideILP(chartData, alpha);
}


template<class TriangleMeshType, class PolyMeshType>
void quadrangulate(
        TriangleMeshType& newSurface,
        const ChartData& chartData,
        const std::vector<int>& ilpResult,
        const int chartSmoothingIterations,
        const int meshSmoothingIterations,
        PolyMeshType& quadrangulatedNewSurface,
        std::vector<int>& quadrangulatedNewSurfaceLabel)
{
    std::vector<std::vector<size_t>> vertexSubsideMap(chartData.subSides.size());
    std::vector<int> cornerVertices(newSurface.vert.size(), -1);

    //Fill fixed vertices (subsides corners)
    for (const ChartSubSide& subside : chartData.subSides) {
        size_t vStart = subside.vertices[0];
        size_t vEnd = subside.vertices[subside.vertices.size() - 1];

        if (cornerVertices[vStart] == -1) {
            cornerVertices[vStart] = quadrangulatedNewSurface.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulatedNewSurface,
                        newSurface.vert.at(vStart).P());
        }

        if (cornerVertices[vEnd] == -1) {
            cornerVertices[vEnd] = quadrangulatedNewSurface.vert.size();
            vcg::tri::Allocator<PolyMeshType>::AddVertex(
                        quadrangulatedNewSurface,
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

                    newVertexId = quadrangulatedNewSurface.vert.size();
                    vcg::tri::Allocator<PolyMeshType>::AddVertex(
                                quadrangulatedNewSurface,
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

        //Input mesh
        Eigen::MatrixXd chartV;
        Eigen::MatrixXi chartF;
        vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(newSurface);
        vcg::tri::UpdateFlags<PolyMeshType>::VertexClearS(newSurface);
        for (const size_t& fId : chart.faces) {
            newSurface.face[fId].SetS();
            for (int k = 0; k < newSurface.face[fId].VN(); k++) {
                newSurface.face[fId].V(k)->SetS();
            }
        }
        std::vector<int> vMap, fMap;
        VCGToEigenSelected(newSurface, chartV, chartF, vMap, fMap, 3);

        assert(chartSides.size() >= 3 && chartSides.size() <= 6);

        //Input subdivisions
        Eigen::VectorXi l(chartSides.size());

        std::vector<double> chartSideLength(chartSides.size());
        std::vector<std::vector<size_t>> chartEigenSides(chartSides.size());

        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& chartSide = chartSides[i];

            size_t targetSize = 0;
            for (const size_t& subSideId : chartSides[i].subsides) {
                targetSize += ilpResult[subSideId];
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

        std::vector<std::vector<size_t>> patchEigenSides = getPatchSides(patchBorders, patchCorners, l);

        assert(chartSides.size() == patchCorners.size());
        assert(chartSides.size() == patchEigenSides.size());

        //Compute quadrangulation
        Eigen::MatrixXd uvMap;
        Eigen::MatrixXd quadrangulationV;
        Eigen::MatrixXi quadrangulationF;
        QuadBoolean::internal::computeQuadrangulation(chartV, chartF, patchV, patchF, chartEigenSides, chartSideLength, patchEigenSides, uvMap, quadrangulationV, quadrangulationF);

        assert(chartV.rows() == uvMap.rows());

        //Get polymesh
        PolyMeshType quadrangulatedChartMesh;
        eigenToVCG(quadrangulationV, quadrangulationF, quadrangulatedChartMesh, 4);

        //Smoothing
        vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulatedChartMesh);
        for (size_t vId : patchBorders) {
            quadrangulatedChartMesh.vert[vId].ClearS();
        }
        vcg::PolygonalAlgorithm<PolyMeshType>::LaplacianReproject(quadrangulatedChartMesh, chartSmoothingIterations, 0.5, true);

        std::vector<int> currentVertexMap(quadrangulatedChartMesh.vert.size(), -1);

        //Map subsides on the vertices of the current mesh (create if necessary)
        for (size_t i = 0; i < chartSides.size(); i++) {
            const ChartSide& side = chartSides[i];
            const std::vector<size_t>& patchSide = patchEigenSides[i];

            size_t currentPatchSideVertex = 0;

            for (size_t j = 0; j < side.subsides.size(); j++) {
                const size_t& subSideId = side.subsides[j];
                const bool& reversed = side.reversedSubside[j];

                //Create new vertices of the subsides
                if (vertexSubsideMap[subSideId].empty()) {
                    //Get fixed corners of the subside
                    const ChartSubSide& subside = chartData.subSides[subSideId];

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
                            size_t newVertexId = quadrangulatedNewSurface.vert.size();

                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulatedNewSurface, coord);

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

                        if (k > 0 && k < ilpResult[subSideId]) {
                            //Average
                            const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[patchSideVId].P();
                            quadrangulatedNewSurface.vert.at(existingVertexId).P() =
                                    (coord + quadrangulatedNewSurface.vert.at(existingVertexId).P())/2;
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
                size_t newId = quadrangulatedNewSurface.vert.size();

                const typename PolyMeshType::CoordType& coord = quadrangulatedChartMesh.vert[i].P();
                vcg::tri::Allocator<PolyMeshType>::AddVertex(quadrangulatedNewSurface, coord);

                currentVertexMap[i] = newId;
            }
        }

        //Set faces
        for (size_t i = 0; i < quadrangulatedChartMesh.face.size(); i++) {
            assert(quadrangulatedChartMesh.face[i].VN() == 4);

            size_t newFaceId = quadrangulatedNewSurface.face.size();

            vcg::tri::Allocator<PolyMeshType>::AddFaces(quadrangulatedNewSurface, 1);

            quadrangulatedNewSurface.face[newFaceId].Alloc(quadrangulatedChartMesh.face[i].VN());
            for (int j = 0; j < quadrangulatedChartMesh.face[i].VN(); j++) {
                int vId = currentVertexMap[vcg::tri::Index(quadrangulatedChartMesh, quadrangulatedChartMesh.face[i].V(j))];
                assert(vId >= 0);

                quadrangulatedNewSurface.face[newFaceId].V(j) = &quadrangulatedNewSurface.vert[vId];
            }
            quadrangulatedNewSurfaceLabel.push_back(chart.label);
        }
    }

    vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulatedNewSurface);
    for (const size_t& borderVertexId : finalMeshBorders) {
        quadrangulatedNewSurface.vert[borderVertexId].ClearS();
    }
    vcg::PolygonalAlgorithm<PolyMeshType>::LaplacianReproject(quadrangulatedNewSurface, meshSmoothingIterations, 0.5, true);

    vcg::tri::Clean<PolyMeshType>::MergeCloseVertex(quadrangulatedNewSurface, 0.00001);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(quadrangulatedNewSurface);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(quadrangulatedNewSurface);
}


}
}
