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
void LaplacianPos(PolyMeshType &poly_m,std::vector<typename PolyMeshType::CoordType> &AvVert)
{
    //cumulate step
    AvVert.clear();
    AvVert.resize(poly_m.vert.size(),typename PolyMeshType::CoordType(0,0,0));
    std::vector<typename PolyMeshType::ScalarType> AvSum(poly_m.vert.size(),0);
    for (size_t i=0;i<poly_m.face.size();i++)
        for (size_t j=0;j<(size_t)poly_m.face[i].VN();j++)
        {
            //get current vertex
            typename PolyMeshType::VertexType *currV=poly_m.face[i].V(j);
            //and its position
            typename PolyMeshType::CoordType currP=currV->P();
            //cumulate over other positions
            typename PolyMeshType::ScalarType W=vcg::PolyArea(poly_m.face[i]);
            //assert(W!=0);
            for (size_t k=0;k<(size_t)poly_m.face[i].VN();k++)
            {
                if (k==j) continue;
                int IndexV=vcg::tri::Index(poly_m,poly_m.face[i].V(k));
                AvVert[IndexV]+=currP*W;
                AvSum[IndexV]+=W;
            }
        }

    //average step
    for (size_t i=0;i<poly_m.vert.size();i++)
    {
        if (AvSum[i]==0)continue;
        AvVert[i]/=AvSum[i];
    }
}

template <class PolyMeshType>
void LaplacianGeodesic(
        PolyMeshType &poly_m,
        int nstep,
        const double maxDistance,
        const double minDumpS = 0.5)
{
    std::vector<typename PolyMeshType::VertexPointer> seedVec;
    for (int i = 0; i < poly_m.vert.size(); i++) {
        if (poly_m.vert[i].IsS()) {
            seedVec.push_back(&poly_m.vert[i]);
        }
    }
    vcg::tri::EuclideanDistance<PolyMeshType> ed;
    vcg::tri::UpdateTopology<PolyMeshType>::VertexFace(poly_m);
    vcg::tri::Geodesic<PolyMeshType>::Compute(poly_m,seedVec, ed);

    std::vector<double> DampS(poly_m.vert.size());
    for (int i = 0; i < poly_m.vert.size(); i++) {
        if (poly_m.vert[i].Q() < maxDistance) {
            DampS[i] = poly_m.vert[i].Q() / maxDistance;
            assert(DampS[i] >= 0 && DampS[i] <= 1);
            DampS[i] = minDumpS + DampS[i]*(1-minDumpS);
        }
        else {
            DampS[i] = std::numeric_limits<double>::max();
        }
    }

    for (int s=0;s<nstep;s++)
    {
        std::vector< typename PolyMesh::CoordType> AvVert;
        LaplacianPos(poly_m,AvVert);

        for (size_t i=0;i<poly_m.vert.size();i++)
        {
            if (DampS[i] > 1)
                continue;

            poly_m.vert[i].P()=poly_m.vert[i].P()*DampS[i]+
                    AvVert[i]*(1-DampS[i]);
        }
    }

}

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
    PolyMeshType tmpPoly;
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(tmpPoly, mesh);

    //Triangulate and get birth data
    birthQuad = splitQuadInTriangle(tmpPoly);

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

template<class TriangleMeshType>
std::vector<std::vector<size_t>> getIntersectionCurves(
        TriangleMeshType& triMesh1,
        TriangleMeshType& triMesh2,
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXi& FB,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J)
{
    std::vector<std::vector<size_t>> intersectionCurves;

    int nFirstFaces = triMesh1.face.size();

    std::set<int> vertexSet1;
    std::set<int> vertexSet2;
    std::unordered_map<int, int> vertexMap;


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
    std::set_intersection(vertexSet1.begin(), vertexSet1.end(), vertexSet2.begin(), vertexSet2.end(), std::inserter(vertexSet, vertexSet.begin()));

    //Create the next map
    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstFaces) {
            for (int j = 0; j < 3; j++) {
                if (vertexSet.find(FR(i, j)) != vertexSet.end() &&
                    vertexSet.find(FR(i,(j+1)%3)) != vertexSet.end() &&
                    vertexSet.find(FR(i,(j+2)%3)) == vertexSet.end())
                {
                    vertexMap.insert(
                                std::make_pair(
                                    FR(i, j),
                                    FR(i, (j+1)%3))
                                );
                }
            }
        }
    }



    while (!vertexSet.empty()) {
        std::vector<size_t> intersectionCurve;

        int vStart = *(vertexSet.begin());

        int vCurrent = vStart;

        bool done = false;
        do {
            vertexSet.erase(vCurrent);
            intersectionCurve.push_back(vCurrent);

            std::unordered_map<int, int>::iterator vNextIt = vertexMap.find(vCurrent);

            if (vNextIt == vertexMap.end()) {
                done = true;
            }
            else {
                vCurrent = vNextIt->second;

                if (vertexSet.find(vCurrent) == vertexSet.end()) {
                    done = true;
                }
            }
        } while (!done);

        assert(vCurrent == vStart);

        intersectionCurve.push_back(vCurrent);

        intersectionCurves.push_back(intersectionCurve);
    }

    return intersectionCurves;
}

template<class TriangleMeshType>
void smoothAlongIntersectionCurves(
        TriangleMeshType& boolean,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        const std::vector<std::vector<size_t>>& intersectionCurves,
        const int intersectionSmoothingInterations,
        const int avgNRing)
{
    typename TriangleMeshType::ScalarType maxDistance = averageEdgeLength(boolean) * avgNRing;

    vcg::tri::UpdateSelection<TriangleMeshType>::VertexClear(boolean);

    if (intersectionCurves.size() == 0)
        return;

    for (const std::vector<size_t>& intersectionCurve : intersectionCurves) {
        for (const size_t& vId : intersectionCurve) {
            boolean.vert[vId].SetS();
        }
    }

    LaplacianGeodesic(boolean, intersectionSmoothingInterations, maxDistance, 0.6);

    std::vector<int> vMap;
    std::vector<int> fMap;
    QuadBoolean::internal::VCGToEigen(boolean, VR, FR, vMap, fMap);
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
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(preservedSurface, mesh1, true);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(preservedSurface, mesh2, true);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(preservedSurface);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(preservedSurface);

    newFaceLabel.resize(preservedSurface.face.size(), -1);
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        newFaceLabel[i] = static_cast<int>(preservedSurface.face[i].Q());
    }
}


template<class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& triResult,
        const size_t& nFirstFaces,
        const std::vector<int>& birthQuad1,
        const std::vector<int>& birthQuad2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const Eigen::VectorXi& J,
        TriangleMeshType& newSurface)
{
    //Selected the new surface triangle faces
    std::vector<bool> isNewSurface(triResult.face.size(), false);

    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstFaces) {
            int firstMeshIndex = birthFace;
            if (!preservedQuad1[birthQuad1[firstMeshIndex]]) {
                isNewSurface[i] = true;
            }
        }
        //The birth face is in the second mesh
        else {
            int secondMeshIndex = birthFace - nFirstFaces;
            if (!preservedQuad2[birthQuad2[secondMeshIndex]]) {
                isNewSurface[i] = true;
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

template<class TriangleMeshType>
std::vector<int> getPatchDecomposition(
        TriangleMeshType& newSurface,
        std::vector<std::vector<size_t>>& partitions,
        std::vector<std::vector<size_t>>& corners,
        const bool initialRemeshing,
        const double edgeFactor)
{
    if (newSurface.face.size() <= 0)
        return std::vector<int>();

#ifdef USE_NEW_DECOMPOSER
    PatchAssembler<TriangleMeshType> patchAssembler(newSurface);
    typename PatchAssembler<TriangleMeshType>::Parameters parameters;
    parameters.InitialRemesh = initialRemeshing;
    parameters.EdgeSizeFactor = edgeFactor;
    patchAssembler.BatchProcess(partitions, corners, parameters);
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

std::vector<int> findBestSideSize(
        const ChartData& chartData,
        const double& alpha)
{
    if (chartData.charts.size() <= 0)
        return std::vector<int>();


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
    if (newSurface.face.size() <= 0)
        return;

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
        if (chartSides.size() < 3 || chartSides.size() > 6) {
            std::cout << "Chart " << cId << " with corners less than 3 or greater than 6!" << std::endl;
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
        igl::writeOBJ(std::string("res/") + std::to_string(cId) + std::string("_chart.obj"), chartV, chartF);
#endif

        //Compute quadrangulation
        Eigen::MatrixXd uvMapV;
        Eigen::MatrixXi uvMapF;
        Eigen::MatrixXd quadrangulationV;
        Eigen::MatrixXi quadrangulationF;
        QuadBoolean::internal::computeQuadrangulation(chartV, chartF, patchV, patchF, chartEigenSides, chartSideLength, patchEigenSides, uvMapV, uvMapF, quadrangulationV, quadrangulationF);

        assert(chartV.rows() == uvMapV.rows());

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

                        //If it is not a corner or if it is not on border
                        if (!subside.isOnBorder && k > 0 && k < ilpResult[subSideId]) {
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

    if (meshSmoothingIterations > 0) {
        vcg::tri::UpdateSelection<PolyMeshType>::VertexAll(quadrangulatedNewSurface);
        for (const size_t& borderVertexId : finalMeshBorders) {
            quadrangulatedNewSurface.vert[borderVertexId].ClearS();
        }
        vcg::PolygonalAlgorithm<PolyMeshType>::LaplacianReproject(quadrangulatedNewSurface, meshSmoothingIterations, 0.5, true);
    }
}


template<class PolyMeshType>
void getResult(
        PolyMeshType& preservedSurface,
        PolyMeshType& quadrangulatedNewSurface,
        PolyMeshType& result,
        const int resultSmoothingIterations,
        const int avgNRing)
{
    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(quadrangulatedNewSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(quadrangulatedNewSurface);
    for (typename PolyMeshType::FaceIterator fIt = quadrangulatedNewSurface.face.begin(); fIt != quadrangulatedNewSurface.face.end(); fIt++) {
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

    vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceBorderFromFF(preservedSurface);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearV(preservedSurface);
    for (typename PolyMeshType::FaceIterator fIt = preservedSurface.face.begin(); fIt != preservedSurface.face.end(); fIt++) {
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

    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, preservedSurface);
    vcg::tri::Append<PolyMeshType, PolyMeshType>::Mesh(result, quadrangulatedNewSurface);

    //NOT NEEDED?
    //vcg::tri::Clean<PolyMeshType>::MergeCloseVertex(quadrangulatedNewSurface, 0.00001);
    vcg::tri::Clean<PolyMeshType>::RemoveDuplicateVertex(result);
    vcg::tri::Clean<PolyMeshType>::RemoveUnreferencedVertex(result);

    if (resultSmoothingIterations > 0) {
        typename PolyMeshType::ScalarType maxDistance = averageEdgeLength(result) * avgNRing;

        vcg::tri::UpdateSelection<PolyMeshType>::VertexClear(result);
        for (typename PolyMeshType::VertexIterator vIt = result.vert.begin(); vIt != result.vert.end(); vIt++) {
            if (vIt->IsV()) {
                vIt->SetS();
            }
        }

        LaplacianGeodesic(result, resultSmoothingIterations, maxDistance, 0.6);
    }


}

}
}
