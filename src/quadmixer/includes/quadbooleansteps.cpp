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

#include "quadbooleansteps.h"

#include "quadpatchtracer.h"
#include "quadpreserved.h"
#include "quadlibiglbooleaninterface.h"
#include "quadfeasibility.h"

#include <quadretopology/includes/qr_convert.h>

#include <map>

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/mesh_to_matrix.h>

#include <vcg/complex/algorithms/polygonal_algorithms.h>

#ifdef SAVE_MESHES_FOR_DEBUG
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
    birthFace = QuadRetopology::internal::splitFacesInTriangles(tmpPoly);

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
    QuadRetopology::internal::eigenToVCG(VR, FR, result);
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

    typename TriangleMeshType::ScalarType maxDistance = std::min(QuadRetopology::internal::averageEdgeLength(boolean) * NRing, boolean.bbox.Diag()*maxBB);

    vcg::tri::UpdateSelection<TriangleMeshType>::VertexClear(boolean);

    if (intersectionVertices.size() == 0)
        return;

    for (const size_t& vId : intersectionVertices) {
        boolean.vert[vId].SetS();
    }

    QuadRetopology::internal::LaplacianGeodesicSmoothing(boolean, intersectionSmoothingInterations, maxDistance, 0.8, smoothedVertices);
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
        std::vector<std::pair<int, int>>& preservedFacesMap,
        std::vector<std::pair<int, int>>& preservedVerticesMap,
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

}

}
