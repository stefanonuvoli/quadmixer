#ifndef QUADPATCHES_H
#define QUADPATCHES_H

#include <vector>
#include <set>
#include <unordered_set>

#include "quadcommontypes.h"

#define MINIMUMSIDELENGTHDIFFERENCE 3
#define MINIMUMSIDEMINLENGTH 2
#define MINIMUMSIDEMAXLENGTH 5

namespace QuadBoolean {

struct QuadPatch {
    std::vector<size_t> faces;
    vcg::face::Pos<PolyMesh::FaceType> startPos;
    size_t sizeX;
    size_t sizeY;
};
struct QuadData {
    std::set<int> labels;
    std::vector<QuadPatch> patches;
};

void computePreservedQuads(
        PolyMesh& triMesh,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J,
        const std::vector<int>& birthQuad,
        const int offset,
        std::vector<bool>& preservedQuad);

std::vector<int> splitQuadPatchesInMaximumRectangles(
        PolyMesh& mesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad,
        const int minQuadSideLength,
        const bool recursive = true);

QuadData getQuadPatchesData(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel);

int mergeQuadPatches(
        PolyMesh& mesh,
        std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedQuad);
int deleteNonConnectedQuadPatches(
        PolyMesh& mesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);
int deleteSmallQuadPatches(
        PolyMesh& mesh,
        const std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad);

int maxHist(const std::vector<int>& row, int& startColumn, int& endColumn);

//InvariantEdgeMap<int> getBorderEdgeMap(
//        PolyMesh& preservedSurfaceMesh,
//        const std::vector<int>& preservedSurfaceLabel);

void getPreservedSurfaceMesh(
        PolyMesh& mesh1,
        PolyMesh& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMesh& preservedSurface,
        std::vector<int>& newFaceLabel);

void getNewSurfaceMesh(
        PolyMesh& triUnion,
        PolyMesh& triMesh1,
        PolyMesh& triMesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const Eigen::VectorXi& J,
        PolyMesh& newSurface);

}

#endif // QUADPATCHES_H
