#include "quadpatches.h"

#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

#include <stack>

namespace QuadBoolean {

void computePreservedQuads(
        PolyMesh& triMesh,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        const Eigen::MatrixXd& VR,
        const Eigen::MatrixXi& FR,
        const Eigen::VectorXi& J,
        const std::vector<int>& birthQuad,
        const int offset,
        std::vector<bool>& preservedQuad)
{
    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i] - offset;

        //If the birth face is in the first mesh
        if (birthFace < triMesh.face.size()) {
            preservedQuad[birthQuad[birthFace]] = true;
        }
    }

    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i] - offset;

        //If the birth face is in the first mesh
        if (birthFace < triMesh.face.size()) {
            bool isNew = true;

            for (int j = 0; j < 3; j++) {
                if (V(F(birthFace, j)) != VR(FR(i, j))) {
                    isNew = false;
                }
            }

            if (!isNew) {
                preservedQuad[birthQuad[birthFace]] = false;
            }
        }

    }
}

std::vector<int> splitQuadPatchesInMaximumRectangles(
        PolyMesh& mesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad,
        const int minimumSideLength,
        const bool recursive)
{
    std::vector<int> newFaceLabel;
    newFaceLabel.resize(faceLabel.size(), -1);

    QuadData quadData = getQuadPatchesData(mesh, faceLabel);

    std::set<int>& labels = quadData.labels;
    std::vector<QuadPatch>& patches = quadData.patches;


    int maxPatchId = *labels.rbegin();

    std::vector<bool> validQuads(mesh.face.size(), false);

    for (const int& pId : labels) {
        if (affectedPatches.find(pId) == affectedPatches.end()) {
            QuadPatch& patch = patches[pId];

            for (size_t fId : patch.faces) {
                validQuads[fId] = true;
                newFaceLabel[fId] = faceLabel[fId];
            }
        }
        else {
            QuadPatch& patch = patches[pId];

            const vcg::face::Pos<PolyMesh::FaceType>& startPos = patch.startPos;
            const size_t& sizeX = patch.sizeX;
            const size_t& sizeY = patch.sizeY;

            assert(!startPos.IsNull());

            vcg::face::Pos<PolyMesh::FaceType> pos;

            std::vector<std::vector<bool>> matrix(sizeX, std::vector<bool>(sizeY));
            std::vector<std::vector<int>> S(sizeX, std::vector<int>(sizeY, 0));

            //Fill matrix
            pos.Set(startPos.F(), startPos.E(), startPos.V());
            for (size_t x = 0; x < sizeX; x++) {
                assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
                pos.FlipE();

                vcg::face::Pos<PolyMesh::FaceType> pos2(pos);
                pos2.FlipV();

                for (size_t y = 0; y < sizeY; y++) {
                    assert(faceLabel[vcg::tri::Index(mesh, pos2.F())] == pId);
                    matrix[x][y] = preservedQuad[vcg::tri::Index(mesh, pos2.F())];

                    pos2.FlipE();
                    pos2.FlipF();
                    pos2.FlipE();
                    pos2.FlipV();
                }

                pos.FlipF();
                pos.FlipE();
                pos.FlipV();
            }


            bool foundRectangle;
            int currentNumberOfRectangle = 0;
            do {
                foundRectangle = false;

                for (size_t x = 0; x < sizeX; x++) {
                    for (size_t y = 0; y < sizeY; y++) {
                        S[x][y] = (matrix[x][y] ? 1 : 0);
                    }
                }

                for (size_t x = 1; x < sizeX; x++) {
                    for (size_t y = 0; y < sizeY; y++) {
                        if (matrix[x][y]) {
                            S[x][y] += S[x-1][y];
                        }
                    }
                }


//                for (size_t x = 0; x < sizeX; x++) {
//                    for (size_t y = 0; y < sizeY; y++) {
//                        std::cout << matrix[x][y] << " ";
//                    }
//                    std::cout << std::endl;
//                }
//                std::cout << std::endl;

//                for (size_t x = 0; x < sizeX; x++) {
//                    for (size_t y = 0; y < sizeY; y++) {
//                        std::cout << S[x][y] << " ";
//                    }
//                    std::cout << std::endl;
//                }
//                std::cout << std::endl;


                int minAreaX = sizeX+1, minAreaY = sizeY+1, maxAreaX = -1, maxAreaY = -1, maxArea = 0;
                for (size_t x = 0; x < sizeX; x++) {
                    int tmpMinAreaY;
                    int tmpMaxAreaY;

                    int tmpArea = maxHist(S[x], tmpMinAreaY, tmpMaxAreaY);

                    if (tmpArea > 0 && tmpArea >= maxArea) {
                        int tmpMaxAreaX = x;
                        int tmpMinAreaX = tmpMaxAreaX - (tmpArea / (tmpMaxAreaY - tmpMinAreaY + 1)) + 1;

                        if (tmpMaxAreaX - tmpMinAreaX + 1 >= minimumSideLength && tmpMaxAreaY - tmpMinAreaY + 1 >= minimumSideLength) {
                            minAreaX = tmpMinAreaX;
                            minAreaY = tmpMinAreaY;
                            maxAreaX = tmpMaxAreaX;
                            maxAreaY = tmpMaxAreaY;
                            maxArea = tmpArea;
                        }
                    }
                }

//                std::cout << minAreaX << "/" << maxAreaX << " " << minAreaY << "/" << maxAreaY << " -> " << maxArea << std::endl;

                bool firstToAdd = true;
                size_t nFaces = 0;
                pos.Set(startPos.F(), startPos.E(), startPos.V());
                for (int x = 0; x < sizeX; x++) {
                    assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
                    //Turn right
                    pos.FlipE();
                    vcg::face::Pos<PolyMesh::FaceType> pos2(pos);
                    pos2.FlipV();

                    for (int y = 0; y < sizeY; y++) {
                        assert(faceLabel[vcg::tri::Index(mesh, pos2.F())] == pId);
                        if (x >= minAreaX && y >= minAreaY && x <= maxAreaX && y <= maxAreaY) {
                            assert(matrix[x][y]);

                            foundRectangle = true;
                            size_t currentFaceId = vcg::tri::Index(mesh, pos2.F());
                            assert(!validQuads[currentFaceId]);

                            validQuads[currentFaceId] = true;
                            nFaces++;
                            matrix[x][y] = false;

                            if (currentNumberOfRectangle > 0) {
                                if (firstToAdd) {
                                    maxPatchId++;
                                    firstToAdd = false;
                                }
                                newFaceLabel[currentFaceId] = maxPatchId;
                                affectedPatches.insert(maxPatchId);
                            }
                            else {
                                newFaceLabel[currentFaceId] = pId;
                            }
                        }

                        //Go forward
                        pos2.FlipE();
                        pos2.FlipF();
                        pos2.FlipE();
                        pos2.FlipV();
                    }

                    //Turn left and go forward
                    pos.FlipF();
                    pos.FlipE();
                    pos.FlipV();
                }

                currentNumberOfRectangle++;

                assert(nFaces == maxArea);
            } while (recursive && foundRectangle);
        }
    }

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (!validQuads[i]) {
            preservedQuad[i] = false;
            assert(newFaceLabel[i] == -1);
        }
    }

    return newFaceLabel;
}

QuadData getQuadPatchesData(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel)
{
    //It works just on closed surfaces
    vcg::tri::RequireFFAdjacency<PolyMesh>(mesh);

    QuadData quadData;

    std::set<int>& labels = quadData.labels;
    std::vector<QuadPatch>& patches = quadData.patches;

    //Get the set of the patches ids
    labels.clear();

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0)
            labels.insert(faceLabel[i]);
    }

    int maxPatchId = *labels.rbegin();

    patches.resize(maxPatchId+1);

    //Get faces in a given patch
    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0)
            patches[faceLabel[i]].faces.push_back(i);
    }

    //Get sizes and start pos of the patches
    for (const int& pId : labels) {
        QuadPatch& patch = patches[pId];

        vcg::face::Pos<PolyMesh::FaceType>& startPos = patch.startPos;
        startPos.SetNull();
        size_t& sizeX = patch.sizeX;
        size_t& sizeY = patch.sizeY;

        //Find a corner of the quad
        int maxNBorders = 0;
        for (const size_t& fId : patch.faces) {
            int nBorders = 0;
            for (int j = 0; j < 4; j++) {
                if (faceLabel[vcg::tri::Index(mesh, mesh.face[fId].FFp(j))] != pId) {
                    nBorders++;
                }
            }

            if (nBorders >= 2 && nBorders > maxNBorders) {
                maxNBorders = nBorders;
                for (int j = 0; j < 4; j++) {
                    if (faceLabel[vcg::tri::Index(mesh, mesh.face[fId].FFp(j))] != pId) {
                        startPos.Set(&mesh.face[fId], j, mesh.face[fId].V(j));
                    }
                }
            }
        }

        assert(!startPos.IsNull());

        //Flip if pos is pointing outside the quad
        startPos.FlipE();
        startPos.FlipF();
        if (faceLabel[vcg::tri::Index(mesh, startPos.F())] != pId) {
            startPos.FlipF();
            startPos.FlipE();
            startPos.FlipV();
        }
        else {
            startPos.FlipF();
            startPos.FlipE();
        }

        vcg::face::Pos<PolyMesh::FaceType> pos;

        //Get size of the matrix
        pos.Set(startPos.F(), startPos.E(), startPos.V());
        bool doneX = false;
        do {
            assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
            sizeX++;

            pos.FlipE();
            pos.FlipF();

            if (faceLabel[vcg::tri::Index(mesh, pos.F())] != pId) {
                doneX = true;
            }
            else {
                pos.FlipE();
                pos.FlipV();
            }
        } while (!doneX);


        pos.Set(startPos.F(), startPos.E(), startPos.V());
        pos.FlipE();
        pos.FlipV();
        bool doneY = false;
        do {
            assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
            sizeY++;

            pos.FlipE();
            pos.FlipF();

            if (faceLabel[vcg::tri::Index(mesh, pos.F())] != pId) {
                doneY = true;
            }
            else {
                pos.FlipE();
                pos.FlipV();
            }
        } while (!doneY);


        assert(sizeX*sizeY == patch.faces.size());
    }

    return quadData;
}


int mergeQuadPatches(
        PolyMesh& mesh,
        std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedQuad)
{
    QuadData quadData = getQuadPatchesData(mesh, faceLabel);

    vcg::tri::UpdateQuality<PolyMesh>::VertexValence(mesh);

    std::set<int>& labels = quadData.labels;
    std::vector<QuadPatch>& patches = quadData.patches;

    for (const int& pId : labels) {
        if (affectedPatches.find(pId) != affectedPatches.end()) {
            const QuadPatch& patch = patches[pId];

            const vcg::face::Pos<PolyMesh::FaceType>& startPos = patch.startPos;
            const size_t& sizeX = patch.sizeX;
            const size_t& sizeY = patch.sizeY;

            assert(!startPos.IsNull());

            vcg::face::Pos<PolyMesh::FaceType> pos;
            pos.Set(startPos.F(), startPos.E(), startPos.V());

            for (size_t i = 0; i < 4; i++) {
                bool validToMerge = true;
                int adjLabel = -1;

                //Go back and check previous face label
                pos.FlipF();
                assert(faceLabel[vcg::tri::Index(mesh, pos.F())] != pId);
                pos.FlipV();

                //Singularity
                if (pos.V()->Q()!=4)
                    validToMerge = false;

                pos.FlipE();
                pos.FlipF();
                int prevLabel = faceLabel[vcg::tri::Index(mesh, pos.F())];
                assert(faceLabel[vcg::tri::Index(mesh, pos.F())] != pId);
                pos.FlipF();
                pos.FlipE();
                pos.FlipV();
                pos.FlipF();

                for (size_t j = 0; j < (i%2 == 0 ? sizeX : sizeY); j++) {

                    assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
                    assert(preservedQuad[vcg::tri::Index(mesh, pos.F())]);

                    if (validToMerge) {
                        pos.FlipF();

                        int adjIndex = vcg::tri::Index(mesh, pos.F());

                        if (!preservedQuad[adjIndex]) {
                            validToMerge = false;
                            assert(faceLabel[adjIndex] == -1);
                        }

                        if (adjLabel < 0) {
                            adjLabel = faceLabel[adjIndex];
                        }
                        else if (faceLabel[adjIndex] != adjLabel) {
                            validToMerge = false;
                        }

                        pos.FlipF();
                    }

                    //Go forward
                    pos.FlipE();
                    pos.FlipF();
                    pos.FlipE();
                    pos.FlipV();
                }

                //Check and go back
                pos.FlipF();
                int nextLabel = faceLabel[vcg::tri::Index(mesh, pos.F())];
                assert(faceLabel[vcg::tri::Index(mesh, pos.F())] != pId);
                pos.FlipF();
                pos.FlipV();

                //Singularity
                if (pos.V()->Q()!=4)
                    validToMerge = false;

                pos.FlipE();
                pos.FlipF();
                pos.FlipE();

                if (adjLabel < 0 || adjLabel == prevLabel || adjLabel == nextLabel || affectedPatches.find(adjLabel) == affectedPatches.end())
                    validToMerge = false;

                if (validToMerge) {
                    const QuadPatch& adjQuadPatch = quadData.patches[adjLabel];
                    assert(sizeX == adjQuadPatch.sizeX ||
                           sizeY == adjQuadPatch.sizeY ||
                           sizeX == adjQuadPatch.sizeY ||
                           sizeY == adjQuadPatch.sizeX);

                    for (const size_t& fId : adjQuadPatch.faces) {
                        faceLabel[fId] = pId;
                        affectedPatches.erase(adjLabel);
                    }

                    return 1 + mergeQuadPatches(mesh, affectedPatches, faceLabel, preservedQuad);
                }

                //Turn
                pos.FlipE();
                pos.FlipV();
            }
        }
    }

    return 0;
}

int deleteNonConnectedQuadPatches(
        PolyMesh& mesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad)
{
    int nDeleted = 0;

    QuadData quadData = getQuadPatchesData(mesh, faceLabel);

    std::set<int>& labels = quadData.labels;
    std::vector<QuadPatch>& patches = quadData.patches;

    //Delete non connected patches
    for (const int& pId : labels) {
        QuadPatch& patch = patches[pId];

        const std::vector<size_t>& patchFaces = patch.faces;

        bool connected = false;

        //Find if the quad is connected
        for (const size_t& fId : patchFaces) {
            if (preservedQuad[fId]) {
                for (int j = 0; j < 4 && !connected; j++) {
                    size_t adjIndex = vcg::tri::Index(mesh, mesh.face[fId].FFp(j));
                    if (preservedQuad[adjIndex] && faceLabel[adjIndex] != pId) {
                        connected = true;
                    }
                }
            }
        }

        if (!connected) {
            nDeleted++;
            for (const size_t& fId : patchFaces) {
                preservedQuad[fId] = false;
                faceLabel[fId] = -1;
            }
        }
    }

    return nDeleted;
}

int deleteSmallQuadPatches(
        PolyMesh& mesh,
        const std::unordered_set<int>& affectedPatches,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad)
{
    int nDeleted = 0;


    QuadData quadData = getQuadPatchesData(mesh, faceLabel);

    std::set<int>& labels = quadData.labels;
    std::vector<QuadPatch>& patches = quadData.patches;

    double avgSize = 0;
    int nInvalid = 0;
    for (const int& pId : labels) {
        if (affectedPatches.find(pId) == affectedPatches.end()) {
            const QuadPatch& patch = patches[pId];

            avgSize += std::min(patch.sizeX, patch.sizeY);
            nInvalid++;
        }
    }
    avgSize /= nInvalid;
    const int minPatchSideLength = std::min(std::max(static_cast<int>(std::round(avgSize)) - MINIMUMSIDELENGTHDIFFERENCE, MINIMUMSIDEMINLENGTH), MINIMUMSIDEMAXLENGTH);

    std::cout << "Small patches: side less than " << minPatchSideLength << std::endl;

    //Delete small patches
    for (const int& pId : labels) {
        if (affectedPatches.find(pId) != affectedPatches.end()) {
            const QuadPatch& patch = patches[pId];
            const std::vector<size_t>& patchFaces = patch.faces;

            if (patch.sizeX < minPatchSideLength || patch.sizeY < minPatchSideLength) {
                nDeleted++;
                for (const size_t& fId : patchFaces) {
                    preservedQuad[fId] = false;
                    faceLabel[fId] = -1;
                }
            }
        }
    }

    return nDeleted;
}



int maxHist(const std::vector<int>& row, int& startColumn, int& endColumn)
{
    // Create an empty stack. The stack holds indexes
    // of hist[] array. The bars stored in stack are
    // always in increasing order of their heights.
    stack<int> s;

    startColumn = -1;
    endColumn = -1;

    int max_area = 0; // Initalize max area
    int tp;  // To store top of stack
    int area_with_top; // To store area with top bar
    // as the smallest bar

    // Run through all bars of given histogram
    int i = 0;
    while (i < row.size())
    {
        // If this bar is higher than the bar on top
        // stack, push it to stack
        if (s.empty() || row[s.top()] <= row[i])
            s.push(i++);

        // If this bar is lower than top of stack,
        // then calculate area of rectangle with stack
        // top as the smallest (or minimum height) bar.
        // 'i' is 'right index' for the top and element
        // before top in stack is 'left index'
        else
        {
            tp = s.top();  // store the top index
            s.pop();  // pop the top

            // Calculate the area with hist[tp] stack
            // as smallest bar
            area_with_top = row[tp] * (s.empty() ? i :  i - s.top() - 1);

            // update max area, if needed
            if (max_area < area_with_top) {
                max_area = area_with_top;
                startColumn = (s.empty() ? 0 : s.top()+1);
                endColumn = i - 1;
            }
        }
    }

    // Now pop the remaining bars from stack and calculate
    // area with every popped bar as the smallest bar
    while (s.empty() == false)
    {
        tp = s.top();
        s.pop();

        area_with_top = row[tp] * (s.empty() ? i : i - s.top() - 1);

        if (max_area < area_with_top) {
            max_area = area_with_top;
            startColumn = (s.empty() ? 0 :  s.top()+1);
            endColumn = i - 1;
        }
    }

    return max_area;
}


//InvariantEdgeMap<int> getBorderEdgeMap(
//        PolyMesh& preservedSurfaceMesh,
//        const std::vector<int>& preservedSurfaceLabel)
//{
//    InvariantEdgeMap<int> map;

//    vcg::tri::UpdateTopology<PolyMesh>::FaceFace(preservedSurfaceMesh);
//    vcg::tri::UpdateFlags<PolyMesh>::FaceBorderFromFF(preservedSurfaceMesh);

//    for (size_t fId = 0; fId < preservedSurfaceMesh.face.size(); fId++) {
//        for (size_t k = 0; k < 4; k++) {
//            if (preservedSurfaceMesh.face[fId].IsB(k)) {
//                Edge edge(preservedSurfaceMesh.face[fId].V(k)->P(), preservedSurfaceMesh.face[fId].V((k+1)%4)->P());
//                map.insert(std::make_pair(edge,preservedSurfaceLabel.at(fId)));
//            }
//        }
//    }

//    return map;
//}


void getPreservedSurfaceMesh(
        PolyMesh& mesh1,
        PolyMesh& mesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMesh& preservedSurface,
        std::vector<int>& newFaceLabel)
{
    int maxLabel1 = 0;
    for (const int& l : faceLabel1) {
        maxLabel1 = std::max(maxLabel1, l);
    }

    //Select the face which are remaining
    vcg::tri::UpdateFlags<PolyMesh>::FaceClearS(mesh1);
    vcg::tri::UpdateFlags<PolyMesh>::FaceClearS(mesh2);
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
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(preservedSurface, mesh1, true);
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(preservedSurface, mesh2, true);
    vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(preservedSurface);
    vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(preservedSurface);

    newFaceLabel.resize(preservedSurface.face.size(), -1);
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        newFaceLabel[i] = static_cast<int>(preservedSurface.face[i].Q());
    }
}


void getNewSurfaceMesh(
        PolyMesh& triUnion,
        PolyMesh& triMesh1,
        PolyMesh& triMesh2,
        const std::vector<bool>& preservedQuad1,
        const std::vector<bool>& preservedQuad2,
        const Eigen::VectorXi& J,
        PolyMesh& newSurface)
{
    int nFirstFaces = triMesh1.face.size();

    //Selected the new surface triangle faces
    std::vector<bool> isNewSurface(triUnion.face.size(), false);
    for (int i = 0; i < J.rows(); i++) {
        int birthFace = J[i];

        //If the birth face is in the first mesh
        if (birthFace < nFirstFaces) {
            int firstMeshIndex = birthFace;
            if (!preservedQuad1[triMesh1.face[firstMeshIndex].Q()]) {
                isNewSurface[i] = true;
            }
        }
        //The birth face is in the second mesh
        else {
            int secondMeshIndex = birthFace-nFirstFaces;
            if (!preservedQuad2[triMesh2.face[secondMeshIndex].Q()]) {
                isNewSurface[i] = true;
            }
        }
    }

    vcg::tri::UpdateFlags<PolyMesh>::FaceClearS(triUnion);
    for (size_t i = 0; i < triUnion.face.size(); i++) {
        if (isNewSurface[i]) {
            triUnion.face[i].SetS();
        }
    }
    vcg::tri::Append<PolyMesh, PolyMesh>::Mesh(newSurface, triUnion, true);
    vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(newSurface);
    vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(newSurface);
}


}
