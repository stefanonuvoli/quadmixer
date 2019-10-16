#include "quadpreserved.h"

#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>


namespace QuadBoolean {
namespace internal {

int maxHist(const std::vector<int>& row, int& startColumn, int& endColumn);

template<class PolyMeshType, class TriangleMeshType>
void computePreservedQuadForMesh(
        PolyMeshType& mesh,
        TriangleMeshType& boolean,
        const bool isQuadMesh,
        const double maxDistance,
        std::vector<bool>& preservedQuad)
{
    preservedQuad.resize(mesh.face.size(), false);

    if (!isQuadMesh) {
        return;
    }

    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(mesh);
    vcg::tri::UpdateNormal<TriangleMeshType>::PerFaceNormalized(boolean);

    vcg::tri::UpdateTopology<TriangleMeshType>::FaceFace(boolean);

    std::map<std::set<typename PolyMeshType::CoordType>, size_t> quadMap;

    for (size_t i = 0; i < mesh.face.size(); i++) {
        std::set<typename PolyMeshType::CoordType> coordSet;

        for (int j = 0; j < mesh.face[i].VN(); j++)
            coordSet.insert(mesh.face[i].V(j)->P());

        quadMap.insert(std::make_pair(coordSet, i));
    }


    std::vector<bool> isNewSurface(boolean.face.size(), true);
    for (size_t i = 0; i < boolean.face.size(); i++) {
        if (isNewSurface[i]) {
            bool closeToIntersectionCurve = false;

            std::set<typename TriangleMeshType::CoordType> coordSet;

            for (int k = 0; k < boolean.face[i].VN(); k++) {
                coordSet.insert(boolean.face[i].V(k)->P());

                if (boolean.face[i].V(k)->Q() < maxDistance) {
                    closeToIntersectionCurve = true;
                }
            }

            for (int k = 0; k < boolean.face[i].VN() && !closeToIntersectionCurve && isNewSurface[i]; k++) {
                typename TriangleMeshType::FacePointer fp = boolean.face[i].FFp(k);

                if (fp == &boolean.face[i])
                    continue;

                std::set<typename PolyMeshType::CoordType> coordSetComplete = coordSet;

                int otherFaceEdge = boolean.face[i].FFi(k);
                int oppositeVert = (otherFaceEdge + 2) % 3;

                coordSetComplete.insert(fp->V(oppositeVert)->P());

                typename std::map<std::set<typename PolyMeshType::CoordType>, size_t>::iterator findIt = quadMap.find(coordSetComplete);
                if (findIt != quadMap.end()) {
                    size_t quadId = findIt->second;
                    if (mesh.face[quadId].N().dot(boolean.face[i].N()) > 0) {
                        quadMap.erase(findIt);

                        size_t adjFaceId = vcg::tri::Index(boolean, fp);

                        isNewSurface[adjFaceId] = false;
                        isNewSurface[i] = false;

                        preservedQuad[quadId] = true;
                    }
                }
            }
        }
    }
}

template<class PolyMeshType>
std::vector<int> splitQuadPatchesInMaximumRectangles(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad,
        const int minimumRectangleArea,
        const bool recursive)
{
    if (minimumRectangleArea <= 0)
        return faceLabel;

    if (!isQuadMesh)
        return faceLabel;

    std::vector<int> newFaceLabel;
    newFaceLabel.resize(faceLabel.size(), -1);

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, isQuadMesh, faceLabel);

    const std::set<int>& labels = quadLayoutData.labels;
    const std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    if (labels.size() == 0) {
        return newFaceLabel;
    }

    int maxPatchId = *labels.rbegin();

    std::vector<bool> validQuads(mesh.face.size(), false);

    for (const int& pId : labels) {
        if (affectedPatches.find(pId) == affectedPatches.end()) {
            const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

            for (size_t fId : quadPatch.faces) {
                validQuads[fId] = true;
                newFaceLabel[fId] = faceLabel[fId];
            }
        }
        else {
            const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

            const vcg::face::Pos<typename PolyMeshType::FaceType>& startPos = quadPatch.startPos;
            const size_t& sizeX = quadPatch.sizeX;
            const size_t& sizeY = quadPatch.sizeY;

            assert(!startPos.IsNull());

            vcg::face::Pos<typename PolyMeshType::FaceType> pos;

            std::vector<std::vector<bool>> matrix(sizeX, std::vector<bool>(sizeY));
            std::vector<std::vector<int>> S(sizeX, std::vector<int>(sizeY, 0));

            //Fill matrix
            pos.Set(startPos.F(), startPos.E(), startPos.V());
            for (size_t x = 0; x < sizeX; x++) {
                assert(faceLabel[vcg::tri::Index(mesh, pos.F())] == pId);
                pos.FlipE();

                vcg::face::Pos<typename PolyMeshType::FaceType> pos2(pos);
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

                        if (tmpMaxAreaX - tmpMinAreaX + 1 >= minimumRectangleArea || tmpMaxAreaY - tmpMinAreaY + 1 >= minimumRectangleArea) {
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
                    vcg::face::Pos<typename PolyMeshType::FaceType> pos2(pos);
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

    quadLayoutData = internal::getQuadLayoutData(mesh, isQuadMesh, newFaceLabel);

    return newFaceLabel;
}

template<class PolyMeshType>
int mergeQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::unordered_set<int>& affectedQuads,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedQuad)
{
    if (!isQuadMesh)
        return 0;

    vcg::tri::UpdateQuality<PolyMeshType>::VertexValence(mesh);

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, isQuadMesh, faceLabel);

    const std::set<int>& labels = quadLayoutData.labels;
    const std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    for (const int& pId : labels) {
        if (affectedQuads.find(pId) != affectedQuads.end()) {
            const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

            const vcg::face::Pos<typename PolyMeshType::FaceType>& startPos = quadPatch.startPos;
            const size_t& sizeX = quadPatch.sizeX;
            const size_t& sizeY = quadPatch.sizeY;

            assert(!startPos.IsNull());

            vcg::face::Pos<typename PolyMeshType::FaceType> pos;
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

                if (adjLabel < 0 || adjLabel == prevLabel || adjLabel == nextLabel || affectedQuads.find(adjLabel) == affectedQuads.end())
                    validToMerge = false;

                if (validToMerge) {
                    const QuadLayoutPatch<PolyMeshType>& adjQuadPatch = quadLayoutData.quadPatches[adjLabel];
                    assert(sizeX == adjQuadPatch.sizeX ||
                           sizeY == adjQuadPatch.sizeY ||
                           sizeX == adjQuadPatch.sizeY ||
                           sizeY == adjQuadPatch.sizeX);

                    for (const size_t& fId : adjQuadPatch.faces) {
                        faceLabel[fId] = pId;
                        affectedQuads.erase(adjLabel);
                    }

                    return 1 + mergeQuadPatches(mesh, isQuadMesh, affectedQuads, faceLabel, preservedQuad);
                }

                //Turn
                pos.FlipE();
                pos.FlipV();
            }
        }
    }

    return 0;
}

template<class PolyMeshType>
int deleteNonConnectedQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad)
{
    if (!isQuadMesh)
        return 0;

    int nDeleted = 0;    

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, isQuadMesh, faceLabel);

    const std::set<int>& labels = quadLayoutData.labels;
    const std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    //Delete non connected patches
    for (const int& pId : labels) {
        const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

        const std::vector<size_t>& patchFaces = quadPatch.faces;

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

template<class PolyMeshType>
int deleteSmallQuadPatches(
        PolyMeshType& mesh,
        const bool isQuadMesh,
        const std::unordered_set<int>& affectedPatches,
        const int minPatchArea,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedQuad)
{
    if (!isQuadMesh)
        return 0;

    int nDeleted = 0;

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, isQuadMesh, faceLabel);

    const std::set<int>& labels = quadLayoutData.labels;
    const std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    //Delete small patches
    for (const int& pId : labels) {
        if (affectedPatches.find(pId) != affectedPatches.end()) {
            const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];
            const std::vector<size_t>& patchFaces = quadPatch.faces;

            if (quadPatch.sizeX*quadPatch.sizeY < minPatchArea) {
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

inline int maxHist(const std::vector<int>& row, int& startColumn, int& endColumn)
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

}
}
