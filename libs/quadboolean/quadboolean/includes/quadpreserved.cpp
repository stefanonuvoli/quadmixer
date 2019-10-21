#include "quadpreserved.h"

#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

#include <vcg/complex/algorithms/geodesic.h>
#include <vcg/space/distance3.h>

namespace QuadBoolean {
namespace internal {

int maxHist(const std::vector<int>& row, int& startColumn, int& endColumn);


template<class PolyMeshType, class TriangleMeshType>
void findPreservedFaces(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        TriangleMeshType& trimesh1,
        TriangleMeshType& trimesh2,
        TriangleMeshType& boolean,
        const std::vector<size_t>& intersectionVertices,
        const bool patchRetraction,
        const double patchRetractionNRing,
        const double maxBB,
        const bool preserveNonQuads,
        const std::vector<std::pair<size_t, size_t>>& birthTriangle,
        const std::vector<int>& birthFace1,
        const std::vector<int>& birthFace2,
        std::vector<bool>& isPreserved1,
        std::vector<bool>& isPreserved2,
        std::vector<bool>& isNewSurface)
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
        vcg::tri::UpdateQuality<TriangleMeshType>::VertexConstant(boolean, 0.0);
        vcg::tri::EuclideanDistance<TriangleMeshType> ed;
        vcg::tri::UpdateTopology<TriangleMeshType>::VertexFace(boolean);
        vcg::tri::Geodesic<TriangleMeshType>::Compute(boolean, seedVec, ed);

        maxDistance = std::min(averageEdgeLength(boolean) * patchRetractionNRing, boolean.bbox.Diag()*maxBB);
    }

    isNewSurface.resize(boolean.face.size(), true);
    isPreserved1.resize(mesh1.face.size(), false);
    isPreserved2.resize(mesh2.face.size(), false);

    //Only faces that are birth faces are set to preserved
    for (size_t i = 0; i < boolean.face.size(); i++) {
        size_t trimeshFaceId = birthTriangle[i].second;

        //Set face as preserved
        if (birthTriangle[i].first == 1) {
            isPreserved1[birthFace1[trimeshFaceId]] = true;
        }
        else {
            isPreserved2[birthFace2[trimeshFaceId]] = true;
        }
    }

    //Identify if the triangle is the new surface
    for (size_t i = 0; i < boolean.face.size(); i++) {
        bool closeToIntersectionCurve = false;
        std::set<typename TriangleMeshType::CoordType> booleanCoordSet;

        //Check if close to intersection curve
        for (int k = 0; k < boolean.face[i].VN(); k++) {
            booleanCoordSet.insert(boolean.face[i].V(k)->P());

            if (boolean.face[i].V(k)->Q() < maxDistance) {
                closeToIntersectionCurve = true;
            }
        }

        //Check coordinates
        TriangleMeshType* currentTrimesh = &trimesh1;
        if (birthTriangle[i].first == 2) {
            currentTrimesh = &trimesh2;
        }

        std::set<typename TriangleMeshType::CoordType> trimeshCoordSet;

        size_t trimeshFaceId = birthTriangle[i].second;

        for (int j = 0; j < currentTrimesh->face[trimeshFaceId].VN(); j++)
            trimeshCoordSet.insert(currentTrimesh->face[trimeshFaceId].V(j)->P());

        //Check if polygonal faces
        bool nonPolygonalCheck = true;
        if (!preserveNonQuads) {
            typename PolyMeshType::FaceType* facePointer;
            if (birthTriangle[i].first == 1) {
                facePointer = &mesh1.face[birthFace1[trimeshFaceId]];
            }
            else {
                facePointer = &mesh2.face[birthFace2[trimeshFaceId]];
            }

            nonPolygonalCheck = facePointer->VN() == 4;
        }

        //Surface has not changed
        if (nonPolygonalCheck && !closeToIntersectionCurve && booleanCoordSet == trimeshCoordSet) {
            isNewSurface[i] = false;
        }
        //A triangle has changed
        else {
            //Set face as not preserved
            if (birthTriangle[i].first == 1) {
                isPreserved1[birthFace1[trimeshFaceId]] = false;
            }
            else {
                isPreserved2[birthFace2[trimeshFaceId]] = false;
            }
        }
    }
}

template<class PolyMeshType>
void findAffectedPatches(
        PolyMeshType& mesh,
        const std::vector<bool>& isPreserved,
        const std::vector<int>& faceLabel,
        std::unordered_set<int>& affectedPatches)
{
    //Find affected patches
    for (int i = 0; i < mesh.face.size(); i++) {
        if (!isPreserved[i]) {
            affectedPatches.insert(faceLabel[i]);
        }
    }
}

template<class PolyMeshType>
void getPreservedSurfaceMesh(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const std::vector<bool>& isPreserved1,
        const std::vector<bool>& isPreserved2,
        const std::vector<int>& faceLabel1,
        const std::vector<int>& faceLabel2,
        PolyMeshType& preservedSurface,
        std::vector<int>& newFaceLabel,
        std::unordered_map<size_t, size_t>& preservedFacesMap,
        std::unordered_map<size_t, size_t>& preservedVerticesMap)
{
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(mesh1);
    vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(mesh2);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearS(mesh1);
    vcg::tri::UpdateFlags<PolyMeshType>::VertexClearS(mesh2);

    //Save vertices id in quality
    for (size_t i = 0; i < mesh1.vert.size(); i++) {
        mesh1.vert[i].Q() = i;
    }
    for (size_t i = 0; i < mesh2.vert.size(); i++) {
        mesh2.vert[i].Q() = mesh1.vert.size() + i;
    }

    //Select the face which are remaining and put their id in quality
    for (size_t i = 0; i < mesh1.face.size(); i++) {
        if (isPreserved1[i]) {
            mesh1.face[i].SetS();
            mesh1.face[i].Q() = i;
        }
    }
    for (size_t i = 0; i < mesh2.face.size(); i++) {
        if (isPreserved2[i]) {
            mesh2.face[i].SetS();
            mesh2.face[i].Q() = mesh1.face.size() + i;
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

    int maxLabel1 = 0;
    for (const int& l : faceLabel1) {
        maxLabel1 = std::max(maxLabel1, l);
    }
    newFaceLabel.resize(preservedSurface.face.size(), -1);
    for (size_t i = 0; i < preservedSurface.face.size(); i++) {
        if (!preservedSurface.face[i].IsD()) {
            size_t currentFaceId = static_cast<size_t>(preservedSurface.face[i].Q());

            preservedFacesMap.insert(std::make_pair(i, currentFaceId));

            //Label restore
            if (currentFaceId < mesh1.face.size()) {
                newFaceLabel[i] = faceLabel1[currentFaceId];
            }
            else {
                newFaceLabel[i] = maxLabel1 + faceLabel2[currentFaceId - mesh1.face.size()];
            }
        }
    }


    for (size_t i = 0; i < preservedSurface.vert.size(); i++) {
        if (!preservedSurface.vert[i].IsD()) {
            size_t currentVertId = static_cast<size_t>(preservedSurface.vert[i].Q());

            preservedVerticesMap.insert(std::make_pair(i, currentVertId));
        }
    }
    vcg::PolygonalAlgorithm<PolyMeshType>::UpdateFaceNormalByFitting(preservedSurface);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalized(preservedSurface);
    vcg::tri::UpdateBounding<PolyMeshType>::Box(preservedSurface);
    vcg::tri::UpdateNormal<PolyMeshType>::PerVertexNormalizedPerFace(preservedSurface);
}


template<class TriangleMeshType>
void getNewSurfaceMesh(
        TriangleMeshType& boolean,
        const std::vector<std::pair<size_t, size_t>>& birthTriangle,
        const std::vector<int>& birthFace1,
        const std::vector<int>& birthFace2,
        const std::vector<bool>& isPreserved1,
        const std::vector<bool>& isPreserved2,
        std::vector<bool>& isNewSurface,
        TriangleMeshType& newSurface)
{
    //Readjusting new surfaces
    for (size_t i = 0; i < boolean.face.size(); i++) {
        if (!isNewSurface[i] && (
            (birthTriangle[i].first == 1 && !isPreserved1[birthFace1[birthTriangle[i].second]]) ||
            (birthTriangle[i].first == 2 && !isPreserved2[birthFace2[birthTriangle[i].second]])
        )) {
            isNewSurface[i] = true;
        }
    }

    vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearS(boolean);
    for (size_t i = 0; i < boolean.face.size(); i++) {
        if (isNewSurface[i]) {
            boolean.face[i].SetS();
        }
    }
    vcg::tri::Append<TriangleMeshType, TriangleMeshType>::Mesh(newSurface, boolean, true);
    vcg::tri::Clean<TriangleMeshType>::RemoveDuplicateVertex(newSurface);
    vcg::tri::Clean<TriangleMeshType>::RemoveUnreferencedVertex(newSurface);
}




template<class PolyMeshType>
std::vector<int> splitPatchesInMaximumRectangles(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedPatches,
        const std::vector<int>& faceLabel,
        std::vector<bool>& isPreserved,
        const int minimumRectangleArea,
        const bool recursive)
{
    if (minimumRectangleArea <= 0)
        return faceLabel;

    std::vector<int> newFaceLabel;
    newFaceLabel.resize(faceLabel.size(), -1);

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, faceLabel);

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

            if (startPos.IsNull())
                continue;

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
                    matrix[x][y] = isPreserved[vcg::tri::Index(mesh, pos2.F())];

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
            isPreserved[i] = false;
            assert(newFaceLabel[i] == -1);
        }
    }

    return newFaceLabel;
}

template<class PolyMeshType>
int mergePatches(
        PolyMeshType& mesh,
        std::unordered_set<int>& affectedQuads,
        std::vector<int>& faceLabel,
        const std::vector<bool>& preservedFace)
{
    vcg::tri::UpdateQuality<PolyMeshType>::VertexValence(mesh);

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, faceLabel);

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
                    assert(preservedFace[vcg::tri::Index(mesh, pos.F())]);

                    if (validToMerge) {
                        pos.FlipF();

                        int adjIndex = vcg::tri::Index(mesh, pos.F());

                        if (!preservedFace[adjIndex]) {
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

                    return 1 + mergePatches(mesh, affectedQuads, faceLabel, preservedFace);
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
int deleteNonConnectedPatches(
        PolyMeshType& mesh,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedFace)
{
    int nDeleted = 0;    

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, faceLabel);

    const std::set<int>& labels = quadLayoutData.labels;
    const std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    //Delete non connected patches
    for (const int& pId : labels) {
        const QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

        const std::vector<size_t>& patchFaces = quadPatch.faces;

        bool connected = false;

        //Find if the quad is connected
        for (const size_t& fId : patchFaces) {
            if (preservedFace[fId]) {
                for (int j = 0; j < 4 && !connected; j++) {
                    size_t adjIndex = vcg::tri::Index(mesh, mesh.face[fId].FFp(j));
                    if (preservedFace[adjIndex] && faceLabel[adjIndex] != pId) {
                        connected = true;
                    }
                }
            }
        }

        if (!connected) {
            nDeleted++;
            for (const size_t& fId : patchFaces) {
                preservedFace[fId] = false;
                faceLabel[fId] = -1;
            }
        }
    }

    return nDeleted;
}

template<class PolyMeshType>
int deleteSmallPatches(
        PolyMeshType& mesh,
        const std::unordered_set<int>& affectedPatches,
        const int minPatchArea,
        std::vector<int>& faceLabel,
        std::vector<bool>& preservedFace)
{
    int nDeleted = 0;

    QuadLayoutData<PolyMeshType> quadLayoutData = getQuadLayoutData(mesh, faceLabel);

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
                    preservedFace[fId] = false;
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
