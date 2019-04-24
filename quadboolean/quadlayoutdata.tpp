#include "quadlayoutdata.h"

#include <stack>

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
QuadLayoutData<PolyMeshType> getQuadLayoutData(
        PolyMeshType& mesh,
        const std::vector<int>& faceLabel)
{
    //It works just on closed surfaces
    vcg::tri::RequireFFAdjacency<PolyMeshType>(mesh);

    QuadLayoutData<PolyMeshType> quadLayoutData;

    std::set<int>& labels = quadLayoutData.labels;
    std::vector<QuadLayoutPatch<PolyMeshType>>& quadPatches = quadLayoutData.quadPatches;

    //Get the set of the patches ids
    labels.clear();

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0)
            labels.insert(faceLabel[i]);
    }

    if (labels.size() == 0)
        return quadLayoutData;

    int maxPatchId = *labels.rbegin();

    quadPatches.resize(maxPatchId+1);

    //Get faces in a given patch
    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0)
            quadPatches[faceLabel[i]].faces.push_back(i);
    }

    //Get sizes and start pos of the patches
    for (const int& pId : labels) {
        QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

        vcg::face::Pos<typename PolyMeshType::FaceType>& startPos = quadPatch.startPos;
        startPos.SetNull();
        size_t& sizeX = quadPatch.sizeX;
        size_t& sizeY = quadPatch.sizeY;

        //Find a corner of the quad
        int maxNBorders = 0;
        for (const size_t& fId : quadPatch.faces) {
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

        vcg::face::Pos<typename PolyMeshType::FaceType> pos;

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


        assert(sizeX*sizeY == quadPatch.faces.size());
    }

    return quadLayoutData;
}

}
}
