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

#include "quadlayoutdata.h"

#include <stack>

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
QuadLayoutData<PolyMeshType> getQuadLayoutData(
        PolyMeshType& mesh,
        const std::vector<int>& faceLabel)
{
    QuadLayoutData<PolyMeshType> quadLayoutData;

    //It works just on closed surfaces
    vcg::tri::RequireFFAdjacency<PolyMeshType>(mesh);

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

    for (int i = 0; i < maxPatchId + 1; i++) {
        quadPatches[i].isQuad = true;
    }

    //Get faces in a given patch
    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0) {
            quadPatches[faceLabel[i]].faces.push_back(i);

            if (mesh.face[i].VN() != 4) {
                quadPatches[faceLabel[i]].isQuad = false;
            }
        }
    }

    //Get sizes and start pos of the patches
    for (const int& pId : labels) {
        QuadLayoutPatch<PolyMeshType>& quadPatch = quadPatches[pId];

        size_t& sizeX = quadPatch.sizeX;
        size_t& sizeY = quadPatch.sizeY;

        vcg::face::Pos<typename PolyMeshType::FaceType>& startPos = quadPatch.startPos;
        startPos.SetNull();

        if (quadPatch.isQuad) {
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

            if(startPos.IsNull()) {
                std::cout << "Couldn't find border for a patch." << std::endl;
                sizeX = 0;
                sizeY = 0;
                continue;
            }


            //Flip if pos is pointing outside the quad
            startPos.FlipE();
            startPos.FlipF();
            if (faceLabel[vcg::tri::Index(mesh, startPos.F())] != pId) {
                startPos.FlipF();
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
        else {
            assert(quadPatch.faces.size() == 1);
            sizeX = 0;
            sizeY = 0;

            typename PolyMeshType::FaceType* f = &mesh.face[quadPatch.faces.at(0)];
            startPos.Set(f, 0, f->V(0));
        }
    }

    return quadLayoutData;
}

}
}
