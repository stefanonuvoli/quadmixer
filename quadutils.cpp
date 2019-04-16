#include "quadutils.h"


#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

namespace QuadBoolean {

void updatePolymeshAttributes(PolyMesh& mesh)
{
    vcg::tri::Clean<PolyMesh>::RemoveDuplicateVertex(mesh);
    vcg::tri::Clean<PolyMesh>::RemoveUnreferencedVertex(mesh);
    vcg::tri::UpdateTopology<PolyMesh>::FaceFace(mesh);
    vcg::tri::UpdateFlags<PolyMesh>::VertexBorderFromFaceAdj(mesh);
}

std::vector<int> splitQuadInTriangle(PolyMesh& mesh) {
    typedef PolyMesh::VertexType VertexType;

    std::vector<int> birthQuad(mesh.face.size()*2, -1);

    for (size_t i = 0; i < mesh.face.size(); i++) {
        size_t vSize = mesh.face[i].VN();

        if (vSize > 3) {
            assert(vSize == 4);

            VertexType* v[4];
            v[0] = mesh.face[i].V(0);
            v[1] = mesh.face[i].V(1);
            v[2] = mesh.face[i].V(2);
            v[3] = mesh.face[i].V(3);

            size_t startIndex = 0;
            if ((v[0]->P() - v[2]->P()).Norm() > (v[1]->P() - v[3]->P()).Norm()) {
                startIndex++;
            }

            size_t newFaceId = mesh.face.size();

            vcg::tri::Allocator<PolyMesh>::AddFaces(mesh,1);
            mesh.face.back().Alloc(3);
            mesh.face.back().V(0) = v[startIndex];
            mesh.face.back().V(1) = v[(startIndex + 1)%4];
            mesh.face.back().V(2) = v[(startIndex + 2)%4];

            mesh.face[i].Dealloc();
            mesh.face[i].Alloc(3);
            mesh.face[i].V(0)=v[(startIndex + 2)%4];
            mesh.face[i].V(1)=v[(startIndex + 3)%4];
            mesh.face[i].V(2)=v[(startIndex + 4)%4];

            birthQuad[i] = i;
            birthQuad[newFaceId] = i;
        }
    }

    return birthQuad;
}

void colorizeMesh(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel)
{
    std::set<int> faceLabelSet;
    for (size_t i = 0; i < faceLabel.size(); i++)
        faceLabelSet.insert(faceLabel[i]);

    float subd = (float) 1 / (std::max(static_cast<size_t>(1), faceLabelSet.size() - 1));

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (faceLabel[i] >= 0) {
            vcg::Color4b color;
            color.SetHSVColor(subd * std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])), 1.0, 1.0);

//            color=vcg::Color4b::Scatter(faceLabel.size(),std::distance(faceLabelSet.begin(), faceLabelSet.find(faceLabel[i])));

            mesh.face[i].C() = color;
        }
    }
}

}
