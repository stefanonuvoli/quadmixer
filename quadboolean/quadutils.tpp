#include "quadutils.h"


#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

namespace QuadBoolean {
namespace internal {

template <class PolyMeshType>
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh) {
    typedef typename PolyMeshType::VertexType VertexType;

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

            vcg::tri::Allocator<PolyMeshType>::AddFaces(mesh,1);
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

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh) {
    typename PolyMeshType::ScalarType length = 0;
    size_t numEdges = 0;
    for (size_t i=0;i<mesh.face.size();i++) {
        if (!mesh.face[i].IsD()) {
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                size_t index0=vcg::tri::Index(mesh,mesh.face[i].V0(j));
                size_t index1=vcg::tri::Index(mesh,mesh.face[i].V1(j));
                typename PolyMeshType::CoordType p0=mesh.vert[index0].P();
                typename PolyMeshType::CoordType p1=mesh.vert[index1].P();
                length += (p0-p1).Norm();
                numEdges++;
            }
        }
    }

    length /= numEdges;

    return length;
}

}
}
