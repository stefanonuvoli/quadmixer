#include "quadconvert.h"
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/complex.h>


template<class M>
void VCGToEigen(
        M& vcgMesh,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        int numVertices,
        int dim)
{
    assert(dim >= 2);
    assert(numVertices > 2);

    V.resize(static_cast<int>(vcgMesh.vert.size()), dim);
    F.resize(static_cast<int>(vcgMesh.face.size()), numVertices);

    for (size_t i = 0; i < vcgMesh.vert.size(); i++){
        for (int j = 0; j < dim; j++) {
            V(static_cast<int>(i), j) = vcgMesh.vert[i].P()[j];
        }
    }
    for (size_t i = 0; i < vcgMesh.face.size(); i++){
        for (int j = 0; j < vcgMesh.face[i].VN(); j++) {
            F(static_cast<int>(i), j) = static_cast<int>(vcg::tri::Index(vcgMesh, vcgMesh.face[i].V(j)));
        }
    }
}

template<class M>
void VCGToEigenSelected(
        M& vcgMesh,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        std::vector<int>& vMap,
        std::vector<int>& fMap,
        int numVertices,
        int dim)
{
    assert(dim >= 2);
    assert(numVertices > 2);


    int nSelectedVertices = 0;
    for (size_t i = 0; i < vcgMesh.vert.size(); i++){
        if (vcgMesh.vert[i].IsS()) {
            nSelectedVertices++;
        }
    }
    int nSelectedFaces = 0;
    for (size_t i = 0; i < vcgMesh.face.size(); i++){
        if (vcgMesh.face[i].IsS()) {
            nSelectedFaces++;
        }
    }

    V.resize(nSelectedVertices, dim);
    F.resize(nSelectedFaces, numVertices);

    vMap.resize(vcgMesh.vert.size(), -1);
    int vId = 0;
    for (size_t i = 0; i < vcgMesh.vert.size(); i++){
        if (vcgMesh.vert[i].IsS()) {
            vMap[i] = vId;
            for (int j = 0; j < dim; j++) {
                V(vId, j) = vcgMesh.vert[i].P()[j];
            }
            vId++;
        }
    }

    fMap.resize(vcgMesh.face.size(), -1);
    int fId = 0;
    for (size_t i = 0; i < vcgMesh.face.size(); i++){
        if (vcgMesh.face[i].IsS()) {
            fMap[i] = fId;
            for (int j = 0; j < vcgMesh.face[i].VN(); j++) {
                F(fId, j) = vMap[vcg::tri::Index(vcgMesh, vcgMesh.face[i].V(j))];
            }
            fId++;
        }
    }
}

template<class M>
void eigenToVCG(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        M& vcgMesh,
        int numVertices,
        int dim)
{
    assert(dim >= 2);
    assert(numVertices > 2);

    vcgMesh.Clear();

    for (int i = 0; i < V.rows(); i++) {
        typename M::CoordType vv(V(i,0), V(i,1), V(i,2));
        vcg::tri::Allocator<M>::AddVertex(vcgMesh, vv);
    }

    vcg::tri::Allocator<M>::AddFaces(vcgMesh, static_cast<size_t>(F.rows()));
    for (int i = 0; i < F.rows(); i++) {
        vcgMesh.face[static_cast<size_t>(i)].Alloc(numVertices);
        for (int j = 0; j < numVertices; j++) {
            vcgMesh.face[static_cast<size_t>(i)].V(j) = &(vcgMesh.vert[static_cast<size_t>(F(i,j))]);
        }
    }
}
