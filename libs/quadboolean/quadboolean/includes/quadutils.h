#ifndef QUADUTILS_H
#define QUADUTILS_H

#include <vector>

namespace QuadBoolean {
namespace internal {

static std::vector<size_t> dummySizetVector;

template <class MeshType>
void updateAllMeshAttributes(MeshType &mesh);

template <class MeshType>
std::vector<std::vector<size_t>> findConnectedComponents(
        const MeshType &mesh);

template<class MeshType>
void LaplacianPos(MeshType &poly_m,std::vector<typename MeshType::CoordType> &AvVert);

template <class MeshType>
void LaplacianGeodesic(
        MeshType &poly_m,
        int nstep,
        const double maxDistance,
        const double minDumpS = 0.5,
        std::vector<size_t>& smoothedVertices = dummySizetVector);

std::vector<size_t> findVertexChainPath(
        const size_t& vCurrentId,
        const std::vector<std::vector<size_t>>& vertexNextMap);


template <class MeshType>
bool isTriangleMesh(MeshType& mesh);

template <class MeshType>
bool isQuadMesh(MeshType& mesh);

template <class MeshType>
std::vector<int> splitFacesInTriangles(MeshType& mesh);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh, const std::vector<size_t>& faces);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh);


}
}

#include "quadutils.cpp"

#endif // QUADUTILS_H
