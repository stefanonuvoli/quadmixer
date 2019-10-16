#ifndef QUADUTILS_H
#define QUADUTILS_H

#include <vector>

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
void LaplacianPos(PolyMeshType &poly_m,std::vector<typename PolyMeshType::CoordType> &AvVert);

template <class PolyMeshType>
void LaplacianGeodesic(
        PolyMeshType &poly_m,
        int nstep,
        const double maxDistance,
        const double minDumpS = 0.5);

std::vector<size_t> findVertexChainPath(
        const size_t& vCurrentId,
        const std::vector<std::vector<size_t>>& vertexNextMap);

template <class PolyMeshType>
bool isTriangleMesh(PolyMeshType& mesh);

template <class PolyMeshType>
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh, const std::vector<size_t>& faces);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh);


}
}

#include "quadutils.cpp"

#endif // QUADUTILS_H
