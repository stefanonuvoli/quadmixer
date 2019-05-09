#ifndef QUADUTILS_H
#define QUADUTILS_H

#include "meshtypes.h"

namespace QuadBoolean {
namespace internal {


template <class PolyMeshType>
bool isTriangleMesh(PolyMeshType& mesh);

template <class PolyMeshType>
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh, const std::vector<size_t>& faces);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh);

bool findVertexChainPathRecursive(
        const size_t& vCurrentId,
        const size_t& vStartId,
        const std::vector<std::vector<size_t>>& vertexNextMap,
        std::vector<size_t>& nextConfiguration);

}
}

#include "quadutils.tpp"

#endif // QUADUTILS_H
