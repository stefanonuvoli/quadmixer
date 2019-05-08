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

}
}

#include "quadutils.tpp"

#endif // QUADUTILS_H
