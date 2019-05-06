#ifndef QUADUTILS_H
#define QUADUTILS_H

#include "meshtypes.h"

namespace QuadBoolean {
namespace internal {

template <class PolyMeshType>
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh);

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh);

}
}

#include "quadutils.tpp"

#endif // QUADUTILS_H
