#ifndef QUADUTILS_H
#define QUADUTILS_H

#include "meshtypes.h"

namespace QuadBoolean {
namespace internal {

template <class PolyMeshType>
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh);

}
}

#include "quadutils.tpp"

#endif // QUADUTILS_H
