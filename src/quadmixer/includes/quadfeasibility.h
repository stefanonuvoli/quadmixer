#ifndef QUADFEASIBILITY_H
#define QUADFEASIBILITY_H

namespace QuadBoolean {
namespace internal {

enum FeasibilityResult { NonConsistant, AlreadyOk, SolvedQuadOnly, SolvedQuadDominant, NonSolved };

template <class PolyMeshType, class TriangleMeshType>
FeasibilityResult solveFeasibility(
        PolyMeshType& preservedSurface,
        TriangleMeshType& newSurface,
        const bool polychordSolver,
        const bool splitSolver);

}
}

#include "quadfeasibility.cpp"

#endif // QUADFEASIBILITY_H
