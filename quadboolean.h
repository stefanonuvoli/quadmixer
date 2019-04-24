#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"

#define DEFAULTMOTORCYCLE true
#define DEFAULTMINRECTANGLESIDE 2
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED true
#define DEFAULTALPHA 0.01
#define DEFAULTCHARTSMOOTHINGITERATION 5
#define DEFAULTMESHSMOOTHINGITERATION 15

namespace QuadBoolean {

template<class PolyMeshType, class TriangleMeshType = PolyMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result);

template<class PolyMeshType, class TriangleMeshType>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const bool motorcycle,
        const size_t minRectangleSide,
        const bool mergeQuads,
        const bool deleteSmall,
        const bool deleteNonConnected,
        const double alpha,
        int chartSmoothingIterations,
        int meshSmoothingIterations);

}

#include "quadboolean.tpp"

#endif // QUADBOOLEAN_H
