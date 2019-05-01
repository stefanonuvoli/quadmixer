#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"

#define DEFAULTMOTORCYCLE true
#define DEFAULTMINRECTANGLESIDE 2
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED true
#define DEFAULTALPHA 0.01
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTMESHSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 3

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
        const int chartSmoothingIterations,
        const int meshSmoothingIterations,
        const int resultSmoothingIterations);

}

#include "quadboolean.tpp"

#endif // QUADBOOLEAN_H
