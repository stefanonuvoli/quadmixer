#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"

#define DEFAULTMOTORCYCLE true
#define DEFAULTMINRECTANGLESIDE 2
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 3
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED true
#define DEFAULTALPHA 0.01
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTMESHSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 3

namespace QuadBoolean {

struct Parameters {
    bool motorcycle;
    int intersectionSmoothingIterations;
    size_t minRectangleSide;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    double alpha;
    int chartSmoothingIterations;
    int meshSmoothingIterations;
    int resultSmoothingIterations;

    Parameters();
};

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
        const Parameters& parameters);

}

#include "quadboolean.tpp"

#endif // QUADBOOLEAN_H
