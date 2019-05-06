#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"

#include "meshtypes.h"

#define DEFAULTMOTORCYCLE true
#define DEFAULTMINRECTANGLEAREA 6
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGAVGNRING 3
#define DEFAULTMERGEQUADS false
#define DEFAULTDELETESMALL false
#define DEFAULTDELETENONCONNECTED false
#define DEFAULTALPHA 0.5
#define DEFAULTINITIALREMESHING true
#define DEFAULTEDGEFACTOR 2
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTMESHSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGAVGNRING 5

namespace QuadBoolean {

struct Parameters {
    bool motorcycle;
    int intersectionSmoothingIterations;
    int intersectionSmoothingAVGNRing;
    size_t minRectangleArea;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    double alpha;
    bool initialRemeshing;
    double edgeFactor;
    int chartSmoothingIterations;
    int meshSmoothingIterations;
    int resultSmoothingIterations;
    int resultSmoothingAVGNRing;

    Parameters();
};

template<class PolyMeshType, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result);

template<class PolyMeshType, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const Parameters& parameters);

}

#include "quadboolean.tpp"

#endif // QUADBOOLEAN_H
