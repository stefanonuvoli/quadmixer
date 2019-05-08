#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"
#include "quadboolean/quadilp.h"

#include "meshtypes.h"

#define DEFAULTMOTORCYCLE false
#define DEFAULTMINRECTANGLEAREA 2
#define DEFAULTMINPATCHAREA 6
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGAVGNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGMAXBB 0.1
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED false
#define DEFAULTILPMETHOD QuadBoolean::ILPMethod::LEASTSQUARES
#define DEFAULTALPHA 0.5
#define DEFAULTBETA 0.7
#define DEFAULTINITIALREMESHING false
#define DEFAULTEDGEFACTOR 2
#define DEFAULTREPROJECT true
#define DEFAULTSPLITCONCAVES false
#define DEFAULTFINALSMOOTHING true
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTMESHSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGAVGNRING 5
#define DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS 5
#define DEFAULTRESULTSMOOTHINGLAPLACIANAVGNRING 3

namespace QuadBoolean {

struct Parameters {
    bool motorcycle;
    int intersectionSmoothingIterations;
    int intersectionSmoothingAVGNRing;
    double intersectionSmoothingMaxBB;
    size_t minRectangleArea;
    size_t minPatchArea;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    ILPMethod ilpMethod;
    double alpha;
    double beta;
    bool initialRemeshing;
    double edgeFactor;
    bool reproject;
    bool splitConcaves;
    bool finalSmoothing;
    int chartSmoothingIterations;
    int meshSmoothingIterations;
    int resultSmoothingIterations;
    int resultSmoothingAVGNRing;    
    int resultSmoothingLaplacianIterations;
    int resultLaplacianAVGNRing;

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
