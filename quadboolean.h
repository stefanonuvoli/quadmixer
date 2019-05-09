#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "quadboolean/quadbooleansteps.h"
#include "quadboolean/quadilp.h"

#include "meshtypes.h"

#define DEFAULTMOTORCYCLE false
#define DEFAULTMINRECTANGLEAREA 2
#define DEFAULTMINPATCHAREA 6
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGNRING 5
#define DEFAULTINTERSECTIONSMOOTHINGMAXBB 0.05
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
#define DEFAULTRESULTSMOOTHINGNRING 5
#define DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS 5
#define DEFAULTRESULTSMOOTHINGLAPLACIANNRING 3

namespace QuadBoolean {

struct Parameters {
    bool motorcycle;
    int intersectionSmoothingIterations;
    int intersectionSmoothingNRing;
    double intersectionSmoothingMaxBB;
    int minRectangleArea;
    int minPatchArea;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    ILPMethod ilpMethod;
    double alpha;
    double beta;
    bool initialRemeshing;
    double initialRemeshingEdgeFactor;
    bool reproject;
    bool splitConcaves;
    bool finalSmoothing;
    int chartSmoothingIterations;
    int quadrangulationSmoothingIterations;
    int resultSmoothingIterations;
    int resultSmoothingNRing;
    int resultSmoothingLaplacianIterations;
    int resultSmoothingLaplacianNRing;

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
