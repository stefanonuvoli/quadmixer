#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "defaultmeshtypes.h"

#include "includes/quadbooleanoperation.h"
#include "includes/quadbooleansteps.h"
#include "includes/quadilp.h"

#define DEFAULTMOTORCYCLE false
#define DEFAULTMINRECTANGLEAREA 2
#define DEFAULTMINPATCHAREA 6
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGMAXBB 0.025
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED false
#define DEFAULTILPMETHOD QuadBoolean::ILPMethod::LEASTSQUARES
#define DEFAULTALPHA 0.5
#define DEFAULTBETA 1
#define DEFAULTINITIALREMESHING true
#define DEFAULTEDGEFACTOR 1
#define DEFAULTREPROJECT true
#define DEFAULTSPLITCONCAVES false
#define DEFAULTFINALSMOOTHING true
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTQUADRANGULATIONSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGNRING 3
#define DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS 2
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

template<class PolyMeshType = PolyMesh, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result);

template<class PolyMeshType = PolyMesh, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const Parameters& parameters);

}

#include "quadboolean.tpp"

#endif // QUADBOOLEAN_H
