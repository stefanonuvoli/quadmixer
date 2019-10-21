#ifndef QUADBOOLEANOPERATION_H
#define QUADBOOLEANOPERATION_H

#include <vector>
#include <unordered_map>

#define DEFAULTMOTORCYCLE true
#define DEFAULTPATCHRETRACTION true
#define DEFAULTPATCHRETRACTIONNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGMAXBB 0.025
#define DEFAULTPRESERVENONQUADS true
#define DEFAULTMINRECTANGLEAREA 2
#define DEFAULTMINPATCHAREA 6
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED false
#define DEFAULTONLYQUADS true
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

enum Operation { UNION, INTERSECTION, DIFFERENCE };

enum ILPMethod { LEASTSQUARES, ABS };

struct OriginEntity {
    size_t meshId;
    size_t id;

    OriginEntity(const size_t& meshId, const size_t& id) : meshId(meshId), id(id) {

    }
};

struct SourceInfo {
    std::unordered_map<size_t, OriginEntity> oldFacesMap;
    std::vector<size_t> newFaces;
    std::unordered_map<size_t, OriginEntity> oldVerticesMap;
    std::vector<size_t> newVertices;
};

struct Parameters {
    bool motorcycle;
    bool patchRetraction;
    double patchRetractionNRing;
    int intersectionSmoothingIterations;
    double intersectionSmoothingNRing;
    double maxBB;
    bool preserveNonQuads;
    int minRectangleArea;
    int minPatchArea;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    bool onlyQuads;
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
    double resultSmoothingNRing;
    int resultSmoothingLaplacianIterations;
    double resultSmoothingLaplacianNRing;

    Parameters() {
        motorcycle = DEFAULTMOTORCYCLE;
        patchRetraction = DEFAULTPATCHRETRACTION;
        patchRetractionNRing = DEFAULTPATCHRETRACTIONNRING;
        intersectionSmoothingIterations = DEFAULTINTERSECTIONSMOOTHINGITERATIONS;
        intersectionSmoothingNRing = DEFAULTINTERSECTIONSMOOTHINGNRING;
        maxBB = DEFAULTINTERSECTIONSMOOTHINGMAXBB;
        preserveNonQuads = DEFAULTPRESERVENONQUADS;
        minRectangleArea = DEFAULTMINRECTANGLEAREA;
        minPatchArea = DEFAULTMINPATCHAREA;
        mergeQuads = DEFAULTMERGEQUADS;
        deleteSmall = DEFAULTDELETESMALL;
        deleteNonConnected = DEFAULTDELETENONCONNECTED;
        onlyQuads = DEFAULTONLYQUADS;
        ilpMethod = DEFAULTILPMETHOD;
        alpha = DEFAULTALPHA;
        beta = DEFAULTBETA;
        initialRemeshing = DEFAULTINITIALREMESHING;
        initialRemeshingEdgeFactor = DEFAULTEDGEFACTOR;
        reproject = DEFAULTREPROJECT;
        splitConcaves = DEFAULTSPLITCONCAVES;
        finalSmoothing = DEFAULTFINALSMOOTHING;
        chartSmoothingIterations = DEFAULTCHARTSMOOTHINGITERATIONS;
        quadrangulationSmoothingIterations = DEFAULTQUADRANGULATIONSMOOTHINGITERATIONS;
        resultSmoothingIterations = DEFAULTRESULTSMOOTHINGITERATIONS;
        resultSmoothingNRing = DEFAULTRESULTSMOOTHINGNRING;
        resultSmoothingLaplacianIterations = DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS;
        resultSmoothingLaplacianNRing = DEFAULTRESULTSMOOTHINGLAPLACIANNRING;
    }
};

}

#endif // QUADBOOLEANOPERATION_H
