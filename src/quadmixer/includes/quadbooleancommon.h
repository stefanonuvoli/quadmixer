/* Copyright(C) 2019


The authors of

QuadMixer: Layout Preserving Blending of Quadrilateral Meshes
SIGGRAPH Asia 2019


All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
****************************************************************************/

#ifndef QUADBOOLEANOPERATION_H
#define QUADBOOLEANOPERATION_H

#include <vector>
#include <unordered_map>

#include <quadretopology/includes/qr_ilp.h>

#define DEFAULTMOTORCYCLE true
#define DEFAULTPATCHRETRACTION true
#define DEFAULTPATCHRETRACTIONNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGITERATIONS 5
#define DEFAULTINTERSECTIONSMOOTHINGNRING 3
#define DEFAULTINTERSECTIONSMOOTHINGMAXBB 0.025
#define DEFAULTPRESERVEPOLYGONS1 true
#define DEFAULTPRESERVEPOLYGONS2 true
#define DEFAULTMINRECTANGLEAREA 2
#define DEFAULTMINPATCHAREA 6
#define DEFAULTMERGEQUADS true
#define DEFAULTDELETESMALL true
#define DEFAULTDELETENONCONNECTED false
#define DEFAULTPOLYCHORDSOLVER true
#define DEFAULTSPLITSOLVER true
#define DEFAULTILPMETHOD QuadRetopology::ILPMethod::LEASTSQUARES
#define DEFAULTALPHA 0.5
#define DEFAULTINITIALREMESHING true
#define DEFAULTEDGEFACTOR 1
#define DEFAULTREPROJECT true
#define DEFAULTSPLITCONCAVES false
#define DEFAULTFINALSMOOTHING true
#define DEFAULTCHARTSMOOTHINGITERATIONS 5
#define DEFAULTQUADRANGULATIONFIXEDSMOOTHINGITERATIONS 5
#define DEFAULTQUADRANGULATIONNONFIXEDSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGITERATIONS 5
#define DEFAULTRESULTSMOOTHINGNRING 3
#define DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS 2
#define DEFAULTRESULTSMOOTHINGLAPLACIANNRING 3

namespace QuadBoolean {

enum Operation { UNION, INTERSECTION, DIFFERENCE };

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
    bool preservePolygons1;
    bool preservePolygons2;
    int minRectangleArea;
    int minPatchArea;
    bool mergeQuads;
    bool deleteSmall;
    bool deleteNonConnected;
    bool polychordSolver;
    bool splitSolver;
    QuadRetopology::ILPMethod ilpMethod;
    double alpha;
    bool initialRemeshing;
    double initialRemeshingEdgeFactor;
    bool reproject;
    bool splitConcaves;
    bool finalSmoothing;
    int chartSmoothingIterations;
    int quadrangulationFixedSmoothingIterations;
    int quadrangulationNonFixedSmoothingIterations;
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
        preservePolygons1 = DEFAULTPRESERVEPOLYGONS1;
        preservePolygons2 = DEFAULTPRESERVEPOLYGONS2;
        minRectangleArea = DEFAULTMINRECTANGLEAREA;
        minPatchArea = DEFAULTMINPATCHAREA;
        mergeQuads = DEFAULTMERGEQUADS;
        deleteSmall = DEFAULTDELETESMALL;
        deleteNonConnected = DEFAULTDELETENONCONNECTED;
        polychordSolver = DEFAULTPOLYCHORDSOLVER;
        splitSolver = DEFAULTSPLITSOLVER;
        ilpMethod = DEFAULTILPMETHOD;
        alpha = DEFAULTALPHA;
        initialRemeshing = DEFAULTINITIALREMESHING;
        initialRemeshingEdgeFactor = DEFAULTEDGEFACTOR;
        reproject = DEFAULTREPROJECT;
        splitConcaves = DEFAULTSPLITCONCAVES;
        finalSmoothing = DEFAULTFINALSMOOTHING;
        chartSmoothingIterations = DEFAULTCHARTSMOOTHINGITERATIONS;
        quadrangulationFixedSmoothingIterations = DEFAULTQUADRANGULATIONFIXEDSMOOTHINGITERATIONS;
        quadrangulationNonFixedSmoothingIterations = DEFAULTQUADRANGULATIONNONFIXEDSMOOTHINGITERATIONS;
        resultSmoothingIterations = DEFAULTRESULTSMOOTHINGITERATIONS;
        resultSmoothingNRing = DEFAULTRESULTSMOOTHINGNRING;
        resultSmoothingLaplacianIterations = DEFAULTRESULTSMOOTHINGLAPLACIANITERATIONS;
        resultSmoothingLaplacianNRing = DEFAULTRESULTSMOOTHINGLAPLACIANNRING;
    }
};

}

#endif // QUADBOOLEANOPERATION_H
