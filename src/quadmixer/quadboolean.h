#ifndef QUADBOOLEAN_H
#define QUADBOOLEAN_H

#include "defaultmeshtypes.h"

#include "includes/quadbooleancommon.h"
#include "includes/quadbooleansteps.h"


namespace QuadBoolean {

static SourceInfo dummyInfo;

template<class PolyMeshType = PolyMesh, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        SourceInfo& info = dummyInfo);

template<class PolyMeshType = PolyMesh, class TriangleMeshType = TriangleMesh>
void quadBoolean(
        PolyMeshType& mesh1,
        PolyMeshType& mesh2,
        const Operation& operation,
        PolyMeshType& result,
        const Parameters& parameters,
        SourceInfo& info = dummyInfo);

}

#include "quadboolean.cpp"

#endif // QUADBOOLEAN_H
