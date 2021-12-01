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
