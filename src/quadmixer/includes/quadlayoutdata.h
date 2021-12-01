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

#ifndef QUADLAYOUTDATA_H
#define QUADLAYOUTDATA_H

#include <vector>
#include <set>

#include <Eigen/Core>

#include <vcg/complex/complex.h>

#define MINIMUMSIDELENGTHDIFFERENCE 3
#define MINIMUMSIDEMINLENGTH 2
#define MINIMUMSIDEMAXLENGTH 5

namespace QuadBoolean {
namespace internal {

template<class PolyMeshType>
struct QuadLayoutPatch {
    std::vector<size_t> faces;
    vcg::face::Pos<typename PolyMeshType::FaceType> startPos;
    bool isQuad;
    size_t sizeX;
    size_t sizeY;
};
template<class PolyMeshType>
struct QuadLayoutData {
    std::set<int> labels;
    std::vector<QuadLayoutPatch<PolyMeshType>> quadPatches;
};


template<class PolyMeshType>
QuadLayoutData<PolyMeshType> getQuadLayoutData(
        PolyMeshType& mesh,
        const std::vector<int>& faceLabel);

}
}

#include "quadlayoutdata.cpp"

#endif // QUADLAYOUTDATA_H
