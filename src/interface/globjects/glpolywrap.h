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

#ifndef GLPOLYWRAP_H
#define GLPOLYWRAP_H

#include <vector>

template<class MeshType>
class GLPolyWrap
{
public:
    GLPolyWrap();

    void GLDraw(bool wireframe = true, int wireframeSize = 1);

    MeshType* mesh;
    bool visible;

    bool wireframe;

    bool transformation;
    bool target1;
    bool target2;

    std::vector<typename MeshType::CoordType> pickedPoints;

    int name;
};

#include "glpolywrap.cpp"

#endif // GLPOLYWRAP_H
