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

#ifndef QUADBOOLEAN_DEFAULTMESHTYPES_H
#define QUADBOOLEAN_DEFAULTMESHTYPES_H

#include <vcg/complex/complex.h>

namespace QuadBoolean {

/* ----- Polygon mesh ----- */

class PolyVertex;
class PolyFace;
class PolyEdge;

struct MyPolyTypes : public vcg::UsedTypes<
        vcg::Use<PolyVertex>::AsVertexType,
        vcg::Use<PolyEdge>::AsEdgeType,
        vcg::Use<PolyFace>::AsFaceType>{};

class PolyVertex : public vcg::Vertex<MyPolyTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Normal3d,
        vcg::vertex::Color4b,
        vcg::vertex::Qualityd,
        vcg::vertex::BitFlags,
        vcg::vertex::VFAdj,
        vcg::vertex::Mark,
        vcg::vertex::CurvatureDird>{};

class PolyFace : public vcg::Face<
        MyPolyTypes,
        vcg::face::PolyInfo,
        vcg::face::VertexRef,
        vcg::face::Normal3d,
        vcg::face::Color4b,
        vcg::face::Qualityd,
        vcg::face::BitFlags,
        vcg::face::PFVAdj,
        vcg::face::PFFAdj,
        vcg::face::PVFAdj,
        vcg::face::CurvatureDird,
        vcg::face::Mark,
        vcg::face::WedgeTexCoord2d> {};

class PolyEdge : public vcg::Edge<
        MyPolyTypes,
        vcg::edge::VertexRef,
        vcg::edge::BitFlags> {};

class PolyMesh : public vcg::tri::TriMesh<
        std::vector<PolyVertex>,
        std::vector<PolyEdge>,
        std::vector<PolyFace>> {};


///* ----- Triangle mesh ----- */

class TriangleVertex;
class TriangleFace;
struct MyTriangleTypes : public vcg::UsedTypes<
        vcg::Use<TriangleVertex>::AsVertexType,
        vcg::Use<TriangleFace>::AsFaceType>{};

class TriangleVertex : public vcg::Vertex<
        MyTriangleTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Normal3d,
        vcg::vertex::VFAdj,
        vcg::vertex::Color4b,
        vcg::vertex::Qualityd,
        vcg::vertex::BitFlags,
        vcg::vertex::CurvatureDird,
        vcg::vertex::Mark>{};

class TriangleFace : public vcg::Face<
        MyTriangleTypes,
        vcg::face::VertexRef,
        vcg::face::Normal3d,
        vcg::face::Color4b,
        vcg::face::Qualityd,
        vcg::face::BitFlags,
        vcg::face::FFAdj,
        vcg::face::VFAdj,
        vcg::face::CurvatureDird,
        vcg::face::Mark,
        vcg::face::WedgeTexCoord2d> {};

class TriangleMesh : public vcg::tri::TriMesh<
        std::vector<TriangleVertex>,
        std::vector<TriangleFace> > {};

}

#endif // QUADBOOLEAN_DEFAULTMESHTYPES_H
