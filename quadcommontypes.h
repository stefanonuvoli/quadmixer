#ifndef QUADCOMMONTYPES_H
#define QUADCOMMONTYPES_H

#include <vcg/complex/complex.h>

namespace QuadBoolean {

using namespace vcg;


/* ----- Polygon mesh ----- */

class PolyVertex;
class PolyFace;
class PolyEdge;

struct MyPolyTypes : public UsedTypes<
        Use<PolyVertex>::AsVertexType,
        Use<PolyEdge>::AsEdgeType,
        Use<PolyFace>::AsFaceType>{};

class PolyVertex : public vcg::Vertex<MyPolyTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Normal3d,
        vcg::vertex::Color4b,
        vcg::vertex::Qualityd,
        vcg::vertex::BitFlags,
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
        vcg::face::CurvatureDird> {};

class PolyEdge : public vcg::Edge<
        MyPolyTypes,
        vcg::edge::VertexRef,
        vcg::edge::BitFlags> {};

class PolyMesh : public vcg::tri::TriMesh<
        std::vector<PolyVertex>,
        std::vector<PolyEdge>,
        std::vector<PolyFace>> {};


///* ----- Triangle mesh ----- */

//class TriangleVertex;
//class TriangleFace;
//struct MyTriangleTypes : public UsedTypes<
//        Use<TriangleVertex>::AsVertexType,
//        Use<TriangleFace>::AsFaceType>{};
//class TriangleVertex : public vcg::Vertex<
//        MyTriangleTypes,
//        vcg::vertex::Coord3d,
//        vcg::vertex::Normal3d,
//        vcg::vertex::Color4b,
//        vcg::vertex::Qualityd,
//        vcg::vertex::BitFlags,
//        vcg::vertex::CurvatureDird>{};

//class TriangleFace : public vcg::Face<
//        MyTriangleTypes,
//        vcg::face::VertexRef,
//        vcg::face::Normal3d,
//        vcg::face::Color4b,
//        vcg::face::Qualityd,
//        vcg::face::BitFlags,
//        vcg::face::FFAdj,
//        vcg::face::CurvatureDird> {};

//class TriangleMesh : public vcg::tri::TriMesh<
//        std::vector<TriangleVertex>,
//        std::vector<TriangleFace> > {};

}

#endif // QUADCOMMONTYPES_H
