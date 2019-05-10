// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "primitive.h"

namespace embree
{
  /*! Precalculated representation for 4 triangles. Stores for each
      triangle a base vertex, two edges, and the geometry normal to
      speed up intersection calculations. */
  struct Triangle4
  {
  public:

    /*! Default constructor. */
    __forceinline Triangle4 () {}

    /*! Construction from vertices and IDs. */
    __forceinline Triangle4 (const sse3f& v0, const sse3f& v1, const sse3f& v2, const ssei& geomIDs, const ssei& primIDs, const ssei& mask, const bool last)
      : v0(v0), e1(v0-v1), e2(v2-v0), Ng(cross(e1,e2)), geomIDs(geomIDs), primIDs(primIDs | (last << 31))
    {
#if defined(RTCORE_RAY_MASK)
      this->mask = mask;
#endif
    }

    /*! Returns if the specified triangle is valid. */
    __forceinline bool valid(const size_t i) const { 
      assert(i<4); 
      return geomIDs[i] != -1; 
    }

    /*! Returns a mask that tells which triangles are valid. */
    __forceinline sseb valid() const { return geomIDs != ssei(-1); }

    /*! Returns the number of stored triangles. */
    __forceinline size_t size() const {
      return bitscan(~movemask(valid()));
    }

    /*! Returns a hash number for the geometry */
    __forceinline size_t hash() const 
    {
      size_t hash = 0x3636;
      for (size_t i=0; i<sizeof(Triangle4)/4; i++)
	hash += ((uint32*)this)[i];
      return hash;
    }

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const 
    {
      sse3f p0 = v0;
      sse3f p1 = v0-e1;
      sse3f p2 = v0+e2;
      sse3f lower = min(p0,p1,p2);
      sse3f upper = max(p0,p1,p2);
      sseb mask = valid();
      lower.x = select(mask,lower.x,ssef(pos_inf));
      lower.y = select(mask,lower.y,ssef(pos_inf));
      lower.z = select(mask,lower.z,ssef(pos_inf));
      upper.x = select(mask,upper.x,ssef(neg_inf));
      upper.y = select(mask,upper.y,ssef(neg_inf));
      upper.z = select(mask,upper.z,ssef(neg_inf));
      return BBox3fa(Vec3fa(reduce_min(lower.x),reduce_min(lower.y),reduce_min(lower.z)),
                    Vec3fa(reduce_max(upper.x),reduce_max(upper.y),reduce_max(upper.z)));
    }

    /*! non temporal store */
    __forceinline static void store_nt(Triangle4* dst, const Triangle4& src)
    {
      store4f_nt(&dst->v0.x,src.v0.x);
      store4f_nt(&dst->v0.y,src.v0.y);
      store4f_nt(&dst->v0.z,src.v0.z);
      store4f_nt(&dst->e1.x,src.e1.x);
      store4f_nt(&dst->e1.y,src.e1.y);
      store4f_nt(&dst->e1.z,src.e1.z);
      store4f_nt(&dst->e2.x,src.e2.x);
      store4f_nt(&dst->e2.y,src.e2.y);
      store4f_nt(&dst->e2.z,src.e2.z);
      store4f_nt(&dst->Ng.x,src.Ng.x);
      store4f_nt(&dst->Ng.y,src.Ng.y);
      store4f_nt(&dst->Ng.z,src.Ng.z);
      store4i_nt(&dst->geomIDs,src.geomIDs);
      store4i_nt(&dst->primIDs,src.primIDs);
#if defined(RTCORE_RAY_MASK)
      store4i_nt(&dst->mask,src.mask);
#endif
    }

    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return (N+3)/4; }

    /*! checks if this is the last triangle in the list */
    __forceinline int last() const { 
      return primIDs[0] & 0x80000000; 
    }

    /*! returns the geometry IDs */
    template<bool list>
    __forceinline ssei geomID() const { 
      return geomIDs; 
    }
    template<bool list>
    __forceinline int geomID(const size_t i) const { 
      assert(i<4); return geomIDs[i]; 
    }

    /*! returns the primitive IDs */
    template<bool list>
    __forceinline ssei primID() const { 
      if (list) return primIDs & 0x7FFFFFFF; 
      else      return primIDs;
    }
    template<bool list>
    __forceinline int  primID(const size_t i) const { 
      assert(i<4); 
      if (list) return primIDs[i] & 0x7FFFFFFF; 
      else      return primIDs[i];
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<4 && prims; i++, prims++)
      {
	const PrimRef& prim = *prims;
	const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(this,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,list && !prims));
    }

    /*! fill triangle from triangle list */
    __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene, const bool list)
    {
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<4 && begin<end; i++, begin++)
      {
	const PrimRef& prim = prims[begin];
        const size_t geomID = prim.geomID();
        const size_t primID = prim.primID();
        const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(this,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,list && begin>=end));
    }
    
  public:
    sse3f v0;      //!< Base vertex of the triangles.
    sse3f e1;      //!< 1st edge of the triangles (v0-v1).
    sse3f e2;      //!< 2nd edge of the triangles (v2-v0).
    sse3f Ng;      //!< Geometry normal of the triangles.
    ssei geomIDs;   //!< user geometry ID
    ssei primIDs;   //!< primitive ID
#if defined(RTCORE_RAY_MASK)
    ssei mask;     //!< geometry mask
#endif
  };

  struct Triangle4Type : public PrimitiveType 
  {
    static Triangle4Type type;
    Triangle4Type ();
    size_t blocks(size_t x) const;
    size_t size(const char* This) const;
    size_t hash(const char* This, size_t num) const;
  };

  struct TriangleMeshTriangle4 : public Triangle4Type
  {
    static TriangleMeshTriangle4 type;
    BBox3fa update(char* prim, size_t num, void* geom) const;
  };
}
