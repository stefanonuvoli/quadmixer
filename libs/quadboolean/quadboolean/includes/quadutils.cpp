#include "quadutils.h"

#include <vector>

#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>
#include <vcg/complex/algorithms/geodesic.h>
#include <vcg/space/distance3.h>

namespace QuadBoolean {
namespace internal {


bool findVertexChainPathRecursive(
        const size_t& vCurrentId,
        const size_t& vStartId,
        const std::vector<std::vector<size_t>>& vertexNextMap,
        std::vector<size_t>& nextConfiguration);

template<class PolyMeshType>
void LaplacianPos(PolyMeshType &poly_m,std::vector<typename PolyMeshType::CoordType> &AvVert)
{
    //cumulate step
    AvVert.clear();
    AvVert.resize(poly_m.vert.size(),typename PolyMeshType::CoordType(0,0,0));
    std::vector<typename PolyMeshType::ScalarType> AvSum(poly_m.vert.size(),0);
    for (size_t i=0;i<poly_m.face.size();i++)
        for (size_t j=0;j<(size_t)poly_m.face[i].VN();j++)
        {
            //get current vertex
            typename PolyMeshType::VertexType *currV=poly_m.face[i].V(j);
            //and its position
            typename PolyMeshType::CoordType currP=currV->P();
            //cumulate over other positions
            typename PolyMeshType::ScalarType W=vcg::PolyArea(poly_m.face[i]);
            //assert(W!=0);
            for (size_t k=0;k<(size_t)poly_m.face[i].VN();k++)
            {
                if (k==j) continue;
                int IndexV=vcg::tri::Index(poly_m,poly_m.face[i].V(k));
                AvVert[IndexV]+=currP*W;
                AvSum[IndexV]+=W;
            }
        }

    //average step
    for (size_t i=0;i<poly_m.vert.size();i++)
    {
        if (AvSum[i]==0)continue;
        AvVert[i]/=AvSum[i];
    }
}

template <class PolyMeshType>
void LaplacianGeodesic(
        PolyMeshType &poly_m,
        int nstep,
        const double maxDistance,
        const double minDumpS,
        std::vector<size_t>& smoothedVertices)
{
    std::vector<typename PolyMeshType::VertexPointer> seedVec;
    for (int i = 0; i < poly_m.vert.size(); i++) {
        if (poly_m.vert[i].IsS()) {
            seedVec.push_back(&poly_m.vert[i]);
        }
    }
    vcg::tri::UpdateQuality<PolyMeshType>::VertexConstant(poly_m, 0);
    vcg::tri::EuclideanDistance<PolyMeshType> ed;
    vcg::tri::UpdateTopology<PolyMeshType>::VertexFace(poly_m);
    vcg::tri::Geodesic<PolyMeshType>::Compute(poly_m,seedVec, ed);

    smoothedVertices.clear();
    std::vector<double> DampS(poly_m.vert.size());
    for (int i = 0; i < poly_m.vert.size(); i++) {        
        if (!poly_m.vert[i].IsD()) {
            if (poly_m.vert[i].Q() < maxDistance) {
                smoothedVertices.push_back(i);
                DampS[i] = poly_m.vert[i].Q() / maxDistance;
                assert(DampS[i] >= 0 && DampS[i] <= 1);
                DampS[i] = minDumpS + DampS[i]*(1-minDumpS);
            }
            else {
                DampS[i] = std::numeric_limits<double>::max();
            }
        }
    }

    for (int s=0;s<nstep;s++)
    {
        std::vector< typename PolyMeshType::CoordType> AvVert;
        LaplacianPos(poly_m,AvVert);

        for (size_t i=0;i<poly_m.vert.size();i++)
        {
            if (!poly_m.vert[i].IsD() && DampS[i] <= 1) {
                poly_m.vert[i].P()=poly_m.vert[i].P()*DampS[i]+
                        AvVert[i]*(1-DampS[i]);
            }
        }
    }
}


template <class PolyMeshType>
bool isTriangleMesh(PolyMeshType& mesh) {
    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (mesh.face[i].VN() != 3)
            return false;
    }

    return true;
}

template <class PolyMeshType>
bool isQuadMesh(PolyMeshType& mesh) {
    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (mesh.face[i].VN() != 4)
            return false;
    }

    return true;
}

template <class PolyMeshType>
std::vector<int> splitFacesInTriangles(PolyMeshType& mesh) {
    typedef typename PolyMeshType::VertexType VertexType;

    size_t numFaces = mesh.face.size();
    std::vector<int> birthFace(mesh.face.size(), -1);

    for (size_t i = 0; i < numFaces; i++) {
        size_t prevSize = mesh.face.size();

        if (mesh.face[i].VN() == 4) {
            VertexType* v[4];
            v[0] = mesh.face[i].V(0);
            v[1] = mesh.face[i].V(1);
            v[2] = mesh.face[i].V(2);
            v[3] = mesh.face[i].V(3);

            size_t startIndex = 0;
            if ((v[0]->P() - v[2]->P()).Norm() > (v[1]->P() - v[3]->P()).Norm()) {
                startIndex++;
            }

            vcg::tri::Allocator<PolyMeshType>::AddFaces(mesh,1);
            mesh.face.back().Alloc(3);
            mesh.face.back().V(0) = v[startIndex];
            mesh.face.back().V(1) = v[(startIndex + 1)%4];
            mesh.face.back().V(2) = v[(startIndex + 2)%4];

            mesh.face[i].Dealloc();
            mesh.face[i].Alloc(3);
            mesh.face[i].V(0)=v[(startIndex + 2)%4];
            mesh.face[i].V(1)=v[(startIndex + 3)%4];
            mesh.face[i].V(2)=v[(startIndex + 4)%4];
        }
        else if (mesh.face[i].VN() > 4) {
            vcg::PolygonalAlgorithm<PolyMeshType>::Triangulate(mesh, i);
        }

        birthFace[i] = static_cast<int>(i);

        for (size_t j = 0; j < mesh.face.size() - prevSize; j++)
            birthFace.push_back(static_cast<int>(i));
    }

#ifndef NDEBUG
    vcg::tri::io::ExporterOBJ<PolyMeshType>::Save(mesh, "res/triang.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

    return birthFace;
}

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh, const std::vector<size_t>& faces) {
    typename PolyMeshType::ScalarType length = 0;
    size_t numEdges = 0;
    for (size_t fId : faces) {
        for (size_t j=0;j<mesh.face[fId].VN();j++)
        {
            size_t index0=vcg::tri::Index(mesh,mesh.face[fId].V0(j));
            size_t index1=vcg::tri::Index(mesh,mesh.face[fId].V1(j));
            typename PolyMeshType::CoordType p0=mesh.vert[index0].P();
            typename PolyMeshType::CoordType p1=mesh.vert[index1].P();
            length += (p0-p1).Norm();
            numEdges++;
        }

    }

    length /= numEdges;

    return length;
}

template<class PolyMeshType>
typename PolyMeshType::ScalarType averageEdgeLength(PolyMeshType& mesh) {
    std::vector<size_t> faces;

    for (size_t i=0;i<mesh.face.size();i++) {
        if (!mesh.face[i].IsD()) {
            faces.push_back(i);
        }
    }

    return averageEdgeLength(mesh, faces);
}

template <class PolyMeshType>
class OrientFaces
{
    typedef typename PolyMeshType::FaceType FaceType;
    typedef typename PolyMeshType::VertexType VertexType;
    typedef typename PolyMeshType::CoordType CoordType;
    typedef typename PolyMeshType::ScalarType ScalarType;

public:

    static void InvertFace(FaceType &f0)
    {
        std::vector<VertexType*> faceVert;
        for (int i=0;i<f0.VN();i++)
            faceVert.push_back(f0.V(i));
        std::reverse(faceVert.begin(),faceVert.end());
        for (int i=0;i<f0.VN();i++)
            f0.V(i)=faceVert[i];
    }

private:

    static bool IsCoherent(const FaceType &f0,const FaceType &f1,const int IndexE)
    {
        assert(f0.cFFp(IndexE)==&f1);
        assert(&f0!=&f1);
        const VertexType *v0=f0.cV(IndexE);
        const VertexType *v1=f0.cV((IndexE+1)%f0.VN());
        int IndexEopp=f0.cFFi(IndexE);
        assert(f1.cFFp(IndexEopp)==&f0);
        const VertexType *v2=f1.cV(IndexEopp);
        const VertexType *v3=f1.cV((IndexEopp+1)%f1.VN());
        if (v0==v2){assert(v1==v3);return false;}
        assert(v0==v3);
        assert(v1==v2);
        return true;
    }

    static void InvertFaces(PolyMeshType &PolyM,
                            const std::vector<int> &ToInvert)
    {
        for (size_t i=0;i<ToInvert.size();i++)
            InvertFace(PolyM.face[ToInvert[i]]);
    }

    static void PropagateFrom(PolyMeshType &PolyM,int &fI0,
                       std::vector<int> &OrientSet0,
                       std::vector<int> &OrientSet1)
    {
        OrientSet0.clear();
        OrientSet1.clear();

        std::vector<int> CoherentSet(PolyM.face.size(),-1);
        CoherentSet[fI0]=0;

        assert(!PolyM.face[fI0].IsS());

        std::vector<int> exploreStack;
        exploreStack.push_back(fI0);
        do{
            //get from the stack and set as explored
            int currF=exploreStack.back();
            exploreStack.pop_back();
            if(PolyM.face[currF].IsS())continue;//already explored
            PolyM.face[currF].SetS();

            //put in the right coherent set
            int currSet=CoherentSet[currF];
            if (currSet==0)
                OrientSet0.push_back(currF);
            else
                OrientSet1.push_back(currF);
            for (int i=0;i<PolyM.face[currF].VN();i++)
            {
                FaceType *f0=&PolyM.face[currF];
                FaceType *f1=PolyM.face[currF].FFp(i);
                if (f1==f0)continue;//border

                int IndexF1=vcg::tri::Index(PolyM,f1);
                if(PolyM.face[IndexF1].IsS())continue;

                exploreStack.push_back(IndexF1);//add to the stack

                //either is coherent or the opposite
                if (IsCoherent(*f0,*f1,i))
                    CoherentSet[IndexF1]=currSet;
                else
                    CoherentSet[IndexF1]=(currSet+1)%2;
            }
        }while(!exploreStack.empty());
    }

public:

    static void AutoOrientFaces(PolyMeshType &PolyM)
    {
        vcg::tri::UpdateTopology<PolyMeshType>::FaceFace(PolyM);
        vcg::tri::UpdateFlags<PolyMeshType>::FaceClearS(PolyM);
        for (int i=0;i<(int)PolyM.face.size();i++)
        {
            if (PolyM.face[i].IsS())continue;
            std::cout<<"Reoriented face."<<std::endl;
            std::vector<int> OrientSet0,OrientSet1;
            PropagateFrom(PolyM,i,OrientSet0,OrientSet1);
            if (OrientSet0.size()<OrientSet1.size())
                InvertFaces(PolyM,OrientSet0);
            else
                InvertFaces(PolyM,OrientSet1);
        }
    }

};


inline bool findVertexChainPathRecursive(
        const size_t& vCurrentId,
        const size_t& vStartId,
        const std::vector<std::vector<size_t>>& vertexNextMap,
        std::vector<size_t>& nextConfiguration)
{
    if (vCurrentId == vStartId)
        return true;

    for (size_t i = 0; i < vertexNextMap[vCurrentId].size(); i++) {
        nextConfiguration[vCurrentId] = i;
        if (findVertexChainPathRecursive(vertexNextMap[vCurrentId][i], vStartId, vertexNextMap, nextConfiguration)) {
            return true;
        }
    }
    return false;
}

inline std::vector<size_t> findVertexChainPath(
        const size_t& vCurrentId,
        const std::vector<std::vector<size_t>>& vertexNextMap)
{
    std::vector<size_t> nextConfiguration(vertexNextMap.size());

    findVertexChainPathRecursive(vCurrentId, vCurrentId, vertexNextMap, nextConfiguration);

    return nextConfiguration;
}


}
}
