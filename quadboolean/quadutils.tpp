#include "quadutils.h"


#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/quality.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>

#include <vector>

namespace QuadBoolean {
namespace internal {


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
std::vector<int> splitQuadInTriangle(PolyMeshType& mesh) {
    typedef typename PolyMeshType::VertexType VertexType;

    std::vector<int> birthQuad(mesh.face.size()*2, -1);

    for (size_t i = 0; i < mesh.face.size(); i++) {
        size_t vSize = mesh.face[i].VN();

        if (vSize == 4) {
            VertexType* v[4];
            v[0] = mesh.face[i].V(0);
            v[1] = mesh.face[i].V(1);
            v[2] = mesh.face[i].V(2);
            v[3] = mesh.face[i].V(3);

            size_t startIndex = 0;
            if ((v[0]->P() - v[2]->P()).Norm() > (v[1]->P() - v[3]->P()).Norm()) {
                startIndex++;
            }

            size_t newFaceId = mesh.face.size();

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

            birthQuad[i] = i;
            birthQuad[newFaceId] = i;
        }
    }

    return birthQuad;
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

    static void InvertFace(FaceType &f0)
    {
        std::vector<VertexType*> faceVert;
        for (int i=0;i<f0.VN();i++)
            faceVert.push_back(f0.V(i));
        std::reverse(faceVert.begin(),faceVert.end());
        for (int i=0;i<f0.VN();i++)
            f0.V(i)=faceVert[i];
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

}
}
