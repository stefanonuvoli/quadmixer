#ifndef EVEN_PAIRING
#define EVEN_PAIRING

#include<common_mesh_functions.h>
#include<vcg/complex/algorithms/polygon_polychord_collapse.h>

namespace QuadBoolean {
namespace internal {

template <class MeshType>
class EvenPairing
{
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;

    //the input mesh to trace
    MeshType &meshTri;
    MeshType &meshQuad;
    std::vector<std::vector<size_t> > Components;
    std::vector<std::set<CoordType> > BorderV;
    std::vector<bool> EvenSubd;

public:
    enum ResultType{NonConsistent,AlreadyOk,Solved,NonSolved};

private:
    //std::deque< vcg::face::Pos<FaceType> > polychords;
    std::vector<std::vector<std::pair<size_t,size_t > > > PolyChoordsQuad;

    std::vector<std::vector<vcg::face::Pos<FaceType> > > PolyChoords;
    std::vector<std::pair<ScalarType,size_t> > PolyCoordLenght;

    ScalarType PolyChLength(std::vector<vcg::face::Pos<FaceType> > &PolyCh)
    {
        ScalarType len=0;
        for (size_t i=0;i<PolyCh.size();i++)
            len+=(PolyCh[i].V()->P()-PolyCh[i].VFlip()->P()).Norm();
        return len;
    }

    bool CheckConsistency()
    {
        std::set<CoordType> Pos0;
        for (size_t i=0;i<meshTri.vert.size();i++)
        {
            if (!meshTri.vert[i].IsB())continue;
            Pos0.insert(meshTri.vert[i].P());
        }
        std::set<CoordType> Pos1;
        for (size_t i=0;i<meshQuad.vert.size();i++)
        {
            if (!meshQuad.vert[i].IsB())continue;
            Pos1.insert(meshQuad.vert[i].P());
        }
        std::set<CoordType> result;
        std::set_difference(Pos0.begin(), Pos0.end(), Pos1.begin(), Pos1.end(),
                            std::inserter(result, result.end()));
        std::cout<<"Size 0 "<<Pos0.size()<<std::endl;
        std::cout<<"Size 1 "<<Pos1.size()<<std::endl;
        std::cout<<"Test "<<result.size()<<std::endl;
    }

public:

    ResultType SolvePairing(bool checkPair)
    {
        Components.clear();
        BorderV.clear();
        EvenSubd.clear();

        //update attributes
        std::cout<<"udpate Attr"<<std::endl;
        vcg::tri::UpdateTopology<MeshType>::FaceFace(meshTri);
        vcg::tri::UpdateTopology<MeshType>::FaceFace(meshQuad);

        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(meshTri);
        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromFaceAdj(meshTri);
        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(meshQuad);
        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromFaceAdj(meshQuad);

        if(checkPair)
            CheckConsistency();

        //first find the connected components
        std::cout<<"finding connected components"<<std::endl;
        FindConnectedComponents<MeshType>(meshTri,Components);
        std::cout<<"there are "<< Components.size()<<" conn components"<<std::endl;

        //if there is only one connected component is ok
        if (Components.size()==1)
            return AlreadyOk;

        //assemble border vertices
        std::cout<<"assemble borders"<<std::endl;
        BorderV.resize(Components.size());
        for (size_t i=0;i<Components.size();i++)
            for (size_t j=0;j<Components[i].size();j++)
            {
                size_t IndexF=Components[i][j];
                assert(IndexF>=0);
                assert(IndexF<meshTri.face.size());
                assert(meshTri.face[IndexF].VN()<=3);
                for (size_t k=0;k<meshTri.face[IndexF].VN();k++)
                {
                    if (!meshTri.face[IndexF].V(k)->IsB())continue;
                    BorderV[i].insert(meshTri.face[IndexF].P(k));
                }
            }

        //then count
        std::cout<<"counting"<<std::endl;
        EvenSubd.resize(Components.size(),false);
        for (size_t i=0;i<Components.size();i++)
        {
            std::cout<<"size "<<BorderV[i].size()<<std::endl;
            if (BorderV[i].size()%2==0)EvenSubd[i]=true;
        }

        size_t NumOdd=0;
        for (size_t i=0;i<EvenSubd.size();i++)
            if (BorderV[i].size()%2>0)NumOdd++;

        //everything is even
        if (NumOdd==0)return AlreadyOk;

        //everything is even
        if (NumOdd%2)return NonConsistent;


        PolyChoordsQuad.clear();
        for (size_t i=0;i<meshQuad.face.size();i++)
        {
           assert(meshQuad.face[i].VN()==4);
           for (size_t j=0;j<meshQuad.face[i].VN();j++)
           {
               if (!vcg::face::IsBorder(meshQuad.face[i],j))continue;
               PolyChoordsQuad.resize(PolyChoordsQuad.size()+1);
               PolyChoordsQuad.back().push_back(std::pair<size_t,size_t>(i,j));
               while (true)
               {
                   size_t CurrFI=PolyChoordsQuad.back().back().first;
                   size_t CurrFE=PolyChoordsQuad.back().back().second;
                   FaceType *CurrF=&meshQuad.face[CurrFI];
                   size_t NextE0=(CurrFE+2)%4;
                   if (vcg::face::IsBorder(*CurrF,NextE0))break;

                   FaceType *NextF=CurrF->FFp(NextE0);
                   size_t NextE=CurrF->FFi(NextE0);
                   size_t IndexNextF=vcg::tri::Index(meshQuad,NextF);
                   PolyChoordsQuad.back().push_back(std::pair<size_t,size_t>(IndexNextF,NextE));
               }
           }
        }


        std::cout<<"sorting"<<std::endl;
        PolyCoordLenght.clear();
        for (size_t i=0;i<PolyChoordsQuad.size();i++)
            PolyCoordLenght.push_back(std::pair<ScalarType,size_t>(PolyChoordsQuad[i].size(),i));

        std::sort(PolyCoordLenght.begin(),PolyCoordLenght.end());

        std::cout<<"solving"<<std::endl;
        std::vector<size_t> Choosen;
        //then solve the association
        for (size_t i=0;i<PolyCoordLenght.size();i++)
        {
            size_t IndexTrace=PolyCoordLenght[i].second;

            //take the first
            size_t IndexF0=PolyChoordsQuad[IndexTrace][0].first;
            size_t IndexE0=PolyChoordsQuad[IndexTrace][0].second;
            assert(vcg::face::IsBorder(meshQuad.face[IndexF0],IndexE0));

            //take the laset
            //std::cout<<"Size "<<PolyChoordsQuad[IndexTrace].size()<<std::endl;
            size_t IndexF1=PolyChoordsQuad[IndexTrace].back().first;
            size_t IndexE1=PolyChoordsQuad[IndexTrace].back().second;
            //assert(vcg::face::IsBorder(meshQuad.face[IndexF1],IndexE1));
            //go on the other side
            //std::cout<<"E0 "<<IndexE1<<std::endl;
            IndexE1=(IndexE1+2)%4;
            //std::cout<<"E1 "<<IndexE1<<std::endl;
            assert(vcg::face::IsBorder(meshQuad.face[IndexF1],IndexE1));

            VertexType *v0=meshQuad.face[IndexF0].V(IndexE0);
            VertexType *v1=meshQuad.face[IndexF1].V(IndexE1);
            assert(v0->IsB());
            assert(v1->IsB());
            CoordType Pos0=v0->P();
            CoordType Pos1=v1->P();
            //get the patches
            int IndexP0=-1;
            int IndexP1=-1;

            for (size_t j=0;j<BorderV.size();j++)
                if (BorderV[j].count(Pos0)>0)IndexP0=j;

            assert(IndexP0>=0);

            for (size_t j=0;j<BorderV.size();j++)
                if (BorderV[j].count(Pos1)>0)IndexP1=j;

            assert(IndexP1>=0);

            if (IndexP0==IndexP1)continue;

            std::cout<<"Found!"<<std::endl;

            Choosen.push_back(IndexTrace);

            EvenSubd[IndexP0]=true;
            EvenSubd[IndexP1]=true;
            bool SolvedAll=true;
            for (size_t j=0;j<EvenSubd.size();j++)
                SolvedAll&=EvenSubd[j];

            if (SolvedAll)
                break;
        }
        bool SolvedAll=true;
        for (size_t j=0;j<EvenSubd.size();j++)
            SolvedAll&=EvenSubd[j];

        if (!SolvedAll)return NonSolved;
        std::cout<<"Splitting"<<std::endl;

        std::map<std::pair<CoordType,CoordType>,CoordType > SplittedBoundary;

        for (size_t j=0;j<Choosen.size();j++)
        {
            size_t IndexTrace=Choosen[j];
            size_t IndexF0=PolyChoordsQuad[IndexTrace][0].first;
            size_t IndexE0=PolyChoordsQuad[IndexTrace][0].second;
            vcg::face::Pos<FaceType> StartPos(&meshQuad.face[IndexF0],IndexE0);

            CoordType Pos0Start=meshQuad.face[IndexF0].P0(IndexE0);
            CoordType Pos1Start=meshQuad.face[IndexF0].P1(IndexE0);

            size_t IndexF1=PolyChoordsQuad[IndexTrace].back().first;
            size_t IndexE1=PolyChoordsQuad[IndexTrace].back().second;
            IndexE1=(IndexE1+2)%4;

            CoordType Pos0End=meshQuad.face[IndexF1].P0(IndexE1);
            CoordType Pos1End=meshQuad.face[IndexF1].P1(IndexE1);

            size_t IndexV0=meshQuad.vert.size();
            vcg::tri::PolychordCollapse<MeshType>::SplitPolychord(meshQuad,StartPos,2);
            size_t IndexV1=meshQuad.vert.size()-1;

            std::pair<CoordType,CoordType> key0(std::min(Pos0Start,Pos1Start),
                                                std::max(Pos0Start,Pos1Start));
            std::pair<CoordType,CoordType> key1(std::min(Pos0End,Pos1End),
                                                std::max(Pos0End,Pos1End));

            SplittedBoundary[key0]=meshQuad.vert[IndexV0].P();
            SplittedBoundary[key1]=meshQuad.vert[IndexV1].P();
        }

        //then cycle over all triangles and split
        size_t size0=meshTri.face.size();
        size_t numSplit=0;
        for (size_t i=0;i<size0;i++)
        {
           assert(meshTri.face[i].VN()==3);
           for (size_t j=0;j<meshTri.face[i].VN();j++)
           {
               if (!vcg::face::IsBorder(meshTri.face[i],j))continue;
               CoordType Pos0=meshTri.face[i].P0(j);
               CoordType Pos1=meshTri.face[i].P1(j);
               std::pair<CoordType,CoordType> key(std::min(Pos0,Pos1),
                                                  std::max(Pos0,Pos1));
               if (SplittedBoundary.count(key)==0)continue;
               numSplit++;

               //split the triangle
               CoordType Pos=SplittedBoundary[key];
               vcg::tri::Allocator<MeshType>::AddVertex(meshTri,Pos);
               vcg::tri::Allocator<MeshType>::AddFaces(meshTri,1);
               meshTri.face.back().Alloc(3);
               VertexType *OldV1=meshTri.face[i].V1(j);
               VertexType *OldV2=meshTri.face[i].V2(j);
               meshTri.face[i].V1(j)=&meshTri.vert.back();
               meshTri.face.back().V0(j)=&meshTri.vert.back();
               meshTri.face.back().V1(j)=OldV1;
               meshTri.face.back().V2(j)=OldV2;
           }
        }
        assert(numSplit==SplittedBoundary.size());

        if(checkPair)
            CheckConsistency();

        return Solved;
    }

    EvenPairing(MeshType &_meshTri,MeshType &_meshQuad):meshTri(_meshTri),meshQuad(_meshQuad)
    {}
};

}
}

#endif
