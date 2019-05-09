#ifndef ENVELOPE_GENERATOR
#define ENVELOPE_GENERATOR

#include<vcg/complex/algorithms/update/color.h>
#include<vcg/complex/algorithms/update/quality.h>
#include<vcg/complex/algorithms/harmonic.h>
#include<vcg/complex/algorithms/smooth.h>
#include<vcg/complex/algorithms/hole.h>
#include<vcg/space/index/grid_util.h>
#include<vcg/complex/algorithms/geodesic.h>
#include<vcg/complex/algorithms/polygonal_algorithms.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>

template <class MeshType>
class EnvelopeGenerator
{
    //MeshType &mesh;
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;

    typedef std::pair<CoordType,CoordType> EdgeCoordKey;

    //std::map<EdgeCoordKey,CoordType> SplitOps;

    template<class MESH_TYPE>
    struct SplitLev : public   std::unary_function<vcg::face::Pos<typename MESH_TYPE::FaceType> ,  typename MESH_TYPE::CoordType >
    {
        typedef typename MESH_TYPE::VertexType VertexType;
        typedef typename MESH_TYPE::FaceType FaceType;
        typedef typename MESH_TYPE::CoordType CoordType;

        std::map<EdgeCoordKey,CoordType> *SplitOps;

        void operator()(typename MESH_TYPE::VertexType &nv,vcg::face::Pos<typename MESH_TYPE::FaceType>  ep)
        {
            VertexType* v0=ep.f->V0(ep.z);
            VertexType* v1=ep.f->V1(ep.z);

            assert(v0!=v1);

            CoordType Pos0=v0->P();
            CoordType Pos1=v1->P();

            EdgeCoordKey CoordK(std::min(Pos0,Pos1),std::max(Pos0,Pos1));

            assert(SplitOps->count(CoordK)>0);
            nv.P()=(*SplitOps)[CoordK];
            nv.Q()=0;
        }

        vcg::TexCoord2<ScalarType> WedgeInterp(vcg::TexCoord2<ScalarType> &t0,
                                               vcg::TexCoord2<ScalarType> &t1)
        {
            (void)t0;
            (void)t1;
            return (vcg::TexCoord2<ScalarType>(0,0));
        }

        SplitLev(std::map<EdgeCoordKey,CoordType> *_SplitOps){SplitOps=_SplitOps;}
        //SplitLevQ(){}
    };

    template <class MESH_TYPE>
    class EdgePred
    {
        typedef typename MESH_TYPE::VertexType VertexType;
        typedef typename MESH_TYPE::FaceType FaceType;
        typedef typename MESH_TYPE::ScalarType ScalarType;

        std::map<EdgeCoordKey,CoordType> *SplitOps;

    public:

        bool operator()(vcg::face::Pos<typename MESH_TYPE::FaceType> ep) const
        {
            VertexType* v0=ep.f->V0(ep.z);
            VertexType* v1=ep.f->V1(ep.z);

            assert(v0!=v1);

            CoordType Pos0=v0->P();
            CoordType Pos1=v1->P();

            EdgeCoordKey CoordK(std::min(Pos0,Pos1),std::max(Pos0,Pos1));

            return (SplitOps->count(CoordK)>0);
        }

        EdgePred(std::map<EdgeCoordKey,CoordType> *_SplitOps){SplitOps=_SplitOps;}
    };

    static void SmoothBorders(MeshType &to_smooth,ScalarType Damp)
    {
        std::vector<CoordType> AvPos(to_smooth.vert.size(),CoordType(0,0,0));
        std::vector<size_t> NumDiv(to_smooth.vert.size(),0);

        vcg::tri::UpdateFlags<MeshType>::VertexClearS(to_smooth);
        for (size_t i=0;i<to_smooth.face.size();i++)
            for (size_t j=0;j<to_smooth.face[i].VN();j++)
            {
                CoordType Pos0=to_smooth.face[i].P0(j);
                CoordType Pos1=to_smooth.face[i].P1(j);

                size_t IndexV0=vcg::tri::Index(to_smooth,to_smooth.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(to_smooth,to_smooth.face[i].V1(j));

                if (!vcg::face::IsBorder(to_smooth.face[i],j))continue;


                AvPos[IndexV0]+=Pos1;
                AvPos[IndexV1]+=Pos0;
                NumDiv[IndexV0]++;
                NumDiv[IndexV1]++;
            }

        for (size_t i=0;i<AvPos.size();i++)
        {

            if (NumDiv[i]==0)continue;
            AvPos[i]/=NumDiv[i];
            to_smooth.vert[i].P()=to_smooth.vert[i].P()*Damp+AvPos[i]*(1-Damp);
            //            else
            //                mesh.vert[i].SetS();
        }


    }

    static void UpdateAttributes(MeshType &curr_mesh)
    {
        vcg::tri::UpdateNormal<MeshType>::PerFaceNormalized(curr_mesh);
        vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(curr_mesh);
        vcg::tri::UpdateBounding<MeshType>::Box(curr_mesh);
        vcg::tri::UpdateTopology<MeshType>::FaceFace(curr_mesh);
        vcg::tri::UpdateTopology<MeshType>::VertexFace(curr_mesh);
        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(curr_mesh);
        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromNone(curr_mesh);
    }

    //    void Expand(MeshType &curr_mesh,MeshType &inflated,ScalarType Offset)
    //    {
    //        vcg::Point3i volumeDim;
    //        ScalarType voxelSize = curr_mesh.bbox.Diag()/50.0f;
    //        bool discretizeFlag = false;
    //        bool multiSampleFlag = false;
    //        bool absDistFlag = false;

    //        vcg::Box3<ScalarType> volumeBox = curr_mesh.bbox;
    //        volumeBox.Offset(volumeBox.Diag()/10.0f+Offset);
    //        vcg::BestDim(volumeBox , voxelSize, volumeDim );

    ////        qDebug("Resampling mesh using a volume of %i x %i x %i",volumeDim[0],volumeDim[1],volumeDim[2]);
    ////        qDebug("     VoxelSize is %f, offset is %f ", voxelSize,offsetThr);
    ////        qDebug("     Mesh Box is %f %f %f",mesh.bbox.DimX(),mesh.bbox.DimY(),mesh.bbox.DimZ() );

    //        MCTriMesh temp_mesh,inflated_temp;
    //        vcg::tri::Append<MCTriMesh,MeshType>::Mesh(temp_mesh,curr_mesh);
    //        vcg::tri::Resampler<MCTriMesh,MCTriMesh,MCTriMesh::ScalarType>::Resample(temp_mesh, inflated_temp, volumeBox, volumeDim, voxelSize, Offset,discretizeFlag,multiSampleFlag,absDistFlag/*, cb*/);

    //    }

    static ScalarType AverageEdgeSize(MeshType &curr_mesh)
    {
        ScalarType AVGEdge=0;
        size_t Num=0;
        for (size_t i=0;i<curr_mesh.face.size();i++)
            for (size_t j=0;j<curr_mesh.face[i].VN();j++)
            {
                size_t IndexV0=vcg::tri::Index(curr_mesh,curr_mesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(curr_mesh,curr_mesh.face[i].V1(j));
                CoordType P0=curr_mesh.vert[IndexV0].P();
                CoordType P1=curr_mesh.vert[IndexV1].P();
                AVGEdge+=(P0-P1).Norm();
                Num++;
            }
        AVGEdge/=Num;
        return (AVGEdge);
    }

    static void RemeshSelected(MeshType &curr_mesh,ScalarType EdgeStep)
    {
        typename vcg::tri::IsotropicRemeshing<MeshType>::Params Par;

        Par.swapFlag     = true;
        Par.collapseFlag = true;
        Par.smoothFlag=true;
        Par.projectFlag=false;
        Par.SetFeatureAngleDeg(100);
        Par.SetTargetLen(EdgeStep);
        Par.selectedOnly=true;

        Par.iter=10;

        std::cout<<"Remeshing step"<<std::endl;
        std::cout<<"Edge Size " <<EdgeStep<<std::endl;

        UpdateAttributes(curr_mesh);
        vcg::tri::IsotropicRemeshing<MeshType>::Do(curr_mesh,Par);

        vcg::tri::UpdateFlags<MeshType>::Clear(curr_mesh);
        UpdateAttributes(curr_mesh);
    }

    static void ExpandSelected(MeshType &curr_mesh,
                               size_t smooth_steps)
    {
        UpdateAttributes(curr_mesh);

        //first find the seeds
        std::vector<VertexType*> seeds;
        for (size_t i=0;i<curr_mesh.face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                bool IsSel0=curr_mesh.face[i].IsS();
                bool IsSel1=curr_mesh.face[i].FFp(j)->IsS();
                if (IsSel0=IsSel1)continue;
                seeds.push_back(curr_mesh.face[i].V0(j));
                seeds.push_back(curr_mesh.face[i].V1(j));
            }
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(curr_mesh);

        std::sort(seeds.begin(),seeds.end());
        auto last = std::unique(seeds.begin(), seeds.end());
        seeds.erase(last, seeds.end());
        vcg::tri::Geodesic<MeshType>::Compute(curr_mesh,seeds);

        //find the radius
        ScalarType MaxRad=0;
        for (size_t i=0;i<curr_mesh.vert.size();i++)
        {
            if (!curr_mesh.vert[i].IsS())continue;
            MaxRad=std::max(MaxRad,curr_mesh.vert[i].Q());
        }
        MaxRad*=0.9;
        //then expand
        for (size_t i=0;i<curr_mesh.vert.size();i++)
        {
            if (!curr_mesh.vert[i].IsS())continue;
            ScalarType Dist=MaxRad-curr_mesh.vert[i].Q();
            ScalarType H=sqrt(MaxRad*MaxRad-Dist*Dist);
            curr_mesh.vert[i].P()+=curr_mesh.vert[i].N()*H;
        }
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceStrict(curr_mesh);
        vcg::tri::Smooth<MeshType>::VertexCoordLaplacian(curr_mesh,smooth_steps,true);
    }

    //    static void GetClosest(const CoordType test_pos,
    //                           const MeshType &test_mesh,
    //                           FaceType *&f,
    //                           CoordType &closest)
    //    {
    //        ScalarType minD=
    //        for ()
    //    }

    static void FindClosestFaceBary(MeshType &test_mesh,
                                    const std::vector<CoordType> &Pos,
                                    std::vector<size_t> &SplitFace,
                                    std::vector<CoordType> &SplitBary)
    {
        //find the faces to be splitted
        vcg::GridStaticPtr<FaceType,typename FaceType::ScalarType> Grid;
        Grid.Set(test_mesh.face.begin(),test_mesh.face.end());

        ScalarType MaxD=test_mesh.bbox.Diag();
        CoordType closestPt,Norm,Bary;
        ScalarType MinD;
        for (size_t i=0;i<Pos.size();i++)
        {
            FaceType *f=vcg::tri::GetClosestFaceBase(test_mesh,Grid,Pos[i],MaxD,MinD,closestPt,Norm,Bary);
            size_t IndexF=vcg::tri::Index(test_mesh,f);
            SplitFace.push_back(IndexF);
            SplitBary.push_back(Bary);
        }
    }

    static void ClampBarycenterCoords(CoordType &bary,ScalarType min_value=0.05)
    {
        if (bary.X()<min_value)
        {
            bary.X()=min_value;
            ScalarType remain=1-bary.X();
            ScalarType tot_val=bary.Y()+bary.Z();
            ScalarType ratio=remain/tot_val;
            bary.Y()*=ratio;
            bary.Z()*=ratio;
        }
        if (bary.Y()<min_value)
        {
            bary.Y()=min_value;
            ScalarType remain=1-bary.Y();
            ScalarType tot_val=bary.X()+bary.Z();
            ScalarType ratio=remain/tot_val;
            bary.X()*=ratio;
            bary.Z()*=ratio;
        }
        if (bary.Z()<min_value)
        {
            bary.Z()=min_value;
            ScalarType remain=1-bary.Z();
            ScalarType tot_val=bary.X()+bary.Y();
            ScalarType ratio=remain/tot_val;
            bary.Y()*=ratio;
            bary.Z()*=ratio;
        }
    }

    static CoordType InterpolatePos(MeshType &test_mesh,
                                    size_t &IndexF,
                                    CoordType &bary)
    {
        CoordType P0=test_mesh.face[IndexF].P(0);
        CoordType P1=test_mesh.face[IndexF].P(1);
        CoordType P2=test_mesh.face[IndexF].P(2);
        return (P0*bary.X()+
                P1*bary.Y()+
                P2*bary.Z());
    }

public:

    static void GenerateEnvelope(MeshType &input_mesh,
                                 const std::vector<size_t> &Vert,
                                 MeshType &envelope0,
                                 MeshType &envelope1,
                                 size_t smooth_steps=5)
    {
        MeshType mesh;
        vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,input_mesh);
        UpdateAttributes(mesh);

        //step 1 compute harmonics
        typename vcg::tri::Harmonic<MeshType, ScalarType>::ConstraintVec constraints;
        typedef typename vcg::tri::Harmonic<MeshType, ScalarType>::Constraint HarmoConstraint;
        assert(Vert.size()%2==0);

        for (size_t i=0;i<Vert.size();i+=2)
        {
            constraints.push_back(HarmoConstraint(&(mesh.vert[Vert[i]]), -1.0));
            constraints.push_back(HarmoConstraint(&(mesh.vert[Vert[i+1]]), 1.0));
        }
        typename MeshType::template PerVertexAttributeHandle<ScalarType> handle;
        handle=vcg::tri::Allocator<MeshType>::template GetPerVertexAttribute<ScalarType>(mesh, "harmonic");
        bool ok = vcg::tri::Harmonic<MeshType, ScalarType>::ComputeScalarField(mesh, constraints, handle);
        assert(ok);
        vcg::tri::UpdateQuality<MeshType>::VertexFromAttributeHandle(mesh,handle);
        vcg::tri::UpdateColor<MeshType>::PerVertexQualityRamp(mesh,-1,1);

        //step 1 storethe harmonic field on  quality
        for (size_t i=0;i<mesh.vert.size();i++)
            mesh.vert[i].Q()=handle[i];

        //vcg::tri::Smooth<MeshType>::VertexQualityLaplacian(mesh,10);

        //step 2 compute edges to be splitted
        std::map<EdgeCoordKey,CoordType> SplitOps;
        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                VertexType *V0=mesh.face[i].V0(j);
                VertexType *V1=mesh.face[i].V1(j);
                ScalarType Harmonic0=V0->Q();
                ScalarType Harmonic1=V1->Q();
                if (fabs(Harmonic0)<0.00001)continue;
                if (fabs(Harmonic1)<0.00001)continue;
                if ((Harmonic0*Harmonic1)>=0)continue;
                CoordType P0=V0->P();
                CoordType P1=V1->P();
                std::pair<CoordType,CoordType> key(std::min(P0,P1),std::max(P0,P1));
                ScalarType SumW=fabs(Harmonic0)+fabs(Harmonic1);
                ScalarType W0=1-fabs(Harmonic0)/SumW;
                ScalarType W1=1-W0;
                SplitOps[key]=P0*W0+P1*W1;
            }
        SplitLev<MeshType> splMd(&SplitOps);
        EdgePred<MeshType> eP(&SplitOps);
        bool done=vcg::tri::RefineE<MeshType,SplitLev<MeshType>,EdgePred<MeshType> >(mesh,splMd,eP);
        UpdateAttributes(mesh);

        //step 3 split into two meshes
        envelope0.Clear();
        envelope1.Clear();
        vcg::tri::UpdateQuality<MeshType>::FaceFromVertex(mesh);
        vcg::tri::UpdateSelection<MeshType>::Clear(mesh);
        for (size_t i=0;i<mesh.face.size();i++)
            if (mesh.face[i].Q()<0)mesh.face[i].SetS();

        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);
        vcg::tri::Append<MeshType,MeshType>::Mesh(envelope0,mesh,true);

        vcg::tri::UpdateSelection<MeshType>::FaceInvert(mesh);
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);

        vcg::tri::Append<MeshType,MeshType>::Mesh(envelope1,mesh,true);
        vcg::tri::UpdateSelection<MeshType>::Clear(mesh);

        UpdateAttributes(envelope0);
        UpdateAttributes(envelope1);

        vcg::tri::Allocator<MeshType>::CompactEveryVector(envelope0);
        vcg::tri::Allocator<MeshType>::CompactEveryVector(envelope1);
        //step 5 close holes
        size_t numF0=envelope0.face.size();
        size_t numF1=envelope1.face.size();

        ScalarType AvgEdge0=AverageEdgeSize(envelope0)/2;
        ScalarType AvgEdge1=AverageEdgeSize(envelope1)/2;


        vcg::tri::Hole<MeshType>::template EarCuttingFill<vcg::tri::TrivialEar<MeshType> >(envelope0,envelope0.face.size(),false);
        vcg::tri::Hole<MeshType>::template EarCuttingFill<vcg::tri::TrivialEar<MeshType> >(envelope1,envelope1.face.size(),false);
        size_t numF0_after=envelope0.face.size();
        size_t numF1_after=envelope1.face.size();

        vcg::tri::UpdateSelection<MeshType>::Clear(envelope0);
        vcg::tri::UpdateSelection<MeshType>::Clear(envelope1);
        for (size_t i=numF0;i<numF0_after;i++)
            envelope0.face[i].SetS();
        for (size_t i=numF1;i<numF1_after;i++)
            envelope1.face[i].SetS();

        RemeshSelected(envelope0,AvgEdge0);
        RemeshSelected(envelope1,AvgEdge1);

        std::cout<<"Done"<<std::endl;

        for (size_t i=numF0;i<envelope0.face.size();i++)
            envelope0.face[i].SetS();

        for (size_t i=numF1;i<envelope1.face.size();i++)
            envelope1.face[i].SetS();

        ExpandSelected(envelope0,smooth_steps);
        ExpandSelected(envelope1,smooth_steps);

    }

    static bool GenerateEnvelope(MeshType &input_mesh,
                                 const std::vector<CoordType> &Pos,
                                 MeshType &envelope0,
                                 MeshType &envelope1,
                                 size_t smooth_steps=5)
    {
        MeshType mesh;
        vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,input_mesh);
        UpdateAttributes(mesh);

        std::vector<size_t> SplitFace;
        std::vector<CoordType> SplitBary;
        FindClosestFaceBary(mesh,Pos,SplitFace,SplitBary);

        //check the faces to be unique with the flag
        vcg::tri::UpdateSelection<MeshType>::Clear(mesh);

        assert(SplitFace.size()==SplitBary.size());
        //then make the barycenter close to border
        for (size_t i=0;i<SplitFace.size();i++)
        {
            size_t IndexF=SplitFace[i];
            std::cout<<"Face "<<IndexF<<std::endl;
            if (mesh.face[IndexF].IsS())
            {
                std::cout<<"WARNING SAME FACE SELECTED TWICE"<<std::endl;
                return false;
            }
            mesh.face[IndexF].SetS();
            ClampBarycenterCoords(SplitBary[i]);
        }

        std::vector<size_t> IndexV;
        //then split the faces
        for (size_t i=0;i<SplitFace.size();i++)
        {
            size_t IndexF=SplitFace[i];
            vcg::PolygonalAlgorithm<MeshType>::Triangulate(mesh,IndexF);
            CoordType Pos=InterpolatePos(mesh,IndexF,SplitBary[i]);
            mesh.vert.back().P()=Pos;
            IndexV.push_back(mesh.vert.size()-1);
        }
        GenerateEnvelope(mesh,IndexV,envelope0,envelope1,smooth_steps);
        return true;
    }
};

#endif
