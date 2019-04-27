#ifndef PATCH_DECOMPOSER
#define PATCH_DECOMPOSER

#include <vcg/math/matrix33.h>
#include <vcg/complex/algorithms/parametrization/tangent_field_operators.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/complex/algorithms/geodesic.h>
#include <wrap/io_trimesh/export.h>
#include <vcg/complex/algorithms/polygonal_algorithms.h>
#include <wrap/igl/smooth_field.h>
#include <vcg/complex/algorithms/hole.h>

#include "field_tracer.h"

namespace QuadBoolean {
namespace internal {

class PFace;
class PVertex;

struct PUsedTypes: public vcg::UsedTypes<vcg::Use<PVertex>  ::AsVertexType,
        vcg::Use<PFace>	::AsFaceType>{};

class PVertex:public vcg::Vertex<	PUsedTypes,
        vcg::vertex::Coord3d,
        vcg::vertex::Normal3d,
        vcg::vertex::Qualityd,
        vcg::vertex::BitFlags>{} ;

class PFace:public vcg::Face<
        PUsedTypes
        ,vcg::face::PolyInfo
        ,vcg::face::PFVAdj	 // Pointer to the vertices (just like FVAdj )
        ,vcg::face::PFFAdj	 // Pointer to edge-adjacent face (just like FFAdj )
        ,vcg::face::BitFlags // bit flags
        ,vcg::face::Normal3d // normal
        ,vcg::face::Qualityd // face quality.
        ,vcg::face::BitFlags> {};// basic bit flags

class PMesh: public
        vcg::tri::TriMesh<
        std::vector<PVertex>,   // vector of edges
        std::vector<PFace >     // the vector of faces
        >
{};

template <class MeshType>
class PatchDecomposer
{
    typedef typename MeshType::CoordType CoordType;
    typedef typename MeshType::ScalarType ScalarType;
    typedef typename MeshType::FaceType FaceType;
    typedef typename MeshType::VertexType VertexType;

    MeshType &mesh;
    MeshType OriginalMesh;
    VertexFieldTracer<MeshType> *FieldTracer;

    void UpdateMeshAttributes(MeshType& targetMesh)
    {
        vcg::tri::UpdateNormal<MeshType>::PerFaceNormalized(targetMesh);
        vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(targetMesh);
        vcg::tri::UpdateBounding<MeshType>::Box(targetMesh);
        vcg::tri::UpdateTopology<MeshType>::FaceFace(targetMesh);
        vcg::tri::UpdateTopology<MeshType>::VertexFace(targetMesh);
        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(targetMesh);
        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromNone(targetMesh);
    }

    void GetBorderTracingDirections(std::vector<std::vector<CoordType> > &VertexDirs,
                                    std::vector<std::vector<CoordType> > &OrthoDirs)
    {
        VertexDirs.clear();
        OrthoDirs.clear();
        VertexDirs.resize(mesh.vert.size());
        OrthoDirs.resize(mesh.vert.size());

        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                if (mesh.face[i].FFp(j)!=&mesh.face[i])continue;
                size_t IndexV0=vcg::tri::Index(mesh,mesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(mesh,mesh.face[i].V1(j));
                CoordType P0=mesh.vert[IndexV0].P();
                CoordType P1=mesh.vert[IndexV1].P();
                CoordType Dir=P1-P0;
                Dir.Normalize();

                CoordType OrthoDir=Dir;
                OrthoDir=mesh.face[i].N()^OrthoDir;

                vcg::Matrix33<ScalarType> Rot0=vcg::RotationMatrix(mesh.face[i].N(),mesh.vert[IndexV0].N());
                vcg::Matrix33<ScalarType> Rot1=vcg::RotationMatrix(mesh.face[i].N(),mesh.vert[IndexV1].N());
                CoordType Dir0=Rot0*Dir;
                CoordType Dir1=Rot1*Dir;

                CoordType OrthoDir0=Rot0*OrthoDir;
                CoordType OrthoDir1=Rot1*OrthoDir;

                Dir0.Normalize();
                Dir1.Normalize();
                OrthoDir0.Normalize();
                OrthoDir1.Normalize();

                VertexDirs[IndexV0].push_back(Dir0);
                VertexDirs[IndexV1].push_back(-Dir1);

                OrthoDirs[IndexV0].push_back(OrthoDir0);
                OrthoDirs[IndexV1].push_back(OrthoDir1);
            }
    }

    void InitTracingDirections()
    {
        vcg::tri::UpdateNormal<MeshType>::PerFaceNormalized(mesh);
        vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(mesh);

        std::vector<std::vector<CoordType> > VertexDirs,OrthoDirs;
        GetBorderTracingDirections(VertexDirs,OrthoDirs);

        for (size_t i=0;i<mesh.vert.size();i++)
        {
            //not border do nothing
            if (!mesh.vert[i].IsB())continue;
            //is convex on border, no tracing
            if (BorderCornersConvex.count(i)>0)continue;
            //is concave on border then add all directions
            if (BorderCornersConcave.count(i)>0)
            {
                bool has_found[4]={false,false,false,false};
                //                //disable empty exit
                for (size_t j=0;j<4;j++)
                {
                    if (FieldTracer->VertNeigh[i][j].size()==0)
                        has_found[j]=true;
                }

                StartTraceV.push_back(i);
                StartTraceDir.push_back(std::vector<size_t>());
                //assert(VertexDirs[i].size()==2);
                for (size_t j=0;j<VertexDirs[i].size();j++)
                {
                    CoordType TargetDir=VertexDirs[i][j];
                    TargetDir.Normalize();
                    size_t FoundDirV=FieldTracer->GetClosestDirTo(i,TargetDir);
                    assert(FoundDirV>=0);
                    assert(FoundDirV<4);
                    has_found[FoundDirV]=true;
                }
                for (size_t j=0;j<4;j++)
                {
                    if (has_found[j])continue;
                    StartTraceDir.back().push_back(j);
                }
            }
            else
            {
                //                //regular border, average the directions
                CoordType TargetDir(0,0,0);
                //std::cout<<"size "<<VertexDirs[i].size()<<std::endl;
                assert(VertexDirs[i].size()==2);

                //rotate by the vertex normal
                CoordType RotDir0=OrthoDirs[i][0];
                CoordType RotDir1=OrthoDirs[i][1];

                TargetDir+=RotDir0;
                TargetDir+=RotDir1;

                TargetDir.Normalize();

                StartTraceV.push_back(i);
                size_t StartDirV=FieldTracer->GetClosestDirTo(i,TargetDir);
                StartTraceDir.push_back(std::vector<size_t >(1,StartDirV));
            }
        }
    }

    void InitStartDirections()
    {
        StartTraceV.clear();
        StartTraceDir.clear();
        InitTracingDirections();
    }

    //the initial starting vertex
    std::vector<size_t > StartTraceV;
    //the initial starting direction
    std::vector<std::vector<size_t > > StartTraceDir;
    //    //the number of traces for the starting nodes that has been chosen
    //    std::vector<size_t> NumberCandidates;

    //the initial candidates
    std::vector<std::vector<size_t > > TraceVertCandidates;
    std::vector<std::vector<size_t > > TraceDirCandidates;
    std::vector<bool> HasBeenChoosen;

    //the choosen one
    std::vector<std::vector<size_t > > TraceVert;
    std::vector<std::vector<size_t > > TraceDir;

    std::vector<std::pair<ScalarType,size_t> > CandidatesPathLenghts;
    std::map<std::pair<CoordType,CoordType>,std::pair<ScalarType,CoordType> >  BorderEdges;

    //std::set<CoordType>  PatchCorners;

    std::set<size_t> BorderCornersConvex;
    std::set<size_t> BorderCornersConcave;

    void DrawTrace(size_t IndexTrace,vcg::Color4b TraceCol)
    {
#ifdef DRAWTRACE
        FieldTracer->DrawTrace(TraceVert[IndexTrace],TraceCol);
#endif
    }

    void InitCandidatesPathLenghts()
    {
        CandidatesPathLenghts.clear();
        for (size_t i=0;i<TraceVertCandidates.size();i++)
        {
            ScalarType currL=FieldTracer->TraceLenght(TraceVertCandidates[i],TraceDirCandidates[i]);
            CandidatesPathLenghts.push_back(std::pair<ScalarType,size_t>(currL,i));
            //CandidatesPathLenghts.push_back(std::pair<ScalarType,size_t>(CandidatePathLenght(i),i));
        }
        std::sort(CandidatesPathLenghts.begin(),CandidatesPathLenghts.end());

//        for (size_t i=0;i<CandidatesPathLenghts.size();i++)
//        {
////            std::cout<<"Test"<<CandidatesPathLenghts[i].first<<
////                       ","<<CandidatesPathLenghts[i].second<<std::endl;
//        }
    }


    void PruneCollidingCandidates()
    {
        std::vector<std::vector<size_t > > TraceVertSwap;
        std::vector<std::vector<size_t > > TraceDirSwap;

        InitCandidatesPathLenghts();
        for (size_t i=0;i<TraceVertCandidates.size();i++)
        {
            size_t TraceIndex0=CandidatesPathLenghts[i].second;
            std::vector<size_t > TraceV0=TraceVertCandidates[TraceIndex0];
            std::vector<size_t > TraceD0=TraceDirCandidates[TraceIndex0];
            bool collide=false;
            for (size_t i=0;i<TraceVertSwap.size();i++)
            {
                std::vector<size_t > TraceV1=TraceVertSwap[i];
                std::vector<size_t > TraceD1=TraceDirSwap[i];

                collide|=FieldTracer->CollideTraces(TraceV0,TraceD0,TraceV1,TraceD1);
                if (collide){/*std::cout<<"Pruned "<<TraceIndex0<<std::endl;*/break;}
            }
            if (!collide)
            {
                TraceVertSwap.push_back(TraceV0);
                TraceDirSwap.push_back(TraceD0);
                //std::cout<<"Added "<<TraceIndex0<<std::endl;
            }
        }
        TraceVertCandidates=TraceVertSwap;
        TraceDirCandidates=TraceDirSwap;
        //InitPathPriority();
    }


    void PruneConcaveCornersCandidates(bool addAll=false)
    {
        std::vector<std::vector<size_t > > TraceVertSwap;
        std::vector<std::vector<size_t > > TraceDirSwap;

        std::vector<size_t> VertAllocator(mesh.vert.size(),1);
        InitCandidatesPathLenghts();
        //first round
        int Num0=TraceVertCandidates.size();
        for (size_t i=0;i<TraceVertCandidates.size();i++)
        {
            size_t TraceIndex0=CandidatesPathLenghts[i].second;
            std::vector<size_t > TraceV0=TraceVertCandidates[TraceIndex0];
            std::vector<size_t > TraceD0=TraceDirCandidates[TraceIndex0];
            size_t IndexV0=TraceV0[0];
            if (VertAllocator[IndexV0]==0)continue;

            bool collide=false;
            for (size_t i=0;i<TraceVertSwap.size();i++)
            {
                std::vector<size_t > TraceV1=TraceVertSwap[i];
                std::vector<size_t > TraceD1=TraceDirSwap[i];

                collide|=FieldTracer->CollideTraces(TraceV0,TraceD0,TraceV1,TraceD1);
                if (collide)break;
            }
            if (!collide)
            {
                TraceVertSwap.push_back(TraceV0);
                TraceDirSwap.push_back(TraceD0);
                VertAllocator[IndexV0]--;
            }
        }

        //second round
        if (addAll)
        {
            VertAllocator.clear();
            VertAllocator.resize(mesh.vert.size(),1);
            for (size_t i=0;i<TraceVertCandidates.size();i++)
            {
                size_t TraceIndex0=CandidatesPathLenghts[i].second;
                std::vector<size_t > TraceV0=TraceVertCandidates[TraceIndex0];
                std::vector<size_t > TraceD0=TraceDirCandidates[TraceIndex0];
                size_t IndexV0=TraceV0[0];
                if (VertAllocator[IndexV0]==0)continue;

                bool collide=false;
                for (size_t i=0;i<TraceVertSwap.size();i++)
                {
                    std::vector<size_t > TraceV1=TraceVertSwap[i];
                    std::vector<size_t > TraceD1=TraceDirSwap[i];

                    collide|=FieldTracer->CollideTraces(TraceV0,TraceD0,TraceV1,TraceD1);
                    if (collide)break;
                }
                if (!collide)
                {
                    TraceVertSwap.push_back(TraceV0);
                    TraceDirSwap.push_back(TraceD0);
                    VertAllocator[IndexV0]--;
                }
            }
        }
        TraceVertCandidates=TraceVertSwap;
        TraceDirCandidates=TraceDirSwap;
        int Num1=TraceVertCandidates.size();
        std::cout<<"Pruned "<<Num0-Num1<<" out ouf "<<Num0<<std::endl;
        //        //InitPathPriority();
    }

    void InitBorderEdgeMap()
    {
        BorderEdges.clear();
        for (size_t i=0;i<TraceVertCandidates.size();i++)
        {
            if (!HasBeenChoosen[i])continue;
            for (size_t j=0;j<TraceVertCandidates[i].size()-1;j++)
            {
                size_t IndexV0=TraceVertCandidates[i][j];
                size_t IndexV1=TraceVertCandidates[i][j+1];
                size_t DirIndexV0=TraceDirCandidates[i][j];

                CoordType PosV0=mesh.vert[IndexV0].P();
                CoordType PosV1=mesh.vert[IndexV1].P();

                ScalarType ELength;
                CoordType Dir;
                FieldTracer->EdgeLengh(IndexV0,IndexV1,DirIndexV0,Dir,ELength);
                std::pair<CoordType,CoordType> Key(std::min(PosV0,PosV1),
                                                   std::max(PosV0,PosV1));
                BorderEdges[Key]=std::pair<ScalarType,CoordType> (ELength,Dir);
            }
        }

        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                if (!vcg::face::IsBorder(mesh.face[i],j))continue;
                CoordType Pos0=mesh.face[i].P0(j);
                CoordType Pos1=mesh.face[i].P1(j);

                std::pair<CoordType,CoordType> key(std::min(Pos0,Pos1),
                                                   std::max(Pos0,Pos1));
                ScalarType ELength=(Pos0-Pos1).Norm();
                CoordType Dir=(Pos0-Pos1).Normalize();
                BorderEdges[key]=std::pair<ScalarType,CoordType> (ELength,Dir);
            }
    }

    void RetrievePartitioningFrom(size_t IndexF,std::vector<size_t> &partition)
    {
        partition.clear();

        std::vector<size_t> stack;
        std::set<size_t> explored;

        stack.push_back(IndexF);
        explored.insert(IndexF);
        do
        {
            size_t currF=stack.back();
            stack.pop_back();

            partition.push_back(currF);
            for (size_t i=0;i<mesh.face[currF].VN();i++)
            {
                if (vcg::face::IsBorder(mesh.face[currF],i))continue;

                CoordType Pos0=mesh.face[currF].P0(i);
                CoordType Pos1=mesh.face[currF].P1(i);

                std::pair<CoordType,CoordType> key(std::min(Pos0,Pos1),
                                                   std::max(Pos0,Pos1));

                if (BorderEdges.count(key)==1)continue;

                int NextFIndex=vcg::tri::Index(mesh,mesh.face[currF].FFp(i));

                if (explored.count(NextFIndex)>0)continue;

                explored.insert(NextFIndex);
                stack.push_back(NextFIndex);
            }
        }while (!stack.empty());
    }


    void RetrievePartitioningAround(const std::vector<size_t> &SeedFaces,
                                    std::vector<std::vector<size_t> > &Partitions)
    {
        Partitions.clear();
        vcg::tri::UpdateFlags<MeshType>::FaceClearV(mesh);
        for (size_t i=0;i<SeedFaces.size();i++)
        {
            if (mesh.face[SeedFaces[i]].IsV())continue;

            //std::cout<<"test"<<std::endl;
            std::vector<size_t> partition;
            RetrievePartitioningFrom(SeedFaces[i],partition);

            for (size_t j=0;j<partition.size();j++)
                mesh.face[partition[j]].SetV();

            Partitions.push_back(partition);
        }
    }

    void GetPartitions(std::vector<std::vector<size_t> > &Partitions)
    {
        InitBorderEdgeMap();
        Partitions.clear();
        vcg::tri::UpdateFlags<MeshType>::FaceClearV(mesh);
        for (size_t i=0;i<mesh.face.size();i++)
        {
            if (mesh.face[i].IsV())continue;

            //std::cout<<"test"<<std::endl;
            std::vector<size_t> partition;
            RetrievePartitioningFrom(i,partition);

            for (size_t j=0;j<partition.size();j++)
                mesh.face[partition[j]].SetV();

            Partitions.push_back(partition);
        }
    }

public:

    enum ColorMode{
        Partitions,
        Manifold,
        Holes,
        NumEdges,
        AreaPerimeterError,
        MaxSideLenghtVariance,
        MaxGeodesicLenghtError,
        MaxAngleDeviation,
        IdealPatchDistortion
    };

private:

    struct PartitionQuality
    {
        bool IsManifold;
        size_t NumHoles;
        size_t NumEdges;
        size_t NumConcave;
        ScalarType AreaPerimeterError;
        ScalarType MaxSideLenghtVariance;
        ScalarType MaxGeodesicLenghtError;
        ScalarType MaxAngleDeviation;
        ScalarType IdealPatchDistortion;
    };

    void SetQualityFaces(const std::vector<std::vector<size_t> > &FacePartitions,
                         const std::vector<PartitionQuality> &PQualities,
                         const ColorMode &CMode)
    {
        assert(CMode!=Partitions);
        assert(CMode!=Manifold);
        assert(CMode!=Holes);
        for (size_t i=0;i<FacePartitions.size();i++)
            for (size_t j=0;j<FacePartitions[i].size();j++)
            {
                size_t IndexF=FacePartitions[i][j];
                ScalarType QualityF=0;
                switch(CMode)
                {
                case NumEdges  :QualityF=PQualities[i].NumEdges;break;
                case AreaPerimeterError:QualityF=PQualities[i].AreaPerimeterError; break;
                case MaxSideLenghtVariance:QualityF=PQualities[i].MaxSideLenghtVariance; break;
                case MaxGeodesicLenghtError:QualityF=PQualities[i].MaxGeodesicLenghtError; break;
                case MaxAngleDeviation:QualityF=PQualities[i].MaxAngleDeviation; break;
                case IdealPatchDistortion:QualityF=PQualities[i].IdealPatchDistortion; break;
                default: assert(0);
                }
                mesh.face[IndexF].Q()=QualityF;
            }
    }

public:

    void ColorPatches(ColorMode CMode)
    {
        InitBorderEdgeMap();
        //InitVerticesEdgeMap();
        std::vector<std::vector<size_t> > FacePartitions;
        GetPartitions(FacePartitions);
        std::vector<PartitionQuality> PQualities(FacePartitions.size());
        std::set<ColorMode> StatsCol;
        StatsCol.insert(CMode);

        for (size_t i=0;i<FacePartitions.size();i++)
        {
            switch(CMode) {
            case Partitions  :
                for (size_t i=0;i<FacePartitions.size();i++)
                    for (size_t j=0;j<FacePartitions[i].size();j++)
                    {
                        size_t IndexF=FacePartitions[i][j];
                        mesh.face[IndexF].C()=vcg::Color4b::Scatter(FacePartitions.size(),i);
                    }break;

            case Manifold  :
                for (size_t i=0;i<FacePartitions.size();i++)
                {
                    GetPartitionStats(FacePartitions[i],StatsCol,PQualities[i],true);
                    bool IsManifold=PQualities[i].IsManifold;
                    for (size_t j=0;j<FacePartitions[i].size();j++)
                    {
                        size_t IndexF=FacePartitions[i][j];
                        if (IsManifold)
                            mesh.face[IndexF].C()=vcg::Color4b(0,0,255,255);
                        else
                            mesh.face[IndexF].C()=vcg::Color4b(255,0,0,255);
                    }
                }break;
            case Holes  :
                for (size_t i=0;i<FacePartitions.size();i++)
                {
                    GetPartitionStats(FacePartitions[i],StatsCol,PQualities[i],true);
                    size_t NumHoles=PQualities[i].NumHoles;
//                    if (NumHoles!=1)
//                    {
//                        MeshType SaveMesh;
//                        GetPartitionMesh(FacePartitions[i],SaveMesh);
//                        //vcg::tri::io::ExporterPLY<MeshType>::Save(SaveMesh,"non_one_hole.ply");
//                    }
                    for (size_t j=0;j<FacePartitions[i].size();j++)
                    {
                        size_t IndexF=FacePartitions[i][j];
                        if (NumHoles==1)
                            mesh.face[IndexF].C()=vcg::Color4b(0,0,255,255);
                        else
                            mesh.face[IndexF].C()=vcg::Color4b(255,0,0,255);
                    }
                }break;
            case NumEdges:
            {
                GetPartitionStats(FacePartitions[i],StatsCol,PQualities[i],true);
                size_t currEdge=PQualities[i].NumEdges;
                vcg::Color4b Col;

                if ((currEdge<=2)||(currEdge>6))
                    Col=vcg::Color4b::Red;
                if (currEdge==3)
                    Col=vcg::Color4b::Blue;
                if (currEdge==4)
                    Col=vcg::Color4b::Green;
                if (currEdge==5)
                    Col=vcg::Color4b::Yellow;
                if (currEdge==6)
                    Col=vcg::Color4b::LightGray;

                for (size_t j=0;j<FacePartitions[i].size();j++)
                    mesh.face[FacePartitions[i][j]].C()=Col;

                break;
            }
            default:
                for (size_t i=0;i<FacePartitions.size();i++)
                    GetPartitionStats(FacePartitions[i],StatsCol,PQualities[i],true);

                SetQualityFaces(FacePartitions,PQualities,CMode);

            }
        }

        if ((CMode==AreaPerimeterError)||
                (CMode==MaxSideLenghtVariance)||
                (CMode==MaxGeodesicLenghtError)||
                (CMode==IdealPatchDistortion))
        {
            for (size_t i=0;i<mesh.face.size();i++)
                mesh.face[i].Q()=- mesh.face[i].Q();
            vcg::tri::UpdateColor<MeshType>::PerFaceQualityRamp(mesh);
        }


    }

private:

    vcg::face::Pos<FaceType> FindCornerHEdge(MeshType &TempMesh)
    {
        vcg::face::Pos<FaceType> BorderHEdge;
        for (size_t i=0;i<TempMesh.face.size();i++)
        {
            for (size_t j=0;j<TempMesh.face[i].VN();j++)
            {
                if (!vcg::face::IsBorder(TempMesh.face[i],j))continue;
                //CoordType CornerPos=TempMesh.face[i].cP0(j);
                //if (PatchCorners.count(CornerPos)==0)continue;
                if (!TempMesh.face[i].V(j)->IsS())continue;
                BorderHEdge=vcg::face::Pos<FaceType>(&TempMesh.face[i],(int)j);
                BorderHEdge.FlipV();
                return BorderHEdge;
            }
        }
        assert(0);
    }


    void GetPartitionMesh(const std::vector<size_t> &PartitionFaces,
                          MeshType &partition_mesh)
    {
        partition_mesh.Clear();

        vcg::tri::UpdateFlags<MeshType>::VertexClearS(mesh);
        vcg::tri::UpdateFlags<MeshType>::FaceClearS(mesh);
        for (size_t i=0;i<PartitionFaces.size();i++)
            mesh.face[PartitionFaces[i]].SetS();

        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);
        vcg::tri::Append<MeshType,MeshType>::Mesh(partition_mesh,mesh,true);
        UpdateMeshAttributes(partition_mesh);
        vcg::tri::UpdateFlags<MeshType>::VertexClearS(partition_mesh);
        vcg::tri::UpdateFlags<MeshType>::FaceClearS(partition_mesh);
        vcg::tri::UpdateFlags<MeshType>::VertexClearS(mesh);
        vcg::tri::UpdateFlags<MeshType>::FaceClearS(mesh);
    }


//    void GetPartitionMesh(const std::vector<size_t> &PartitionFaces,
//                          MeshType &partition_mesh)
//    {

//        int t0=clock();
//        partition_mesh.Clear();

//        std::map<size_t,size_t> VertMap;
//        std::vector<CoordType> PosV;

//        for (size_t i=0;i<PartitionFaces.size();i++)
//        {
//            FaceType *f=&mesh.face[PartitionFaces[i]];
//            for (size_t j=0;j<3;j++)
//            {
//                size_t IndexV=vcg::tri::Index(mesh,f->V(j));
//                if (VertMap.count(IndexV)==0)
//                {
//                    PosV.push_back(f->P(j));
//                    VertMap[IndexV]=PosV.size()-1;
//                }
//            }
//        }
//        partition_mesh.vert.resize(PosV.size());
//        partition_mesh.vn=PosV.size();
//        for (size_t i=0;i<PosV.size();i++)
//            partition_mesh.vert[i].P()=PosV[i];

//        int t1=clock();

//        partition_mesh.face.resize(PartitionFaces.size());
//        partition_mesh.fn=PartitionFaces.size();
//        for (size_t i=0;i<PartitionFaces.size();i++)
//        {
//            FaceType *f=&mesh.face[PartitionFaces[i]];
//            for (size_t j=0;j<3;j++)
//            {
//                size_t IndexVOld=vcg::tri::Index(mesh,f->V(j));
//                assert(VertMap.count(IndexVOld)>0);
//                size_t IndexVNew=VertMap[IndexVOld];
//                partition_mesh.face[i].V(j)=&partition_mesh.vert[IndexVNew];
//            }
//        }

//        int t2=clock();
//        vcg::tri::UpdateTopology<MeshType>::FaceFace(partition_mesh);
//        vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(partition_mesh);
//        vcg::tri::UpdateFlags<MeshType>::VertexBorderFromNone(partition_mesh);
//        vcg::tri::UpdateNormal<MeshType>::PerFaceNormalized(partition_mesh);
//        vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(partition_mesh);

//    }

#define CONVEX 4.0
#define CONCAVE 4.0

    void GetCorners(MeshType &TestMesh,
                    std::vector<size_t> &IndexCorners)
    {
        IndexCorners.clear();
        std::vector<std::vector<CoordType> > DirVert(TestMesh.vert.size());
        for (size_t i=0;i<TestMesh.face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                if (!vcg::face::IsBorder(TestMesh.face[i],j))continue;
                CoordType Dir0=TestMesh.face[i].P1(j)-TestMesh.face[i].P0(j);
                Dir0.Normalize();
                CoordType Dir1=-Dir0;
                size_t IndexV0=vcg::tri::Index(TestMesh,TestMesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(TestMesh,TestMesh.face[i].V1(j));
                CoordType SourceN=TestMesh.face[i].N();
                CoordType TargetN0=TestMesh.face[i].V0(j)->N();
                CoordType TargetN1=TestMesh.face[i].V1(j)->N();
                vcg::Matrix33<ScalarType> Rot0=vcg::RotationMatrix(SourceN,TargetN0);
                vcg::Matrix33<ScalarType> Rot1=vcg::RotationMatrix(SourceN,TargetN1);
                CoordType Dir0Rot=Rot0*Dir0;
                CoordType Dir1Rot=Rot1*Dir1;
                DirVert[IndexV0].push_back(Dir0Rot);
                DirVert[IndexV1].push_back(Dir1Rot);
            }

        for (size_t i=0;i<DirVert.size();i++)
        {
            if (DirVert[i].size()!=2)continue;
            CoordType Dir0=DirVert[i][0];
            CoordType Dir1=DirVert[i][1];
            ScalarType Angle=vcg::math::ToDeg(vcg::Angle(Dir0,Dir1));
            if (Angle<(180-45))IndexCorners.push_back(i);
        }

    }

    void GetCorners(MeshType &TestMesh,
                    std::vector<size_t> &ConvexV,
                    std::vector<size_t> &ConcaveV)
    {
        ConvexV.clear();
        ConcaveV.clear();
        std::vector<size_t> CornerV;

        GetCorners(TestMesh,CornerV);

        vcg::tri::UpdateFlags<MeshType>::VertexClearV(TestMesh);
        for (size_t i=0;i<CornerV.size();i++)
            TestMesh.vert[CornerV[i]].SetV();

        vcg::tri::UpdateSelection<MeshType>::VertexCornerBorder(TestMesh,M_PI-0.1);//M_PI/CONVEX);
        for (size_t i=0;i<TestMesh.vert.size();i++)
        {
            if (!TestMesh.vert[i].IsS())continue;
            if (!TestMesh.vert[i].IsV())continue;
            ConvexV.push_back(i);
        }
        vcg::tri::UpdateSelection<MeshType>::VertexCornerBorder(TestMesh,M_PI+0.1);//M_PI/CONCAVE);
        for (size_t i=0;i<TestMesh.vert.size();i++)
        {
            if (!TestMesh.vert[i].IsB())continue;
            if (!TestMesh.vert[i].IsV())continue;
            if (TestMesh.vert[i].IsS())continue;
            ConcaveV.push_back(i);
        }
    }


    void PartitionData(const std::vector<size_t> &PartitionFaces,
                       const std::set<ColorMode> &UpdateStats,
                       size_t &NumHoles,
                       size_t &NumConcave,
                       bool &IsManifold,
                       ScalarType &Area,
                       size_t &NEdges,
                       ScalarType &Perimeter,
                       std::vector<ScalarType> &SideAngles,
                       std::vector<ScalarType> &SidesLenght,
                       std::vector<CoordType> &Corners,
                       bool UseRealLenght=false)
    {
        MeshType TempMesh;
        GetPartitionMesh(PartitionFaces,TempMesh);

        //select border corners
        std::vector<size_t> IndexVConvex,IndexVConcave;
        GetCorners(TempMesh,IndexVConvex,IndexVConcave);
        NEdges=IndexVConvex.size()+IndexVConcave.size();
        NumConcave=IndexVConcave.size();

        //std::cout<<"NEdges "<<NEdges<<std::endl;



        //check manifoldness
        IsManifold=true;
        if (UpdateStats.count(Manifold)>0)
        {
            size_t nonManifE=vcg::tri::Clean<MeshType>::CountNonManifoldEdgeFF(TempMesh);
            if (nonManifE>0)IsManifold=false;
            size_t nonManifV=vcg::tri::Clean<MeshType>::CountNonManifoldVertexFF(TempMesh);
            if (nonManifV>0)IsManifold=false;
        }

        //check number of holes
        if (UpdateStats.count(Holes)>0)
            NumHoles=vcg::tri::Clean<MeshType>::CountHoles(TempMesh);

        //compute area
        Area=0;
        if (UpdateStats.count(AreaPerimeterError)>0)
            for (size_t i=0;i<TempMesh.face.size();i++)
                Area+=(vcg::DoubleArea(TempMesh.face[i])/2);

        if ((UpdateStats.count(AreaPerimeterError)==0)&&
                (UpdateStats.count(MaxSideLenghtVariance)==0)&&
                (UpdateStats.count(MaxGeodesicLenghtError)==0)&&
                (UpdateStats.count(MaxAngleDeviation)==0)&&
                (UpdateStats.count(IdealPatchDistortion)==0))
        {
            //            int t2=clock();
            //            std::cout<<"t0 "<<t1-t0<<std::endl;
            //            std::cout<<"t1 "<<t2-t1<<std::endl;
            return;
        }

        //get first border half edge
        Perimeter=0;

        //        size_t Num=0;
        //        for (size_t i=0;i<TempMesh.vert.size();i++)
        //            if (TempMesh.vert[i].IsS())Num++;
        //        std::cout<<"Num "<<Num<<std::endl;

        vcg::tri::UpdateFlags<MeshType>::VertexClearS(TempMesh);
        for (size_t i=0;i<IndexVConvex.size();i++)
            TempMesh.vert[IndexVConvex[i]].SetS();

        for (size_t i=0;i<IndexVConcave.size();i++)
            TempMesh.vert[IndexVConcave[i]].SetS();

        vcg::face::Pos<FaceType> BorderHEdge=FindCornerHEdge(TempMesh);
        vcg::face::Pos<FaceType> StartingHEdge=BorderHEdge;
        //set the first corner
        Corners.push_back(BorderHEdge.VFlip()->P());

        //then start computing lenghts
        SidesLenght.clear();
        SideAngles.clear();
        SidesLenght.resize(SidesLenght.size()+1,0);
        SideAngles.resize(SideAngles.size()+1,0);
        int i=0;
        do
        {
            CoordType PosV0=BorderHEdge.V()->P();
            //bool OnCorner=(PatchCorners.count(PosV0)==1);
            bool OnCorner=BorderHEdge.V()->IsS();
            CoordType PosV1=BorderHEdge.VFlip()->P();


            //            BorderHEdge.V()->C()=vcg::Color4b(255,0,0,255);
            //            if (i%2==0)
            //                BorderHEdge.F()->C()=vcg::Color4b(255,0,0,255);
            //            else
            //                BorderHEdge.F()->C()=vcg::Color4b(0,0,255,255);

            if ((UpdateStats.count(AreaPerimeterError)>0)||
                    (UpdateStats.count(MaxSideLenghtVariance)>0)||
                    (UpdateStats.count(MaxGeodesicLenghtError)>0)||
                    (UpdateStats.count(MaxAngleDeviation)>0)||
                    (UpdateStats.count(IdealPatchDistortion)>0))
            {
                std::pair<CoordType,CoordType> key(std::min(PosV0,PosV1),std::max(PosV0,PosV1));

                ScalarType EdgeL=(PosV0-PosV1).Norm();
                if (!UseRealLenght)
                {
                    assert(BorderEdges.count(key)>0);
                    EdgeL=BorderEdges[key].first;
                }
                CoordType Dir0=BorderEdges[key].second;

                Perimeter+=EdgeL;

                //Sides.back().push_back(BorderHEdge);
                SidesLenght.back()+=EdgeL;

                BorderHEdge.NextB();

                //CoordType TestPos=BorderHEdge.P();
                BorderHEdge.V()->C()=vcg::Color4b(255,0,0,255);

                if (!OnCorner)
                {
                    CoordType PosV2=BorderHEdge.V()->P();
                    std::pair<CoordType,CoordType> key1(std::min(PosV0,PosV2),std::max(PosV0,PosV2));
                    assert(BorderEdges.count(key1)>0);
                    CoordType Dir1=BorderEdges[key1].second;

                    Dir0.Normalize();
                    Dir1.Normalize();
                    ScalarType currA=vcg::AngleN(Dir0,Dir1);
                    currA*=180.0/3.14;
                    CoordType side=Dir0^Dir1;
                    if (side*BorderHEdge.V()->N()<0)currA=-currA;
                    SideAngles.back()+=currA;
                }

                if ((OnCorner)&&(StartingHEdge!=BorderHEdge))
                {
                    SidesLenght.resize(SidesLenght.size()+1,0);
                    SideAngles.resize(SideAngles.size()+1,0);
                    i=(i+1)%2;
                }
            }
            if ((UpdateStats.count(IdealPatchDistortion)>0)||(UpdateStats.count(MaxGeodesicLenghtError)>0))
            {
                if ((OnCorner)&&(StartingHEdge!=BorderHEdge))
                    Corners.push_back(PosV0);
            }
        }while (StartingHEdge!=BorderHEdge);
        //        int t3=clock();
        //        std::cout<<"t0 "<<t1-t0<<std::endl;
        //        std::cout<<"t1 "<<t3-t1<<std::endl;
    }

    void GetPartitionStats(const std::vector<size_t> &Patch,
                           const std::set<ColorMode> &UpdateStats,
                           PartitionQuality &PQuality,
                           bool UseRealLenght=false)
    {
        ScalarType Area;ScalarType Perimeter;
        std::vector<ScalarType> SideAngles;
        std::vector<ScalarType> SidesLenght;
        std::vector<CoordType> Corners;

        PartitionData(Patch,UpdateStats,PQuality.NumHoles,PQuality.NumConcave,PQuality.IsManifold,Area,PQuality.NumEdges,Perimeter,SideAngles,SidesLenght,Corners,UseRealLenght);

        PQuality.AreaPerimeterError=0;
        if (UpdateStats.count(AreaPerimeterError)>0)
        {
            ScalarType EdgeSize=Perimeter/(ScalarType)SidesLenght.size();
            ScalarType IdealArea=0;
            if (SidesLenght.size()==3)IdealArea=0.433*EdgeSize*EdgeSize;
            if (SidesLenght.size()==4)IdealArea=EdgeSize*EdgeSize;
            if (SidesLenght.size()==5)IdealArea=1.75*EdgeSize*EdgeSize;
            if (SidesLenght.size()==6)IdealArea=2.598*EdgeSize*EdgeSize;
            PQuality.AreaPerimeterError=(fabs(IdealArea-Area)/IdealArea);
        }

        PQuality.MaxAngleDeviation=0;
        if (UpdateStats.count(MaxAngleDeviation)>0)
        {
            for (size_t j=0;j<SideAngles.size();j++)
                PQuality.MaxAngleDeviation=std::min(PQuality.MaxAngleDeviation,SideAngles[j]);
        }

        PQuality.MaxSideLenghtVariance=0;
        if (UpdateStats.count(MaxSideLenghtVariance)>0)
        {
            ScalarType IdealEdge=Perimeter/SidesLenght.size();
            for (size_t j=0;j<SidesLenght.size();j++)
                PQuality.MaxSideLenghtVariance=std::max(PQuality.MaxSideLenghtVariance,fabs(SidesLenght[j]-IdealEdge)/IdealEdge);
        }

        PQuality.MaxGeodesicLenghtError=0;
        if (UpdateStats.count(MaxGeodesicLenghtError)>0)
        {
            /*std::cout<<"Side0 "<<Corners.size()<<std::endl;
            std::cout<<"Side1 "<<SidesLenght.size()<<std::endl;*/
            assert(Corners.size()==SidesLenght.size());
            for (size_t j=0;j<Corners.size();j++)
            {
                ScalarType EuclideanLenght=(Corners[j]-Corners[(j+1)%Corners.size()]).Norm();
                ScalarType GeodesicLenghtError=fabs(SidesLenght[j]-EuclideanLenght)/EuclideanLenght;
                PQuality.MaxGeodesicLenghtError=std::max(PQuality.MaxGeodesicLenghtError,GeodesicLenghtError);
            }
        }
        PQuality.IdealPatchDistortion=0;
        if (UpdateStats.count(IdealPatchDistortion)>0)
        {
            PMesh PolygonMesh;
            vcg::tri::Allocator<PMesh>::AddVertices(PolygonMesh,Corners.size());
            vcg::tri::Allocator<PMesh>::AddFaces(PolygonMesh,1);
            PolygonMesh.face.back().Alloc(Corners.size());
            for (size_t j=0;j<Corners.size();j++)
            {
                PolygonMesh.vert[j].P()=Corners[j];
                PolygonMesh.face.back().V(j)=&PolygonMesh.vert[j];
            }
            vcg::PolygonalAlgorithm<PMesh>::UpdateFaceNormals(PolygonMesh);
            PQuality.IdealPatchDistortion=vcg::PolyAspectRatio(PolygonMesh.face.back(),false);
        }

    }

    bool IsOkPartitionHoles(std::vector<size_t> &Partition)
    {
        std::set<ColorMode> Stats;
        Stats.insert(Holes);

        PartitionQuality PQuality;
        GetPartitionStats(Partition,Stats,PQuality,true);

        if (PQuality.NumHoles!=1)return false;

        return true;
    }


    bool IsOkPartition(std::vector<size_t> &Partition)
    {
        std::set<ColorMode> Stats;
        GetStatsSet(Stats);

        PartitionQuality PQuality;
        GetPartitionStats(Partition,Stats,PQuality,true);

        if ((Param.TopologyChecks)&&(!PQuality.IsManifold))return false;
        if ((Param.TopologyChecks)&&(PQuality.NumHoles!=1))return false;
        if (PQuality.NumEdges<Param.MinSides)return false;
        if (PQuality.NumEdges>Param.MaxSides)return false;

        if (Param.MinAreaPerimeterRatio>0)
            if (PQuality.AreaPerimeterError>Param.MinAreaPerimeterRatio)return false;

        if (Param.MaxAngleDeviation<0)
            if (PQuality.MaxAngleDeviation<Param.MaxAngleDeviation)return false;

        if (Param.MaxEdgeLenghtVariance>0)
            if (PQuality.MaxSideLenghtVariance>Param.MaxEdgeLenghtVariance)return false;

        if (Param.MaxGeodesicToEuclideanRatio>0)
            if (PQuality.MaxGeodesicLenghtError>Param.MaxGeodesicToEuclideanRatio)return false;

        if (Param.MaxDistortion>0)
            if (PQuality.IdealPatchDistortion>Param.MaxDistortion)return false;

        return true;
    }

    bool IsOkPartitions(std::vector<std::vector<size_t> > &Partitions)
    {
        for (size_t i=0;i<Partitions.size();i++)
            if (!IsOkPartition(Partitions[i]))return false;

        return true;
    }

    void GetFacesSurroundingCandidate(const size_t &IndexCandidate,
                                      std::vector<size_t> &CandidateFaces)
    {
        CandidateFaces.clear();
        std::set<std::pair<size_t,size_t> > EdgeSet;
        for (size_t i=0;i<TraceVertCandidates[IndexCandidate].size()-1;i++)
        {
            size_t IndexV0=TraceVertCandidates[IndexCandidate][i];
            size_t IndexV1=TraceVertCandidates[IndexCandidate][i+1];
            std::pair<size_t,size_t>  Key(std::min(IndexV0,IndexV1),
                                          std::max(IndexV0,IndexV1));
            EdgeSet.insert(Key);
        }

        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                size_t IndexV0=vcg::tri::Index(mesh,mesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(mesh,mesh.face[i].V1(j));

                std::pair<size_t,size_t> key(std::min(IndexV0,IndexV1),
                                             std::max(IndexV0,IndexV1));
                if (EdgeSet.count(key)==0)continue;
                CandidateFaces.push_back(i);
            }
        std::sort(CandidateFaces.begin(),CandidateFaces.end());
        std::vector<size_t>::iterator last = std::unique(CandidateFaces.begin(), CandidateFaces.end());
        CandidateFaces.erase(last, CandidateFaces.end());
    }

    void GetPartitionsAfterRemoving(size_t &IndexCandidate,
                                    std::vector<std::vector<size_t> > &Partitions)
    {
        HasBeenChoosen[IndexCandidate]=false;
        InitBorderEdgeMap();
        //InitVerticesEdgeMap();

        //then retrieve the faces
        std::vector<size_t> CandidateFaces;
        GetFacesSurroundingCandidate(IndexCandidate,CandidateFaces);

        //std::cout<<"There are "<<CandidateFaces.size()<<"candidate faces "<<std::endl;

        //and get surrounding patches
        RetrievePartitioningAround(CandidateFaces,Partitions);
        //std::cout<<"There are "<<Partitions.size()<<"partitions "<<std::endl;

        HasBeenChoosen[IndexCandidate]=true;

    }

    void FindPartitioningPaths()
    {
        InitCandidatesPathLenghts();
        std::reverse(CandidatesPathLenghts.begin(),CandidatesPathLenghts.end());
        HasBeenChoosen.clear();
        HasBeenChoosen.resize(TraceVertCandidates.size(),true);
        bool removed=false;
        do
        {
            removed=false;
            for (size_t i=0;i<CandidatesPathLenghts.size();i++)
            {
                size_t IndexP=CandidatesPathLenghts[i].second;
                if (!HasBeenChoosen[IndexP])continue;

                std::vector<std::vector<size_t> > Partitions;
                GetPartitionsAfterRemoving(IndexP,Partitions);

                //check if I can sefely remove or not
                if (IsOkPartitions(Partitions))
                {
                    removed=true;
                    HasBeenChoosen[IndexP]=false;
                }
            }
        }while (removed);
        TraceVert.clear();
        TraceDir.clear();
        size_t num_removed=0;
        for (size_t i=0;i<TraceVertCandidates.size();i++)
        {
            if (!HasBeenChoosen[i]){num_removed++;continue;}
            TraceVert.push_back(TraceVertCandidates[i]);
            TraceDir.push_back(TraceDirCandidates[i]);
        }
        std::cout<<"Removed "<<num_removed<<std::endl;
    }

    void SmoothPartitionsStep()
    {
        InitBorderEdgeMap();
        std::vector<CoordType> AvPos(mesh.vert.size(),CoordType(0,0,0));
        std::vector<size_t> NumDiv(mesh.vert.size(),0);

        vcg::tri::UpdateFlags<MeshType>::VertexClearS(mesh);
        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                CoordType Pos0=mesh.face[i].P0(j);
                CoordType Pos1=mesh.face[i].P1(j);
                size_t IndexV0=vcg::tri::Index(mesh,mesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(mesh,mesh.face[i].V1(j));

                std::pair<CoordType,CoordType> key(std::min(Pos0,Pos1),
                                                   std::max(Pos0,Pos1));

                if (vcg::face::IsBorder(mesh.face[i],j))continue;
                if (BorderEdges.count(key)==0)continue;
                mesh.vert[IndexV0].SetS();
                mesh.vert[IndexV1].SetS();
                AvPos[IndexV0]+=Pos1;
                AvPos[IndexV1]+=Pos0;
                NumDiv[IndexV0]++;
                NumDiv[IndexV1]++;
            }
        for (size_t i=0;i<AvPos.size();i++)
        {
            if (mesh.vert[i].IsB()){
                mesh.vert[i].SetS();
                continue;
            }
            if (NumDiv[i]==0)continue;
            AvPos[i]/=NumDiv[i];
            mesh.vert[i].P()=mesh.vert[i].P()*0.5+AvPos[i]*0.5;
        }
        vcg::tri::UpdateSelection<MeshType>::VertexInvert(mesh);
        vcg::tri::Smooth<MeshType>::VertexCoordLaplacian(mesh,1,true);

    }

    void SmoothPartitionsSteps(size_t SmoothSteps)
    {

        //save old normal
        std::vector<CoordType> OldNorm;
        for (size_t i=0;i<mesh.face.size();i++)
            OldNorm.push_back(mesh.face[i].N());

        MeshType currM;
        vcg::tri::Append<MeshType,MeshType>::Mesh(currM,mesh);
        UpdateMeshAttributes(currM);

        vcg::GridStaticPtr<FaceType,typename FaceType::ScalarType> Grid;
        Grid.Set(currM.face.begin(),currM.face.end());
        for (size_t i=0;i<SmoothSteps;i++)
        {
            SmoothPartitionsStep();
            ScalarType maxD=mesh.bbox.Diag();
            ScalarType minD=0;
            CoordType closestPT;
            for (size_t i=0;i<mesh.vert.size();i++)
            {
                FaceType *f=vcg::tri::GetClosestFaceBase(mesh,Grid,mesh.vert[i].P(),maxD,minD,closestPT);
                mesh.vert[i].P()=closestPT;
            }
        }

        UpdateMeshAttributes(mesh);
        //vcg::tri::io::ExporterPLY<MeshType>::Save(mesh,"testMesh.ply");
        for (size_t i=0;i<mesh.face.size();i++)
        {
            vcg::Matrix33<ScalarType> Rot;
            Rot=vcg::RotationMatrix(OldNorm[i],mesh.face[i].N());
            mesh.face[i].PD1()=Rot*mesh.face[i].PD1();
            mesh.face[i].PD2()=mesh.face[i].PD1()^mesh.face[i].N();
            mesh.face[i].PD1().Normalize();
            mesh.face[i].PD2().Normalize();
        }

        vcg::tri::CrossField<MeshType>::SetVertCrossVectorFromFace(mesh);
    }

    void DetectBorderConstraints()
    {

        BorderCornersConvex.clear();
        BorderCornersConcave.clear();

        std::vector<size_t> ConvexV,ConcaveV;
        GetCorners(mesh,ConvexV,ConcaveV);
        BorderCornersConvex.insert(ConvexV.begin(),ConvexV.end());
        BorderCornersConcave.insert(ConcaveV.begin(),ConcaveV.end());

        //then find the non-manifold ones
        vcg::tri::UpdateSelection<MeshType>::Clear(mesh);
        size_t non_manifV=vcg::tri::Clean<MeshType>::CountNonManifoldVertexFF(mesh);
        if (non_manifV>0)std::cout<<"WARNING: Non Manifold Vertices: "<<non_manifV<<std::endl;
        for (size_t i=0;i<mesh.vert.size();i++)
        {
            if (!mesh.vert[i].IsB())continue;
            if (!mesh.vert[i].IsS())continue;
            BorderCornersConvex.insert(i);
        }
    }


    std::set<std::pair<size_t,size_t> > NonTracedCorners;
    std::set<std::pair<size_t,size_t> > NonExpandedCorners;

    void TraceConcaveCorners()
    {
        NonTracedCorners.clear();
        NonExpandedCorners.clear();

        assert(StartTraceV.size()==StartTraceDir.size());
        size_t non_expanded=0;
        size_t non_traced=0;
        size_t tested=0;
        std::cout<<"Tracing Concave Corners"<<std::endl;

        for (size_t i=0;i<StartTraceV.size();i++)
        {
            int StartV=StartTraceV[i];
            if (BorderCornersConcave.count(StartV)==0)continue;
            for (size_t j=0;j<StartTraceDir[i].size();j++)
            {
                tested++;
                std::vector<size_t> IndexV,IndexDir;
                bool traced=FieldTracer->TraceFrom(StartTraceV[i],
                                                   StartTraceDir[i][j],
                                                   IndexV,IndexDir);
                if (!traced)
                {
                    non_traced++;
                    NonTracedCorners.insert(std::pair<size_t,size_t>(StartTraceV[i],StartTraceDir[i][j]));
                    std::cout<<"Non Traced "<<StartTraceV[i]<<" "<<StartTraceDir[i][j]<<std::endl;
                    continue;
                }

                //return;
                bool expanded=FieldTracer->ExpandPath(IndexV,IndexDir);
                if (!expanded)
                {
                    non_expanded++;
                    NonExpandedCorners.insert(std::pair<size_t,size_t>(i,j));
                    std::cout<<"Non Expanded "<<StartTraceV[i]<<" "<<StartTraceDir[i][j]<<std::endl;
                    continue;
                }

                TraceVertCandidates.push_back(IndexV);
                TraceDirCandidates.push_back(IndexDir);
            }
        }
        std::cout<<"Non Traced "<<non_traced<<std::endl;
        std::cout<<"Non Expanded "<<non_expanded<<std::endl;
        std::cout<<"Tested "<<tested<<std::endl;
    }

    //    void CheckConcavePaths()
    //    {

    //    }

    void InitField(MeshType &TempMesh)
    {
        typename vcg::tri::FieldSmoother<MeshType>::SmoothParam SParam;
        SParam.align_borders=true;
        SParam.curvRing=1;
        SParam.alpha_curv=0;
        SParam.Ndir=4;
        SParam.sharp_thr=0;
        SParam.curv_thr=0;
        SParam.SmoothM=vcg::tri::SMNPoly;
        //the number of faces of the ring used ot esteem the curvature
        std::cout<<"before"<<std::endl;
        vcg::tri::FieldSmoother<MeshType>::SmoothDirections(TempMesh,SParam);
        vcg::tri::CrossField<MeshType>::SetVertCrossVectorFromFace(TempMesh);
        std::cout<<"after"<<std::endl;
    }

    void Reset()
    {
        StartTraceV.clear();
        StartTraceDir.clear();

        TraceVertCandidates.clear();
        HasBeenChoosen.clear();
        TraceDirCandidates.clear();

        TraceVert.clear();
        TraceDir.clear();

        CandidatesPathLenghts.clear();

        BorderEdges.clear();

        //PatchCorners.clear();


        if (FieldTracer!=NULL)
            delete(FieldTracer);

        FieldTracer=new VertexFieldTracer<MeshType>(mesh);

    }

    void GetStatsSet(std::set<ColorMode> &Stats)
    {
        Stats.clear();
        Stats.insert(Partitions);
        Stats.insert(Manifold);
        Stats.insert(Holes);
        Stats.insert(NumEdges);
        if (Param.MinAreaPerimeterRatio>0)
            Stats.insert(AreaPerimeterError);
        if (Param.MaxEdgeLenghtVariance>0)
            Stats.insert(MaxSideLenghtVariance);
        if (Param.MaxGeodesicToEuclideanRatio>0)
            Stats.insert(MaxGeodesicLenghtError);
        if (Param.MaxAngleDeviation>0)
            Stats.insert(MaxAngleDeviation);
        if (Param.MaxDistortion>0)
            Stats.insert(IdealPatchDistortion);
    }

    void SplitMeshNonOkPartition(std::vector<std::vector<size_t> > &FacePartitions,
                                 MeshType &OKMesh,
                                 MeshType &NonOKMesh,
                                 bool checkHolesOnly)
    {
        OKMesh.Clear();
        NonOKMesh.Clear();

        //then save the partitions on quality
        for (size_t i=0;i<FacePartitions.size();i++)
            for (size_t j=0;j<FacePartitions[i].size();j++)
                mesh.face[FacePartitions[i][j]].Q()=i;

        //vcg::tri::UpdateSelection<MeshType>::FaceClear(mesh);
        std::vector<size_t> IndexF;
        for (size_t i=0;i<FacePartitions.size();i++)
        {
            if ((checkHolesOnly)&&(IsOkPartitionHoles(FacePartitions[i])))continue;
            if ((!checkHolesOnly)&&(IsOkPartition(FacePartitions[i])))continue;

            for (size_t j=0;j<FacePartitions[i].size();j++)
                IndexF.push_back(FacePartitions[i][j]);
                //mesh.face[FacePartitions[i][j]].SetS();
        }
        vcg::tri::UpdateSelection<MeshType>::FaceClear(mesh);
        for (size_t i=0;i<IndexF.size();i++)
            mesh.face[IndexF[i]].SetS();

        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);
        vcg::tri::Append<MeshType,MeshType>::Mesh(NonOKMesh,mesh,true);

        vcg::tri::UpdateSelection<MeshType>::VertexClear(mesh);
        vcg::tri::UpdateSelection<MeshType>::FaceInvert(mesh);
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(mesh);
        vcg::tri::Append<MeshType,MeshType>::Mesh(OKMesh,mesh,true);

        vcg::tri::UpdateSelection<MeshType>::FaceClear(mesh);
        vcg::tri::UpdateSelection<MeshType>::VertexClear(mesh);
    }

    void TraceFromBorders()
    {
        InitStartDirections();
        for (size_t i=0;i<StartTraceV.size();i++)
        {
            //std::cout<<"Size "<<StartTraceDir[i].size()<<std::endl;

            for (size_t j=0;j<StartTraceDir[i].size();j++)
            {
                std::vector<size_t> IndexV,IndexDir;
                bool traced=FieldTracer->TraceFrom(StartTraceV[i],StartTraceDir[i][j],IndexV,IndexDir);
                if (!traced)continue;
                bool expanded=FieldTracer->ExpandPath(IndexV,IndexDir);
                if (!expanded)continue;

                TraceVertCandidates.push_back(IndexV);
                TraceDirCandidates.push_back(IndexDir);
            }
        }
        std::cout<<"Traced successfully "<<TraceVertCandidates.size()<<" out of "<<StartTraceV.size()<<std::endl;
        PruneCollidingCandidates();
        TraceVert=TraceVertCandidates;
        TraceDir=TraceDirCandidates;
        //for (size_t i=0;i<Param.SmoothSteps;i++)
        SmoothPartitionsSteps(Param.SmoothSteps);
        FindPartitioningPaths();
    }

    void GetClosingMesh(MeshType &ToClose,MeshType &Closing)
    {
        Closing.Clear();
        size_t holeSize=ToClose.face.size();
        size_t f0=ToClose.face.size();        
        UpdateMeshAttributes(ToClose);
        vcg::tri::Hole<MeshType>::template EarCuttingFill<vcg::tri::TrivialEar<MeshType> >(ToClose,holeSize);
        size_t f1=ToClose.face.size();
        std::cout<<"*** F0 "<<f0<<std::endl;
        std::cout<<"*** F1 "<<f1<<std::endl;

        vcg::tri::UpdateSelection<MeshType>::FaceClear(ToClose);
        for (size_t i=f0;i<f1;i++)
            ToClose.face[i].SetS();
        vcg::tri::UpdateSelection<MeshType>::VertexFromFaceLoose(ToClose);

        vcg::tri::Append<MeshType,MeshType>::Mesh(Closing,ToClose,true);

        UpdateMeshAttributes(Closing);
        for (size_t i=0;i<3;i++)
        {
            vcg::PolygonalAlgorithm<MeshType>::Triangulate(Closing);
            UpdateMeshAttributes(Closing);
        }
        Remesh(Closing);
    }

    void GetCurrentPathCoords(std::vector<std::vector<CoordType > > &PathPos)
    {
        PathPos.clear();
        for (size_t i=0;i<TraceVert.size();i++)
        {
            PathPos.push_back(std::vector<CoordType >());
            for (size_t j=0;j<TraceVert[i].size();j++)
            {
                CoordType Pos0=mesh.vert[TraceVert[i][j]].P();;
                PathPos.back().push_back(Pos0);
            }
        }
    }

    void FilterPathCoords(const MeshType &FilterMesh,
                          std::vector<std::vector<CoordType > > &PathPos)
    {
        std::set<std::pair<CoordType,CoordType> > EdgeSet;
        for (size_t i=0;i<FilterMesh.face.size();i++)
            for (size_t j=0;j<3;j++)
            {
                CoordType Pos0=FilterMesh.face[i].cP0(j);
                CoordType Pos1=FilterMesh.face[i].cP1(j);
                EdgeSet.insert(std::pair<CoordType,CoordType>(std::min(Pos0,Pos1),std::max(Pos0,Pos1)));
            }

        std::vector<std::vector<CoordType > > SwapPos;
        for (size_t i=0;i<PathPos.size();i++)
        {
            SwapPos.push_back(std::vector<CoordType >());
            for (size_t j=0;j<PathPos[i].size()-1;j++)
            {
                CoordType Pos0=PathPos[i][j];
                CoordType Pos1=PathPos[i][j+1];
                std::pair<CoordType,CoordType> Key(std::min(Pos0,Pos1),std::max(Pos0,Pos1));
                if (EdgeSet.count(Key)==0)continue;
                SwapPos.back().push_back(Pos0);
                SwapPos.back().push_back(Pos1);
            }
            auto last = std::unique(SwapPos.back().begin(), SwapPos.back().end());
            SwapPos.back().erase(last, SwapPos.back().end());
        }

        PathPos.clear();
        PathPos=SwapPos;

        //then split the one that need
        for (size_t i=0;i<PathPos.size();i++)
        {
            if (PathPos[i].size()==0)continue;
            for (size_t j=0;j<PathPos[i].size()-1;j++)
            {
                CoordType Pos0=PathPos[i][j];
                CoordType Pos1=PathPos[i][j+1];
                std::pair<CoordType,CoordType> Key(std::min(Pos0,Pos1),std::max(Pos0,Pos1));
                if (EdgeSet.count(Key)==0){
                    PathPos.push_back(std::vector<CoordType >());
                    PathPos.back()=std::vector<CoordType>(PathPos[i].begin()+j+1,PathPos[i].end());
                    PathPos[i].resize(j+1);
                }
            }
        }


        //
    }

    void PathCoordsToIndex(const MeshType &TraceMesh,
                           std::vector<std::vector<CoordType > > &PathPos,
                           std::vector<std::vector<size_t> > &PathIndex)
    {
        std::map<CoordType,size_t> VertSet;
        for (size_t i=0;i<TraceMesh.vert.size();i++)
            VertSet[TraceMesh.vert[i].P()]=i;

        PathIndex.clear();
        int found=0;
        int non_found=0;
        for (size_t i=0;i<PathPos.size();i++)
        {
            if (PathPos[i].size()==0)continue;
            PathIndex.push_back(std::vector<size_t>());
            for (size_t j=0;j<PathPos[i].size();j++)
            {
                //assert(VertSet.count(PathPos[i][j])>0);
                if (VertSet.count(PathPos[i][j])==0){non_found++;continue;}
                found++;
                PathIndex.back().push_back(VertSet[PathPos[i][j]]);
            }
        }
        std::cout<<"found: "<<found<<std::endl;
        std::cout<<"not found: "<<non_found<<std::endl;
    }

    void SplitBadPartitions()
    {
        std::vector<std::vector<CoordType > > PathPos0;
        GetCurrentPathCoords(PathPos0);
        //PathDir0=TraceDir;

        std::cout<<"*** Getting non ok partitions ***"<<std::endl;
        std::vector<std::vector<size_t> > FacePartitions;
        GetPartitions(FacePartitions);
        //SavePartitionOnQuality(FacePartitions,mesh);
        //size_t Offset0=FacePartitions.size();

        MeshType OKMesh0,NonOKMesh0;
        SplitMeshNonOkPartition(FacePartitions,OKMesh0,NonOKMesh0,false);
        if (NonOKMesh0.face.size()==0)return;

        std::cout<<"*** filtering ***"<<std::endl;
        //FilterPathCoords(OKMesh0,PathPos0);
        mesh.Clear();

        std::cout<<"*** Retracing bad partitions ***"<<std::endl;
        vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,NonOKMesh0);
        UpdateMeshAttributes(mesh);
        vcg::tri::CrossField<MeshType>::SetVertCrossVectorFromFace(mesh);

        //std::cout<<"A"<<std::endl;
        Reset();
        //std::cout<<"B"<<std::endl;

        //std::cout<<"C"<<std::endl;
        DetectBorderConstraints();
        //std::cout<<"D"<<std::endl;
        FieldTracer->Init(BorderCornersConvex);
        //std::cout<<"E"<<std::endl;
        //std::cout<<"AZZ"<<std::endl;
        InitStartDirections();
        //std::cout<<"BAOI"<<std::endl;
        TraceFromBorders();
        //std::cout<<"DE"<<std::endl;
        std::vector<std::vector<CoordType > > PathPos1;
        GetCurrentPathCoords(PathPos1);
        //PathDir1=TraceDir;

        std::cout<<"*** Getting non ok partitions ***"<<std::endl;
        FacePartitions.clear();
        GetPartitions(FacePartitions);
        //size_t Offset1=FacePartitions.size();
        //SavePartitionOnQuality(FacePartitions,mesh,Offset0);
        //return;

        std::cout<<"*** getting the ones with more than 1 hole ***"<<std::endl;
        MeshType OKMesh1,NonOKMesh1;
        vcg::tri::io::ExporterPLY<MeshType>::Save(mesh,"testA.ply");
        std::cout<<"there are "<<FacePartitions.size()<<" partitions"<<std::endl;
        SplitMeshNonOkPartition(FacePartitions,OKMesh1,NonOKMesh1,true);
        vcg::tri::io::ExporterPLY<MeshType>::Save(OKMesh1,"test0.ply");
        vcg::tri::io::ExporterPLY<MeshType>::Save(NonOKMesh1,"test1.ply");
        MeshType Closing;
        if (NonOKMesh1.face.size()>0)
        {
            //std::cout<<"*** NonOKMesh "<<NonOKMesh.face.size()<<std::endl;
            GetClosingMesh(NonOKMesh1,Closing);
            //std::cout<<"*** de boia 0 ***"<<std::endl;

            mesh.Clear();
            vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,Closing);
            UpdateMeshAttributes(mesh);
            TraceVert.clear();
            TraceDir.clear();
            //std::cout<<"*** de boia 1 ***"<<std::endl;

            GetPartitions(FacePartitions);
            std::cout<<"*** de boia 2 ***"<<std::endl;
            //SavePartitionOnQuality(FacePartitions,mesh,Offset1);

            std::cout<<"*** reassemble ***"<<std::endl;
        }

        mesh.Clear();
        vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,OKMesh0);
        vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,OKMesh1);
        if (NonOKMesh1.face.size()>0)
            vcg::tri::Append<MeshType,MeshType>::Mesh(mesh,Closing);

        vcg::tri::Clean<MeshType>::RemoveDuplicateVertex(mesh);
        vcg::tri::Clean<MeshType>::RemoveUnreferencedVertex(mesh);
        vcg::tri::Allocator<MeshType>::CompactEveryVector(mesh);
        UpdateMeshAttributes(mesh);
        //GetPartitionFromQuality(mesh,FacePartitions);

        std::vector<std::vector<CoordType > > PathPos2=PathPos0;
        PathPos2.insert(PathPos2.end(),PathPos1.begin(),PathPos1.end());

        //        std::vector<std::vector<CoordType > > PathDir2=PathDir0;
        //        PathDir2.insert(PathPos2.end(),PathPos1.begin(),PathPos1.end());

        Reset();
        FilterPathCoords(mesh,PathPos2);
        PathCoordsToIndex(mesh,PathPos2,TraceVert);

        TraceVertCandidates=TraceVert;
        TraceDir.resize(TraceVert.size());

        for (size_t i=0;i<TraceVert.size();i++)
        {
            TraceDir[i].resize(TraceVert[i].size(),0);

            CoordType Pos0=mesh.vert[TraceVert[i][0]].P();
            CoordType Pos1=mesh.vert[TraceVert[i][1]].P();
            CoordType Dir=Pos1-Pos0;
            Dir.Normalize();
            TraceDir[i][0]=FieldTracer->GetClosestDirTo(TraceVert[i][0],Dir);

            for (size_t j=1;j<TraceVert[i].size();j++)
                TraceDir[i][j]=FieldTracer->FollowDirection(TraceVert[i][j-1],TraceVert[i][j],TraceDir[i][j-1]);
        }
        TraceDirCandidates=TraceDir;
        HasBeenChoosen=std::vector<bool>(TraceVertCandidates.size(),true);

        SmoothPartitionsSteps(Param.SmoothSteps);
        ColorPatches(Partitions);
        std::cout<<"*** DONE ***"<<std::endl;

    }


    void Remesh(MeshType &to_remesh)
    {
        ScalarType EdgeStep=0;
        size_t Num=0;
        for (size_t i=0;i<to_remesh.face.size();i++)
            for (size_t j=0;j<to_remesh.face[i].VN();j++)
            {
                size_t IndexV0=vcg::tri::Index(to_remesh,to_remesh.face[i].V0(j));
                size_t IndexV1=vcg::tri::Index(to_remesh,to_remesh.face[i].V1(j));
                CoordType P0=to_remesh.vert[IndexV0].P();
                CoordType P1=to_remesh.vert[IndexV1].P();
                EdgeStep+=(P0-P1).Norm();
                Num++;
            }
        EdgeStep/=Num;
        typename vcg::tri::IsotropicRemeshing<MeshType>::Params Par;

        Par.swapFlag     = true;
        Par.collapseFlag = true;
        Par.smoothFlag=true;
        Par.projectFlag=Param.Reproject;
        Par.SetFeatureAngleDeg(100);
        Par.SetTargetLen(EdgeStep);
        Par.selectedOnly=true;
        Par.iter=20;

        vcg::tri::UpdateSelection<MeshType>::FaceAll(to_remesh);
        for (size_t i=0;i<to_remesh.face.size();i++)
            for (size_t j=0;j<to_remesh.face[i].VN();j++)
            {
                if (!vcg::face::IsBorder(to_remesh.face[i],j))continue;
                to_remesh.face[i].ClearS();
            }
        std::cout<<"first pre-remeshing step"<<std::endl;
        std::cout<<"Edge Size " <<EdgeStep<<std::endl;
        MeshType Reproject;
        vcg::tri::Append<MeshType,MeshType>::Mesh(Reproject,to_remesh);
        UpdateMeshAttributes(Reproject);

        vcg::tri::IsotropicRemeshing<MeshType>::Do(to_remesh,Reproject,Par);
        std::cout<<std::flush;
    }

public:

    void InitField()
    {
        InitField(mesh);
    }

    void RemeshStep()
    {
        if (Param.InitialSmooth)
        {
            vcg::tri::UpdateSelection<MeshType>::VertexFromBorderFlag(mesh);

            if (Param.Reproject)
            {
                for (size_t i=0;i<mesh.vert.size();i++)
                {
                    if (mesh.vert[i].IsS())
                        mesh.vert[i].ClearS();
                    else
                        mesh.vert[i].SetS();
                }
                vcg::PolygonalAlgorithm<MeshType>::LaplacianReproject(mesh,3,0.5,true);
            }
            else
            {
                vcg::PolygonalAlgorithm<MeshType>::Laplacian(mesh,true,3,0.5);
            }
//            vcg::tri::Smooth<MeshType>::VertexCoordLaplacian(mesh,3,true);
            UpdateMeshAttributes(mesh);
        }

        //refine close to the border
        vcg::tri::UpdateSelection<MeshType>::FaceClear(mesh);
        for (size_t i=0;i<mesh.face.size();i++)
            for (size_t j=0;j<mesh.face[i].VN();j++)
            {
                if (!vcg::face::IsBorder(mesh.face[i],j))continue;
                mesh.face[i].SetS();
            }
        vcg::PolygonalAlgorithm<MeshType>::Triangulate(mesh,true,true);

        UpdateMeshAttributes(mesh);

        Remesh(mesh);

        UpdateMeshAttributes(mesh);
    }

#ifdef DRAWTRACE
    void GLDrawStartingDir()
    {
        //

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        //        vcg::glColor(vcg::Color4b(0,255,0,255));
        glDisable(GL_LIGHTING);
        glDepthRange(0,0.99998);
        //        glPointSize(12);
        //        glBegin(GL_POINTS);
        //        for (size_t i=0;i<StartTraceV.size();++i)
        //            vcg::glVertex(mesh.vert[StartTraceV[i]].P());
        //        glEnd();

        //        assert(StartTraceDir.size()==StartTraceV.size());

        //        std::set<std::pair<size_t,size_t> >::iterator IteNonTraced=NonTracedCorners.begin();
        //        for (IteNonTraced;IteNonTraced!=NonTracedCorners.end();IteNonTraced++)
        //        {
        //            size_t IndexV=(*IteNonTraced).first;
        //            size_t IndexDir=(*IteNonTraced).second;
        //            CoordType Pos0=mesh.vert[IndexV].P();
        //            CoordType Dir=FieldTracer->GetDirection(IndexV,IndexDir);
        //            CoordType Pos1=Pos0+Dir*mesh.bbox.Diag()*0.002;
        //            vcg::glColor(vcg::Color4b(255,0,0,255));
        //            glLineWidth(10);
        //            glBegin(GL_LINES);
        //            vcg::glVertex(Pos0);
        //            vcg::glVertex(Pos1);
        //            glEnd();

        //        }

        for (size_t i=0;i<StartTraceV.size();++i)
        {
            CoordType Pos0=mesh.vert[StartTraceV[i]].P();
            if (BorderCornersConcave.count(StartTraceV[i])==0)continue;
            for (size_t j=0;j<StartTraceDir[i].size();++j)
            {
                CoordType Dir=FieldTracer->GetDirection(StartTraceV[i],StartTraceDir[i][j]);
                CoordType Pos1=Pos0+Dir*mesh.bbox.Diag()*0.01;
                //                        if (NonTracedCorners.count(std::pair<size_t,size_t>(StartTraceV[i],StartTraceDir[i][j]))==1)
                //                        {
                vcg::glColor(vcg::Color4b(255,0,0,255));
                glLineWidth(20);
                glBegin(GL_LINES);
                vcg::glVertex(Pos0);
                vcg::glVertex(Pos1);
                glEnd();
                //                        }
                //                        else continue;
                //                if (NonExpandedCorners.count(std::pair<size_t,size_t>(i,j))==1)
                //                {
                //                    vcg::glColor(vcg::Color4b(255,255,0,255));
                //                    glLineWidth(10);
                //                }
                //                else
                //                {
                //                    vcg::glColor(vcg::Color4b(0,0,255,255));
                //                    glLineWidth(5);
                //                }

            }
        }
        glPopAttrib();
    }
#endif

    void InitTracing()
    {
        Reset();

        DetectBorderConstraints();

        FieldTracer->Init(BorderCornersConvex);

        InitStartDirections();
    }

    void GLDraw()
    {
        for (size_t i=0;i<TraceVert.size();i++)
        {
            vcg::Color4b TraceCol=vcg::Color4b::Scatter(TraceVert.size(),i);
            DrawTrace(i,TraceCol);
        }

        //        glPushAttrib(GL_ALL_ATTRIB_BITS);
        //        glDisable(GL_LIGHTING);
        //        glDepthRange(0,0.99998);
        //        glPointSize(20);

        //        glBegin(GL_POINTS);
        //        for (size_t i=0;i<mesh.vert.size();++i)
        //        {
        //            if (BorderCornersConcave.count(i)>0)
        //            {
        //                vcg::glColor(vcg::Color4b(0,0,255,255));
        //                vcg::glVertex(mesh.vert[i].P());
        //            }else
        //                if (BorderCornersConvex.count(i)>0)
        //                {
        //                    vcg::glColor(vcg::Color4b(255,0,0,255));
        //                    vcg::glVertex(mesh.vert[i].P());
        //                }
        ////            if (PatchCorners.count(mesh.vert[i].P())>0)
        ////            {
        ////                vcg::glColor(vcg::Color4b(0,255,0,255));
        ////                vcg::glVertex(mesh.vert[i].P());
        ////            }
        //        }
        //        glEnd();

        //        glPopAttrib();

    }

    struct Parameters
    {
        bool TopologyChecks;
        size_t MinSides,MaxSides;
        ScalarType MinAreaPerimeterRatio;
        ScalarType MaxAngleDeviation;
        ScalarType MaxEdgeLenghtVariance;
        ScalarType MaxGeodesicToEuclideanRatio;
        ScalarType MaxDistortion;
        size_t SmoothSteps;
        bool Reproject;
        bool InitialRemesh;
        bool InitialSmooth;

        Parameters()
        {
            TopologyChecks=true;
            MinAreaPerimeterRatio=-1;//0.5;
            MinSides=3;
            MaxSides=6;
            MaxAngleDeviation=-1;//-90;
            MaxEdgeLenghtVariance=-1;
            MaxGeodesicToEuclideanRatio=-1;//0.5;
            MaxDistortion=-1;//0.5;
            SmoothSteps=10;
            Reproject=false;
            InitialRemesh=true;
            InitialSmooth=false;
        }
    };

    Parameters Param;

    void SetParam(const Parameters &_Param)
    {
        Param=_Param;
    }

    void TraceBorders()
    {
        TraceVertCandidates.clear();
        TraceDirCandidates.clear();


        TraceConcaveCorners();

        PruneConcaveCornersCandidates(false);

        HasBeenChoosen.clear();
        HasBeenChoosen.resize(TraceVertCandidates.size(),true);

        //InitVerticesEdgeMap();

        TraceVert=TraceVertCandidates;
        TraceDir=TraceDirCandidates;

        SmoothPartitionsSteps(Param.SmoothSteps);

        vcg::tri::UpdateNormal<MeshType>::PerFace(mesh);
        vcg::tri::UpdateNormal<MeshType>::PerVertex(mesh);

        ColorPatches(Partitions);

        SplitBadPartitions();
    }

    //Parameters Param;

    void BatchProcess(std::vector<std::vector<size_t> > &Partitions,
                      std::vector<std::vector<size_t> > &Corners)
    {
        UpdateMeshAttributes(mesh);

//        if (Param.InitialRemesh)
//            RemeshStep();

//       InitField();

//       InitTracing();

//       TraceBorders();
        if (Param.InitialRemesh)
        RemeshStep();

        InitField();

//        mesh.UpdateAttributes();
//        vcg::tri::CrossField<MyTriMesh>::SetVertCrossVectorFromFace(mesh);

        //PatchDeco.SetParam(Param);
        InitTracing();

        TraceBorders();

        //RType=OldRType=QuadBoolean::internal::PatchDecomposer<MyTriMesh>::Partitions;


       GetPartitions(Partitions);

       vcg::tri::Clean<MeshType>::SplitNonManifoldVertex(mesh,0);
       UpdateMeshAttributes(mesh);
       int nonManifV=vcg::tri::Clean<MeshType>::CountNonManifoldVertexFF(mesh);
       int nonManifE=vcg::tri::Clean<MeshType>::CountNonManifoldEdgeFF(mesh);
       std::cout<<"non manuf V:"<<nonManifV<<std::endl;
       std::cout<<"non manuf E:"<<nonManifE<<std::endl;
       //copy index on quality
       for (size_t i=0;i<mesh.vert.size();i++)
        mesh.vert[i].Q()=i;

       Corners.resize(Partitions.size());
       std::cout<<"There are:"<<Partitions.size()<<std::endl;
       for (size_t i=0;i<Partitions.size();i++)
       {
           MeshType TempMesh;
           GetPartitionMesh(Partitions[i],TempMesh);
           std::vector<size_t> IndexC;
           GetCorners(TempMesh,IndexC);

//           std::cout<<"Partition: "<<i<<std::endl;
//           std::cout<<"Corners: "<<IndexC.size()<<std::endl;

           for (size_t j=0;j<IndexC.size();j++)
           {
               IndexC[j]=TempMesh.vert[IndexC[j]].Q();
               //std::cout<<"Index "<<IndexC[j]<<std::endl;
            }
           Corners[i]=IndexC;
       }
    }

    PatchDecomposer(MeshType &_mesh):mesh(_mesh)
    {
        FieldTracer=NULL;
    }

    ~PatchDecomposer()
    {
        if (FieldTracer!=NULL)
            delete(FieldTracer);
    }
};

}
}
#endif
