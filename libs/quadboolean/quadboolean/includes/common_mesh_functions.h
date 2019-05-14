#ifndef COMMON_MESH_FUNCTIONS
#define COMMON_MESH_FUNCTIONS

namespace QuadBoolean {
namespace internal {


template <class MeshType>
void UpdateAttributes(MeshType &curr_mesh)
{
    vcg::tri::UpdateNormal<MeshType>::PerFaceNormalized(curr_mesh);
    vcg::tri::UpdateNormal<MeshType>::PerVertexNormalized(curr_mesh);
    vcg::tri::UpdateBounding<MeshType>::Box(curr_mesh);
    vcg::tri::UpdateTopology<MeshType>::FaceFace(curr_mesh);
    vcg::tri::UpdateTopology<MeshType>::VertexFace(curr_mesh);
    vcg::tri::UpdateFlags<MeshType>::FaceBorderFromFF(curr_mesh);
    vcg::tri::UpdateFlags<MeshType>::VertexBorderFromNone(curr_mesh);
}

template <class MeshType>
void FindConnectedComponents(const MeshType &mesh,
                             std::vector<std::vector<size_t> > &Components)
{
    Components.clear();
    std::set<size_t> explored;
    //get connected components
    for (size_t i=0;i<mesh.face.size();i++)
    {
        std::vector<size_t> stack;
        size_t IndexF=i;
        if (explored.count(IndexF)>0)continue;

        stack.push_back(IndexF);
        explored.insert(IndexF);
        Components.resize(Components.size()+1);
        do
        {
            size_t currF=stack.back();
            stack.pop_back();

            Components.back().push_back(currF);
            for (size_t i=0;i<mesh.face[currF].VN();i++)
            {
                if (vcg::face::IsBorder(mesh.face[currF],i))continue;

                int NextFIndex=vcg::tri::Index(mesh,mesh.face[currF].cFFp(i));

                if (explored.count(NextFIndex)>0)continue;

                explored.insert(NextFIndex);
                stack.push_back(NextFIndex);
            }
        }while (!stack.empty());
    }
    //sort and make unique
    for (size_t i=0;i<Components.size();i++)
    {
        std::sort(Components[i].begin(),Components[i].end());
        auto last=std::unique(Components[i].begin(),Components[i].end());
        Components[i].erase(last, Components[i].end());
    }
}

}
}

#endif
