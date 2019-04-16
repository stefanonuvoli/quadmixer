#include "quadbooleaninterface.h"

#ifdef __GNUC__
#ifndef __clang__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-template-friend"
#include <igl/copyleft/cgal/CSGTree.h>
#pragma GCC diagnostic pop
#else //__clang__
#include <igl/copyleft/cgal/CSGTree.h>
#endif //__clang__
#else //__GNUC__
#include <igl/copyleft/cgal/CSGTree.h>
#endif //__GNUC__


#include <vcg/complex/algorithms/mesh_to_matrix.h>

void trimeshUnion(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J)
{
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,igl::MESH_BOOLEAN_TYPE_UNION,VR,FR,J);
}
