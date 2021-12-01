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

#include "quadlibiglbooleaninterface.h"

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

namespace QuadBoolean {
namespace internal {

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

void trimeshDifference(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J)
{
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,igl::MESH_BOOLEAN_TYPE_MINUS,VR,FR,J);
}

void trimeshIntersection(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J)
{
    igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,igl::MESH_BOOLEAN_TYPE_INTERSECT,VR,FR,J);
}

}
}
