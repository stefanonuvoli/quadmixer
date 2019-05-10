#ifndef QUADLIBIGLBOOLEANINTERFACE_H
#define QUADLIBIGLBOOLINTERFACE_H

#include <Eigen/Core>

namespace QuadBoolean {
namespace internal {

void trimeshUnion(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

void trimeshDifference(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

void trimeshIntersection(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

}
}

#endif // QUADLIBIGLBOOLEANINTERFACE_H
