#ifndef QUADBOOLEANINTERFACE_H
#define QUADBOOLINTERFACE_H

#include <Eigen/Core>

void trimeshUnion(
        const Eigen::MatrixXd& VA,
        const Eigen::MatrixXi& FA,
        const Eigen::MatrixXd& VB,
        const Eigen::MatrixXi& FB,
        Eigen::MatrixXd& VR,
        Eigen::MatrixXi& FR,
        Eigen::VectorXi& J);

#endif // QUADBOOLEANINTERFACE_H
