#ifndef QUADCONVERT_H
#define QUADCONVERT_H

#include <Eigen/Core>
#include <vector>

template<class M>
void VCGToEigen(
        M& vcgMesh,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        int numVertices = 3,
        int dim = 3);

template<class M>
void VCGToEigenSelected(
        M& vcgMesh,
        Eigen::MatrixXd& V,
        Eigen::MatrixXi& F,
        std::vector<int>& vMap,
        std::vector<int>& fMap,
        int numVertices = 3,
        int dim = 3);

template<class M>
void eigenToVCG(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& F,
        M& vcgMesh,
        int numVertices = 3,
        int dim = 3);

#include "quadconvert.tpp"

#endif // QUADCONVERT_H
