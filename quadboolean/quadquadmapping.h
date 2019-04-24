#ifndef QUADQUADMAPPING_H
#define QUADQUADMAPPING_H

#include <vector>

#include <Eigen/Core>

namespace QuadBoolean {
namespace internal {

std::vector<std::vector<size_t>> getPatchSides(
        const std::vector<size_t>& borders,
        const std::vector<size_t>& corners,
        const Eigen::VectorXi& l);

void computeQuadrangulation(
        const Eigen::MatrixXd& chartV,
        const Eigen::MatrixXi& chartF,
        const Eigen::MatrixXd& patchV,
        const Eigen::MatrixXi& patchF,        
        const std::vector<std::vector<size_t>>& chartSides,
        const std::vector<double>& chartSideLengths,
        const std::vector<std::vector<size_t>>& patchSides,
        Eigen::MatrixXd& V_uv,
        Eigen::MatrixXd& quadrangulationV,
        Eigen::MatrixXi& quadrangulationF);

}

}


#endif // QUADQUADMAPPING_H