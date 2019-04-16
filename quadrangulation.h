#ifndef QUADRANGULATION_QUADRANGULATION_H
#define QUADRANGULATION_QUADRANGULATION_H

#include <vector>

#include <Eigen/Core>

namespace QuadBoolean {

std::vector<double> getSideLengths(
        const std::vector<std::vector<unsigned int>>& sides,
        const Eigen::MatrixXd& vertices);

std::vector<std::vector<unsigned int>> getSides(
        const std::vector<unsigned int>& borders,
        const std::vector<unsigned int>& singularities);

void computeQuadrangulation(
        const Eigen::MatrixXd& chartV,
        const Eigen::MatrixXi& chartF,
        const Eigen::MatrixXd& patchV,
        const Eigen::MatrixXi& patchF,        
        const std::vector<std::vector<unsigned int>>& chartSides,
        const std::vector<std::vector<unsigned int>>& patchSides,
        const std::vector<double>& chartSideLengths,
        Eigen::MatrixXd& V_uv,
        Eigen::MatrixXd& quadrangulationV,
        Eigen::MatrixXi& quadrangulationF);

}


#endif // QUADRANGULATION_QUADRANGULATION_H
