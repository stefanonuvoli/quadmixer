#ifndef QUADRANGULATION_PATTERNS_H
#define QUADRANGULATION_PATTERNS_H

#include <vector>

#include <Eigen/Core>

namespace QuadBoolean {

void computePattern(
        const Eigen::VectorXi &l,
        Eigen::MatrixXd& patchV,
        Eigen::MatrixXi& patchF,
        std::vector<unsigned int>& borders,
        std::vector<unsigned int>& corners);

}

#endif // QUADRANGULATION_PATTERNS_H
