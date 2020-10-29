#ifndef QUADPATTERNS_H
#define QUADPATTERNS_H

#include <vector>

#include <Eigen/Core>

namespace QuadBoolean {
namespace internal {

void computePattern(
        const Eigen::VectorXi &l,
        Eigen::MatrixXd& patchV,
        Eigen::MatrixXi& patchF,
        std::vector<size_t>& borders,
        std::vector<size_t>& corners,
        std::vector<std::vector<size_t>>& sides);

}
}

#endif // QUADPATTERNS_H
