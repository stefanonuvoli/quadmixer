#include "patterns.h"

#include "patterns/ktmethod/patchgen/PatchParam.h"
#include "patterns/meshtypes.h"
#include "patterns/patchg.h"

#include <iostream>

#include <igl/readOBJ.h>

#include "quadconvert.h"

namespace QuadBoolean {

void computePattern(
        const Eigen::VectorXi &l,
        Eigen::MatrixXd& patchV,
        Eigen::MatrixXi& patchF,
        std::vector<unsigned int>& borders,
        std::vector<unsigned int>& corners)
{
    PatchG<PMesh,patchgen::PatchParam> patch;

    long int num_sides = l.size();
    if (num_sides < 2 || 6 < num_sides) {
        std::cout << "num_sides=" << num_sides << " is unsupported.\n";
        return;
    }
    if (l.sum() % 2 != 0) {
        std::cout << "The sum of number of edge subdivisions should be even.\n";
        return;
    }
    if (l.sum() < 4) {
        std::cout << "Input numbers are too small.\n";
        return;
    }

    patchgen::PatchParam param;
    patch.generate_topology(l,param);
    patch.determine_geometry(l);

    VCGToEigen(patch.mesh, patchV, patchF, 4);

    std::copy(patch.borders.begin(), patch.borders.end(), std::back_inserter(borders));
    std::copy(patch.corners.begin(), patch.corners.end(), std::back_inserter(corners));
}

}
