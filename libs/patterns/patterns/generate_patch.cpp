#include "generate_patch.h"

#include <patchgen/generate_topology.h>
#include <determine_geometry.h>

using namespace Eigen;

void patterns::generatePatch(const Eigen::VectorXi& l, patchgen::PatchParam& param, Patch& patch) {
    patchgen::generate_topology(l, param, patch);
    patterns::determine_geometry(patch, l);

    for (auto v : patch.vertices()) {
        auto p = patch.data(v).laplaceDirect.value;
        patch.set_point(v, patterns::Patch::Point(p.x(), p.y(), p.z()));
    }
}

void patterns::generatePatch(const patchgen::PatchParam& param, Patch& patch) {
    patchgen::generate_topology(param, patch);
    patterns::determine_geometry(patch, param.l);

    for (auto v : patch.vertices()) {
        auto p = patch.data(v).laplaceDirect.value;
        patch.set_point(v, patterns::Patch::Point(p.x(), p.y(), p.z()));
    }
}
