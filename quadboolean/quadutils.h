#ifndef QUADUTILS_H
#define QUADUTILS_H

#include "meshtypes.h"

namespace QuadBoolean {
namespace internal {

std::vector<int> splitQuadInTriangle(PolyMesh& mesh);

void colorizeMesh(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel);

}
}

#endif // QUADUTILS_H
