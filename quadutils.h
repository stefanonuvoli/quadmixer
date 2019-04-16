#ifndef QUADUTILS_H
#define QUADUTILS_H

#include "quadcommontypes.h"

namespace QuadBoolean {

void updatePolymeshAttributes(PolyMesh& mesh);
std::vector<int> splitQuadInTriangle(PolyMesh& mesh);
void colorizeMesh(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel);

}

#endif // QUADUTILS_H
