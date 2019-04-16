#ifndef QUADTRIANGLECHARTS_H
#define QUADTRIANGLECHARTS_H

#include "quadcommontypes.h"

namespace QuadBoolean {

struct ChartSide {
    std::array<int, 2> incidentCharts;
    std::array<int, 2> incidentChartSideId;
    std::vector<PolyMesh::VertexType*> vertices;

    bool isOnBorder;

    double length;
    int size;
};

struct Chart {
    int label;

    std::vector<size_t> faces;
    std::vector<size_t> borderFaces;

    std::vector<size_t> chartSides;

    std::vector<size_t> adjacentCharts;
};

struct ChartData {
    std::set<int> labels;

    std::vector<Chart> charts;

    std::vector<ChartSide> sides;
};

ChartData getCharts(PolyMesh& mesh, const std::vector<int>& faceLabel);

}

#endif // QUADTRIANGLECHARTS_H
