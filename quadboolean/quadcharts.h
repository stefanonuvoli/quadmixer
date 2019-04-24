#ifndef QUADTRIANGLECHARTS_H
#define QUADTRIANGLECHARTS_H

#include <vector>
#include <array>
#include <set>
#include <cmath>

#define CORNERMINANGLE M_PI/8

namespace QuadBoolean {
namespace internal {

struct ChartSubSide {
    std::array<int, 2> incidentCharts;
    std::array<int, 2> incidentChartSideId;
    std::vector<size_t> vertices;

    bool isOnBorder;

    double length;
    int size;
};

struct ChartSide {
    std::vector<size_t> vertices;
    std::vector<size_t> subsides;
    std::vector<bool> reversedSubside;
    int size;
    double length;
};

struct Chart {
    int label;

    std::vector<size_t> faces;
    std::vector<size_t> borderFaces;

    std::vector<size_t> adjacentCharts;

    std::vector<size_t> chartSubSides;

    std::vector<ChartSide> chartSides;
};

struct ChartData {
    std::set<int> labels;
    std::vector<Chart> charts;
    std::vector<ChartSubSide> subSides;
};

template<class TriangleMeshType>
ChartData getCharts(TriangleMeshType& mesh, const std::vector<int>& faceLabel);

}
}

#include "quadcharts.tpp"

#endif // QUADTRIANGLECHARTS_H
