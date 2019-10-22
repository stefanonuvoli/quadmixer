#ifndef QUADBOOLEAN_QUADCHARTS_H
#define QUADBOOLEAN_QUADCHARTS_H

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

    double length;
    int size;

    bool isOnBorder;
};

struct ChartSide {
    std::vector<size_t> vertices;
    std::vector<size_t> subsides;
    std::vector<bool> reversedSubside;

    double length;
    int size;
};

struct Chart {
    std::vector<size_t> faces;
    std::vector<size_t> borderFaces;

    std::vector<size_t> adjacentCharts;

    std::vector<size_t> chartSubSides;

    std::vector<ChartSide> chartSides;

    int label;

    bool isTriangle;
};

struct ChartData {
    std::set<int> labels;
    std::vector<Chart> charts;
    std::vector<ChartSubSide> subSides;
};

//template<class TriangleMeshType>
//ChartData getCharts(TriangleMeshType& mesh, const std::vector<int>& faceLabel);

template<class TriangleMeshType>
ChartData getPatchDecompositionChartData(
        TriangleMeshType& mesh,
        const std::vector<int>& faceLabel,
        const std::vector<std::vector<size_t>>& corners);

}
}

#include "quadcharts.cpp"

#endif // QUADBOOLEAN_QUADCHARTS_H
