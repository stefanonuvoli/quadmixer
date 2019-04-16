#ifndef QUADILP_H
#define QUADILP_H

#include "quadcharts.h"

namespace QuadBoolean {

int getAverageEdgeLength(ChartData& chartData);
std::vector<int> solveChartSideILP(ChartData& chartData, double avgSubdivision);

}

#endif // QUADILP_H
