#ifndef QUADILP_H
#define QUADILP_H

#include "quadcharts.h"

namespace QuadBoolean {
namespace internal {

enum ILPMethod { LEASTSQUARES, ABS };

std::vector<int> solveChartSideILP(
        const ChartData& chartData,
        const double weight,
        const ILPMethod& method = LEASTSQUARES);

}
}

#endif // QUADILP_H
