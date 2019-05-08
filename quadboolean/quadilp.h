#ifndef QUADILP_H
#define QUADILP_H

#include "quadcharts.h"

namespace QuadBoolean {

enum ILPMethod { LEASTSQUARES, ABS };

namespace internal {

template<class TriangleMeshType>
std::vector<int> solveChartSideILP(
        TriangleMeshType& mesh,
        const ChartData& chartData,
        const double alpha,
        const double beta,
        const ILPMethod& method);


}
}

#include "quadilp.tpp"

#endif // QUADILP_H
