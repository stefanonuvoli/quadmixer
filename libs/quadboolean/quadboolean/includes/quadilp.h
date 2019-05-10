#ifndef QUADBOOLEAN_QUADILP_H
#define QUADBOOLEAN_QUADILP_H

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

#endif // QUADBOOLEAN_QUADILP_H
