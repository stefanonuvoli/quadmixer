#ifndef QUADBOOLEAN_QUADILP_H
#define QUADBOOLEAN_QUADILP_H

#include "quadcharts.h"
#include "quadlayoutdata.h"
#include "quadbooleancommon.h"

namespace QuadBoolean {

namespace internal {

enum ILPStatus { SOLUTIONFOUND, SOLUTIONWRONG, INFEASIBLE };


template<class TriangleMeshType>
std::vector<int> solveILP(
        TriangleMeshType& mesh,
        ChartData& chartData,
        const bool onlyQuads,
        const double alpha,
        const double beta,
        const ILPMethod& method,
        const bool regularity,
        const double timeLimit,
        double& gap,
        ILPStatus& status);

}
}

#include "quadilp.cpp"

#endif // QUADBOOLEAN_QUADILP_H
