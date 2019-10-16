#ifndef QUADBOOLEAN_QUADILP_H
#define QUADBOOLEAN_QUADILP_H

#include "quadcharts.h"
#include "quadlayoutdata.h"
#include "quadbooleancommon.h"

namespace QuadBoolean {

namespace internal {

enum ILPStatus { SOLUTIONFOUND, SOLUTIONWRONG, INFEASIBLE };

//template<class TriangleMeshType>
//std::vector<int> solveChartSideILPFreeBorders(
//        TriangleMeshType& mesh,
//        const ChartData& chartData,
//        const double alpha,
//        const double beta,
//        const ILPMethod& method,
//        const bool regularity,
//        const double timeLimit,
//        double& gap,
//        ILPStatus& status);

template<class TriangleMeshType>
std::vector<int> solveChartSideILPFixedBorders(
        TriangleMeshType& mesh,
        const ChartData& chartData,
        const double alpha,
        const double beta,
        const ILPMethod& method,
        const bool regularity,
        const double timeLimit,
        double& gap,
        ILPStatus& status);

template<class PolyMeshType, class TriangleMeshType>
void fixNonFeasibleChart(
        PolyMeshType& mesh,
        TriangleMeshType& trimesh,
        std::vector<int>& birthQuad,
        std::vector<int>& faceLabel,
        const QuadLayoutData<PolyMeshType>& quadLayoutData,
        TriangleMeshType& newSurface,
        std::vector<int>& newSurfaceLabel,
        std::vector<std::vector<typename TriangleMeshType::CoordType>>& newSurfaceCornerPoints,
        std::vector<bool>& preservedQuad,
        const std::set<typename TriangleMeshType::CoordType>& pointSet,
        size_t& currentLabel,
        std::vector<bool>& isQuadPatchRemoved);

}
}

#include "quadilp.cpp"

#endif // QUADBOOLEAN_QUADILP_H
