#include <gurobi_c++.h>

#include "quadilp.h"

#include "quadutils.h"

#define MINSIDEVALUE 1
#define AVERAGELENGTHSMOOTHITERATIONS 3
#define MAXCOST 1000000.0

namespace QuadBoolean {
namespace internal {


template<class TriangleMeshType>
std::vector<double> getSmoothedChartAverageEdgeLength(
        TriangleMeshType& mesh,
        const ChartData& chartData,
        const double quadRegularityWeight);

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
//        ILPStatus& status)
//{
//    using namespace std;

//    vector<int> result(chartData.subSides.size(), -1);

//    try {
//        GRBEnv env = GRBEnv();

//        GRBModel model = GRBModel(env);

////        model.set(GRB_IntParam_OutputFlag, 0);
//        model.set(GRB_DoubleParam_TimeLimit, timeLimit);

//        // Create variables
//        GRBQuadExpr obj;

//        vector<GRBVar> vars(chartData.subSides.size());
//        vector<GRBVar> diff;
//        vector<GRBVar> abs;

//        vector<GRBVar> free(chartData.charts.size());

//        std::vector<double> avgLength = getSmoothedChartAverageEdgeLength(mesh, chartData, beta);

//        for (size_t i = 0; i < chartData.subSides.size(); i++) {
//            const ChartSubSide& subside = chartData.subSides[i];

//            //If it is not a border (free)
//            if (!subside.isOnBorder) {
//                assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);

//                vars[i] = model.addVar(MINSIDEVALUE, GRB_INFINITY, 0.0, GRB_INTEGER, "s" + to_string(i));
//            }
//            else {
//                vars[i] = model.addVar(subside.size - 1, subside.size + 1, 0.0, GRB_INTEGER, "s" + to_string(i));
//            }
//        }

//        if (regularity) {
//            //Regularity
//            for (size_t i = 0; i < chartData.subSides.size(); i++) {
//                const ChartSubSide& subside = chartData.subSides[i];

//                //If it is not a border (free)
//                if (!subside.isOnBorder) {
//                    assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);
//                    assert(avgLength[subside.incidentCharts[0]] > 0 && avgLength[subside.incidentCharts[1]] > 0);


//                    double incidentChartAverageLength = (avgLength[subside.incidentCharts[0]] + avgLength[subside.incidentCharts[1]])/2;
//                    int sideSubdivision = static_cast<int>(std::round(subside.length / incidentChartAverageLength));

//                    size_t dId = diff.size();
//                    size_t aId = abs.size();

//                    if (method == LEASTSQUARES) {
//                        obj += alpha * (vars[i] - sideSubdivision) * (vars[i] - sideSubdivision);
//                    }
//                    else {
//                        diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
//                        abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

//                        model.addConstr(diff[dId] == vars[i] - sideSubdivision, "dc" + to_string(dId));
//                        model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

//                        obj += alpha * abs[aId];
//                    }
//                }
//            }
//        }


//        for (size_t i = 0; i < chartData.charts.size(); i++) {
//            const Chart& chart = chartData.charts[i];
//            if (chart.faces.size() > 0) {
//                size_t nSides = chart.chartSides.size();
//                //Quad case
//                if (nSides == 4) {
//                    for (size_t j = 0; j <= 1; j++) {
//                        const ChartSide& side1 = chart.chartSides[j];
//                        const ChartSide& side2 = chart.chartSides[(j+2)%4];

//                        GRBLinExpr subSide1Sum = 0;
//                        for (const size_t& subSideId : side1.subsides) {
//                            subSide1Sum += vars[subSideId];
//                        }
//                        GRBLinExpr subSide2Sum = 0;
//                        for (const size_t& subSideId : side2.subsides) {
//                            subSide2Sum += vars[subSideId];
//                        }

//                        if (method == LEASTSQUARES) {
//                            obj += (1-alpha) * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
//                        }
//                        else {
//                            size_t dId = diff.size();
//                            size_t aId = abs.size();

//                            diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
//                            abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

//                            model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
//                            model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

//                            obj += (1-alpha) * abs[aId];
//                        }
//                    }
//                }
////                //Other cases
////                else {
////                    for (size_t j1 = 0; j1 < nSides; j1++) {
////                        for (size_t j2 = j1+1; j2 < nSides; j2++) {
////                            const ChartSide& side1 = chart.chartSides[j1];
////                            const ChartSide& side2 = chart.chartSides[j2];

////                            GRBLinExpr subSide1Sum = 0;
////                            for (const size_t& subSideId : side1.subsides) {
////                                subSide1Sum += vars[subSideId];
////                            }
////                            GRBLinExpr subSide2Sum = 0;
////                            for (const size_t& subSideId : side2.subsides) {
////                                subSide2Sum += vars[subSideId];
////                            }

////                            if (method == LEASTSQUARES) {
////                                obj += (1-alpha) * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
////                            }
////                            else {
////                                size_t dId = diff.size();
////                                size_t aId = abs.size();

////                                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
////                                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

////                                model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
////                                model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

////                                obj += (1-alpha) * abs[aId];
////                            }

////                        }
////                    }
////                }
//            }

//            //Even side size sum constraint in a chart
//            for (size_t i = 0; i < chartData.charts.size(); i++) {
//                const Chart& chart = chartData.charts[i];
//                if (chart.faces.size() > 0) {
//                    if (chart.chartSides.size() < 3 || chart.chartSides.size() > 6) {
//                        std::cout << "Chart " << i << " has " << chart.chartSides.size() << " sides." << std::endl;
//                        continue;
//                    }

//                    GRBLinExpr sumExp = 0;
//                    for (const size_t& subSideId : chart.chartSubSides) {
//                        sumExp += vars[subSideId];
//                    }
//                    free[i] = model.addVar(2, GRB_INFINITY, 0.0, GRB_INTEGER, "f" + to_string(i));
//                    model.addConstr(free[i]*2 == sumExp);
//                }
//            }
//        }


////        model.update();

//        //Set objective function
//        model.setObjective(obj, GRB_MINIMIZE);


////        model.write("out.lp");

//        //Optimize model
//        model.optimize();

//        for (size_t i = 0; i < chartData.subSides.size(); i++) {
//            result[i] = static_cast<int>(std::round(vars[i].get(GRB_DoubleAttr_X)));
////            std::cout << subSide.size << " -> " << result[i] << std::endl;
//        }

//        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

//        gap = model.get(GRB_DoubleAttr_MIPGap);

//        status = ILPStatus::SOLUTIONFOUND;

//        for (size_t i = 0; i < chartData.charts.size(); i++) {
//            const Chart& chart = chartData.charts[i];

//            if (chart.faces.size() > 0) {
//                int sizeSum = 0;
//                for (const size_t& subSideId : chart.chartSubSides) {
//                    sizeSum += result[subSideId];
//                }

//                if (sizeSum % 2 == 1) {
//                    std::cout << "Error not even, chart: " << i << " -> ";
//                    for (const size_t& subSideId : chart.chartSubSides) {
//                        std::cout << result[subSideId] << " ";
//                    }
//                    std::cout << " = " << sizeSum << " - FREE: " << free[i].get(GRB_DoubleAttr_X) << std::endl;

//                    status = ILPStatus::SOLUTIONWRONG;
//                }
//            }
//        }

//    } catch(GRBException e) {
//        cout << "Error code = " << e.getErrorCode() << endl;
//        cout << e.getMessage() << endl;

//        status = ILPStatus::INFEASIBLE;
//    }

//    return result;
//}

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
        ILPStatus& status)
{
    using namespace std;

    vector<int> result(chartData.subSides.size(), -1);

    try {
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);

//        model.set(GRB_IntParam_OutputFlag, 0);

        model.set(GRB_DoubleParam_TimeLimit, timeLimit);

        // Create variables
        GRBQuadExpr obj;

        vector<GRBVar> vars(chartData.subSides.size());
        vector<GRBVar> diff;
        vector<GRBVar> abs;

        vector<GRBVar> free(chartData.charts.size());

        std::vector<double> avgLength = getSmoothedChartAverageEdgeLength(mesh, chartData, beta);

        for (size_t i = 0; i < chartData.subSides.size(); i++) {
            const ChartSubSide& subside = chartData.subSides[i];

            //If it is not a border (free)
            if (!subside.isOnBorder) {
                assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);

                vars[i] = model.addVar(MINSIDEVALUE, GRB_INFINITY, 0.0, GRB_INTEGER, "s" + to_string(i));
            }
        }

        if (regularity) {
            const double cost = alpha;
            //Regularity
            for (size_t i = 0; i < chartData.subSides.size(); i++) {
                const ChartSubSide& subside = chartData.subSides[i];

                //If it is not a border (free)
                if (!subside.isOnBorder) {
                    assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);
                    assert(avgLength[subside.incidentCharts[0]] > 0 && avgLength[subside.incidentCharts[1]] > 0);

                    double incidentChartAverageLength = (avgLength[subside.incidentCharts[0]] + avgLength[subside.incidentCharts[1]])/2;
                    int sideSubdivision = static_cast<int>(std::round(subside.length / incidentChartAverageLength));

                    size_t dId = diff.size();
                    size_t aId = abs.size();

                    if (method == LEASTSQUARES) {
                        obj += cost * (vars[i] - sideSubdivision) * (vars[i] - sideSubdivision);
                    }
                    else {
                        diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                        abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                        model.addConstr(diff[dId] == vars[i] - sideSubdivision, "dc" + to_string(dId));
                        model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                        obj += cost * abs[aId];
                    }
                }
            }
        }

        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                size_t nSides = chart.chartSides.size();

                //Quad case
                if (nSides == 4) {
                    const double quadCost = (1-alpha)/2;

                    for (size_t j = 0; j <= 1; j++) {
                        bool areBorders = true;

                        const ChartSide& side1 = chart.chartSides[j];
                        const ChartSide& side2 = chart.chartSides[(j+2)%4];

                        GRBLinExpr subSide1Sum = 0;
                        for (const size_t& subSideId : side1.subsides) {
                            const ChartSubSide& subSide = chartData.subSides[subSideId];
                            if (subSide.isOnBorder) {
                                subSide1Sum += subSide.size;
                            }
                            else {
                                subSide1Sum += vars[subSideId];
                                areBorders = false;
                            }
                        }
                        GRBLinExpr subSide2Sum = 0;
                        for (const size_t& subSideId : side2.subsides) {
                            const ChartSubSide& subSide = chartData.subSides[subSideId];
                            if (subSide.isOnBorder) {
                                subSide2Sum += subSide.size;
                            }
                            else {
                                subSide2Sum += vars[subSideId];
                                areBorders = false;
                            }
                        }

                        if (!areBorders) {
                            if (method == LEASTSQUARES) {
                                obj += quadCost * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
                            }
                            else {
                                size_t dId = diff.size();
                                size_t aId = abs.size();

                                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                                model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
                                model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                                obj += quadCost * abs[aId];
                            }
                        }

                    }
                }
                //Other cases
                else {
                    const double patchCost = (1-alpha)/((nSides * (nSides-1))/2.0);
                    for (size_t j1 = 0; j1 < nSides; j1++) {
                        for (size_t j2 = j1+1; j2 < nSides; j2++) {
                            const ChartSide& side1 = chart.chartSides[j1];
                            const ChartSide& side2 = chart.chartSides[j2];

                            bool areBorders = true;

                            GRBLinExpr subSide1Sum = 0;
                            for (const size_t& subSideId : side1.subsides) {
                                const ChartSubSide& subSide = chartData.subSides[subSideId];
                                if (subSide.isOnBorder) {
                                    subSide1Sum += subSide.size;
                                }
                                else {
                                    subSide1Sum += vars[subSideId];
                                    areBorders = false;
                                }
                            }
                            GRBLinExpr subSide2Sum = 0;
                            for (const size_t& subSideId : side2.subsides) {
                                const ChartSubSide& subSide = chartData.subSides[subSideId];
                                if (subSide.isOnBorder) {
                                    subSide2Sum += subSide.size;
                                }
                                else {
                                    subSide2Sum += vars[subSideId];
                                    areBorders = false;
                                }
                            }


                            if (!areBorders) {
                                if (method == LEASTSQUARES) {
                                    obj += patchCost * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
                                }
                                else {
                                    size_t dId = diff.size();
                                    size_t aId = abs.size();

                                    diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                                    abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                                    model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
                                    model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                                    obj += patchCost * abs[aId];
                                }
                            }

                        }
                    }
                }
            }

            //Even side size sum constraint in a chart
            for (size_t i = 0; i < chartData.charts.size(); i++) {
                const Chart& chart = chartData.charts[i];
                if (chart.faces.size() > 0) {
                    if (chart.chartSides.size() < 3 || chart.chartSides.size() > 6) {
                        std::cout << "Chart " << i << " has " << chart.chartSides.size() << " sides." << std::endl;
                        continue;
                    }

                    GRBLinExpr sumExp = 0;
                    for (const size_t& subSideId : chart.chartSubSides) {
                        const ChartSubSide& subSide = chartData.subSides[subSideId];
                        if (subSide.isOnBorder) {
                            sumExp += subSide.size;
                        }
                        else {
                            sumExp += vars[subSideId];
                        }
                    }
                    free[i] = model.addVar(2, GRB_INFINITY, 0.0, GRB_INTEGER, "f" + to_string(i));
                    model.addConstr(free[i]*2 == sumExp);
                }
            }
        }


//        model.update();

        //Set objective function
        model.setObjective(obj, GRB_MINIMIZE);


//        model.write("out.lp");

        //Optimize model
        model.optimize();

        for (size_t i = 0; i < chartData.subSides.size(); i++) {
            const ChartSubSide& subSide = chartData.subSides[i];
            if (subSide.isOnBorder) {
                result[i] = subSide.size;
            }
            else {
                result[i] = static_cast<int>(std::round(vars[i].get(GRB_DoubleAttr_X)));
            }
//            std::cout << subSide.size << " -> " << result[i] << std::endl;
        }

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        gap = model.get(GRB_DoubleAttr_MIPGap);

        status = ILPStatus::SOLUTIONFOUND;


        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];

            if (chart.faces.size() > 0) {
                int sizeSum = 0;
                for (const size_t& subSideId : chart.chartSubSides) {
                    sizeSum += result[subSideId];
                }

                if (sizeSum % 2 == 1) {
                    std::cout << "Error not even, chart: " << i << " -> ";
                    for (const size_t& subSideId : chart.chartSubSides) {
                        std::cout << result[subSideId] << " ";
                    }
                    std::cout << " = " << sizeSum << " - FREE: " << free[i].get(GRB_DoubleAttr_X) << std::endl;

                    status = ILPStatus::SOLUTIONWRONG;
                }
            }
        }

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;

        status = ILPStatus::INFEASIBLE;
    }

    return result;
}

template<class TriangleMeshType>
std::vector<double> getSmoothedChartAverageEdgeLength(
        TriangleMeshType& mesh,
        const ChartData& chartData,
        const double quadRegularityWeight)
{
    std::vector<double> avgLengths(chartData.charts.size() , -1);

    const size_t iterations = AVERAGELENGTHSMOOTHITERATIONS;

    //Fill charts with a border
    for (size_t i = 0; i < chartData.charts.size(); i++) {
        const Chart& chart = chartData.charts[i];
        if (chart.faces.size() > 0) {
            double currentQuadLength = 0;
            int numSides = 0;

            for (size_t sId : chart.chartSubSides) {
                const ChartSubSide& side = chartData.subSides[sId];
                if (side.isOnBorder) {
                    currentQuadLength += side.length / side.size;
                    numSides++;
                }
            }

            if (numSides > 0) {
                currentQuadLength /= numSides;                
                avgLengths[i] = currentQuadLength;
            }
        }
    }

    //Fill charts with no borders
    bool done;
    do {
        done = true;
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0 && avgLengths[i] < 0) {
                double currentLength = 0;
                size_t numAdjacentCharts = 0;

                for (size_t adjId : chart.adjacentCharts) {
                    if (avgLengths[adjId] > 0) {
                        currentLength += avgLengths[adjId];
                        numAdjacentCharts++;
                        done = false;
                    }
                }

                if (currentLength > 0) {
                    currentLength /= numAdjacentCharts;
                    avgLengths[i] = currentLength;
                }
            }
        }
    } while (!done);


    //Weight on triangle average
    if (quadRegularityWeight < 1) {
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                double currentTriangleLength = averageEdgeLength(mesh, chart.faces);

                assert(avgLengths[i] > 0);

                avgLengths[i] = avgLengths[i]*quadRegularityWeight + currentTriangleLength*(1-quadRegularityWeight);
            }
        }
    }

    //Smoothing
    for (size_t k = 0; k < iterations; k++) {
        std::vector<double> lastAvgLengths = avgLengths;

        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                assert(lastAvgLengths[i] > 0);

                double currentLength = 0;
                size_t numAdjacentCharts = 0;

                for (size_t adjId : chart.adjacentCharts) {
                    if (avgLengths[adjId] > 0) {
                        currentLength += lastAvgLengths[adjId];
                        numAdjacentCharts++;
                    }
                }

                if (currentLength > 0) {
                    currentLength += lastAvgLengths[i];

                    currentLength /= (numAdjacentCharts + 1);
                    avgLengths[i] = currentLength;
                }
            }
        }
    }

    return avgLengths;
}

//template<class PolyMeshType, class TriangleMeshType>
//void fixNonFeasibleChart(
//        PolyMeshType& mesh,
//        TriangleMeshType& trimesh,
//        std::vector<int>& birthQuad,
//        std::vector<int>& faceLabel,
//        const QuadLayoutData<PolyMeshType>& quadLayoutData,
//        TriangleMeshType& newSurface,
//        std::vector<int>& newSurfaceLabel,
//        std::vector<std::vector<typename TriangleMeshType::CoordType>>& newSurfaceCornerPoints,
//        std::vector<bool>& preservedQuad,
//        const std::set<typename TriangleMeshType::CoordType>& targetPointSet,
//        size_t& currentLabel,
//        std::vector<bool>& isQuadPatchAdded)
//{
//    vcg::tri::UpdateTopology<PolyMeshType>::VertexFace(mesh);

//    std::set<int> quadPatchToAdd;
//    for (size_t i = 0; i < mesh.face.size(); i++) {
//        if (preservedQuad[i]) {
//            for (int j = 0; j < mesh.face[i].VN(); j++) {
//                if (targetPointSet.find(mesh.face[i].V(j)->P()) != targetPointSet.end()) {
//                    std::vector<typename PolyMeshType::FacePointer> vertexFaces;
//                    std::vector<int> vertexFaceIndexes;
//                    vcg::face::VFStarVF(mesh.face[i].V(j), vertexFaces, vertexFaceIndexes);

//                    for (size_t k = 0; k < vertexFaces.size(); k++) {
//                        size_t fId = vcg::tri::Index(mesh, vertexFaces.at(k));
//                        if (faceLabel[fId] >= 0) {
//                            quadPatchToAdd.insert(faceLabel[fId]);
//                        }
//                    }

//                }
//            }
//        }
//    }

//    for (const int& label : quadPatchToAdd) {
//        if (isQuadPatchAdded[label])
//            continue;

//        isQuadPatchAdded[label] = true;

//        std::vector<typename PolyMeshType::CoordType> corners(4);

//        const QuadLayoutPatch<PolyMeshType>& quadPatch = quadLayoutData.quadPatches[label];
//        const vcg::face::Pos<typename PolyMeshType::FaceType>& startPos = quadPatch.startPos;
//        const size_t& sizeX = quadPatch.sizeX;
//        const size_t& sizeY = quadPatch.sizeY;

//        assert(!startPos.IsNull());

//        vcg::face::Pos<typename PolyMeshType::FaceType> pos;

//        pos.Set(startPos.F(), startPos.E(), startPos.V());

//        for (int i = 0; i < 4; i++) {
//            pos.FlipV();
//            pos.FlipE();
//            pos.FlipF();
//            pos.FlipE();
//            for (size_t j = 0; j < (i%2 == 0 ? sizeX : sizeY); j++) {
//                corners[i] = pos.V()->P();
//                pos.FlipE();
//                pos.FlipF();
//                pos.FlipE();
//                pos.FlipV();
//            }
//            pos.FlipE();
//            pos.FlipV();
//        }


//        vcg::tri::UpdateSelection<TriangleMeshType>::FaceClear(trimesh);
//        vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearV(trimesh);
//        vcg::tri::UpdateFlags<TriangleMeshType>::FaceClearV(newSurface);
//        for (int i = 0; i < trimesh.face.size(); i++) {
//            assert(birthQuad[i] >= 0);
//            if (faceLabel[birthQuad[i]] == label) {
//                trimesh.face[i].SetS();
//                trimesh.face[i].SetV();
//                preservedQuad[birthQuad[i]] = false;
//            }
//        }

//        vcg::tri::Append<TriangleMeshType, TriangleMeshType>::Mesh(newSurface, trimesh, true);

//        newSurfaceLabel.resize(newSurface.face.size());
//        for (size_t i = 0; i < newSurface.face.size(); i++) {
//            if (newSurface.face[i].IsV()) {
//#ifndef NDEBUG
//                newSurface.face[i].C() = vcg::Color4b(0,0,255,255);
//#endif
//                newSurfaceLabel[i] = currentLabel;
//            }
//        }

//        newSurfaceCornerPoints.push_back(corners);

//        currentLabel++;
//    }
//}

}
}


