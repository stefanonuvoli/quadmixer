#include "quadilp.h"


#include "gurobi_c++.h"

#define MINSIDEVALUE 1
#define AVERAGELENGTHSMOOTHITERATIONS 5

namespace QuadBoolean {
namespace internal {

std::vector<double> getAverageEdgeLength(const ChartData& chartData);

std::vector<int> solveChartSideILP(
        const ChartData& chartData,
        const double weight,
        const ILPMethod& method)
{
    using namespace std;

    vector<int> result(chartData.subSides.size(), -1);

    try {
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);

//        model.set(GRB_IntParam_OutputFlag, 0);
        model.set(GRB_DoubleParam_TimeLimit, 1.5);

        // Create variables
        GRBQuadExpr obj;

        vector<GRBVar> vars(chartData.subSides.size());
        vector<GRBVar> diff;
        vector<GRBVar> abs;

        vector<GRBVar> free(chartData.charts.size());

        std::vector<double> avgLength = getAverageEdgeLength(chartData);

        for (size_t i = 0; i < chartData.subSides.size(); i++) {
            const ChartSubSide& subside = chartData.subSides[i];

            //If it is not a border (free)
            if (!subside.isOnBorder) {
                assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);

                vars[i] = model.addVar(MINSIDEVALUE, GRB_INFINITY, 0.0, GRB_INTEGER, "s" + to_string(i));

//                double incidentChartAverageLength = (avgLength[subside.incidentCharts[0]] + avgLength[subside.incidentCharts[1]])/2;
//                int sideSubdivision = static_cast<int>(std::round(subside.length / incidentChartAverageLength));

//                vars[i] = model.addVar(std::min(MINSIDEVALUE, sideSubdivision - 1), sideSubdivision+1, 0.0, GRB_INTEGER, "s" + to_string(i));

//                vars[i].set(GRB_DoubleAttr_Start, sideSubdivision);
            }
            else {
                vars[i] = model.addVar(subside.size-1, subside.size, 0.0, GRB_INTEGER, "s" + to_string(i));
            }
        }


        //Regularity
        for (size_t i = 0; i < chartData.subSides.size(); i++) {
            const ChartSubSide& subside = chartData.subSides[i];

            int sideSubdivision;
            //If it is not a border (free)
            if (!subside.isOnBorder) {
                assert(subside.incidentCharts[0] >= 0 && subside.incidentCharts[1] >= 0);
                assert(avgLength[subside.incidentCharts[0]] > 0 && avgLength[subside.incidentCharts[1]] > 0);

                double incidentChartAverageLength = (avgLength[subside.incidentCharts[0]] + avgLength[subside.incidentCharts[1]])/2;
                sideSubdivision = static_cast<int>(std::round(subside.length / incidentChartAverageLength));
            }
            else {
                sideSubdivision = subside.size;
            }

            size_t dId = diff.size();
            size_t aId = abs.size();

            if (method == LEASTSQUARES) {
                obj += weight * (vars[i] - sideSubdivision) * (vars[i] - sideSubdivision);
            }
            else {
                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                model.addConstr(diff[dId] == vars[i] - sideSubdivision, "dc" + to_string(dId));
                model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                obj += weight * abs[aId];
            }
        }

        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                size_t nSides = chart.chartSides.size();
                //Quad case
                if (nSides == 4) {
                    for (size_t j = 0; j <= 1; j++) {
                        const ChartSide& side1 = chart.chartSides[j];
                        const ChartSide& side2 = chart.chartSides[(j+2)%4];

                        GRBLinExpr subSide1Sum = 0;
                        for (const size_t& subSideId : side1.subsides) {
                            subSide1Sum += vars[subSideId];
                        }
                        GRBLinExpr subSide2Sum = 0;
                        for (const size_t& subSideId : side2.subsides) {
                            subSide2Sum += vars[subSideId];
                        }

                        if (method == LEASTSQUARES) {
                            obj += (1-weight) * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
                        }
                        else {
                            size_t dId = diff.size();
                            size_t aId = abs.size();

                            diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                            abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                            model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
                            model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                            obj += (1-weight) * abs[aId];
                        }
                    }
                }
                //Other cases
                else {
                    for (size_t j1 = 0; j1 < nSides; j1++) {
                        for (size_t j2 = j1+1; j2 < nSides; j2++) {
                            const ChartSide& side1 = chart.chartSides[j1];
                            const ChartSide& side2 = chart.chartSides[j2];
                            GRBLinExpr subSide1Sum = 0;
                            for (const size_t& subSideId : side1.subsides) {
                                subSide1Sum += vars[subSideId];
                            }
                            GRBLinExpr subSide2Sum = 0;
                            for (const size_t& subSideId : side2.subsides) {
                                subSide2Sum += vars[subSideId];
                            }


                            if (method == LEASTSQUARES) {
                                obj += (1-weight) * (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
                            }
                            else {
                                size_t dId = diff.size();
                                size_t aId = abs.size();

                                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                                model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
                                model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                                obj += (1-weight) * abs[aId];
                            }
                        }
                    }
                }
            }

            //Even side size sum constraint in a chart
            for (size_t i = 0; i < chartData.charts.size(); i++) {
                const Chart& chart = chartData.charts[i];
                if (chart.faces.size() > 0) {
                    GRBLinExpr sumExp = 0;
                    for (const size_t& subSideId : chart.chartSubSides) {
                        sumExp += vars[subSideId];
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
            result[i] = static_cast<int>(std::round(vars[i].get(GRB_DoubleAttr_X)));

#ifndef NDEBUG
            if (chartData.subSides[i].isOnBorder && result[i] != chartData.subSides[i].size) {
                std::cout << "Different border size: from " << result[i] << " to " << chartData.subSides[i].size << std::endl;
            }
#endif
//            std::cout << subSide.size << " -> " << result[i] << std::endl;
        }

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;


    #ifndef NDEBUG
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

                }
                assert(sizeSum % 2 == 0);
            }
        }
    #endif

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }

    return result;
}

std::vector<double> getAverageEdgeLength(const ChartData& chartData) {
    std::vector<double> avgLengths(chartData.charts.size() , -1);

    const size_t iterations = AVERAGELENGTHSMOOTHITERATIONS;

    //Fill charts with a border
    for (size_t i = 0; i < chartData.charts.size(); i++) {
        const Chart& chart = chartData.charts[i];
        if (chart.faces.size() > 0) {
            double currentLength = 0;
            int numBorderSides = 0;

            for (size_t sId : chart.chartSubSides) {
                const ChartSubSide& side = chartData.subSides[sId];
                if (side.isOnBorder) {
                    currentLength += side.length / side.size;
                    numBorderSides++;
                }
            }

            if (numBorderSides > 0) {
                currentLength /= numBorderSides;
                avgLengths[i] = currentLength;
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

}
}


