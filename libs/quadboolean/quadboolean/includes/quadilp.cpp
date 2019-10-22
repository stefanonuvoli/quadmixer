#include <gurobi_c++.h>

#include "quadilp.h"

#include "quadutils.h"

#define MINSIDEVALUE 1
#define AVERAGELENGTHSMOOTHITERATIONS 1
#define MAXCOST 1000000.0

namespace QuadBoolean {
namespace internal {


template<class TriangleMeshType>
std::vector<double> getSmoothedChartAverageEdgeLength(
        TriangleMeshType& mesh,
        const ChartData& chartData,
        const double quadRegularityWeight);

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
        GRBQuadExpr obj = 0;

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

        std::cout << chartData.subSides.size() << " subsides!" << std::endl;

        if (regularity) {
            GRBQuadExpr regExpr = 0;
            const double regCost = alpha;
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
                        regExpr += (vars[i] - sideSubdivision) * (vars[i] - sideSubdivision);
                    }
                    else {
                        diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                        abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                        model.addConstr(diff[dId] == vars[i] - sideSubdivision, "dc" + to_string(dId));
                        model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                        regExpr += abs[aId];
                    }
                }
            }
            obj += regCost * regExpr;
        }

        GRBQuadExpr isoExpr = 0;
        const double isoCost = (1-alpha);
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];
            if (chart.faces.size() > 0) {
                size_t nSides = chart.chartSides.size();

                //Quad case
                if (nSides == 4) {

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
                                isoExpr += (subSide1Sum - subSide2Sum) * (subSide1Sum - subSide2Sum);
                            }
                            else {
                                size_t dId = diff.size();
                                size_t aId = abs.size();

                                diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                                abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                                model.addConstr(diff[dId] == subSide1Sum - subSide2Sum, "dc" + to_string(dId));
                                model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                                isoExpr += abs[aId];
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

                    if (onlyQuads) {
                        model.addConstr(free[i]*2 == sumExp);
                    }
                    else {
                        if (method == LEASTSQUARES) {
                            obj += (free[i]*2 - sumExp) * (free[i]*2 - sumExp) * MAXCOST;
                        }
                        else {
                            size_t dId = diff.size();
                            size_t aId = abs.size();

                            diff.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(dId)));
                            abs.push_back(model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(aId)));

                            model.addConstr(diff[dId] == free[i]*2 - sumExp, "dc" + to_string(dId));
                            model.addGenConstrAbs(abs[aId], diff[dId], "ac" + to_string(aId));

                            obj += abs[aId] * MAXCOST;
                        }
                    }
                }
            }
        }
        obj += isoCost * isoExpr;


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
        }

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        gap = model.get(GRB_DoubleAttr_MIPGap);

        status = ILPStatus::SOLUTIONFOUND;


        for (size_t i = 0; i < chartData.charts.size(); i++) {
            Chart& chart = chartData.charts[i];

            if (chart.faces.size() > 0) {
                int sizeSum = 0;
                for (const size_t& subSideId : chart.chartSubSides) {
                    sizeSum += result[subSideId];
                }

                if (sizeSum % 2 == 1) {
                    if (onlyQuads) {
                        std::cout << "Error not even, chart: " << i << " -> ";
                        for (const size_t& subSideId : chart.chartSubSides) {
                            std::cout << result[subSideId] << " ";
                        }
                        std::cout << " = " << sizeSum << " - FREE: " << free[i].get(GRB_DoubleAttr_X) << std::endl;

                        status = ILPStatus::SOLUTIONWRONG;
                    }
                    else {
                        exit(-1);
                        std::vector<int> sideSubdivision(chart.chartSides.size(), 0);
                        for (size_t j = 0; j < chart.chartSides.size(); j++) {
                            const ChartSide& side = chart.chartSides[j];

                            sideSubdivision[j] = 0;
                            for (const size_t& subSideId : side.subsides) {
                                sideSubdivision[j] += result[subSideId];
                            }
                        }

                        //Find the one to be split in triangle
                        bool reversed = false;
                        int bestIndex = -1;
                        int bestScore = std::numeric_limits<int>::max();

                        for (size_t j = 0; j < sideSubdivision.size(); j++) {
                            int score = ((sideSubdivision[j] - 1) - sideSubdivision[(j+2)%sideSubdivision.size()]);
                            score = score * score;

                            if (score < bestScore) {
                                bestIndex = j;
                                bestScore = score;
                            }
                        }

                        for (size_t j = 0; j < sideSubdivision.size(); j++) {
                            int score = ((sideSubdivision[sideSubdivision.size()-1-j] - 1) - sideSubdivision[sideSubdivision.size()-1-((j+2)%sideSubdivision.size())]);
                            score = score * score;

                            if (score < bestScore) {
                                reversed = true;
                                bestIndex = j;
                                bestScore = score;
                            }
                        }

                        chart.chartSides[bestIndex];

                        Chart newTriangleChart;
                        newTriangleChart.isTriangle = true;
                        newTriangleChart.faces;

                                //TODO;
                    }
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
                const ChartSubSide& subside = chartData.subSides[sId];
                if (subside.isOnBorder) {
                    currentQuadLength += subside.length / subside.size;
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

}
}


