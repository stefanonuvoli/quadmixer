#include "quadilp.h"
#include "gurobi_c++.h"

#define MINSIDEVALUE 1

namespace QuadBoolean {

int getAverageEdgeLength(ChartData& chartData) {
    double avgLength = 0;
    int numBorderSides = 0;
    for (size_t i = 0; i < chartData.sides.size(); i++) {
        const ChartSide& side = chartData.sides[i];
        if (side.isOnBorder) {
            avgLength += side.length / side.size;
            numBorderSides++;
        }
    }

    avgLength /= numBorderSides;
    return static_cast<int>(avgLength);
}

std::vector<int> solveChartSideILP(ChartData& chartData, double avgEdgeLength) {
    using namespace std;

#ifndef NDEBUG
    for (size_t i = 0; i < chartData.charts.size(); i++) {
        const Chart& chart = chartData.charts[i];

        int sizeSum = 0;
        for (const size_t& sideId : chart.chartSides) {
            sizeSum += chartData.sides[sideId].size;
        }

        assert(sizeSum % 2 == 1);
    }
#endif

    vector<int> result;

    try {
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);

        // Create variables
        GRBQuadExpr obj;
        vector<GRBVar> vars(chartData.sides.size());
        vector<GRBVar> diff(chartData.sides.size());
        vector<GRBVar> abs(chartData.sides.size());

        vector<GRBVar> free(chartData.charts.size());
        for (size_t i = 0; i < chartData.sides.size(); i++) {
            const ChartSide& side = chartData.sides[i];

            //If it is a border (fixed)
            if (side.isOnBorder) {
                vars[i] = model.addVar(side.size, side.size, 0.0, GRB_INTEGER, "s" + to_string(i)); //TODO REMOVE!
            }
            //If it is not a border (free)
            else {
                const double sideSubdivision = std::round(side.length / avgEdgeLength);
                vars[i] = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "s" + to_string(i));
                diff[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_INTEGER, "d" + to_string(i));
                abs[i] = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "a" + to_string(i));
                model.addConstr(diff[i] == vars[i] - sideSubdivision);
                model.addGenConstrAbs(abs[i], diff[i]);
                obj += abs[i];
//                obj += (vars[i] - sideSubdivision)*(vars[i] - sideSubdivision);
            }
        }

        //Even side size in a chart
        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];

            GRBLinExpr sum = 0;
            for (const size_t& sideId : chart.chartSides) {
                sum += vars[sideId];
            }

            free[i] = model.addVar(0, GRB_INFINITY, 0.0, GRB_INTEGER, "sumf" + to_string(i));
            model.addConstr(sum - (2 * free[i]) == 0);
        }


        //Set objective function
        model.setObjective(obj, GRB_MINIMIZE);


        //Optimize model
        model.optimize();

        result.resize(chartData.sides.size());
        for (size_t i = 0; i < chartData.sides.size(); i++) {
            result[i] = vars[i].get(GRB_DoubleAttr_X);
            std::cout << chartData.sides[i].size << " -> " << result[i] << std::endl;
        }

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;


        for (size_t i = 0; i < chartData.charts.size(); i++) {
            const Chart& chart = chartData.charts[i];

            int sizeSum = 0;
            for (const size_t& sideId : chart.chartSides) {
                sizeSum += result[sideId];
            }

            if (sizeSum % 2 == 1) {
                std::cout << "Error not even, chart: " << i << std::endl;
            }
        }


    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }
    return result;
}

}
