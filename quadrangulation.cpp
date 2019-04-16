#include "quadrangulation.h"

#include <igl/lscm.h>

#include <igl/AABB.h>
#include <igl/in_element.h>


namespace QuadBoolean {

Eigen::VectorXd pointToBarycentric(
        const Eigen::VectorXd& t1,
        const Eigen::VectorXd& t2,
        const Eigen::VectorXd& t3,
        const Eigen::VectorXd& p);

Eigen::VectorXd barycentricToPoint(
        const Eigen::VectorXd& t1,
        const Eigen::VectorXd& t2,
        const Eigen::VectorXd& t3,
        const Eigen::VectorXd& p);


std::vector<double> getSideLengths(
        const std::vector<std::vector<unsigned int>>& sides,
        const Eigen::MatrixXd& vertices);

double getSingleSideLength(
        const std::vector<unsigned int>& side,
        const Eigen::MatrixXd& vertices);

void computeQuadrangulation(
        const Eigen::MatrixXd& chartV,
        const Eigen::MatrixXi& chartF,
        const Eigen::MatrixXd& patchV,
        const Eigen::MatrixXi& patchF,
        const std::vector<std::vector<unsigned int>>& chartSides,
        const std::vector<std::vector<unsigned int>>& patchSides,
        const std::vector<double>& chartSideLengths,
        Eigen::MatrixXd& uvMap,
        Eigen::MatrixXd& quadrangulationV,
        Eigen::MatrixXi& quadrangulationF)
{


    Eigen::VectorXi b;
    Eigen::MatrixXd bc;

    int chartBorderSize = 0;
    for (const std::vector<unsigned int>& side : chartSides)
        chartBorderSize += side.size()-1;

    b.resize(chartBorderSize);
    bc.resize(chartBorderSize, 2);

    int fixedId = 0;
    for (size_t sId = 0; sId < chartSides.size(); sId++) {
        //Get first and last corner of the side
        const unsigned int& firstPatchSideCornerId = patchSides[sId][0];
        const unsigned int& lastPatchSideCornerId = patchSides[sId][patchSides[sId].size() - 1];

        //Coordinate of the current corner
        const Eigen::VectorXd& cornerCoord = patchV.row(firstPatchSideCornerId);

        //Get vector of the side
        const Eigen::VectorXd vector = patchV.row(lastPatchSideCornerId) - patchV.row(firstPatchSideCornerId);
        double currentLength = 0;
        for (size_t i = 0; i < chartSides[sId].size() - 1; i++) {
            if (i > 0) {
                currentLength += (chartV.row(chartSides[sId][i]) - chartV.row(chartSides[sId][i-1])).norm();
            }

            unsigned int vId = chartSides[sId][i];

            double lengthRatio = currentLength / chartSideLengths[sId];

            const Eigen::VectorXd uv = cornerCoord + (vector * lengthRatio);

            b(fixedId) = static_cast<int>(vId);

            //Flip x with y
            bc(fixedId, 0) = uv(1);
            bc(fixedId, 1) = uv(0);

            fixedId++;
        }

    }

    //Apply Least Square Conformal Maps
    igl::lscm(chartV, chartF, b, bc, uvMap);

    //AABB tree for point location
    igl::AABB<Eigen::MatrixXd, 2> tree;
    tree.init(uvMap, chartF);

    quadrangulationV.resize(patchV.rows(), 3);
    for (int i = 0; i < patchV.rows(); i++) {
        const Eigen::VectorXd& Q = patchV.row(i);

        Eigen::MatrixXd Q2D(1,2);
        Q2D(0,0) = Q.x();
        Q2D(0,1) = Q.y();

        Eigen::VectorXi I;
        igl::in_element(uvMap, chartF, Q2D, tree, I);

        int triIndex = I(0);

        if (triIndex < 0) {
            Eigen::VectorXi sqrD;
            Eigen::MatrixXd C;
            tree.squared_distance(uvMap, chartF, Q2D, sqrD, I, C);

            triIndex = I(0);

            assert(triIndex > -1);
        }

        const Eigen::VectorXi& tri = chartF.row(triIndex);

        Eigen::VectorXd baryc = pointToBarycentric(
                    uvMap.row(tri(0)),
                    uvMap.row(tri(1)),
                    uvMap.row(tri(2)),
                    Q2D.row(0));

        Eigen::VectorXd mappedPoint = barycentricToPoint(
                    chartV.row(tri(0)),
                    chartV.row(tri(1)),
                    chartV.row(tri(2)),
                    baryc);

        quadrangulationV.row(i) = mappedPoint;
    }
    quadrangulationF = patchF;
}



Eigen::VectorXd pointToBarycentric(
        const Eigen::VectorXd& t1,
        const Eigen::VectorXd& t2,
        const Eigen::VectorXd& t3,
        const Eigen::VectorXd& p)
{
    double det = (t2.y() - t3.y()) * (t1.x() - t3.x()) + (t3.x() - t2.x()) * (t1.y() - t3.y());

    Eigen::VectorXd baryc(3);

    baryc(0) = ((t2.y() - t3.y()) * (p.x() - t3.x()) + (t3.x() - t2.x()) * (p.y() - t3.y())) / det;
    baryc(1) = ((t3.y() - t1.y()) * (p.x() - t3.x()) + (t1.x() - t3.x()) * (p.y() - t3.y())) / det;
    baryc(2) = 1 - baryc(0) - baryc(1);

    return baryc;
}


Eigen::VectorXd barycentricToPoint(
        const Eigen::VectorXd& t1,
        const Eigen::VectorXd& t2,
        const Eigen::VectorXd& t3,
        const Eigen::VectorXd& baryc)
{
    Eigen::VectorXd coordinates =
            t1 * baryc(0) +
            t2 * baryc(1) +
            t3 * baryc(2);

    return coordinates;
}

std::vector<std::vector<unsigned int>> getSides(
        const std::vector<unsigned int>& borders,
        const std::vector<unsigned int>& corners)
{
    std::vector<std::vector<unsigned int>> sides(corners.size());
    size_t bId = 0;
    for (size_t sId = 0; sId < corners.size(); sId++) {
        while (borders[bId] != corners[sId]) {
            bId = (bId + 1) % borders.size();
        }

        std::vector<unsigned int> side;
        while (borders[bId] != corners[(sId + 1) % corners.size()]) {
            side.push_back(borders[bId]);

            bId = (bId + 1) % borders.size();
        }
        side.push_back(corners[(sId + 1) % corners.size()]);

        sides[sId] = side;
    }

    return sides;
}

std::vector<double> getSideLengths(
        const std::vector<std::vector<unsigned int>>& sides,
        const Eigen::MatrixXd& vertices)
{
    std::vector<double> lengths(sides.size());

    for (size_t sId = 0; sId < sides.size(); sId++) {
        lengths[sId] = getSingleSideLength(sides[sId], vertices);
    }

    return lengths;
}

double getSingleSideLength(
        const std::vector<unsigned int>& side,
        const Eigen::MatrixXd& vertices)
{
        double length = 0;

        for (size_t i = 1; i < side.size(); i++) {
            length += (vertices.row(side[i]) - vertices.row(side[i-1])).norm();
        }

        return length;
}

}
