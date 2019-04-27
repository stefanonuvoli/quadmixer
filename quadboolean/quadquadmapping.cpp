#include "quadquadmapping.h"

#include <igl/lscm.h>

#include <igl/AABB.h>
#include <igl/in_element.h>


namespace QuadBoolean {
namespace internal {

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


void computeQuadrangulation(
        const Eigen::MatrixXd& chartV,
        const Eigen::MatrixXi& chartF,
        const Eigen::MatrixXd& patchV,
        const Eigen::MatrixXi& patchF,
        const std::vector<std::vector<size_t>>& chartSides,
        const std::vector<double>& chartSideLength,
        const std::vector<std::vector<size_t>>& patchSides,
        Eigen::MatrixXd& uvMap,
        Eigen::MatrixXd& quadrangulationV,
        Eigen::MatrixXi& quadrangulationF)
{
    Eigen::VectorXi b;
    Eigen::MatrixXd bc;

    int chartBorderSize = 0;
    for (const std::vector<size_t>& side : chartSides)
        chartBorderSize += side.size()-1;

    b.resize(chartBorderSize);
    bc.resize(chartBorderSize, 2);

    int fixedId = 0;
    for (size_t sId = 0; sId < chartSides.size(); sId++) {
        //Get first and last corner of the side
        const size_t& firstPatchSideCornerId = patchSides[sId][0];
        const size_t& lastPatchSideCornerId = patchSides[sId][patchSides[sId].size() - 1];

        //Coordinate of the current corner
        const Eigen::VectorXd& cornerCoord = patchV.row(firstPatchSideCornerId);

        //Get vector of the side
        const Eigen::VectorXd vector = patchV.row(lastPatchSideCornerId) - patchV.row(firstPatchSideCornerId);
        double currentLength = 0;
        for (size_t i = 0; i < chartSides[sId].size() - 1; i++) {
            const std::vector<size_t>& chartSide = chartSides[sId];

            if (i > 0) {
                currentLength += (chartV.row(chartSide[i]) - chartV.row(chartSide[i-1])).norm();
            }

            size_t vId = chartSide[i];

            double lengthRatio = currentLength / chartSideLength[sId];
            assert(lengthRatio >= 0 && lengthRatio < 1);

            const Eigen::VectorXd uv = cornerCoord + (vector * lengthRatio);

            b(fixedId) = static_cast<int>(vId);

            //Flip x with y
            bc(fixedId, 0) = uv(1);
            bc(fixedId, 1) = uv(0);

            fixedId++;
        }
    }

    if (b.size() < chartV.rows()) {
        //Apply Least Square Conformal Maps
        igl::lscm(chartV, chartF, b, bc, uvMap);
    }
    else {
        //Get the UV map with all fixed border
        uvMap = bc;
    }

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

    //Flip of faces
    for (int i = 0; i < quadrangulationF.rows(); i++) {
        assert(quadrangulationF.cols() == 4);
        for (int j = 0; j < quadrangulationF.cols()/2; j++) {
            std::swap(quadrangulationF(i,j), quadrangulationF(i, quadrangulationF.cols() - 1 - j));
        }
    }
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

    baryc(0) = std::max(std::min(baryc(0), 1.0), 0.0);
    baryc(1) = std::max(std::min(baryc(1), 1.0), 0.0);
    baryc(2) = std::max(std::min(baryc(2), 1.0), 0.0);

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

std::vector<std::vector<size_t>> getPatchSides(
        Eigen::MatrixXd& patchV,
        Eigen::MatrixXi& patchF,
        std::vector<size_t>& borders,
        std::vector<size_t>& corners,
        const Eigen::VectorXi& l)
{
    assert(corners.size() == l.size());

    std::vector<std::vector<size_t>> sides(corners.size());
    size_t startCornerId = 0;

    bool foundSolution;
    do {
        size_t bId = 0;
        while (borders[bId] != corners[startCornerId]) {
            bId = (bId + 1) % borders.size();
        }

        foundSolution = true;
        size_t cId = startCornerId;
        size_t sId = 0;
        do {
            assert(borders[bId] == corners[cId]);

            cId = (cId + 1) % corners.size();

            std::vector<size_t> side;

            while (borders[bId] != corners[cId]) {
                side.push_back(borders[bId]);
                bId = (bId + 1) % borders.size();
            }

            assert(borders[bId] == corners[cId]);
            assert(side[side.size() - 1] != corners[cId]);

            if (side.size() != l(sId)) {
                foundSolution = false;
                break;
            }

            side.push_back(corners[cId]);

            sides[sId] = side;
            sId++;
        } while (startCornerId != cId);

       startCornerId++;
    } while (!foundSolution && startCornerId < corners.size());

    if (!foundSolution) {
        for (int i = 0; i < patchV.rows(); i++) {
            for (int j = 0; j < patchV.cols(); j++) {
                patchV(i,j) = 0 - patchV(i,j);
            }
        }

        for (int i = 0; i < patchF.rows(); i++) {
            assert(patchF.cols() == 4);
            for (int j = 0; j < patchF.cols()/2; j++) {
                std::swap(patchF(i,j), patchF(i, patchF.cols() - 1 - j));
            }
        }

        std::reverse(corners.begin(), corners.end());
        std::reverse(borders.begin(), borders.end());

        startCornerId = 0;
        do {
            assert(startCornerId < corners.size());

            size_t bId = 0;
            while (borders[bId] != corners[startCornerId]) {
                bId = (bId + 1) % borders.size();
            }

            foundSolution = true;
            size_t cId = startCornerId;
            size_t sId = 0;
            do {
                assert(borders[bId] == corners[cId]);

                cId = (cId + 1) % corners.size();

                std::vector<size_t> side;

                while (borders[bId] != corners[cId]) {
                    side.push_back(borders[bId]);
                    bId = (bId + 1) % borders.size();
                }

                assert(borders[bId] == corners[cId]);
                assert(side[side.size() - 1] != corners[cId]);

                if (side.size() != l(sId)) {
                    foundSolution = false;
                    break;
                }

                side.push_back(corners[cId]);

                sides[sId] = side;
                sId++;
            } while (startCornerId != cId);

            startCornerId++;

        } while (!foundSolution);
    }

    assert(foundSolution);

    return sides;
}

}
}
