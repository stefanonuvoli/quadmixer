#include "quadcharts.h"

#include <map>

#include "assert.h"

#include <vcg/complex/complex.h>

#include <unordered_set>

#define MAXITERATIONS 100000
#include <wrap/io_trimesh/export.h>

#include "quadutils.h"

namespace QuadBoolean {
namespace internal {

template<class TriangleMeshType>
void findChartFacesAndBorderFaces(
        TriangleMeshType& mesh,
        const std::vector<int>& faceLabel,
        ChartData& chartData);

//template<class TriangleMeshType>
//double dotProjectedOntoPlane(
//        const typename TriangleMeshType::CoordType& edge1,
//        const typename TriangleMeshType::CoordType& edge2,
//        const typename TriangleMeshType::CoordType& normal);

//It works just on triangle meshes
template<class TriangleMeshType>
ChartData getPatchDecompositionChartData(
        TriangleMeshType& mesh,
        const std::vector<int>& faceLabel,
        const std::vector<std::vector<size_t>>& corners)
{
    typedef std::map<std::pair<size_t, size_t>, std::pair<int, int>> EdgeLabelMap;
    typedef std::map<std::pair<size_t, size_t>, int> EdgeSubSideMap;

    ChartData chartData;

    if (mesh.face.size() == 0)
        return chartData;

    //Region growing algorithm for getting charts
    findChartFacesAndBorderFaces(mesh, faceLabel, chartData);

    //TODO SPLIT IN FUNCTIONS
    EdgeSubSideMap edgeSubSideMap;
    for (const int& pId : chartData.labels) {
        Chart& chart = chartData.charts[pId];

        assert(chart.label == pId);

        if (chart.faces.size() == 0)
            continue;

        std::unordered_set<size_t> cornerSet(corners[pId].begin(), corners[pId].end());

        EdgeLabelMap edgeLabelMap;
        std::vector<std::vector<size_t>> vertexNextMap(mesh.vert.size());

        std::set<size_t> remainingVertices;

#ifndef NDEBUG
        vcg::tri::UpdateSelection<TriangleMeshType>::FaceClear(mesh);
        for (const size_t& fId : chart.faces) {
            mesh.face[fId].SetS();
        }
        TriangleMeshType newMesh;
        vcg::tri::Append<TriangleMeshType, TriangleMeshType>::Mesh(newMesh, mesh, true);
        vcg::tri::io::ExporterOBJ<TriangleMeshType>::Save(newMesh, "res/lastChartComputed.obj", vcg::tri::io::Mask::IOM_FACECOLOR);
#endif

        //Fill edge map and next vertex map
        for (const size_t& fId : chart.borderFaces) {
            typename TriangleMeshType::FaceType* currentFacePointer = &mesh.face[fId];
            vcg::face::Pos<typename TriangleMeshType::FaceType> pos(currentFacePointer, 0);

            for (int k = 0; k < currentFacePointer->VN(); k++) {
                pos.FlipF();
                size_t adjFace = vcg::tri::Index(mesh, pos.F());
                int adjLabel = faceLabel[adjFace];

                bool isBorderEdge = false;
                int adjChartLabel = -2;

                if (currentFacePointer == pos.F()) {
                    adjChartLabel = -1;
                    isBorderEdge = true;
                }
                else if (adjLabel != chart.label) {
                    adjChartLabel = adjLabel;
                    isBorderEdge = true;
                }
                pos.FlipF();

                //For each border edge
                if (isBorderEdge) {
                    assert(adjChartLabel > -2);

                    typename TriangleMeshType::VertexType* vStart = pos.V();
                    pos.FlipV();
                    typename TriangleMeshType::VertexType* vEnd = pos.V();
                    pos.FlipV();

                    size_t vStartId = vcg::tri::Index(mesh, vStart);
                    size_t vEndId = vcg::tri::Index(mesh, vEnd);

                    std::pair<size_t, size_t> edge(vStartId, vEndId);
                    if (edge.first > edge.second) {
                        std::swap(edge.first, edge.second);
                    }

                    edgeLabelMap.insert(std::make_pair(edge, std::make_pair(chart.label, adjChartLabel)));
                    vertexNextMap[vStartId].push_back(vEndId);

                    remainingVertices.insert(vStartId);
                    remainingVertices.insert(vEndId);
                }

                pos.FlipV();
                pos.FlipE();
            }
        }

        do {
            //Find first label
            size_t vStartId;
            size_t vCurrentId;
            size_t vNextId;

            //Corner detection variables
            typename TriangleMeshType::CoordType lastEdgeVec;
            bool isCorner = false;

            vCurrentId = *remainingVertices.begin();

            std::vector<size_t> nextConfiguration = findVertexChainPath(vCurrentId, vertexNextMap);
            vNextId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];

            //Get last edge vector
            lastEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
            lastEdgeVec.Normalize();

//            std::pair<size_t, size_t> startEdge(vCurrentId, vNextId);
//            if (startEdge.first > startEdge.second) {
//                std::swap(startEdge.first, startEdge.second);
//            }

            int currentLabel;

            //Iterate in the borders to get the first corner
            vStartId = vCurrentId;
            size_t firstCornerIterations = 0;
            do {
                //Next border edge
                vCurrentId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];
                vNextId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];

                typename TriangleMeshType::CoordType currentEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
                currentEdgeVec.Normalize();

                //Check if it is a corner
                isCorner = cornerSet.find(vCurrentId) != cornerSet.end();

                lastEdgeVec = currentEdgeVec;

                firstCornerIterations++;
            } while (!isCorner && vCurrentId != vStartId && firstCornerIterations < MAXITERATIONS);

#ifndef NDEBUG
            if (firstCornerIterations >= MAXITERATIONS) {
                std::cout << "Errore: error iterating! Cannot find the first corner or get back to the start vertex." << std::endl;
            }
#endif

#ifndef NDEBUG
            if (vCurrentId == vStartId) {
                std::cout << "Warning 1: input mesh is not well-defined: no corners!" << std::endl;
            }
#endif
            vStartId = vCurrentId;
            vNextId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];

            ChartSide currentSide;
            currentSide.length = 0;
            currentSide.size = 0;


            int adjChartLabel;
            do {
                size_t subSideId = chartData.subSides.size();
                ChartSubSide currentSubSide;

                //Get edge
                std::pair<size_t, size_t> edge(vCurrentId, vNextId);
                if (edge.first > edge.second) {
                    std::swap(edge.first, edge.second);
                }
                //Get current label on the other side
                const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
                assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
                adjChartLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;

                std::unordered_set<size_t> cornerSetAdj;
                if (adjChartLabel >= 0)
                    cornerSetAdj.insert(corners[adjChartLabel].begin(), corners[adjChartLabel].end());

                double length = 0;

                bool newSubSide = false;

                bool firstIteration = true;
                isCorner = false;
                bool isAdjCorner = false;
                size_t vSubSideStartId = vCurrentId;
                size_t iterations = 0;
                do {
                    typename TriangleMeshType::CoordType currentEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
                    currentEdgeVec.Normalize();

                    std::pair<size_t, size_t> edge(vCurrentId, vNextId);
                    if (edge.first > edge.second) {
                        std::swap(edge.first, edge.second);
                    }

                    //Check if it is a corner
                    if (!firstIteration) {
                        isCorner = cornerSet.find(vCurrentId) != cornerSet.end();
                        isAdjCorner = cornerSetAdj.find(vCurrentId) != cornerSetAdj.end();
                    }

                    //Get current label on the other subside
                    const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
                    assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
                    currentLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;

                    if (!isCorner && !isAdjCorner && currentLabel == adjChartLabel) {
                        EdgeSubSideMap::iterator findIt = edgeSubSideMap.find(edge);

                        //If the subside has already been processed
                        if (findIt == edgeSubSideMap.end()) {
                            currentSubSide.vertices.push_back(vCurrentId);

                            length += (mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P()).Norm();

                            edgeSubSideMap.insert(std::make_pair(edge, subSideId));

                            newSubSide = true;
                        }
                        else if (firstIteration) {
                            subSideId = findIt->second;
                        }
                        firstIteration = false;

                        remainingVertices.erase(vCurrentId);

                        //Next border edge
                        vCurrentId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];
                        vNextId = vertexNextMap[vCurrentId][nextConfiguration[vCurrentId]];

                        lastEdgeVec = currentEdgeVec;
                    }
                } while (!isCorner && !isAdjCorner && currentLabel == adjChartLabel && vCurrentId != vSubSideStartId && iterations < MAXITERATIONS);
#ifndef NDEBUG
                if (iterations >= MAXITERATIONS) {
                    std::cout << "Error: error iterating! Cannot find a corner or get back to the start vertex." << std::endl;
                }
#endif
#ifndef NDEBUG
                if (vCurrentId == vSubSideStartId) {
                    std::cout << "Warning 2: input mesh is not well-defined: single border chart with no corners!" << std::endl;
                }
#endif

                //True if the subside is reversed (from the last to the first vertex)
                bool reversed;

                if (newSubSide) {
                    //Add last vertex
                    currentSubSide.vertices.push_back(vCurrentId);

                    //Create new side
                    int chartSubSideId = chart.chartSubSides.size();
                    chart.chartSubSides.push_back(subSideId);

                    currentSubSide.incidentCharts[0] = chart.label;
                    currentSubSide.incidentCharts[1] = adjChartLabel;

                    currentSubSide.length = length;

                    currentSubSide.incidentChartSideId[0] = chartSubSideId;

                    if (adjChartLabel >= 0) {
                        currentSubSide.isOnBorder = false;

                        chart.adjacentCharts.push_back(adjChartLabel);
                    }
                    else {
                        currentSubSide.isOnBorder = true;
                        currentSubSide.incidentChartSideId[1] = -1;
                    }

                    assert(currentSubSide.vertices.size() >= 2);
                    currentSubSide.size = currentSubSide.vertices.size() - 1;

                    chartData.subSides.push_back(currentSubSide);


                    //Pop last vertex
                    if (currentSide.vertices.size() > 0) {
                        assert(currentSide.vertices.back() == currentSubSide.vertices.front());
                        currentSide.vertices.pop_back();
                    }
                    currentSide.vertices.insert(
                                currentSide.vertices.end(),
                                currentSubSide.vertices.begin(),
                                currentSubSide.vertices.end());

                    reversed = false;
                }
                else {
                    assert(currentSubSide.vertices.size() == 0);

                    //Add the side to other chart
                    if (adjChartLabel >= 0) {
                        int chartSideId = chart.chartSubSides.size();
                        chart.chartSubSides.push_back(subSideId);

                        assert(chartData.subSides[subSideId].incidentCharts[1] == chart.label);
                        chartData.subSides[subSideId].incidentChartSideId[1] = chartSideId;

                        chart.adjacentCharts.push_back(adjChartLabel);

                        //Pop last vertex
                        if (currentSide.vertices.size() > 0) {
                            assert(currentSide.vertices.back() == chartData.subSides[subSideId].vertices.back());
                            currentSide.vertices.pop_back();
                        }

                        //Add side vertices
                        currentSide.vertices.insert(
                                    currentSide.vertices.end(),
                                    chartData.subSides[subSideId].vertices.rbegin(),
                                    chartData.subSides[subSideId].vertices.rend());
                    }

                    reversed = true;
                }

                currentSide.subsides.push_back(subSideId);
                currentSide.reversedSubside.push_back(reversed);

                currentSide.length += chartData.subSides[subSideId].length;
                currentSide.size += chartData.subSides[subSideId].size;

                if (isCorner) {
                    chart.chartSides.push_back(currentSide);
                    currentSide = ChartSide();
                }

            } while (vCurrentId != vStartId);

#ifndef NDEBUG
            if (!isCorner) {
                std::cout << "Warning 4: Chart has no final corner!" << std::endl;
            }
#endif

        } while (!remainingVertices.empty());

#ifndef NDEBUG
        if (chart.chartSides.size() < 3 || chart.chartSides.size() > 6) {
            std::cout << "Warning 3: Chart " << pId << " has " << chart.chartSides.size() << " sides." << std::endl;
        }
#endif
    }

    return chartData;
}

template<class TriangleMeshType>
void findChartFacesAndBorderFaces(
        TriangleMeshType& mesh,
        const std::vector<int>& faceLabel,
        ChartData& chartData)
{
    if (mesh.face.size() == 0)
        return;

    std::set<int>& labels = chartData.labels;
    std::vector<Chart>& charts = chartData.charts;

    for (size_t fId = 0; fId < mesh.face.size(); fId++) {
        if (faceLabel.at(fId) >= 0)
            labels.insert(faceLabel.at(fId));
    }

    int maxChartLabel = *labels.rbegin();
    charts.resize(maxChartLabel + 1);

    std::vector<bool> visited(mesh.face.size(), false);

    for (size_t i = 0; i < mesh.face.size(); i++) {
        if (!visited[i]) {
            //Region growing to get chart
            std::stack<size_t> stack;
            stack.push(i);

            Chart chart;

            chart.label = faceLabel[i];

            if (chart.label >= 0) {
                std::set<size_t> borderFacesSet;
                do {
                    size_t fId = stack.top();
                    stack.pop();
                    assert(faceLabel[fId] == chart.label);

                    if (!visited[fId]) {
                        chart.faces.push_back(fId);

                        typename TriangleMeshType::FaceType* currentFacePointer = &mesh.face[fId];
                        vcg::face::Pos<typename TriangleMeshType::FaceType> pos(currentFacePointer, 0);
                        for (int k = 0; k < 3; k++) {
                            pos.FlipF();
                            size_t adjFace = vcg::tri::Index(mesh, pos.F());

                            //Saving border faces
                            if (currentFacePointer == pos.F() || faceLabel[adjFace] != chart.label) {
                                borderFacesSet.insert(fId);
                            }
                            else {
                                if (!visited[adjFace]) {
                                    stack.push(adjFace);
                                }
                            }
                            pos.FlipF();

                            //Next edge
                            pos.FlipV();
                            pos.FlipE();
                        }

                        visited[fId] = true;
                    }
                }
                while (!stack.empty());

                assert(borderFacesSet.size() >= 1);

                std::copy(borderFacesSet.begin(), borderFacesSet.end(), std::back_inserter(chart.borderFaces));

                charts[chart.label] = chart;
            }

        }
    }
}



////It works just on triangle meshes
//template<class TriangleMeshType>
//ChartData getCharts(
//        TriangleMeshType& mesh,
//        const std::vector<int>& faceLabel)
//{
//    typedef std::map<std::pair<size_t, size_t>, std::pair<int, int>> EdgeLabelMap;
//    typedef std::map<std::pair<size_t, size_t>, int> EdgeSubSideMap;

//    ChartData chartData;

//    //Region growing algorithm for getting charts
//    findChartFaces(mesh, faceLabel, chartData);

//    const double cornerCosLimit = cos(CORNERMINANGLE);


//    //TODO SPLIT IN FUNCTIONS
//    EdgeSubSideMap edgeSubSideMap;
//    for (const int& pId : chartData.labels) {
//        Chart& chart = chartData.charts[pId];

//        assert(chart.label == pId);

//        if (chart.faces.size() == 0)
//            continue;

//        EdgeLabelMap edgeLabelMap;
//        std::map<size_t, size_t> vertexNextMap;

//        std::set<size_t> remainingVertices;

//        //Fill edge map and next vertex map
//        for (const size_t& fId : chart.borderFaces) {
//            typename TriangleMeshType::FaceType* currentFacePointer = &mesh.face[fId];
//            vcg::face::Pos<typename TriangleMeshType::FaceType> pos(currentFacePointer, 0);

//            for (int k = 0; k < currentFacePointer->VN(); k++) {
//                pos.FlipF();
//                size_t adjFace = vcg::tri::Index(mesh, pos.F());
//                int adjLabel = faceLabel[adjFace];

//                bool isBorderEdge = false;
//                int adjChartLabel = -2;

//                if (currentFacePointer == pos.F()) {
//                    adjChartLabel = -1;
//                    isBorderEdge = true;
//                }
//                else if (adjLabel != chart.label) {
//                    adjChartLabel = adjLabel;
//                    isBorderEdge = true;
//                }
//                pos.FlipF();

//                //For each border edge
//                if (isBorderEdge) {
//                    assert(adjChartLabel > -2);

//                    typename TriangleMeshType::VertexType* vStart = pos.V();
//                    pos.FlipV();
//                    typename TriangleMeshType::VertexType* vEnd = pos.V();
//                    pos.FlipV();

//                    size_t vStartId = vcg::tri::Index(mesh, vStart);
//                    size_t vEndId = vcg::tri::Index(mesh, vEnd);

//                    std::pair<size_t, size_t> edge(vStartId, vEndId);
//                    if (edge.first > edge.second) {
//                        std::swap(edge.first, edge.second);
//                    }

//                    edgeLabelMap.insert(std::make_pair(edge, std::make_pair(chart.label, adjChartLabel)));
//                    vertexNextMap.insert(std::make_pair(vStartId, vEndId));

//                    remainingVertices.insert(vStartId);
//                    remainingVertices.insert(vEndId);
//                }

//                pos.FlipV();
//                pos.FlipE();
//            }
//        }

//        do {
//            //Find first label
//            size_t vStartId;
//            size_t vCurrentId;
//            size_t vNextId;

//            //Corner detection variables
//            bool firstIteration = true;
//            typename TriangleMeshType::CoordType lastEdgeVec;
//            bool isCorner = false;

//            vCurrentId = *remainingVertices.begin();
//            vNextId = vertexNextMap.at(vCurrentId);

//            //Get last edge vector
//            lastEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
//            lastEdgeVec.Normalize();

//            std::pair<size_t, size_t> startEdge(vCurrentId, vNextId);
//            if (startEdge.first > startEdge.second) {
//                std::swap(startEdge.first, startEdge.second);
//            }

//            int currentLabel;

//            //Iterate in the borders to get the first corner
//            vStartId = vCurrentId;
//            do {
//                //Next border edge
//                vCurrentId = vertexNextMap.at(vCurrentId);
//                vNextId = vertexNextMap.at(vCurrentId);

//                typename TriangleMeshType::CoordType currentEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
//                currentEdgeVec.Normalize();

//                //Check if it is a corner
//                double currentDot = dotProjectedOntoPlane<TriangleMeshType>(currentEdgeVec, lastEdgeVec, mesh.vert[vCurrentId].N());
//                if (std::fabs(currentDot) < cornerCosLimit) {
//                    isCorner = true;
//                }

//                //Get edge
//                std::pair<size_t, size_t> edge(vCurrentId, vNextId);
//                if (edge.first > edge.second) {
//                    std::swap(edge.first, edge.second);
//                }
//                //Get current label on the other side
//                const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
//                assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
//                currentLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;

//                lastEdgeVec = currentEdgeVec;
//                firstIteration = false;
//            } while (!isCorner && vCurrentId != vStartId);

//#ifndef NDEBUG
//            if (vCurrentId == vStartId) {
//                std::cout << "Warning 1: input mesh is not well-defined: no corners!" << std::endl;
//            }
//#endif

//            //Get all the sub sides
//            vStartId = vCurrentId;
//            vNextId = vertexNextMap.at(vCurrentId);

//            ChartSide currentSide;
//            currentSide.length = 0;
//            currentSide.size = 0;

//            int adjChartLabel;
//            do {
//                size_t subSideId = chartData.subSides.size();
//                ChartSubSide currentSubSide;

//                adjChartLabel = currentLabel;
//                double length = 0;

//                bool newSubSide = false;

//                firstIteration = true;
//                isCorner = false;
//                size_t vSubSideStartId = vCurrentId;
//                do {
//                    typename TriangleMeshType::CoordType currentEdgeVec = mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P();
//                    currentEdgeVec.Normalize();

//                    std::pair<size_t, size_t> edge(vCurrentId, vNextId);
//                    if (edge.first > edge.second) {
//                        std::swap(edge.first, edge.second);
//                    }

//                    //Check if it is a corner
//                    if (!firstIteration) {
//                        double currentDot = dotProjectedOntoPlane<TriangleMeshType>(currentEdgeVec, lastEdgeVec, mesh.vert[vCurrentId].N());
//                        if (std::fabs(currentDot) < cornerCosLimit) {
//                            isCorner = true;
//                        }
//                    }

//                    //Get current label on the other subside
//                    const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
//                    assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
//                    currentLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;

//                    if (!isCorner && currentLabel == adjChartLabel) {
//                        EdgeSubSideMap::iterator findIt = edgeSubSideMap.find(edge);

//                        //If the subside has already been processed
//                        if (findIt == edgeSubSideMap.end()) {
//                            currentSubSide.vertices.push_back(vCurrentId);

//                            length += (mesh.vert[vNextId].P() - mesh.vert[vCurrentId].P()).Norm();

//                            edgeSubSideMap.insert(std::make_pair(edge, subSideId));

//                            newSubSide = true;
//                        }
//                        else if (firstIteration) {
//                            subSideId = findIt->second;
//                        }
//                        firstIteration = false;

//                        //Next border edge
//                        vCurrentId = vertexNextMap.at(vCurrentId);
//                        vNextId = vertexNextMap.at(vCurrentId);

//                        remainingVertices.erase(vCurrentId);

//                        lastEdgeVec = currentEdgeVec;
//                    }
//                } while (!isCorner && currentLabel == adjChartLabel && vCurrentId != vSubSideStartId);

//#ifndef NDEBUG
//                if (vCurrentId == vSubSideStartId) {
//                    std::cout << "Warning 2: input mesh is not well-defined: single border chart with no corners!" << std::endl;
//                }
//#endif

//                //Subside appears reversed
//                bool reversed;

//                if (newSubSide) {
//                    //Add last vertex
//                    currentSubSide.vertices.push_back(vCurrentId);

//                    //Create new side
//                    int chartSubSideId = chart.chartSubSides.size();
//                    chart.chartSubSides.push_back(subSideId);

//                    currentSubSide.incidentCharts[0] = chart.label;
//                    currentSubSide.incidentCharts[1] = adjChartLabel;

//                    currentSubSide.length = length;

//                    currentSubSide.incidentChartSideId[0] = chartSubSideId;

//                    if (adjChartLabel >= 0) {
//                        currentSubSide.isOnBorder = false;

//                        chart.adjacentCharts.push_back(adjChartLabel);
//                    }
//                    else {
//                        currentSubSide.isOnBorder = true;
//                        currentSubSide.incidentChartSideId[1] = -1;
//                    }

//                    assert(currentSubSide.vertices.size() >= 2);
//                    currentSubSide.size = currentSubSide.vertices.size() - 1;

//                    chartData.subSides.push_back(currentSubSide);


//                    //Pop last vertex
//                    if (currentSide.vertices.size() > 0) {
//                        assert(currentSide.vertices.back() == currentSubSide.vertices.front());
//                        currentSide.vertices.pop_back();
//                    }
//                    currentSide.vertices.insert(
//                                currentSide.vertices.end(),
//                                currentSubSide.vertices.begin(),
//                                currentSubSide.vertices.end());

//                    reversed = false;
//                }
//                else {
//                    assert(currentSubSide.vertices.size() == 0);

//                    //Add the side to other chart
//                    if (adjChartLabel >= 0) {
//                        int chartSideId = chart.chartSubSides.size();
//                        chart.chartSubSides.push_back(subSideId);

//                        assert(chartData.subSides[subSideId].incidentCharts[1] == chart.label);
//                        chartData.subSides[subSideId].incidentChartSideId[1] = chartSideId;

//                        chart.adjacentCharts.push_back(adjChartLabel);

//                        //Pop last vertex
//                        if (currentSide.vertices.size() > 0) {
//                            assert(currentSide.vertices.back() == chartData.subSides[subSideId].vertices.back());
//                            currentSide.vertices.pop_back();
//                        }

//                        //Add side vertices
//                        currentSide.vertices.insert(
//                                    currentSide.vertices.end(),
//                                    chartData.subSides[subSideId].vertices.rbegin(),
//                                    chartData.subSides[subSideId].vertices.rend());
//                    }

//                    reversed = true;
//                }

//                currentSide.subsides.push_back(subSideId);
//                currentSide.reversedSubside.push_back(reversed);

//                currentSide.length += chartData.subSides[subSideId].length;
//                currentSide.size += chartData.subSides[subSideId].size;

//                if (isCorner) {
//                    chart.chartSides.push_back(currentSide);
//                    currentSide = ChartSide();
//                }

//            } while (vCurrentId != vStartId);

//            assert(isCorner);

//        } while (!remainingVertices.empty());

//        if (chart.chartSides.size() < 3 || chart.chartSides.size() > 6) {
//            std::cout << "Warning 3: Chart " << pId << " has " << chart.chartSides.size() << " sides." << std::endl;
//        }
//    }

//    return chartData;
//}


//template<class TriangleMeshType>
//double dotProjectedOntoPlane(
//        const typename TriangleMeshType::CoordType& v1,
//        const typename TriangleMeshType::CoordType& v2,
//        const typename TriangleMeshType::CoordType& normal)
//{
//    typename TriangleMeshType::CoordType proj1 = v1 - (normal * v1.dot(normal));
//    typename TriangleMeshType::CoordType proj2 = v2 - (normal * v2.dot(normal));
//    proj1.Normalize();
//    proj2.Normalize();

//    return proj1.dot(proj2);
//}

}
}
