#include "quadcharts.h"
#include <map>

namespace QuadBoolean {


void findChartFaces(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel,
        ChartData& chartData);

//It works just on triangle meshes
ChartData getCharts(PolyMesh& mesh, const std::vector<int>& faceLabel)
{
    typedef std::map<std::pair<PolyMesh::CoordType, PolyMesh::CoordType>, std::pair<int, int>> EdgeLabelMap;
    typedef std::map<std::pair<PolyMesh::CoordType, PolyMesh::CoordType>, int> EdgeSideMap;
    vcg::tri::UpdateTopology<PolyMesh>::FaceFace(mesh);

    ChartData chartData;

    //Region growing algorithm for getting charts
    findChartFaces(mesh, faceLabel, chartData);


    //TODO SPLIT IN FUNCTIONS
    EdgeSideMap edgeSideMap;
    for (const int& pId : chartData.labels) {
        Chart& chart = chartData.charts[pId];

        assert(chart.label == pId);

        if (chart.faces.size() == 0)
            continue;

        EdgeLabelMap edgeLabelMap;
        std::map<PolyMesh::VertexType*, PolyMesh::VertexType*> vertexNextMap;

        std::set<PolyMesh::VertexType*> remainingVertices;

        //Fill edge map and next vertex map
        for (const size_t& fId : chart.borderFaces) {
            PolyMesh::FaceType* currentFacePointer = &mesh.face[fId];
            vcg::face::Pos<PolyMesh::FaceType> pos(currentFacePointer, 0);

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

                    PolyMesh::VertexType* vStart = pos.V();
                    pos.FlipV();
                    PolyMesh::VertexType* vEnd = pos.V();
                    pos.FlipV();

                    std::pair<PolyMesh::CoordType, PolyMesh::CoordType> edge(vStart->P(), vEnd->P());
                    if (edge.first > edge.second) {
                        std::swap(edge.first, edge.second);
                    }

                    edgeLabelMap.insert(std::make_pair(edge, std::make_pair(chart.label, adjChartLabel)));
                    vertexNextMap.insert(std::make_pair(vStart, vEnd));

                    remainingVertices.insert(vStart);
                    remainingVertices.insert(vEnd);
                }

                pos.FlipV();
                pos.FlipE();
            }
        }

        do {
            //Find first label
            PolyMesh::VertexType* vStart;
            PolyMesh::VertexType* vCurrent;
            PolyMesh::VertexType* vNext;

            vCurrent = *remainingVertices.begin();
            vNext = vertexNextMap.at(vCurrent);

            std::pair<PolyMesh::CoordType, PolyMesh::CoordType> startEdge(vCurrent->P(), vNext->P());
            if (startEdge.first > startEdge.second) {
                std::swap(startEdge.first, startEdge.second);
            }

            int adjChartLabel;
            int currentLabel;

            //Get current label on the other side
            const std::pair<int,int>& startEdgeLabels = edgeLabelMap.at(startEdge);
            assert(startEdgeLabels.first == chart.label || startEdgeLabels.second == chart.label);
            adjChartLabel = startEdgeLabels.first == chart.label ? startEdgeLabels.second : startEdgeLabels.first;

            //Iterate in the borders to get the first corner
            vStart = vCurrent;
            do {
                //Next border edge
                vCurrent = vertexNextMap.at(vCurrent);
                vNext = vertexNextMap.at(vCurrent);

                std::pair<PolyMesh::CoordType, PolyMesh::CoordType> edge(vCurrent->P(), vNext->P());
                if (edge.first > edge.second) {
                    std::swap(edge.first, edge.second);
                }

                //Get current label on the other side
                const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
                assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
                currentLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;
            } while (currentLabel == adjChartLabel && vCurrent != vStart);

            if (vCurrent == vStart) {
                std::cout << "Warning 1: input mesh is not well-defined: single border chart!" << std::endl;
            }

            //Get all the sides
            vStart = vCurrent;
            vNext = vertexNextMap.at(vCurrent);
            do {
                int sideId = chartData.sides.size();
                ChartSide currentSide;

                adjChartLabel = currentLabel;
                double length = 0;

                bool newSide = false;
                bool firstEdge = true;

                PolyMesh::VertexType* vSideStart = vCurrent;
                do {
                    std::pair<PolyMesh::CoordType, PolyMesh::CoordType> edge(vCurrent->P(), vNext->P());
                    if (edge.first > edge.second) {
                        std::swap(edge.first, edge.second);
                    }

                    //Get current label on the other side
                    const std::pair<int,int>& currentEdgeLabels = edgeLabelMap.at(edge);
                    assert(currentEdgeLabels.first == chart.label || currentEdgeLabels.second == chart.label);
                    currentLabel = currentEdgeLabels.first == chart.label ? currentEdgeLabels.second : currentEdgeLabels.first;

                    if (currentLabel == adjChartLabel) {
                        EdgeSideMap::iterator findIt = edgeSideMap.find(edge);

                        //If the side has already been processed
                        if (findIt == edgeSideMap.end()) {
                            currentSide.vertices.push_back(vCurrent);

                            length += (vNext->P() - vCurrent->P()).Norm();

                            edgeSideMap.insert(std::make_pair(edge, sideId));

                            newSide = true;
                        }
                        else if (firstEdge) {
                            sideId = findIt->second;
                        }

                        firstEdge = false;

                        //Next border edge
                        vCurrent = vertexNextMap.at(vCurrent);
                        vNext = vertexNextMap.at(vCurrent);

                        remainingVertices.erase(vCurrent);
                    }
                } while (currentLabel == adjChartLabel && vCurrent != vSideStart);

                //Add last vertex
                currentSide.vertices.push_back(vCurrent);

                if (vCurrent == vSideStart) {
                    std::cout << "Warning 2: input mesh is not well-defined: single border chart!" << std::endl;
                }

                if (newSide) {
                    //Create new side
                    int chartSideId = chart.chartSides.size();
                    chart.chartSides.push_back(sideId);

                    currentSide.incidentCharts[0] = chart.label;
                    currentSide.incidentCharts[1] = adjChartLabel;

                    currentSide.length = length;

                    currentSide.incidentChartSideId[0] = chartSideId;

                    if (adjChartLabel >= 0) {
                        currentSide.isOnBorder = false;
                    }
                    else {
                        currentSide.isOnBorder = true;
                        currentSide.incidentChartSideId[1] = -1;
                    }

                    assert(currentSide.vertices.size() >= 2);
                    currentSide.size = currentSide.vertices.size() - 1;

                    chartData.sides.push_back(currentSide);

                    chart.adjacentCharts.push_back(adjChartLabel);
                }
                else {
                    //Add the side to other chart
                    if (adjChartLabel >= 0) {
                        int chartSideId = chart.chartSides.size();
                        chart.chartSides.push_back(sideId);

                        assert(chartData.sides[sideId].incidentCharts[1] == chart.label);
                        chartData.sides[sideId].incidentChartSideId[1] = chartSideId;

                        chart.adjacentCharts.push_back(adjChartLabel);
                    }
                }

            } while (vCurrent != vStart);

        } while (!remainingVertices.empty());
    }

    return chartData;
}

void findChartFaces(
        PolyMesh& mesh,
        const std::vector<int>& faceLabel,
        ChartData& chartData)
{
    std::set<int>& labels = chartData.labels;
    std::vector<Chart>& charts = chartData.charts;

    for (size_t fId = 0; fId < mesh.face.size(); fId++) {
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

            std::set<size_t> borderFacesSet;
            do {
                size_t fId = stack.top();
                stack.pop();
                assert(faceLabel[fId] == chart.label);

                if (!visited[fId]) {
                    chart.faces.push_back(fId);

                    PolyMesh::FaceType* currentFacePointer = &mesh.face[fId];
                    vcg::face::Pos<PolyMesh::FaceType> pos(currentFacePointer, 0);
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
