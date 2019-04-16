#ifndef GLCHARTSIDESWRAP_H
#define GLCHARTSIDESWRAP_H

#include "quadcharts.h"

template<class MeshType>
class GLChartSidesWrap
{
public:
    GLChartSidesWrap();

    void GLDraw();

    MeshType* mesh;
    QuadBoolean::ChartData* chartData;
    std::vector<int>* ilpResult;

    bool visible;
};

#include "glchartsideswrap.tpp"

#endif // GLCHARTSIDESWRAP_H
