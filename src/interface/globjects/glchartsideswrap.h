#ifndef GLCHARTSIDESWRAP_H
#define GLCHARTSIDESWRAP_H

#include <quadboolean/includes/quadcharts.h>

template<class MeshType>
class GLChartSidesWrap
{
public:
    GLChartSidesWrap();

    void GLDraw();

    MeshType* mesh;
    QuadBoolean::internal::ChartData* chartData;
    std::vector<int>* ilpResult;

    bool visible;
    bool ilpVisible;
};

#include "glchartsideswrap.tpp"

#endif // GLCHARTSIDESWRAP_H
