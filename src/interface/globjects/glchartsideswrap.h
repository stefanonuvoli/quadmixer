#ifndef GLCHARTSIDESWRAP_H
#define GLCHARTSIDESWRAP_H

#include <quadretopology/includes/qr_charts.h>

template<class MeshType>
class GLChartSidesWrap
{
public:
    GLChartSidesWrap();

    void GLDraw();

    MeshType* mesh;
    QuadRetopology::ChartData* chartData;
    std::vector<int>* ilpResult;

    bool visible;
    bool ilpVisible;
};

#include "glchartsideswrap.cpp"

#endif // GLCHARTSIDESWRAP_H
