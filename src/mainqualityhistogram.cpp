#include <iostream>

#include <quadboolean/defaultmeshtypes.h>

#include <wrap/io_trimesh/import_obj.h>

#include <vcg/complex/algorithms/polygonal_algorithms.h>

int main(int argc, char **argv )
{
    if(argc<4)
    {
        std::cout << "Usage qualityhistogram <mesh1.obj> <template|angle> <result.csv>\n";
        return -1;
    }

    std::string mesh1Name = argv[1];
    std::string qualityType = argv[2];
    std::string qualityFile = argv[3];

    using namespace QuadBoolean;

    PolyMesh mesh;

    int loadMask = 0;
    vcg::tri::io::ImporterOBJ<PolyMesh>::Open(mesh, mesh1Name.c_str(), loadMask);

    if (qualityType == "angle") {
        vcg::PolygonalAlgorithm<PolyMesh>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QAngle);
    }
    else {
        vcg::PolygonalAlgorithm<PolyMesh>::UpdateQuality(mesh, vcg::PolygonalAlgorithm<PolyMesh>::QTemplate);
    }

    std::ofstream file;
    file.open(qualityFile);

    for (size_t i = 0; i < mesh.face.size(); i++) {
        file << mesh.face[i].Q() << std::endl;
    }

    file.close();

    return 0;
}
