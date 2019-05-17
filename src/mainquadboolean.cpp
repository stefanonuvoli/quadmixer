#include <iostream>

#include <quadboolean/quadboolean.h>
#include <quadboolean/defaultmeshtypes.h>

#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/export_obj.h>

int main(int argc, char **argv )
{
    if(argc<5)
    {
        std::cout << "Usage Quadrangulation <mesh1.obj> <+|-|*> <mesh2.obj> <result.obj>\n";
        return -1;
    }

    std::string mesh1Name = argv[1];
    std::string operation = argv[2];
    std::string mesh2Name = argv[3];
    std::string meshResultName = argv[4];

    using namespace QuadBoolean;

    PolyMesh mesh1, mesh2;

    int loadMask = 0;

    vcg::tri::io::ImporterOBJ<PolyMesh>::Open(mesh1, mesh1Name.c_str(), loadMask);
    vcg::tri::io::ImporterOBJ<PolyMesh>::Open(mesh2, mesh2Name.c_str(), loadMask);

    PolyMesh result;

    quadBoolean(mesh1, mesh2, QuadBoolean::Operation::UNION, result);
    vcg::tri::io::ExporterOBJ<PolyMesh>::Save(mesh2, mesh2Name.c_str(), loadMask);

    return 0;
}
