#include <iostream>

#include <QApplication>
#include "quadbooleanwindow.h"

#include <GL/glut.h>

int main(int argc, char *argv[])
{
    glutInit(&argc,argv);

    QApplication app(argc, argv);
    QuadBooleanWindow qbw;
    qbw.showMaximized();
    return app.exec();
}


//#include <iostream>

//#include "quadboolean.h"

//#include <wrap/io_trimesh/import_obj.h>

//int main(int argc, char **argv )
//{
////    if(argc<3)
////    {
////        std::cout << "Usage Quadrangulation <meshfilename1.obj> <meshfilename2.obj>\n";
////        return -1;
////    }
////    assert(argc>=3);

////    std::string meshName1 = argv[1];
////    std::string meshName2 = argv[2];

//    using namespace QuadBoolean;

//    //TODO REMOVE
//    const std::string filename = "fertility.obj";

//    std::string meshName1("/mnt/OS/Workspace/Dataset/quadboolean/dataset/" + filename);
//    std::string meshName2("/mnt/OS/Workspace/Dataset/quadboolean/dataset/" + filename);

//    std::string PolyMeshPath;

//    PolyMesh mesh1, mesh2;

//    int loadMask = 0;

//    vcg::tri::io::ImporterOBJ<PolyMesh>::Open(mesh1, meshName1.c_str(), loadMask);
//    vcg::tri::io::ImporterOBJ<PolyMesh>::Open(mesh2, meshName2.c_str(), loadMask);

//    //TODO REMOVE
//    vcg::tri::UpdateBounding<PolyMesh>::Box(mesh2);
//    const PolyMesh::CoordType translation(mesh2.bbox.DimX()/4, mesh2.bbox.DimY()/4, mesh2.bbox.DimZ()/4);
////    const PolyMesh::CoordType translation(23, 0, 0);
//    for (size_t i = 0; i < mesh2.vert.size(); i++) {
//        mesh2.vert[i].P() = mesh2.vert[i].P() + translation;
//    }

//    PolyMesh result;
//    quadUnion(mesh1, mesh2, result);

//    return 0;
//}
