//#include <QDirIterator>
//#include <QStringList>
//#include <iostream>

//#include "quadboolean/quadutils.h"

//#include "meshtypes.h"
//#include <wrap/io_trimesh/export.h>
//#include <wrap/io_trimesh/import.h>

//int main(int argc, char *argv[])
//{
//    if (argc >= 3) {
//        QString dir(argv[1]);
//        std::string outputDir(argv[2]);
//        QDirIterator it(dir, QStringList() << "*.obj", QDir::Files, QDirIterator::Subdirectories);
//        while (it.hasNext()) {
//            QuadBoolean::PolyMesh mesh, triangulatedMesh;

//            std::string file = it.next().toStdString();

//            int loadMask;
//            vcg::tri::io::Importer<QuadBoolean::PolyMesh>::Open(mesh, file.c_str(), loadMask);

//            std::cout << file << " -> ";

//            if (mesh.vert.size() < 200) {
//                cout << std::endl;
//                continue;
//            }

//            vcg::tri::Append<QuadBoolean::PolyMesh, QuadBoolean::PolyMesh>::Mesh(triangulatedMesh, mesh);

//            std::string path = file.substr(0, file.find_last_of("/"));
//            std::string fullname = file.substr(file.find_last_of("/") + 1);
//            std::string name = fullname.substr(0, fullname.find_last_of("."));
//            std::string ext = fullname.substr(fullname.find_last_of(".") + 1);

//            std::string newFileTriangulated = outputDir + "/" + name + "_triangulated." + ext;
//            std::string newFile = outputDir + "/" + name + "." + ext;

//            std::cout << newFileTriangulated << std::endl;

//            std::vector<int> birthQuad = QuadBoolean::internal::splitQuadInTriangle(triangulatedMesh);

//            ofstream myfile;
//            std::string newFileTxt = outputDir + "/" + name + "_triangulated_map.txt";
//            myfile.open(newFileTxt);
//            myfile << triangulatedMesh.face.size();
//            for (int i = 0; i < triangulatedMesh.face.size(); i++) {
//                myfile << birthQuad[i] << std::endl;
//                triangulatedMesh.face[i].C() = mesh.face[birthQuad[i]].C();
//            }
//            myfile.close();

//            vcg::tri::io::ExporterOBJ<QuadBoolean::PolyMesh>::Save(triangulatedMesh, newFileTriangulated.c_str(), loadMask);
//            vcg::tri::io::ExporterOBJ<QuadBoolean::PolyMesh>::Save(mesh, newFile.c_str(), loadMask);
//        }

//    }
//}
