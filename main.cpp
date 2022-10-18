#include <QApplication>
#include <QDebug>

#include <iostream>

#include "MainWindow.h"
#include "CubeTetrahedron.h"
#include "ColorTransformation.h"
#include <vector>

void tetrahedron();

int main(int argc, char *argv[]) {
    ColorTransformation ct;
    ct.print();
    ct.export2PLY("/home/imanol/testCube.ply");
    ct.export2PLYTetras("/home/imanol/testCubeTetras.ply");
    std::vector<std::vector<float>> cp = {{0,0,0}, {0,0,1}, {0,1,0}, {0,1,1}, {1,0,0}, {1,0,1}, {1,1,0}, {1,1,1}, {2,0,0}, {2,0,1}, {2,1,0}, {2,1,1}};
    std::cout << "Control Points: " << std::endl;
    for(const auto &p : cp) {
        std::cout << p[0] <<", " << p[1] << ", " << p[2] << std::endl;
    }
    ct.setControlPoints(cp);

    ct.computeBiharmonicCoordinates();

    ct.updateColorTransformation();
    ct.export2PLY("/home/imanol/testTransfCube.ply");

    ct.print();

    return 1;

    //std::vector<std::vector<float>> cp = {{4,4,4}};
    ct.setControlPoints(cp);

    ct.computeBiharmonicCoordinates();

    std::vector<float> cp2 = {4, 4, 5};
    ct.updateControlPoint(0, cp2);

    ct.updateColorTransformation();

    ct.print();


    return 0;


    QApplication a(argc, argv);
    //qDebug() << "Hello World";
    /*QPushButton button("Hello, from Clion", nullptr);
    button.resize(200,200);
    button.show();*/
    MainWindow mainWindow;
    mainWindow.show();

    return QApplication::exec();
}