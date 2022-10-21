#include <QApplication>
#include <QDebug>

#include <iostream>

#include "MainWindow.h"
#include "CubeTetrahedron.h"
#include "ColorTransformation.h"
#include <vector>

void tetrahedron();

int main(int argc, char *argv[]) {
    /*ColorTransformation ct;
    ct.print();

    //ct.export2PLY("/home/imanol/testCube.ply");

    std::cout << "Testing sampling..." << std::endl;
    float x_i, y_i, z_i;
    float x_o, y_o, z_o;
    x_i = 0.05; y_i = 0.05; z_i = 0.05;
    ct.sample(x_i, y_i, z_i, x_o, y_o, z_o);
    std::cout << x_i << ", " << y_i << ", " << z_i << " => " << x_o<< ", " << y_o << ", " << z_o << std::endl;

    x_i = 1.05; y_i = 0.95; z_i = 0.5;
    ct.sample(x_i, y_i, z_i, x_o, y_o, z_o);
    std::cout << x_i << ", " << y_i << ", " << z_i << " => " << x_o<< ", " << y_o << ", " << z_o << std::endl;

    //ct.export2PLY("/home/imanol/testCube.ply");
    //ct.export2PLYTetras("/home/imanol/testCubeTetras.ply");
    std::vector<std::vector<float>> cp = {{0,0,0}, {0,0,0.95}, {0,1,0}, {0,1,1}, {2,0,0}, {2,0,1}, {2,1,0}, {2,1,1}};
    std::cout << "Control Points: " << std::endl;
    for(const auto &p : cp) {
        std::cout << p[0] <<", " << p[1] << ", " << p[2] << std::endl;
    }
    ct.setControlPoints(cp);

    ct.computeBiharmonicCoordinates();

    std::vector<float> p = {2,0,1};
    ct.updateControlPoint(4, p);
    p = {2,0,2};
    ct.updateControlPoint(5, p);
    p = {2,1,1};
    ct.updateControlPoint(6, p);
    p = {2,1,2};
    ct.updateControlPoint(7, p);

    ct.updateColorTransformation();
    ct.export2PLY("/home/imanol/testTransfCube.ply");

    std::cout << "Testing Sampling..." << std::endl;
    x_i = 0.05; y_i = 0.05; z_i = 0.05;
    ct.sample(x_i, y_i, z_i, x_o, y_o, z_o);
    std::cout << x_i << ", " << y_i << ", " << z_i << " => " << x_o<< ", " << y_o << ", " << z_o << std::endl;

    x_i = 1.05; y_i = 0.95; z_i = 0.5;
    ct.sample(x_i, y_i, z_i, x_o, y_o, z_o);
    std::cout << x_i << ", " << y_i << ", " << z_i << " => " << x_o<< ", " << y_o << ", " << z_o << std::endl;




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

    */


    QApplication a(argc, argv);
    //qDebug() << "Hello World";
    /*QPushButton button("Hello, from Clion", nullptr);
    button.resize(200,200);
    button.show();*/
    MainWindow mainWindow;
    mainWindow.show();

    return QApplication::exec();
}