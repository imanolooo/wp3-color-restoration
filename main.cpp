#include <QApplication>
#include <QDebug>

#include <iostream>

#include "MainWindow.h"
#include "CubeTetrahedron.h"

void tetrahedron();

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    //qDebug() << "Hello World";
    /*QPushButton button("Hello, from Clion", nullptr);
    button.resize(200,200);
    button.show();*/
    MainWindow mainWindow;
    mainWindow.show();

    return QApplication::exec();
}