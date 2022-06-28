//
// Created by imanol on 23/5/22.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include <iostream>
#include <QColorDialog>
#include <QFileDialog>
#include <QImageReader>
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "PointCloud.h"
#include "ColorTransformation.h"
#include "ColorTransformation2D.h"
#include <color.hpp>

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow) {
    _imageLabel = new QLabel();
    _imageLabel->setBackgroundRole(QPalette::Base);
    _imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    _imageLabel->setScaledContents(true);

    ui->setupUi(this);

    loadPickedColors();
    updatePickedColorsGUI();


}

MainWindow::~MainWindow() {
    if(_imageLabel != nullptr)  delete _imageLabel;
    delete ui;

}

void MainWindow::loadPickedColors() {

    //White
    _pickedColors["white"].push_back(PickedColor("white", "AbsS", 4786,6094,5,230,226,221));
    _pickedColors["white"].push_back(PickedColor("white", "AbsN", 3420,4228,1,246,247,249));
    _pickedColors["white"].push_back(PickedColor("white", "AbsS", 1463,2737,2,221,217,211));
    _pickedColors["white"].push_back(PickedColor("white", "AbsC", 6686,3144,1,226,226,221));

    //Garnet
    _pickedColors["garnet"].push_back(PickedColor("garnet", "AbsS", 3764,5481,2,115,81,76));
    _pickedColors["garnet"].push_back(PickedColor("garnet", "AbsN", 3650,3585,1,192,143,138));
    _pickedColors["garnet"].push_back(PickedColor("garnet", "AbsS", 1721,2691,1,135,76,75));
    _pickedColors["garnet"].push_back(PickedColor("garnet", "AbsC", 7324,2915,3,138,73,52));

    //Light grey
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsS", 4655,5928,2,196,194,196));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsN", 3561,4014,1,203,202,208));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsS", 1223,2886,2,203,197,191));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsC", 7042,3786,1,224,224,223));

    //Medium grey
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsS", 4682,591,2,113,115,126));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsN", 3600,3936,3,200,193,194));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsS", 1171,3019,3,171,171,175));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsC", 6777,3092,2,182,181,175));

    //Dark grey
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsS", 4767,5874,2,86,88,102));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsN", 3470,4059,3,159,156,162));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsS", 1686,2807,1,146,145,152));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsC", 6908,2994,3,80,86,93));

    //Black
    _pickedColors["black"].push_back(PickedColor("black", "AbsS", 462,4625,3,71,70,75));
    _pickedColors["black"].push_back(PickedColor("black", "AbsN", 3222,3379,1,122,124,140));
    _pickedColors["black"].push_back(PickedColor("black", "AbsS", 1265,2801,1,93,92,102));
    _pickedColors["black"].push_back(PickedColor("black", "AbsC", 7333,3056,0,56,48,51));

    //Ocher
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsS", 3800,5256,2,204,155,95));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsN", 2943,3932,1,234,200,154));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsS", 1577,2645,3,173,132,91));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsC", 6752,3019,1,219,162,78));

    //Light ocher
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsS", 3944,3887,3,234,222,203));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsN", 3629,3743,3,233,220,204));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsS", 1225,2853,3,224,191,158));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsC", 6702,2835,3,229,214,189));

    //Pink
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsS", 3212,532,3,236,200,186));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsN", 3311,4184,3,215,176,171));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsS", 1609,2822,1,210,172,150));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsC", 6636,3034,1,217,175,139));

    //Orange
    _pickedColors["orange"].push_back(PickedColor("orange", "AbsS", 3156,3547,3,216,136,111));

    //Green
    _pickedColors["green"].push_back(PickedColor("green", "AbsN", 3730,3904,2,137,145,140));
    _pickedColors["green"].push_back(PickedColor("green", "AbsS", 4280,3589,3,135,138,125));
    _pickedColors["green"].push_back(PickedColor("green", "AbsN", 3883,4113,1,140,145,145));
    _pickedColors["green"].push_back(PickedColor("green", "AbsS", 1553,2682,3,107,109,105));
    _pickedColors["green"].push_back(PickedColor("green", "AbsC", 6323,3535,1,127,127,94));

    //Light green
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsS", 4534,5953,2,175,183,157));
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsN", 3618,3689,1,201,196,168));
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsC", 6906,3884,1,174,178,148));

    //Red
    _pickedColors["red"].push_back(PickedColor("red", "AbsS", 3199,5214,2,189,103,86));
    _pickedColors["red"].push_back(PickedColor("red", "AbsN", 3239,4081,3,196,152,149));
    _pickedColors["red"].push_back(PickedColor("red", "AbsS", 1449,2984,1,178,109,95));
    _pickedColors["red"].push_back(PickedColor("red", "AbsC", 6855,6847,1,214,131,95));

    for(const auto &color : _pickedColors)
        ui->currentColor->addItem(color.first.c_str());

}

void MainWindow::updatePickedColorsGUI() {
    clearPickedColorsGUI();

    QString colorName = ui->currentColor->currentText();
    QGridLayout * pickedColorLayout = new QGridLayout();
    QLabel * color;
    unsigned int counter = 0;
    for(const auto &pickedColor : _pickedColors[colorName.toStdString()]) {
        color = new QLabel(this);
        color->setFixedSize(50,50);
        QString r = QString::number(pickedColor.red());
        QString g = QString::number(pickedColor.green());
        QString b = QString::number(pickedColor.blue());
        color->setStyleSheet("QLabel{background-color:rgb("+r+","+g+","+b+"); border-radius:5}");
        pickedColorLayout->addWidget(color, counter/4, counter%4);
        ++counter;
    }
    ui->pickedColorsGroup->setLayout(pickedColorLayout);
}

void MainWindow::currentColorChanged(QString text) {
    updatePickedColorsGUI();
}

void MainWindow::clearPickedColorsGUI() {
    QLayout * pickedColorLayout = ui->pickedColorsGroup->layout();

    if(pickedColorLayout == nullptr)
        return;

    QLayoutItem *child;
    while((child = pickedColorLayout->takeAt(0)) != nullptr)
    {
        pickedColorLayout->removeItem(child);
        delete child->widget();
        delete child;
    }

    delete pickedColorLayout;
}

void MainWindow::changeFinalColor() {
    //computing the avg color
    QString colorName = ui->currentColor->currentText();
    float r = 0, g = 0, b = 0;

    for(const auto &pickedColor : _pickedColors[colorName.toStdString()]) {
        r += pickedColor.red();
        g += pickedColor.green();
        b += pickedColor.blue();
    }
    r /= _pickedColors[colorName.toStdString()].size();
    g /= _pickedColors[colorName.toStdString()].size();
    b /= _pickedColors[colorName.toStdString()].size();


    //showing the dialog
    QColor initial(r,g,b);
    QColor finalColor = QColorDialog::getColor(initial, this, "Select color...", QColorDialog::DontUseNativeDialog);

    //updating the label
    QString r_str = QString::number(finalColor.red());
    QString g_str = QString::number(finalColor.green());
    QString b_str = QString::number(finalColor.blue());
    ui->finalColorLabel->setFixedSize(150,150);
    ui->finalColorLabel->setStyleSheet("QLabel{background-color:rgb("+r_str+","+g_str+","+b_str+"); border-radius:5}");


}

void MainWindow::on_actionLoad_Image_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this,
                                            tr("Open Image..."), "/home/imanol/data/wp3-color_restoration", tr("Image Files (*.png *.jpg)"));


    QImageReader reader(fileName);
    const QImage newImage = reader.read();
    if(newImage.isNull()) {
        std::cout << "Unable to load the image." << std::endl;
        return;
    }

    _image = newImage;
    _imageLabel->setPixmap(QPixmap::fromImage(_image));

    //loading mask if exist
    fileName.insert(fileName.lastIndexOf("."), "-mask");
    std::cout << fileName.toStdString() << std::endl;
    QImageReader maskReader(fileName);

    const QImage maskImage = maskReader.read();
    if(maskImage.isNull()) {
        std::cout << "No mask image found!" << std::endl;
    } else {
        _maskImage = maskImage;
    }

    std::cout << _maskImage.width() << " " << _maskImage.height() << std::endl;


    ui->scrollAreaImage->setBackgroundRole(QPalette::Dark);
    ui->scrollAreaImage->setWidget(_imageLabel);
    ui->scrollAreaImage->setVisible(true);
    ui->scrollAreaImage->setWidgetResizable(false);

    _scaleFactor = 1;
}

void MainWindow::on_actionExport_Image_to_PLY_triggered() {
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Export Image to PLY..."), "/home/imanol/data/wp3-color_restoration", tr("PLY Files (*.ply)"));

    PointCloud pc(_image, _maskImage);

    //add the picked colours
    for (auto const& pickCol : _pickedColors) {
        for(auto const& c : pickCol.second) {
            Point p(c.red()/255.f, c.green()/255.f, c.blue()/255.f, c.red()/255.f, c.green()/255.f, c.blue()/255.f);
            pc.addPoint(p);
        }
    }

    pc.exportToPLY(fileName.toStdString());
}

void MainWindow::on_actionZoom_in_triggered() {
    scaleImage(1.25);
}

void MainWindow::on_actionZoom_out_triggered() {
    scaleImage(0.8);
}

void MainWindow::scaleImage(double factor) {
    _scaleFactor *= factor;

    _imageLabel->resize(_scaleFactor * _imageLabel->pixmap()->size());

    adjustScrollBar(ui->scrollAreaImage->horizontalScrollBar(), factor);
    adjustScrollBar(ui->scrollAreaImage->verticalScrollBar(), factor);
}

void MainWindow::adjustScrollBar(QScrollBar *scrollBar, double factor) {
    scrollBar->setValue(int(factor * scrollBar->value()
                            + ((factor - 1) * scrollBar->pageStep()/2)));
}

void MainWindow::on_actionFit_in_view_triggered() {

    double sfX = (double)ui->scrollAreaImage->width()/(double)_imageLabel->pixmap()->width();
    double sfY = (double)ui->scrollAreaImage->height()/(double)_imageLabel->pixmap()->height();

    scaleImage(std::min(sfX,sfY)/_scaleFactor);



}

void MainWindow::on_actionColor_Transformation_triggered() {
    ColorTransformation2D test2D(5, 5, 1);
    std::vector<std::vector<float>> controlPoints;
    for(auto &pc : _pickedColors) {
        //TODO: Compute aa color representing the picked colors (now is the first)
        std::cout << pc.first << std::endl;
        color::rgb<float> rgb( { pc.second[0].red()/255.f, pc.second[0].green()/255.f, pc.second[0].blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        std::vector<float> color = {lab[0], lab[1], lab[2]};
        controlPoints.push_back(color);
    }

    controlPoints.clear();
//    controlPoints.push_back({0.1,0.1,0.1});
//    controlPoints.push_back({0.1,1.1,1.1});
    std::vector<std::vector<float>> newControlPoints;
//    newControlPoints.push_back({0.1, 0.4,0.1});
//    newControlPoints.push_back({0.1, 1.3, 1.3});
    test2D.initControlPoints(controlPoints, newControlPoints);
    std::vector<float> p({-0.75,-0.75, 0});
    std::vector<float> pt;
    test2D.sample(p,pt);
    return;

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Export Image to PLY..."), "/home/imanol/data/wp3-color_restoration", tr("PLY Files (*.ply)"));

    ColorTransformation colorTransf;

    //setting control Points
//    std::vector<std::vector<float>> controlPoints;
    for(auto &pc : _pickedColors) {
        //TODO: Compute aa color representing the picked colors (now is the first)
        std::cout << pc.first << std::endl;
        std::vector<float> color = {pc.second[0].red(), pc.second[0].green(), pc.second[0].blue()};
        controlPoints.push_back(color);
    }
    colorTransf.setControlPoints(controlPoints);

    colorTransf.computeBiharmonicCoordinates();

    //update control points
    //updateControlPoint

    colorTransf.updateColorTransformation();

    //Transform the point cloud.
    PointCloud pc(_image, _maskImage);
    pc.transform(colorTransf);

    pc.exportToPLY(fileName.toStdString());
}

void MainWindow::on_actionCompute_LAB_triggered() {
    for(auto &pc : _pickedColors) {
        std::cout << pc.first << "\t";//std::endl;
        std::vector<float> avg(3,0);
        int counter = 0;
        for(auto &c : pc.second) {
            std::vector<float> color = {c.red(), c.green(), c.blue()};
            avg[0] += color[0];
            avg[1] += color[1];
            avg[2] += color[2];
            counter++;
        }
        avg[0] /= counter;
        avg[1] /= counter;
        avg[2] /= counter;
        std::cout << avg[0] << "\t" << avg[1] << "\t" << avg[2] << "\t";
        color::rgb<float> rgb( { avg[0]/255.f, avg[1]/255.f, avg[2]/255.f});
        color::lab<float> lab;
        lab = rgb;
        std::cout << lab[0] << "\t" << lab[1] << "\t" << lab[2] << std::endl;
    }
}


