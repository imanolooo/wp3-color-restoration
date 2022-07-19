//
// Created by imanol on 23/5/22.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include <iostream>
#include <QColorDialog>
#include <QFileDialog>
#include <QImageReader>
#include <QElapsedTimer>
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "PointCloud.h"
#include "ColorTransformation.h"
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

    _ct2D = NULL;

}

MainWindow::~MainWindow() {
    if(_imageLabel != nullptr)  delete _imageLabel;
    delete ui;
    if(_ct2D != NULL) delete _ct2D;
}

void MainWindow::loadPickedColors() {

    //White
    _pickedColors["white"].push_back(PickedColor("white", "AbsS", 4786,6094,5,230,226,221));
    _pickedColors["white"].push_back(PickedColor("white", "AbsN", 3420,4228,1,246,247,249));
    _pickedColors["white"].push_back(PickedColor("white", "AbsS", 1463,2737,2,221,217,211));
    _pickedColors["white"].push_back(PickedColor("white", "AbsC", 6686,3144,1,226,226,221));
    //_finalColors.emplace("white",PickedColor("white", "0422", 2232, 4456, 0, 239, 240, 244));
    _finalColors.emplace("white",PickedColor("white", "carlos", -1, -1, 0, 212, 195, 179));
    float r = 0, g = 0, b = 0;
    for(auto const &c : _pickedColors["white"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["white"].size();     g /= _pickedColors["white"].size();     b /= _pickedColors["white"].size();
    _avgColors.emplace("white",PickedColor("white", "compute", -1, -1, 0, r, g, b));


    //Garnet
    _pickedColors["maroon"].push_back(PickedColor("maroon", "AbsS", 3764,5481,2,115,81,76));
    _pickedColors["maroon"].push_back(PickedColor("maroon", "AbsN", 3650,3585,1,192,143,138));
    _pickedColors["maroon"].push_back(PickedColor("maroon", "AbsS", 1721,2691,1,135,76,75));
    _pickedColors["maroon"].push_back(PickedColor("maroon", "AbsC", 7324,2915,3,138,73,52));
    //_finalColors.emplace("garnet", PickedColor("garnet", "530", 3562, 1771, 0, 95, 1, 1));
    _finalColors.emplace("maroon", PickedColor("maroon", "carlos", -1, -1, 0, 99, 28, 17));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["maroon"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["maroon"].size();     g /= _pickedColors["maroon"].size();     b /= _pickedColors["maroon"].size();
    _avgColors.emplace("maroon",PickedColor("maroon", "compute", -1, -1, 0, r, g, b));

    //Light grey
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsS", 4655,5928,2,196,194,196));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsN", 3561,4014,1,203,202,208));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsS", 1223,2886,2,203,197,191));
    _pickedColors["light grey"].push_back(PickedColor("light grey", "AbsC", 7042,3786,1,224,224,223));
    //_finalColors.emplace("light grey", PickedColor("light grey", "0832", 4435, 2277, 0, 196, 197, 217));
    _finalColors.emplace("light grey", PickedColor("light grey", "0832", 4435, 2277, 0, 128, 123, 125));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["light grey"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["light grey"].size();     g /= _pickedColors["light grey"].size();     b /= _pickedColors["light grey"].size();
    _avgColors.emplace("light grey",PickedColor("light grey", "compute", -1, -1, 0, r, g, b));

    //Medium grey
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsS", 4682,591,2,113,115,126));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsN", 3600,3936,3,200,193,194));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsS", 1171,3019,3,171,171,175));
    _pickedColors["medium grey"].push_back(PickedColor("medium grey", "AbsC", 6777,3092,2,182,181,175));
    //_finalColors.emplace("dark grey", PickedColor("dark grey", "0832", 4385, 1778, 0, 82, 80, 94));
    _finalColors.emplace("medium grey", PickedColor("medium grey", "carlos", -1, -1, 0, 44, 43, 56));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["medium grey"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["medium grey"].size();     g /= _pickedColors["medium grey"].size();     b /= _pickedColors["medium grey"].size();
    _avgColors.emplace("medium grey",PickedColor("medium grey", "compute", -1, -1, 0, r, g, b));

    //Dark grey
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsS", 4767,5874,2,86,88,102));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsN", 3470,4059,3,159,156,162));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsS", 1686,2807,1,146,145,152));
    _pickedColors["dark grey"].push_back(PickedColor("dark grey", "AbsC", 6908,2994,3,80,86,93));
    //_finalColors.emplace("medium grey", PickedColor("medium grey", "0469", 2347, 607, 0, 191, 189, 194));
    _finalColors.emplace("dark grey", PickedColor("dark grey", "carlos", -1, -1, 0, 33, 32, 32));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["dark grey"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["dark grey"].size();     g /= _pickedColors["dark grey"].size();     b /= _pickedColors["dark grey"].size();
    _avgColors.emplace("dark grey",PickedColor("dark grey", "compute", -1, -1, 0, r, g, b));

    //Black
    _pickedColors["black"].push_back(PickedColor("black", "AbsS", 462,4625,3,71,70,75));
    _pickedColors["black"].push_back(PickedColor("black", "AbsN", 3222,3379,1,122,124,140));
    _pickedColors["black"].push_back(PickedColor("black", "AbsS", 1265,2801,1,93,92,102));
    _pickedColors["black"].push_back(PickedColor("black", "AbsC", 7333,3056,0,56,48,51));
    //_finalColors.emplace("black", PickedColor("black", "0422", 2163, 2288, 0, 64, 72, 91));
    _finalColors.emplace("black", PickedColor("black", "carlos", -1, -1, 0, 27, 21, 20));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["black"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["black"].size();     g /= _pickedColors["black"].size();     b /= _pickedColors["black"].size();
    _avgColors.emplace("black",PickedColor("black", "compute", -1, -1, 0, r, g, b));

    //Ocher
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsS", 3800,5256,2,204,155,95));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsN", 2943,3932,1,234,200,154));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsS", 1577,2645,3,173,132,91));
    _pickedColors["ocher"].push_back(PickedColor("ocher", "AbsC", 6752,3019,1,219,162,78));
    //_finalColors.emplace("ocher", PickedColor("ocher", "0488", 3084, 861, 0, 233, 168, 84));
    _finalColors.emplace("ocher", PickedColor("ocher", "carlos", -1, -1, 0, 204, 158, 60));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["ocher"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["ocher"].size();     g /= _pickedColors["ocher"].size();     b /= _pickedColors["ocher"].size();
    _avgColors.emplace("ocher",PickedColor("ocher", "compute", -1, -1, 0, r, g, b));

    //Light ocher
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsS", 3944,3887,3,234,222,203));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsN", 3629,3743,3,233,220,204));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsS", 1225,2853,3,224,191,158));
    _pickedColors["lighter ocher"].push_back(PickedColor("lighter ocher", "AbsC", 6702,2835,3,229,214,189));
    //_finalColors.emplace("lighter ocher", PickedColor("lighter ocher", "0488", 3340, 900, 0, 231, 209, 172));
    _finalColors.emplace("lighter ocher", PickedColor("lighter ocher", "carlos", -1, -1, 0, 240, 210, 164));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["lighter ocher"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["lighter ocher"].size();     g /= _pickedColors["lighter ocher"].size();     b /= _pickedColors["lighter ocher"].size();
    _avgColors.emplace("lighter ocher",PickedColor("lighter ocher", "compute", -1, -1, 0, r, g, b));

    //Pink
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsS", 3212,532,3,236,200,186));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsN", 3311,4184,3,215,176,171));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsS", 1609,2822,1,210,172,150));
    _pickedColors["pink"].push_back(PickedColor("pink", "AbsC", 6636,3034,1,217,175,139));
    //_finalColors.emplace("pink", PickedColor("pink", "0491", 2844, 2946, 0, 238, 202, 190));
    _finalColors.emplace("pink", PickedColor("pink", "carlos", -1, -1, 0, 219, 178, 170));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["pink"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["pink"].size();     g /= _pickedColors["pink"].size();     b /= _pickedColors["pink"].size();
    _avgColors.emplace("pink",PickedColor("pink", "compute", -1, -1, 0, r, g, b));

    //Orange
    _pickedColors["orange"].push_back(PickedColor("orange", "AbsS", 3156,3547,3,216,136,111));
    //_finalColors.emplace("orange", PickedColor("orange", "0832", 906, 3459, 0, 239, 166, 123));
    _finalColors.emplace("orange", PickedColor("orange", "carlos", -1, -1, 0, 255, 114, 50));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["orange"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["orange"].size();     g /= _pickedColors["orange"].size();     b /= _pickedColors["orange"].size();
    _avgColors.emplace("orange",PickedColor("orange", "compute", -1, -1, 0, r, g, b));

    //Green
    _pickedColors["green"].push_back(PickedColor("green", "AbsN", 3730,3904,2,137,145,140));
    _pickedColors["green"].push_back(PickedColor("green", "AbsS", 4280,3589,3,135,138,125));
    _pickedColors["green"].push_back(PickedColor("green", "AbsN", 3883,4113,1,140,145,145));
    _pickedColors["green"].push_back(PickedColor("green", "AbsS", 1553,2682,3,107,109,105));
    _pickedColors["green"].push_back(PickedColor("green", "AbsC", 6323,3535,1,127,127,94));
    //_finalColors.emplace("green", PickedColor("green", "0491", 1077, 720, 0, 106, 101, 97));
    _finalColors.emplace("green", PickedColor("green", "carlos", -1, -1, 0, 45, 52, 47));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["green"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["green"].size();     g /= _pickedColors["green"].size();     b /= _pickedColors["green"].size();
    _avgColors.emplace("green",PickedColor("green", "compute", -1, -1, 0, r, g, b));

    //Light green
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsS", 4534,5953,2,175,183,157));
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsN", 3618,3689,1,201,196,168));
    _pickedColors["light green"].push_back(PickedColor("light green", "AbsC", 6906,3884,1,174,178,148));
    //_finalColors.emplace("light green", PickedColor("light green", "0104", 2003, 2886, 0, 193, 202, 181));
    _finalColors.emplace("light green", PickedColor("light green", "carlos", -1, -1, 0, 105, 116, 87));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["light green"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["light green"].size();     g /= _pickedColors["light green"].size();     b /= _pickedColors["light green"].size();
    _avgColors.emplace("light green",PickedColor("light green", "compute", -1, -1, 0, r, g, b));

    //Red
    _pickedColors["red"].push_back(PickedColor("red", "AbsS", 3199,5214,2,189,103,86));
    _pickedColors["red"].push_back(PickedColor("red", "AbsN", 3239,4081,3,196,152,149));
    _pickedColors["red"].push_back(PickedColor("red", "AbsS", 1449,2984,1,178,109,95));
    _pickedColors["red"].push_back(PickedColor("red", "AbsC", 6855,6847,1,214,131,95));
    //_finalColors.emplace("red", PickedColor("red", "0466", 1086, 3161, 0, 167, 64, 45));
    _finalColors.emplace("red", PickedColor("red", "carlos", -1, -1, 0, 151, 51, 33));
    r = 0; g = 0; b = 0;
    for(auto const &c : _pickedColors["red"]) {
        r += c.red();        g += c.green();        b += c.blue();
    }
    r /= _pickedColors["red"].size();     g /= _pickedColors["red"].size();     b /= _pickedColors["red"].size();
    _avgColors.emplace("red",PickedColor("red", "compute", -1, -1, 0, r, g, b));

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
    updateOriginalColorsGUI();
    updateFinalColorsGUI();
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

void MainWindow::updateOriginalColorsGUI() {
    //updating the label
    QString colorName = ui->currentColor->currentText();
    QString r_str = QString::number(_avgColors.at(colorName.toStdString()).red());
    QString g_str = QString::number(_avgColors.at(colorName.toStdString()).green());
    QString b_str = QString::number(_avgColors.at(colorName.toStdString()).blue());
    ui->originalColorLabel->setFixedSize(150,150);
    ui->originalColorLabel->setStyleSheet("QLabel{background-color:rgb("+r_str+","+g_str+","+b_str+"); border-radius:5}");
}

void MainWindow::updateFinalColorsGUI() {
    //get the color
    QString colorName = ui->currentColor->currentText();
    float r = 0, g = 0, b = 0;
    r = _finalColors.at(colorName.toStdString()).red();
    g = _finalColors.at(colorName.toStdString()).green();
    b = _finalColors.at(colorName.toStdString()).blue();

    //updating the label
    QString r_str = QString::number(r);
    QString g_str = QString::number(g);
    QString b_str = QString::number(b);
    ui->finalColorLabel->setFixedSize(150,150);
    ui->finalColorLabel->setStyleSheet("QLabel{background-color:rgb("+r_str+","+g_str+","+b_str+"); border-radius:5}");

    //set label to the final color with the luminance of the original
    float ro = _avgColors.at(colorName.toStdString()).red();
    float go = _avgColors.at(colorName.toStdString()).green();
    float bo = _avgColors.at(colorName.toStdString()).blue();

    color::rgb<float> rgb({r/255.f, g/255.f, b/255.f});
    color::lab<float> lab;
    lab = rgb;

    color::rgb<float> rgbo({ro/255.f, go/255.f, bo/255.f});
    color::lab<float> labo;
    labo = rgbo;

    lab[0] = labo[0];//set the luminance of the origin.
    rgb = lab;

    r_str = QString::number(std::min(255.f, std::max(0.f, rgb[0]*255.f)));
    g_str = QString::number(std::min(255.f, std::max(0.f, rgb[1]*255.f)));
    b_str = QString::number(std::min(255.f, std::max(0.f, rgb[2]*255.f)));

    ui->finalColorWithCurrentLLabel->setFixedSize(150,150);
    ui->finalColorWithCurrentLLabel->setStyleSheet("QLabel{background-color:rgb("+r_str+","+g_str+","+b_str+"); border-radius:5}");
}

void MainWindow::changeFinalColor() {
//TODO: Implement buttons of final and original color
    /*
    //showing the dialog
    QColor initial(r,g,b);
    QColor finalColor = QColorDialog::getColor(initial, this, "Select color...", QColorDialog::DontUseNativeDialog);

    //updating the label
    QString r_str = QString::number(finalColor.red());
    QString g_str = QString::number(finalColor.green());
    QString b_str = QString::number(finalColor.blue());
    ui->finalColorLabel->setFixedSize(150,150);
    ui->finalColorLabel->setStyleSheet("QLabel{background-color:rgb("+r_str+","+g_str+","+b_str+"); border-radius:5}");
*/

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

void MainWindow::on_actionExport_2DColorTransf_to_PLY_triggered() {
    if(_ct2D == NULL) {
        std::cerr << "Color transformation not created yet!" << std::endl;
        return;
    }
    QString fileNameOri = QFileDialog::getSaveFileName(this,
                                                    tr("Export original 2D Color Transformation to PLY..."), "/home/imanol/data/wp3-color_restoration", tr("PLY Files (*.ply)"));
    QString fileNameTransf = QFileDialog::getSaveFileName(this,
                                                    tr("Export transformed 2D Color Transformation to PLY..."), "/home/imanol/data/wp3-color_restoration", tr("PLY Files (*.ply)"));

    _ct2D->export2PLY(fileNameOri.toStdString(), fileNameTransf.toStdString());
}

void MainWindow::on_actionPrint_color_info_triggered() {
    std::vector<std::string> names;
    std::vector<float> a_values;
    std::vector<float> b_values;

    std::cout << "Average colors..." << std::endl;
    for(auto const& pc : _pickedColors) {
        names.push_back(pc.first);
        float r = 0, g = 0, b = 0;
        for(auto const& c : pc.second) {
            r += c.red()/255.f;
            g += c.green()/255.f;
            b += c.blue()/255.f;
        }
        r /= pc.second.size();
        g /= pc.second.size();
        b /= pc.second.size();
        color::rgb<float> rgb( { r, g, b});
        color::lab<float> lab;
        lab = rgb;
        a_values.push_back(lab[1]);
        b_values.push_back(lab[2]);
    }

    for(auto i = 0; i < names.size(); ++i)
        std::cout << names[i] << ", ";
    std::cout << std::endl;
    for(auto i = 0; i < a_values.size(); ++i)
        std::cout << a_values[i] << ", ";
    std::cout << std::endl;
    for(auto i = 0; i < b_values.size(); ++i)
        std::cout << b_values[i] << ", ";
    std::cout << std::endl;

    names.clear();
    a_values.clear();
    b_values.clear();


    std::cout << "Final colors..." << std::endl;
    for(auto const& c : _finalColors) {
        names.push_back(c.first);
        color::rgb<float> rgb( { c.second.red()/255.f, c.second.green()/255.f, c.second.blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        a_values.push_back(lab[1]);
        b_values.push_back(lab[2]);
    }

    for(auto i = 0; i < names.size(); ++i)
        std::cout << names[i] << ", ";
    std::cout << std::endl;
    for(auto i = 0; i < a_values.size(); ++i)
        std::cout << a_values[i] << ", ";
    std::cout << std::endl;
    for(auto i = 0; i < b_values.size(); ++i)
        std::cout << b_values[i] << ", ";
    std::cout << std::endl;
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
    _ct2D = new ColorTransformation2D(255, 255, 1);
    std::vector<std::vector<float>> sourceColors;
    std::vector<std::vector<float>> targetColors;

    std::cout << "Computing the color control points..." << std::endl;
    for(auto const &pc : _pickedColors) {
        std::cout << pc.first << std::endl;
        float r = 0, g = 0, b = 0;
        for(auto const& c : pc.second) {
            r += c.red()/255.f;
            g += c.green()/255.f;
            b += c.blue()/255.f;
        }
        r /= pc.second.size();
        g /= pc.second.size();
        b /= pc.second.size();
        color::rgb<float> rgb( { r, g, b});
        color::lab<float> lab;
        lab = rgb;
        std::vector<float> color = {0/*lab[0]*/, lab[1], lab[2]};
        sourceColors.push_back(color);
    }

    for(auto const &c : _finalColors) {
        color::rgb<float> rgb({ c.second.red()/255.f, c.second.green()/255.f, c.second.blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        std::vector<float> color = {0/*lab[0]*/, lab[1], lab[2]};
        targetColors.push_back(color);
    }

    std::cout << "Initializing the color control points..." << std::endl;
    _ct2D->initControlPoints(sourceColors, targetColors);

    QElapsedTimer timer;
    timer.start();
    std::cout << "Transforming the image..." << std::endl;
    QImage target = _image;
    for(auto i = 0; i < target.width(); ++i) {
        for(auto j = 0; j < target.height(); ++j) {
            if(qRed(_maskImage.pixel(i,j)) > 128) {
                color::rgb<float> rgb({ qRed(target.pixel(i,j))/255.f, qGreen(target.pixel(i,j))/255.f, qBlue(target.pixel(i,j))/255.f});
                color::lab<float> lab;
                lab = rgb;
                std::vector<float> p({0/*lab[0]*/, lab[1], lab[2]});
                std::vector<float> pt;
                _ct2D->sample(p, pt);
                pt[0] = lab[0];//keeping the lightness of the source
                lab = color::lab<float>({pt[0], pt[1], pt[2]});
                rgb = lab;
                /*if(rgb[0]*255 < 0.f || rgb[0]*255 > 255.f || rgb[1]*255 < 0.f || rgb[1]*255 > 255.f || rgb[2]*255 < 0.f || rgb[2]*255 > 255.f)
                {
                    std::cout << "Original Color" << std::endl;
                    color::rgb<float> rgb2({ qRed(target.pixel(i,j))/255.f, qGreen(target.pixel(i,j))/255.f, qBlue(target.pixel(i,j))/255.f});
                    color::lab<float> lab2;
                    lab2 = rgb2;
                    std::cout << "RGB " << rgb2[0]*255 << " " << rgb2[1]*255 << " " << rgb2[2]*255 << std::endl;
                    std::cout << "LAB " << lab2[0] << " " << lab2[1] << " " << lab2[2] << std::endl;
                    std::cout << "Final Color" << std::endl;
                    std::cout << "RGB " << rgb[0]*255 << " " << rgb[1]*255 << " " << rgb[2]*255 << std::endl;
                    std::cout << "LAB " << lab[0] << " " << lab[1] << " " << lab[2] << std::endl;
                    std::cout << std::endl;
                }*/
                target.setPixelColor(i, j, QColor(std::min(255.f,std::max(0.f,rgb[0]*255)),
                                                  std::min(255.f,std::max(0.f,rgb[1]*255)),
                                                  std::min(255.f,std::max(0.f,rgb[2]*255))));
            }
        }
    }
    std::cout << "Done in " << timer.elapsed() << std::endl;

    std::cout << "Saving the images..." << std::endl;
    _image.save("sourceImage.png");
    target.save("targetImage.png");

    /* TESTING THE BIHARMONICS
     * controlPoints.clear();
    *//*  ncp(0,0) = -1;  ncp(0,1) = -1;  ncp(0,2) = 0;
    ncp(1,0) =  1;  ncp(1,1) = -1;  ncp(1,2) = 0;
    ncp(2,0) = -1;  ncp(2,1) =  1;  ncp(2,2) = 0;
    ncp(3,0) =  1;  ncp(3,1) =  1;  ncp(3,2) = 0;*//*
    controlPoints.push_back({0,-1.1,-1.1});
    controlPoints.push_back({0,1,-1.1});
    controlPoints.push_back({0,-0.9,1});
    controlPoints.push_back({0,0.9,0.9});
    std::vector<std::vector<float>> newControlPoints;
    newControlPoints.push_back({0,-1.4,-1.4});
    newControlPoints.push_back({0,1,-1.4});
    newControlPoints.push_back({0,-0.9,1});
    newControlPoints.push_back({0,0.9,0.9});
    test2D.initControlPoints(controlPoints, newControlPoints);
    std::vector<float> p({0,-0.75,-0.75});
    std::vector<float> pt;
    test2D.sample(p,pt);
    p = {0, 1.2, -1.2};
    test2D.sample(p,pt);
    return;*/

    /*EXPORT TO PLY
     *
     * QString fileName = QFileDialog::getSaveFileName(this,
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

    pc.exportToPLY(fileName.toStdString());*/
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

void MainWindow::on_actionPrint_Transformation_Errors_triggered() {
    if(_ct2D == NULL) {
        std::cerr << "Color transformation unavailable..." << std::endl;
        return;
    }

    std::cout << "Computing error per color..." << std::endl;
    for(auto const & c : _pickedColors) {
        std::cout << "\t" << c.first << std::endl;
        float ro = _avgColors.at(c.first).red();
        float go = _avgColors.at(c.first).green();
        float bo = _avgColors.at(c.first).blue();
        color::rgb<float> rgbo({ro/255.f, go/255.f, bo/255.f});
        color::lab<float> labo;
        labo = rgbo;

        float r = _finalColors.at(c.first).red();
        float g = _finalColors.at(c.first).green();
        float b = _finalColors.at(c.first).blue();
        color::rgb<float> rgb({r/255.f, g/255.f, b/255.f});
        color::lab<float> lab;
        lab = rgb;

        std::vector<float> p({0/*lab[0]*/, labo[1], labo[2]});
        std::vector<float> pt;
        _ct2D->sample(p, pt);

        float A = pt[1] - lab[1];
        float B = pt[2] - lab[2];
        std::cout << "\t\tA: " << std::abs(A) << std::endl;
        std::cout << "\t\tB: " << std::abs(B) << std::endl;
        std::cout << "\t\tAB: " << std::sqrt(A*A + B*B) << std::endl;
    }




}

