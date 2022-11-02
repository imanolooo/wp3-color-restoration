//
// Created by imanol on 23/5/22.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include <iostream>
#include <QColorDialog>
#include <QFileDialog>
#include <QDirIterator>
#include <QImageReader>
#include <QElapsedTimer>
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "PointCloud.h"
#include "ColorTransformation.h"

#include <color.hpp>//https://github.com/dmilos/color
#include <json.hpp>//https://github.com/nlohmann/json
#include "happly.h"

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
    _ct3D = NULL;

    _need2ComputeBHC = true;

    loadImage("/home/imanol/Dades/wp3-color_restoration/textures/srgb/LOW-Pedret_XII_color_absS.png");
}

MainWindow::~MainWindow() {
    if(_imageLabel != nullptr)  delete _imageLabel;
    delete ui;
    if(_ct2D != NULL) delete _ct2D;
}

void MainWindow::loadPickedColors() {
    std::string srcPaletteFile = "/home/imanol/Dades/wp3-color_restoration/json/PaletteSrcColors-RoserBego270922.json";
    std::string dstPaletteFile = "/home/imanol/Dades/wp3-color_restoration/json/PaletteDstColors-RoserBego270922.json";

    std::ifstream fSrc(srcPaletteFile);
    std::ifstream fDst(dstPaletteFile);
    nlohmann::json dataSrc = nlohmann::json::parse(fSrc);
    nlohmann::json dataDst = nlohmann::json::parse(fDst);

    std::cout << "Reading src colors..." << std::endl;
    for(auto const &element : dataSrc)
    {
        std::cout << "\t" << element << std::endl;
        _pickedColors[element["color"]].push_back(PickedColor(element["color"], element["file"],
                                                              element["pixel"]["x"], element["pixel"]["y"], element["kernel"],
                                                              element["rgb"]["r"], element["rgb"]["g"], element["rgb"]["b"]));
    }

    std::cout << "Computing avg colors..." << std::endl;
    for(auto const &pc : _pickedColors) {
        std::cout << "\t" << pc.first << std::endl;
        float r = 0, g = 0, b = 0;
        for(auto const& c : pc.second) {
            r += c.red(); g += c.green(); b += c.blue();
        }
        r /= pc.second.size();  g /= pc.second.size(); b /= pc.second.size();
        _avgColors.emplace(pc.first,PickedColor(pc.first, "compute", -1, -1, 0, r, g, b));
    }

    std::cout << "Reading dst colors..." << std::endl;
    for(auto const &element : dataDst)
    {
        std::cout << "\t" << element << std::endl;
        _finalColors.emplace(element["color"], PickedColor(element["color"], element["file"],
                                                              element["pixel"]["x"], element["pixel"]["y"], element["kernel"],
                                                              element["rgb"]["r"], element["rgb"]["g"], element["rgb"]["b"]));
    }

    std::cout << "Adding color names to GUI..." << std::endl;
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

void MainWindow::avgCurrentColor() {
    _need2ComputeBHC = true;
    QString colorName = ui->currentColor->currentText();
    float r = 0, g = 0, b = 0;
    for(auto const &c : _pickedColors[colorName.toStdString()]) {
        r += c.red(); g += c.green(); b += c.blue();
    }
    r /= _pickedColors[colorName.toStdString()].size(); g /= _pickedColors[colorName.toStdString()].size(); b /= _pickedColors[colorName.toStdString()].size();

    _avgColors.at(colorName.toStdString()).setRed(r);
    _avgColors.at(colorName.toStdString()).setGreen(g);
    _avgColors.at(colorName.toStdString()).setBlue(b);

    updateOriginalColorsGUI();
}

void MainWindow::changeCurrentColor() {
    _need2ComputeBHC = true;
    QString colorName = ui->currentColor->currentText();
    int r = _avgColors.at(colorName.toStdString()).red();
    int g = _avgColors.at(colorName.toStdString()).green();
    int b = _avgColors.at(colorName.toStdString()).blue();

    QColor initial(r,g,b);
    QColor finalColor = QColorDialog::getColor(initial, this, "Select color...", QColorDialog::DontUseNativeDialog);

    _avgColors.at(colorName.toStdString()).setRed(finalColor.red());
    _avgColors.at(colorName.toStdString()).setGreen(finalColor.green());
    _avgColors.at(colorName.toStdString()).setBlue(finalColor.blue());

    updateOriginalColorsGUI();
}
void MainWindow::changeTargetColor() {
    QString colorName = ui->currentColor->currentText();
    int r = _finalColors.at(colorName.toStdString()).red();
    int g = _finalColors.at(colorName.toStdString()).green();
    int b = _finalColors.at(colorName.toStdString()).blue();

    QColor initial(r,g,b);
    QColor finalColor = QColorDialog::getColor(initial, this, "Select color...", QColorDialog::DontUseNativeDialog);
    if(!finalColor.isValid())   finalColor = initial;
    /* Alternative to the previous line to avoid grey layer on main window. We need to make the dialog as member of mainwindow and create a slot to handle its result.
    QColorDialog dialog(initial, this);
    dialog.setOption(QColorDialog::DontUseNativeDialog);
    dialog.open();
     */


    _finalColors.at(colorName.toStdString()).setRed(finalColor.red());
    _finalColors.at(colorName.toStdString()).setGreen(finalColor.green());
    _finalColors.at(colorName.toStdString()).setBlue(finalColor.blue());

    updateFinalColorsGUI();
}

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

void MainWindow::on_actionLoad_Image_triggered() {
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Image..."), "/home/imanol/data/wp3-color_restoration",
                                                    tr("Image Files (*.png *.jpg)"));
    loadImage(fileName.toStdString());
}

void MainWindow::loadImage(std::string path){
    QString fileName(path.c_str());
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
        _maskImage = QImage();
    } else {
        _maskImage = maskImage;
    }



    ui->scrollAreaImage->setBackgroundRole(QPalette::Dark);
    ui->scrollAreaImage->setWidget(_imageLabel);
    ui->scrollAreaImage->setVisible(true);
    ui->scrollAreaImage->setWidgetResizable(false);

    _scaleFactor = 1;
    scaleImage(_scaleFactor);

    if(!ui->originalRadioButton->isChecked())
        ui->originalRadioButton->setChecked(true);

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

void MainWindow::on_actionExport_ColorTransf_to_PLY_triggered() {
    if(_ct2D == NULL && _ct3D == NULL) {
        std::cerr << "Color transformation not created yet!" << std::endl;
        return;
    }
    QString fileNameOri = QFileDialog::getSaveFileName(this,
                                                    tr("Export original Color Transformation to PLY..."), "/home/imanol/Dades/wp3-color_restoration", tr("PLY Files (*.ply)"));
    QString fileNameTransf = QFileDialog::getSaveFileName(this,
                                                    tr("Export transformed Color Transformation to PLY..."), "/home/imanol/Dades/wp3-color_restoration", tr("PLY Files (*.ply)"));

    std::cout << "Exporting transformations to PLY..." << std::flush;
    if(_ct2D != NULL)   _ct2D->export2PLY(fileNameOri.toStdString(), fileNameTransf.toStdString());
    if(_ct3D != NULL)   _ct3D->export2PLY(fileNameOri.toStdString(), fileNameTransf.toStdString());
    std::cout << "Done!" << std::endl;
}

void MainWindow::on_actionPrint_color_info_triggered() {
    std::vector<std::string> names;
    std::vector<float> a_values;
    std::vector<float> b_values;
    std::vector<std::string> colors;

    std::cout << "Scale of Colors..." << std::endl;
    for(auto const& c : _finalColors) {
        names.push_back(c.first);
        colors.push_back(rgb2HexString(c.second.red(), c.second.green(), c.second.blue()));
    }
    std::cout << "\tdomain=[";
    auto size = names.size();
    for(auto i = 0; i < size; ++i) {
        std::cout << "\"" << names[i] << "\"";
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]," << std::endl;

    std::cout << "\trange=[";
    for(auto i = 0; i < size; ++i) {
        std::cout << "\"" << colors[i] << "\"";
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "Picked Colors..." << std::endl;
    names.clear(); a_values.clear(); b_values.clear(); colors.clear();
    for(auto const& pc : _pickedColors) {
        std::string name = pc.first;
        for(auto i = 0; i < pc.second.size(); ++i) {
            names.push_back(name + std::to_string(i));
            PickedColor const& c = pc.second[i];
            color::rgb<float> rgb({ c.red()/255.f, c.green()/255.f, c.blue()/255.f});
            color::lab<float> lab;
            lab = rgb;
            a_values.push_back(lab[1]);
            b_values.push_back(lab[2]);
            colors.push_back(name);
        }
    }
    printInfoVectorsColor(names, a_values, b_values, colors);

    std::cout << "Average colors..." << std::endl;
    names.clear(); a_values.clear(); b_values.clear(); colors.clear();
    for(auto const& c : _avgColors) {
        names.push_back(c.first);
        auto r = c.second.red();
        auto g = c.second.green();
        auto b = c.second.blue();
        color::rgb<float> rgb( { r/255.f, g/255.f, b/255.f});
        color::lab<float> lab;
        lab = rgb;
        a_values.push_back(lab[1]);
        b_values.push_back(lab[2]);
        //colors.push_back(rgb2HexString(r, g, b));
    }
    printInfoVectorsColor(names, a_values, b_values, colors);

    std::cout << "Final colors..." << std::endl;
    names.clear(); a_values.clear(); b_values.clear(); colors.clear();
    for(auto const& c : _finalColors) {
        names.push_back(c.first);
        auto r = c.second.red();
        auto g = c.second.green();
        auto b = c.second.blue();
        color::rgb<float> rgb( { r/255.f, g/255.f, b/255.f});
        color::lab<float> lab;
        lab = rgb;
        a_values.push_back(lab[1]);
        b_values.push_back(lab[2]);
        PickedColor const& avgC = _avgColors.at(c.first);
        //colors.push_back(rgb2HexString(avgC.red(), avgC.green(), avgC.blue()));
    }
    printInfoVectorsColor(names, a_values, b_values, colors);

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
    QElapsedTimer testTimer;
    testTimer.start();
    ui->correctedRadioButton->setEnabled(true);

    _ct2D = new ColorTransformation2D(255, 255, 1);
    std::vector<std::vector<float>> sourceColors;
    std::vector<std::vector<float>> targetColors;

    computeSourceAndTargetColors(sourceColors, targetColors, false);

    std::cout << "Initializing the color control points..." << std::endl;
    _ct2D->initControlPoints(sourceColors, targetColors);

    QElapsedTimer timer;
    timer.start();
    std::cout << "Transforming the image..." << std::endl;
    _correctedImage = _image;
    for(auto i = 0; i < _correctedImage.width(); ++i) {
        for(auto j = 0; j < _correctedImage.height(); ++j) {
            if(_maskImage.width() == 0 || qRed(_maskImage.pixel(i,j)) > 128) {
                color::rgb<float> rgb({ qRed(_correctedImage.pixel(i,j))/255.f, qGreen(_correctedImage.pixel(i,j))/255.f, qBlue(_correctedImage.pixel(i,j))/255.f});
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
                _correctedImage.setPixelColor(i, j, QColor(std::min(255.f,std::max(0.f,rgb[0]*255)),
                                                  std::min(255.f,std::max(0.f,rgb[1]*255)),
                                                  std::min(255.f,std::max(0.f,rgb[2]*255))));
            }
        }
    }
    std::cout << "Done in " << timer.elapsed() << std::endl;

    std::cout << "Saving the images..." << std::endl;
    _image.save("sourceImage.png");
    _correctedImage.save("targetImage.png");

    if(!ui->correctedRadioButton->isChecked())
        ui->correctedRadioButton->setChecked(true);
    else
        correctedRadioButtonClicked(true);
    std::cout << "Transformation finished!" << std::endl;

    printInfoPickedColor("yellow ocher");


    std::cout << "Study of the first ocher chrown of a virgin " << std::endl;
    color::rgb<float> rgb({ 175/255.f, 130/255.f, 88/255.f});
    std::cout << "RGB ini " << rgb[0] << " " << rgb[1] << " " << rgb[2] << std::endl;
    color::lab<float> lab;
    lab = rgb;
    std::cout << "lab ini " << lab[0] << " " << lab[1] << " " << lab[2] << std::endl;
    std::vector<float> p({0/*lab[0]*/, lab[1], lab[2]});
    std::vector<float> pt;
    _ct2D->sample(p, pt);
    pt[0] = lab[0];//keeping the lightness of the source
    lab = color::lab<float>({pt[0], pt[1], pt[2]});
    std::cout << "lab final " << lab[0] << " " << lab[1] << " " << lab[2] << std::endl;
    rgb = lab;
    std::cout << "RGB final " << rgb[0] << " " << rgb[1] << " " << rgb[2] << std::endl;

    std::cout << "All work done in " << testTimer.elapsed() << " ms" << std::endl;
}

void MainWindow::on_actionColor_Transformation_3D_triggered() {
    ui->correctedRadioButton->setEnabled(true);

    std::vector<std::vector<float>> sourceColors;
    std::vector<std::vector<float>> targetColors;
    computeSourceAndTargetColors(sourceColors, targetColors, true);

    if(_need2ComputeBHC) {
        std::cout << "Building the color transformation structures..." << std::endl;
        std::vector<float> dim = {100, 255, 255};
        std::vector<float> orig = {0, -128, -128};
        std::vector<int> res = {30, 30, 30};

        _ct3D = new ColorTransformation(dim, orig, res);

        std::cout << "Initializing the color control points..." << std::endl;
        _ct3D->setControlPoints(sourceColors);//, targetColors);

        _ct3D->computeBiharmonicCoordinates();
    }
    _need2ComputeBHC = false;

    std::cout << "Updating control points..." << std::endl;
    for(auto i = 0; i < targetColors.size(); ++i)
        _ct3D->updateControlPoint(i, targetColors[i]);
    _ct3D->updateColorTransformation();

    QElapsedTimer timer;
    timer.start();
    //_ct3D->print();
    std::cout << "Transforming the image..." << std::endl;
    _correctedImage = _image;
    for(auto i = 0; i < _correctedImage.width(); ++i) {
        for(auto j = 0; j < _correctedImage.height(); ++j) {
            if(_maskImage.width() == 0 || qRed(_maskImage.pixel(i,j)) > 128) {
                //std::cout << i << ", " << j << std::endl;
                color::rgb<float> rgb({ qRed(_correctedImage.pixel(i,j))/255.f, qGreen(_correctedImage.pixel(i,j))/255.f, qBlue(_correctedImage.pixel(i,j))/255.f});
                color::lab<float> lab;
                lab = rgb;
                std::vector<float> p({lab[0], lab[1], lab[2]});
                std::vector<float> pt({0, 0, 0});
                //std::cout << "Presampling" << std::endl;
                _ct3D->sample(p, pt);
                //std::cout << "Transformation: " << p[0] << ", " << p[1] << ", " << p[2] << " => " << pt[0] << ", " << pt[1] << ", " << pt[2] << std::endl;
                pt[0] = lab[0];//keeping the lightness of the source
                lab = color::lab<float>({pt[0], pt[1], pt[2]});
                rgb = lab;
                _correctedImage.setPixelColor(i, j, QColor(std::min(255.f,std::max(0.f,rgb[0]*255)),
                                                           std::min(255.f,std::max(0.f,rgb[1]*255)),
                                                           std::min(255.f,std::max(0.f,rgb[2]*255))));
            }
        }
    }
    std::cout << "Done in " << timer.elapsed() << std::endl;

    std::cout << "Saving the images..." << std::endl;
    _image.save("sourceImage.png");
    _correctedImage.save("targetImage.png");

    if(!ui->correctedRadioButton->isChecked())
        ui->correctedRadioButton->setChecked(true);
    else
        correctedRadioButtonClicked(true);
    std::cout << "Transformation finished!" << std::endl;


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

void MainWindow::originalRadioButtonClicked(bool active) {
    if(!active) return;

    _imageLabel->setPixmap(QPixmap::fromImage(_image));
}
void MainWindow::correctedRadioButtonClicked(bool active) {
    if(!active) return;

    _imageLabel->setPixmap(QPixmap::fromImage(_correctedImage));
}

void MainWindow::printInfoPickedColor(std::string colorName) {
    std::cout << "Testing " + colorName + " dispersion..." << std::endl;
    if(_pickedColors.find(colorName) == _pickedColors.end()) {
        std::cout << "Color not found." << std::endl;
        return;
    }
    std::vector<std::string> vNames;
    std::vector<float> vASrc, vADst;
    std::vector<float> vBSrc, vBDst;
    std::vector<PickedColor> & vPC = _pickedColors[colorName];
    for(auto i = 0; i < vPC.size(); ++i) {
        vNames.push_back(colorName+std::to_string(i));
        color::rgb<float> rgb({ vPC[i].red()/255.f, vPC[i].green()/255.f, vPC[i].blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        vASrc.push_back(lab[1]);
        vBSrc.push_back(lab[2]);
        std::vector<float> p({0, lab[1], lab[2]});
        std::vector<float> pt;
        _ct2D->sample(p, pt);
        lab = color::lab<float>({pt[0], pt[1], pt[2]});
        vADst.push_back(lab[1]);
        vBDst.push_back(lab[2]);
    }

    vNames.push_back(colorName+" AVG");
    PickedColor & pc = _avgColors.at(colorName);
    color::rgb<float> rgb({ pc.red()/255.f, pc.green()/255.f, pc.blue()/255.f});
    color::lab<float> lab;
    lab = rgb;
    vASrc.push_back(lab[1]);
    vBSrc.push_back(lab[2]);
    std::vector<float> p({0, lab[1], lab[2]});
    std::vector<float> pt;
    _ct2D->sample(p, pt);
    lab = color::lab<float>({pt[0], pt[1], pt[2]});
    vADst.push_back(lab[1]);
    vBDst.push_back(lab[2]);

    std::cout << "Init Colors" << std::endl;
    printInfoVectorsColor(vNames, vASrc, vBSrc);

    std::cout << "Final Colors" << std::endl;
    printInfoVectorsColor(vNames, vADst, vBDst);
}

std::string MainWindow::rgb2HexString(unsigned int r, unsigned int g, unsigned int b) {
    char hexColor[8];
    std::snprintf(hexColor, sizeof hexColor, "#%02x%02x%02x", r, g, b);

    return std::string(hexColor);
}

void MainWindow::printInfoVectorsColor(const std::vector<std::string> &names, const std::vector<float> &a_values, const std::vector<float> &b_values,
                                       const std::vector<std::string> &colors) {
    auto size = names.size();

    std::cout << "\t\'Name\' : [";
    for(auto i = 0; i < size; ++i) {
        std::cout << "\"" << names[i] << "\"";
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]," << std::endl;

    std::cout << "\t\'a\' : [";
    for(auto i = 0; i < size; ++i) {
        std::cout << a_values[i];
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]," << std::endl;

    std::cout << "\t\'b\' : [";
    for(auto i = 0; i < size; ++i) {
        std::cout << b_values[i];
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]," << std::endl;

    if(colors.size() != size)   return;

    std::cout << "\t\'Color\' : [";
    for(auto i = 0; i < size; ++i) {
        std::cout << "\"" << colors[i] << "\"";
        if(size - 1 != i)   std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void MainWindow::computeSourceAndTargetColors(std::vector<std::vector<float>> &sourceColors, std::vector<std::vector<float>> &targetColors, bool considerLightness) {
    std::cout << "Computing the color control points..." << std::endl;
    for(auto const &c : _avgColors) {
        std::cout << c.first << std::endl;
        color::rgb<float> rgb( { c.second.red()/255.f, c.second.green()/255.f, c.second.blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        std::vector<float> color = {lab[0], lab[1], lab[2]};
        if(!considerLightness) color[0] = 0;
        sourceColors.push_back(color);
    }

    for(auto const &c : _finalColors) {
        color::rgb<float> rgb({ c.second.red()/255.f, c.second.green()/255.f, c.second.blue()/255.f});
        color::lab<float> lab;
        lab = rgb;
        std::vector<float> color = {lab[0], lab[1], lab[2]};
        if(!considerLightness) color[0] = 0;
        targetColors.push_back(color);
    }
}

void MainWindow::on_actionExport_Palette_to_PLY_triggered() {

    QString fileNameSamples = QFileDialog::getSaveFileName(this,
                                                    tr("Samples..."), "/home/imanol/data/wp3-color_restoration/",
                                                    tr("PointCloud Files (*.ply)"));
    QString fileNameAVG = QFileDialog::getSaveFileName(this,
                                                           tr("Palette..."), "/home/imanol/data/wp3-color_restoration/",
                                                           tr("PointCloud Files (*.ply)"));

    QString fileNameFinal = QFileDialog::getSaveFileName(this,
                                                       tr("Final..."), "/home/imanol/data/wp3-color_restoration/",
                                                       tr("PointCloud Files (*.ply)"));



    std::vector<std::vector<float>> sourceColors;
    std::vector<std::vector<float>> targetColors;

    computeSourceAndTargetColors(sourceColors, targetColors, true);

    std::vector<std::array<double,3>> positions, colors;
    positions.reserve(sourceColors.size());
    colors.reserve(sourceColors.size());
    for(const auto &v : sourceColors)
        positions.push_back({v[0], v[1], v[2]});

    colors.push_back({0,0,0});
    colors.push_back({68./255.,68./255.,68./255.});
    colors.push_back({144./255.,20./255.,20/255.});
    colors.push_back({0,108./255.,0});
    colors.push_back({65./255.,1.,86./255.});
    colors.push_back({174./255.,174./255.,174./255.});
    colors.push_back({116./255.,116./255.,116./255.});
    colors.push_back({1.,0,1.});
    colors.push_back({1.,0,0});
    colors.push_back({223./255.,223./255.,223./255.});
    colors.push_back({1,230./255.,163./255.});
    colors.push_back({1.,253./255.,0});

    //range=["#000000", "#444444", "#901414", "#006c00", "#41ff56", "#aeaeae", "#747474", "#ff00ff", "#ff0000", "#dfdfdf", "#ffe6a3", "#fffd00"]

    //AVG
    happly::PLYData plyOutAVG;
    plyOutAVG.addVertexPositions(positions);
    plyOutAVG.addVertexColors(colors);
    //plyOut.addFaceIndices(_faces);
    plyOutAVG.write(fileNameAVG.toStdString(), happly::DataFormat::ASCII);

    //Final
    positions.clear();
    for(const auto &v : targetColors)
        positions.push_back({v[0], v[1], v[2]});

    happly::PLYData plyOutFinal;
    plyOutFinal.addVertexPositions(positions);
    plyOutFinal.addVertexColors(colors);
    //plyOut.addFaceIndices(_faces);
    plyOutFinal.write(fileNameFinal.toStdString(), happly::DataFormat::ASCII);

    //Samples
    std::vector<std::array<double,3>> colorsSamples;
    positions.clear();
    int i = 0;
    for(const auto &pc : _pickedColors) {
        for(const auto &c : pc.second) {
            color::rgb<double> rgb( { c.red()/255.f, c.green()/255.f, c.blue()/255.f});
            color::lab<double> lab;
            lab = rgb;
            positions.push_back({lab[0], lab[1], lab[2]});
            colorsSamples.push_back(colors[i]);
        }
        i++;
    }

    happly::PLYData plyOutSamples;
    plyOutSamples.addVertexPositions(positions);
    plyOutSamples.addVertexColors(colorsSamples);
    //plyOut.addFaceIndices(_faces);
    plyOutSamples.write(fileNameSamples.toStdString(), happly::DataFormat::ASCII);
}

void MainWindow::on_actionTest_Transformation_on_Images_triggered() {
    std::cout << "Testing the transformation in a set of images... " << std::endl;
    std::string path = "/home/imanol/Dades/wp3-color_restoration/Images4Testing/";
    QString pathDir = QFileDialog::getExistingDirectory(this, "Select direcrtory", path.c_str());
    QDirIterator dirIterator(pathDir, {"*.jpg", "*.jpeg", "*.png"}, QDir::Files);
    while(dirIterator.hasNext()) {
        std::string imagePath = dirIterator.next().toStdString();
        if(imagePath.find("corrected") == std::string::npos) {
            std::cout << "Handling " << imagePath << std::endl;
            loadImage(imagePath);
            on_actionColor_Transformation_3D_triggered();
            std::string correctedPath = imagePath.substr(0, imagePath.find_last_of(".")) + "-corrected.png";
            _correctedImage.save(correctedPath.c_str());
        }
    }
}
