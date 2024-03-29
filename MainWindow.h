//
// Created by imanol on 23/5/22.
// inspired on https://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html
//

#ifndef COLOR_RESTORATION_MAINWINDOW_H
#define COLOR_RESTORATION_MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QScrollBar>
#include "PickedColor.h"
#include "ColorTransformation2D.h"
#include "ColorTransformation.h"
#include "LabelImage.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    void loadPickedColors();
    void updatePickedColorsGUI();
    void clearPickedColorsGUI();

public slots:
    void on_actionLoad_Image_triggered();
    void on_actionPrint_color_info_triggered();


    void on_actionExport_Image_to_PLY_triggered();
    void on_actionExport_ColorTransf_to_PLY_triggered();
    void on_actionExport_Palette_to_PLY_triggered();
    void on_actionExport_transf_per_zone_triggered();
    void on_actionExport_transf_per_L_triggered();
    void on_actionExport_weights_triggered();
    void on_actionExport_deformation_factors_triggered();

    void on_actionFit_in_view_triggered();
    void on_actionZoom_in_triggered();
    void on_actionZoom_out_triggered();

    void on_actionColor_Transformation_triggered();
    void on_actionColor_Transformation_3D_triggered();
    void on_actionLoad_Last_TColor_Transformation_3D_triggered();
    void on_actionCompute_LAB_triggered();
    void on_actionPrint_Transformation_Errors_triggered();
    void on_actionTest_Transformation_on_Images_triggered();

    void currentColorChanged(QString text);
    void updateOriginalColorsGUI();
    void updateFinalColorsGUI();
    void avgCurrentColor();
    void changeCurrentColor();
    void changeTargetColor();
    void originalRadioButtonClicked(bool b);
    void correctedRadioButtonClicked(bool b);

    void exportSelection(QPoint start, QPoint end);

private:
    void loadImage(std::string path);
    void computeSourceAndTargetColors(std::vector<std::vector<float>> &sourceColors, std::vector<std::vector<float>> &targetColors, bool considerLightness);
    void adjustScrollBar(QScrollBar *scrollBar, double factor);
    void scaleImage(double factor);
    void printInfoPickedColor(std::string colorName);
    void printInfoVectorsColor(const std::vector<std::string> &names, const std::vector<float> &a_values, const std::vector<float> &b_values,
                               const std::vector<std::string> &colors = std::vector<std::string>());
    std::string rgb2HexString(unsigned int r, unsigned int g, unsigned int b);

    std::map<std::string, std::vector<PickedColor>> _pickedColors;
    std::map<std::string, PickedColor> _avgColors;
    std::map<std::string, PickedColor> _finalColors;
    Ui::MainWindow *ui;
    QImage _image, _maskImage, _correctedImage;
    LabelImage * _imageLabel;
    double _scaleFactor;
    ColorTransformation2D * _ct2D;
    ColorTransformation * _ct3D;
    bool _need2ComputeBHC;
};


#endif //COLOR_RESTORATION_MAINWINDOW_H
