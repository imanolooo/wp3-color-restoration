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
    void on_actionExport_Image_to_PLY_triggered();

    void on_actionFit_in_view_triggered();
    void on_actionZoom_in_triggered();
    void on_actionZoom_out_triggered();

    void on_actionColor_Transformation_triggered();

    void currentColorChanged(QString text);
    void changeFinalColor();

private:
    void adjustScrollBar(QScrollBar *scrollBar, double factor);
    void scaleImage(double factor);

    std::map<std::string, std::vector<PickedColor>> _pickedColors;
    Ui::MainWindow *ui;
    QImage _image;
    QLabel * _imageLabel;
    double _scaleFactor;
};


#endif //COLOR_RESTORATION_MAINWINDOW_H
