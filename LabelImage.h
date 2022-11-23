//
// Created by imanol on 15/11/22.
//

#ifndef COLOR_RESTORATION_LABELIMAGE_H
#define COLOR_RESTORATION_LABELIMAGE_H


#include <QLabel>
#include <QRubberBand>

class LabelImage : public QLabel {
Q_OBJECT

public:
    LabelImage(QWidget * parent = 0);
    void setImage(const QImage & img);
    void enableSelection() { _state = 1; }

signals:
    void selectionCompleted(QPoint start, QPoint end);

protected:
    void mousePressEvent(QMouseEvent * event);
    void mouseMoveEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);

private:
    QRubberBand * _rubberBand;
    QPoint _start;
    int _state;

};


#endif //COLOR_RESTORATION_IMAGE_H
