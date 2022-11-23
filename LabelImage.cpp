//
// Created by imanol on 15/11/22.
//

#include "LabelImage.h"

#include <iostream>
#include <QMouseEvent>

LabelImage::LabelImage(QWidget *parent) : QLabel(parent){
    _rubberBand = new QRubberBand(QRubberBand::Rectangle, this);
    _state = 0;
}

void LabelImage::setImage(const QImage &img) {
    setPixmap(QPixmap::fromImage(img));
}

void LabelImage::mousePressEvent(QMouseEvent *event) {
    if(_state != 0) {
        _start = event->pos();
        _rubberBand ->setGeometry(QRect(_start,QSize()).normalized());
        _rubberBand->show();
    }
    QLabel::mousePressEvent(event);
}

void LabelImage::mouseMoveEvent(QMouseEvent *event) {
    if(_state != 0)
        _rubberBand ->setGeometry(QRect(_start,event->pos()).normalized());
    QLabel::mouseMoveEvent(event);

}

void LabelImage::mouseReleaseEvent(QMouseEvent *event) {
    if(_state != 0)
        selectionCompleted(_start, event->pos());

    _rubberBand->hide();
    _state = 0;
    QLabel::mouseReleaseEvent(event);
}


