//
// Created by imanol on 25/5/22.
//

#include "Point.h"

Point::Point(float x, float y, float z, float r, float g, float b) : _x(x), _y(y), _z(z), _r(r), _g(g), _b(b) {

}

void Point::updatePosition(const float x, const float y, const float z) {
    _x = x;
    _y = y;
    _z = z;
}

void Point::updateColor(const float r, const float g, const float b) {
    _r = r;
    _g = g;
    _b = b;
}
