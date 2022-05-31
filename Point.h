//
// Created by imanol on 25/5/22.
//

#ifndef COLOR_RESTORATION_POINT_H
#define COLOR_RESTORATION_POINT_H


class Point {
public:
    Point(float x, float y, float z, float r, float g, float b);
    void updatePosition(const float x, const float y, const float z);
    void updateColor(const float r, const float g, const float b);

    float x() const { return _x; }
    float y() const { return _y; }
    float z() const { return _z; }
    float r() const { return _r; }
    float g() const { return _g; }
    float b() const { return _b; }

private:
    float _x, _y, _z;
    float _r, _g, _b;
};


#endif //COLOR_RESTORATION_POINT_H
