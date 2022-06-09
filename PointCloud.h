//
// Created by imanol on 25/5/22.
//

#ifndef COLOR_RESTORATION_POINTCLOUD_H
#define COLOR_RESTORATION_POINTCLOUD_H


#include <QImage>
#include "Point.h"
#include "ColorTransformation.h"

class PointCloud {
public:
    PointCloud() { ; }
    explicit PointCloud(const QImage img, const QImage mask);

    void addPoint(Point& p);
    void transform(ColorTransformation &ct);

    void exportToPLY(const std::string path);

private:
    std::vector<Point> _points;

};


#endif //COLOR_RESTORATION_POINTCLOUD_H
