//
// Created by imanol on 25/5/22.
//

#include "PointCloud.h"
#include "happly.h"
#include <color.hpp>

PointCloud::PointCloud(const QImage img, const QImage mask) {
    _points.reserve(img.width()*img.height());
    std::vector<std::vector<std::vector<bool>>> rgb(256, std::vector<std::vector<bool>>(256, std::vector<bool>(256,false)));
    std::vector<std::vector<int>> lab_count(std::vector<std::vector<int>>(256, std::vector<int>(256,0)));

    int width = img.width();
    int height = img.height();
    for(auto i = 0; i < width; ++i) {
        for (auto j = 0; j < height; ++j) {
            //TODO: use a mask instead of alpha
            if((mask.width() != 0 && mask.height() != 0 && qRed(mask.pixel(i,j)) != 255) || (mask.width() == 0 && mask.height() == 0 && qAlpha(img.pixel(i,j)) != 255)) {
                continue;
            }

            int r = qRed(img.pixel(i,j));
            int g = qGreen(img.pixel(i,j));
            int b = qBlue(img.pixel(i,j));

            rgb[r][g][b] = true;
            //rgb
           // _points.push_back(Point(r/255.f, g/255.f, b/255.f, r/255.f, g/255.f, b/255.f));

            color::rgb<float> c_rgb( { r/255.f, g/255.f, b/255.f});
            color::lab<float> lab;
            lab = c_rgb;
            lab_count[(int)lab[1]+127][(int)lab[2]+127] = lab_count[(int)lab[1]+127][(int)lab[2]+127] + 1;
        }
    }

    for(auto i = 0; i < 256; ++i) {
        for(auto j = 0; j < 256; ++j) {
            std::cout << i-127 << ", ";
        }
    }
    std::cout << std::endl << std::endl;

    for(auto i = 0; i < 256; ++i) {
        for(auto j = 0; j < 256; ++j) {
            std::cout << j-127 << ", ";
        }
    }
    std::cout << std::endl << std::endl;

    for(auto i = 0; i < 256; ++i) {
        for(auto j = 0; j < 256; ++j) {
            std::cout << lab_count[i][j] << ", ";
        }
    }
    std::cout << std::endl << std::endl;

    exit(1);


    //alternative create _points from rgb, to have only one point per color.
    for(auto r = 0; r < 256; ++r) {
        for(auto g = 0; g < 256; ++g) {
            for(auto b = 0; b < 256; ++b) {
                if(rgb[r][g][b])
                    _points.push_back(Point(r/255.f, g/255.f, b/255.f, r/255.f, g/255.f, b/255.f));
            }
        }
    }
}

void PointCloud::exportToPLY(const std::string path) {
    std::vector<std::array<double,3>> positions;
    std::vector<std::array<double,3>> colors;

    positions.reserve(_points.size());
    colors.reserve(_points.size());

    for(auto p : _points) {
        positions.push_back({p.x(), p.y(), p.z()});
        colors.push_back({p.r(), p.g(), p.b()});
    }

    happly::PLYData plyOut;
    plyOut.addVertexPositions(positions);
    plyOut.addVertexColors(colors);

    plyOut.write(path, happly::DataFormat::Binary);
}

void PointCloud::addPoint(Point &p) {
    _points.push_back(p);
}

void PointCloud::transform(ColorTransformation &ct) {
    for(auto &p : _points) {
        int r_i = p.r()*255;
        int g_i = p.r()*255;
        int b_i = p.r()*255;
        float r_o, g_o, b_o;
        ct.sample(r_i, g_i, b_i, r_o, g_o, b_o);
        p.updatePosition(r_o/255., g_o/255., b_o/255);
        p.updateColor(r_o/255., g_o/255., b_o/255);
    }
}
