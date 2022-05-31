//
// Created by imanol on 27/5/22.
//

#ifndef COLOR_RESTORATION_COLORTRANSFORMATION_H
#define COLOR_RESTORATION_COLORTRANSFORMATION_H


#include "CubeTetrahedron.h"
#include <igl/biharmonic_coordinates.h>

class ColorTransformation {
public:
    ColorTransformation();

    void sample(const int r_i, const int g_i, const int b_i, float &r_o, float &g_o, float &b_o);

    void setControlPoints(std::vector<std::vector<float> > &cp);
    void updateControlPoint(int index, std::vector<float> &pos);
    void computeBiharmonicCoordinates();
    void updateColorTransformation();

private:
    CubeTetrahedron _ct;
    std::vector<std::pair<int,std::vector<float>>> _cp; //int => index in the cubeTetrahedron, vector<float> coordinates

    Eigen::MatrixXd _W; //biharmonic coordinates weights;


};


#endif //COLOR_RESTORATION_COLORTRANSFORMATION_H
