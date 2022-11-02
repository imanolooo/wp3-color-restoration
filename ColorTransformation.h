//
// Created by imanol on 27/5/22.
//

#ifndef COLOR_RESTORATION_COLORTRANSFORMATION_H
#define COLOR_RESTORATION_COLORTRANSFORMATION_H


#include "CubeTetrahedron.h"
#include <igl/biharmonic_coordinates.h>

class ColorTransformation {
public:
    ColorTransformation(const std::vector<float> &dim, const std::vector<float> &orig, const std::vector<int> &res);

    void sample(const std::vector<float> &input, std::vector<float> &output);

    void setControlPoints(std::vector<std::vector<float> > &cp);
    void updateControlPoint(int index, std::vector<float> &pos);
    void computeBiharmonicCoordinates();
    void updateColorTransformation();

    void print();
    void export2PLY(const std::string path, const std::string pathTransf) { _ct.export2PLY(path, pathTransf); }
    void export2PLYTetras(const std::string path) { _ct.export2PLYTetras(path); }

    CubeTetrahedron* cubeTetra() { return &_ct; }

private:
    CubeTetrahedron _ct;
    std::vector<std::pair<int,std::vector<float>>> _cp; //int => index in the cubeTetrahedron, vector<float> coordinates

    Eigen::MatrixXd _W; //biharmonic coordinates weights;
    std::vector<std::vector<int>> _S;

    bool _boundaries;

};


#endif //COLOR_RESTORATION_COLORTRANSFORMATION_H
