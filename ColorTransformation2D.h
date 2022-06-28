//
// Created by imanol on 21/6/22.
//

#ifndef COLOR_RESTORATION_COLORTRANSFORMATION2D_H
#define COLOR_RESTORATION_COLORTRANSFORMATION2D_H


#include <Eigen/Core>
#include <vector>

class ColorTransformation2D {
public:
    ColorTransformation2D(const int width, const int height, const float step = 1);
    void initControlPoints(const std::vector<std::vector<float>> &cp, const std::vector<std::vector<float>> &newcp);
    void sample(const std::vector<float> &p, std::vector<float> &pTransformed) const;

private:
    bool computeBarycentricCoordinates(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &p,
                                        float &u, float &v) const;
    void removeRow(Eigen::MatrixXi& matrix, unsigned int rowToRemove);

    Eigen::MatrixXd _vertices, _verticesTransformed;
    Eigen::MatrixXi _faces;
    Eigen::MatrixXd _weights;

    std::vector<float>  _origin;
    int                 _width, _height;
    float               _step;

    std::vector<std::pair<int,int>>    _cellsWithControlPoint;//first cell, second index to cp new vertex
};


#endif //COLOR_RESTORATION_COLORTRANSFORMATION2D_H
