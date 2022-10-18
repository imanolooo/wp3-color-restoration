//
// Created by imanol on 27/5/22.
//

#include <cassert>
#include <QElapsedTimer>
#include "ColorTransformation.h"


ColorTransformation::ColorTransformation() {
    std::vector<float> dim = {2, 1, 1};
    std::vector<float> orig = {0, 0, 0};
    std::vector<float> res = {2, 2, 2};
    _ct = CubeTetrahedron(dim, orig, res);


}

void ColorTransformation::setControlPoints(std::vector<std::vector<float>> &cp) {

    for(auto &p : cp) {
        std::pair<int, std::vector<float>> controlPoint;
        controlPoint.first = _ct.look4NearestVert(p);
        //TODO: Update position of the vertex that will be a control point
        controlPoint.second = p;
        _cp.push_back(controlPoint);
    }

}

void ColorTransformation::updateControlPoint(int index, std::vector<float> &pos) {
    _cp[index].second = pos;

}

void ColorTransformation::computeBiharmonicCoordinates() {
    //V => Verts
    Eigen::MatrixXd V(_ct.numVerts(),3);
    for(auto i = 0; i < _ct.numVerts(); ++i) {
        float x, y, z;
        _ct.vert(i, x, y, z);
        V(i,0) = x;
        V(i,1) = y;
        V(i,2) = z;
    }

    //T => Tetrahedrons
    Eigen::MatrixXi T(_ct.numTetras(), 4);
    for(auto i = 0; i < _ct.numTetras(); ++i) {
        int v1, v2, v3, v4;
        _ct.tetra(i, v1, v2, v3, v4);
        T(i,0) = v1;
        T(i,1) = v2;
        T(i,2) = v3;
        T(i,3) = v4;
    }
    //_ct.clearTetras();

    //S => list of lists (of dim = 1 per points, dim > 1 per regions) of indexes of control points.
    std::vector<std::vector<int>> S;
    for(auto &cp : _cp) {
        S.push_back({cp.first});
    }
    //For 3D k needs to be 3.
    int k = 2;

    QElapsedTimer timer;
    timer.start();
    std::cout << "Computing biharmonic coordinates... " << std::endl;
    igl::biharmonic_coordinates(V,T,S,k,_W);
    std::cout << "Done in " << timer.elapsed() << std::endl;

}

void ColorTransformation::updateColorTransformation() {
    //V => High Res Verts
    Eigen::MatrixXd V(_ct.numVerts(),3);

    //L => Low Res Verts
    Eigen::MatrixXd L(_cp.size(),3);
    for(auto i = 0; i < _cp.size(); ++i) {
        L(i,0) = _cp[i].second[0];
        L(i,1) = _cp[i].second[1];
        L(i,2) = _cp[i].second[2];
    }

    V = _W * L;

    for(auto i = 0; i < V.rows(); ++i)
        _ct.updateVert(i, V(i,0), V(i,1), V(i,2));
}

void ColorTransformation::sample(const int r_i, const int g_i, const int b_i, float &r_o, float &g_o, float &b_o) {
    _ct.sample(r_i, g_i, b_i, r_o, g_o, b_o);
}

void ColorTransformation::print() {
    int numVerts = _ct.numVerts();
    std::cout << "Num Verts: " << numVerts << std::endl;
    for(auto i = 0; i < numVerts; ++i) {
        float x, y, z;
        _ct.vert(i, x, y, z);
        std::cout << i << " => " << x << ", " << y << ", " << z << std::endl;
    }

    int numTetras = _ct.numTetras();
    std::cout << "Num Tetras: " << numTetras << std::endl;
    for (auto i = 0; i < numTetras; ++i) {
        int i1, i2, i3, i4;
        _ct.tetra(i, i1, i2, i3, i4);
        std::cout << i << " => " << i1 << ", " << i2 << ", " << i3 << ", " << i4 << std::endl;
    }
}
