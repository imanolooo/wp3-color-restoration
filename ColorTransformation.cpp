//
// Created by imanol on 27/5/22.
//

#include <cassert>
#include <QElapsedTimer>
#include "ColorTransformation.h"


ColorTransformation::ColorTransformation(const std::vector<float> &dim, const std::vector<float> &orig, const std::vector<int> &res) {
    _ct = CubeTetrahedron(dim, orig, res);
}

void ColorTransformation::setControlPoints(std::vector<std::vector<float>> &cp) {
    std::map<int, std::vector<std::vector<float>>> cpChecker;
    for(auto &p : cp) {
        std::pair<int, std::vector<float>> controlPoint;
        controlPoint.first = _ct.look4NearestVert(p);
        _ct.updateVert(controlPoint.first, p[0], p[1], p[2]);
        controlPoint.second = p;
        _cp.push_back(controlPoint);
        cpChecker[controlPoint.first].push_back(controlPoint.second);
    }
    for(const auto &check : cpChecker){
        if(check.second.size() > 1) {
            std::cout << "Problem! 2 control points lies in same vertex!" << std::endl;
            std::cout << "Vertex " << check.first << std::endl;
            for(const auto &p : check.second) {
                std::cout << p[0] << ", " << p[1] << ", " << p [2] << std::endl;
            }
        }
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

    _ct.updatedVertices(V);
}

void ColorTransformation::sample(const std::vector<float> &input, std::vector<float> &output) {
    std::cout << "Sampling " << std::endl;
    _ct.sample(input, output);
}

void ColorTransformation::print() {
    int numVerts = _ct.numVerts();
    std::cout << "Num Verts: " << numVerts << std::endl;
    for(auto i = 0; i < 31*31+1/*numVerts*/; ++i) {
        float x, y, z;
        _ct.vert(i, x, y, z);
        std::cout << i << " => " << x << ", " << y << ", " << z << std::endl;
    }

    int numTetras = _ct.numTetras();
    std::cout << "Num Tetras: " << numTetras << std::endl;
    /*for (auto i = 0; i < numTetras; ++i) {
        int i1, i2, i3, i4;
        _ct.tetra(i, i1, i2, i3, i4);
        std::cout << i << " => " << i1 << ", " << i2 << ", " << i3 << ", " << i4 << std::endl;
    }*/
}
