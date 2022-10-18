//
// Created by imanol on 27/5/22.
//

#include "CubeTetrahedron.h"

#include <cmath>
#include <iostream>
#include <array>

#include "happly.h"

CubeTetrahedron::CubeTetrahedron(std::vector<float> &dimensions, std::vector<float> &origin, std::vector<float> &resolution)
    : _dimensions(dimensions), _origin(origin), _resolution(resolution) {

    //TODO: Use origin and resolution to create the points.

    std::vector<float> end = {_origin[0]+_dimensions[0], _origin[1]+_dimensions[1], _origin[2]+_dimensions[2]};
    std::vector<float> incr = {_dimensions[0]/_resolution[0], _dimensions[1]/_resolution[1], _dimensions[2]/_resolution[2]};

    //generate vertices - corners
    for(auto x = origin[0]; x <= end[0]; x+=incr[0]) {
        for(auto y = origin[1]; y <= end[1]; y+=incr[1]) {
            for(auto z = origin[2]; z <= end[2]; z+=incr[2]) {
                _vertices.push_back({(float)x, (float)y, (float)z});
            }
        }
    }
    //generate vertices - middle
    std::vector<float> current = {_origin[0], _origin[1], _origin[2]};
    for(auto x = 0; x < _resolution[0]; ++x) {
        current[1] = _origin[1];
        for(auto y = 0; y < _resolution[1]; ++y) {
            current[2] = _origin[2];
            for(auto z = 0; z < _resolution[2]; ++z) {
                _vertices.push_back({current[0]+incr[0]*0.5f, current[1]+incr[1]*0.5f, current[2]+incr[2]*0.5f});
                current[2] += incr[2];
            }
            current[1] += incr[1];
        }
        current[0] += incr[0];
    }

    //generate tetrahedrons
    int index = 0;
    int cubeCounter = 0;
    for(auto x = 0; x < _resolution[0]; ++x) {
        for(auto y = 0; y < _resolution[1]; ++y) {
            for(auto z = 0; z < _resolution[2]; ++z) {
                unsigned int v0 = index;
                unsigned int v1 = index+1;
                unsigned int v2 = index+(_resolution[2]+1);
                unsigned int v3 = index+(_resolution[2]+1)+1;
                unsigned int v4 = index+(_resolution[2]+1)*(_resolution[1]+1);
                unsigned int v5 = index+(_resolution[2]+1)*(_resolution[1]+1)+1;
                unsigned int v6 = index+(_resolution[2]+1)*(_resolution[1]+1)+(_resolution[2]+1);
                unsigned int v7 = index+(_resolution[2]+1)*(_resolution[1]+1)+(_resolution[2]+1)+1;
                unsigned int v8 = (_resolution[0]+1)*(_resolution[1]+1)*(_resolution[2]+1) + cubeCounter;

                _tetras.push_back({v1, v0, v2, v8});//left
                _tetras.push_back({v1, v2, v3, v8});
                _tetras.push_back({v2, v7, v3, v8});//back
                _tetras.push_back({v2, v6, v7, v8});
                _tetras.push_back({v6, v5, v7, v8});//right
                _tetras.push_back({v6, v4, v5, v8});
                _tetras.push_back({v0, v5, v4, v8});//front
                _tetras.push_back({v0, v1, v5, v8});
                _tetras.push_back({v1, v3, v7, v8});//top
                _tetras.push_back({v1, v7, v5, v8});
                _tetras.push_back({v0, v4, v6, v8});//bottom
                _tetras.push_back({v0, v6, v2, v8});

                _faces.push_back({v0, v1, v2});//left
                _faces.push_back({v1, v3, v2});
                _faces.push_back({v2, v3, v6});//back
                _faces.push_back({v3, v7, v6});
                _faces.push_back({v4, v6, v5});//right
                _faces.push_back({v6, v7, v5});
                _faces.push_back({v4, v5, v1});//front
                _faces.push_back({v0, v4, v1});
                _faces.push_back({v0, v2, v4});//bottom
                _faces.push_back({v4, v2, v6});
                _faces.push_back({v5, v3, v1});//top
                _faces.push_back({v5, v7, v3});//top

                index++;
                cubeCounter++;
            }
            index++;
        }
        index += _resolution[2] + 1;
    }
}

void CubeTetrahedron::vert(const int i, float &x, float &y, float &z) const {
    x = _vertices[i][0];
    y = _vertices[i][1];
    z = _vertices[i][2];
}

void CubeTetrahedron::tetra(const int i, int &v1, int &v2, int &v3, int &v4) const {
    v1 = _tetras[i][0];
    v2 = _tetras[i][1];
    v3 = _tetras[i][2];
    v4 = _tetras[i][3];
}

void CubeTetrahedron::updateVert(const int i, const float x, const float y, const float z) {
    _vertices[i][0] = x;
    _vertices[i][1] = y;
    _vertices[i][2] = z;
}

void CubeTetrahedron::sample(const int i, const int j, const int k, float &x, float &y, float &z) {
    /*unsigned int index = i + j * _width + k * _width * _depth;//usar _resolution
    x = _vertices[i][0];
    y = _vertices[i][1];
    z = _vertices[i][2];*/
}

void CubeTetrahedron::clearTetras() {
    _tetras.clear();
    std::vector<std::vector<unsigned int>>().swap(_tetras);
}

unsigned int CubeTetrahedron::look4NearestVert(const std::vector<float> &p) {
    std::vector<float> incr = {_dimensions[0]/_resolution[0], _dimensions[1]/_resolution[1], _dimensions[2]/_resolution[2]};

    std::vector<float> ijk = {(p[0]-_origin[0])/incr[0], (p[1]-_origin[1])/incr[1], (p[2]-_origin[2])/incr[2]};
    for(auto &x : ijk)
        x = std::round(x);
    //TODO: clip to 0 and maxRes to handle points outside the grid

    std::cout << "Nearest position " << ijk[0]*incr[0] << ", " << ijk[1]*incr[1] << ", " << ijk[2]*incr[2] << std::endl;

    unsigned int index = ijk[0] * (_resolution[1]+1)*(_resolution[2]+1) + ijk[1] * (_resolution[2]+1) + ijk[2];
    std::cout << "Index " << index << std::endl;

    return index;


}

void CubeTetrahedron::export2PLY(const std::string &path) {
    std::vector<std::array<double,3>> positions;
    positions.reserve(_vertices.size());
    for(const auto &v : _vertices)
        positions.push_back({v[0], v[1], v[2]});
    happly::PLYData plyOut;
    plyOut.addVertexPositions(positions);
    //plyOut.addVertexColors(colors);
    plyOut.addFaceIndices(_faces);
    plyOut.write(path, happly::DataFormat::Binary);
}

void CubeTetrahedron::export2PLYTetras(const std::string &path) {
    std::vector<std::array<double,3>> positions;
    positions.reserve(_vertices.size());
    for(const auto &v : _vertices)
        positions.push_back({v[0], v[1], v[2]});
    happly::PLYData plyOut;
    plyOut.addVertexPositions(positions);
    //plyOut.addVertexColors(colors);
    //plyOut.addFaceIndices(_faces);
    std::vector<std::vector<unsigned int>> faces;
    for(const auto &t : _tetras) {
        faces.push_back({t[0], t[1], t[3]});
        faces.push_back({t[0], t[2], t[1]});
        faces.push_back({t[1], t[2], t[3]});
        faces.push_back({t[0], t[3], t[2]});
    }
    plyOut.addFaceIndices(faces);

    plyOut.write(path, happly::DataFormat::Binary);
}

