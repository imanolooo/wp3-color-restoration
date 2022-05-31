//
// Created by imanol on 27/5/22.
//

#include "CubeTetrahedron.h"

CubeTetrahedron::CubeTetrahedron(int width, int depth, int height) : _width(width), _depth(depth), _height(height) {
    //generate vertices;
    for(auto z = 0; z < height; ++z) {
        for(auto y = 0; y < depth; ++y) {
            for(auto x = 0; x < width; ++x) {
                _vertices.push_back({(float)x, (float)y, (float)z});
            }
        }
    }

    //generate tetrahedrons
    int index = 0;
    for(int z = 0; z < height-1; ++z) {
        for(int y = 0; y < depth-1; ++y) {
            for(int x = 0; x < width-1; ++x) {
                int v0 = index;
                int v1 = index+1;
                int v2 = index+width;
                int v3 = index+width+1;
                int v4 = index+(width*height);
                int v5 = index+(width*height)+1;
                int v6 = index+(width*height)+width;
                int v7 = index+(width*height)+width+1;

                _tetras.push_back({v0, v1, v3, v5});
                _tetras.push_back({v0, v3, v2, v6});
                _tetras.push_back({v0, v5, v6, v4});
                _tetras.push_back({v3, v6, v5, v7});
                _tetras.push_back({v0, v3, v6, v5});

                index++;
            }
        }
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
    unsigned int index = i + j * _width + k * _width * _depth;
    x = _vertices[i][0];
    y = _vertices[i][1];
    z = _vertices[i][2];
}

void CubeTetrahedron::clearTetras() {
    _tetras.clear();
    std::vector<std::vector<int>>().swap(_tetras);
}

