//
// Created by imanol on 27/5/22.
//

#include <vector>

#ifndef COLOR_RESTORATION_CUBETETRAHEDRON_H
#define COLOR_RESTORATION_CUBETETRAHEDRON_H


class CubeTetrahedron {
public:
    explicit CubeTetrahedron() {;}
    CubeTetrahedron(int width, int depth, int height);
    void sample(const int i, const int j, const int k, float &x, float &y, float &z);

    void clearTetras();

    unsigned int numVerts() const  { return _vertices.size(); }
    unsigned int numTetras() const { return _tetras.size(); }
    unsigned int width() const { return _width; }
    unsigned int height() const { return _height; }
    unsigned int depth() const { return _depth; }

    void updateVert(const int i, const float x, const float y, const float z);
    void vert(const int i, float &x, float &y, float &z) const;
    void tetra(const int i, int &v1, int &v2, int &v3, int &v4) const;

private:
    unsigned int _width, _height, _depth;
    std::vector<std::vector<float>> _vertices;
    std::vector<std::vector<int>> _tetras;

};


#endif //COLOR_RESTORATION_CUBETETRAHEDRON_H
