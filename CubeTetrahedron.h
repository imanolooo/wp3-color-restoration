//
// Created by imanol on 27/5/22.
//

#include <vector>

#ifndef COLOR_RESTORATION_CUBETETRAHEDRON_H
#define COLOR_RESTORATION_CUBETETRAHEDRON_H

#include <string>
/*
 * EACH CUBE INSIDE THE GRID FOLLOWS THE NEXT ORDER
 *          3           7
 *      1           5
 *
 *
 *          2           6
 *      0           4
 */


class CubeTetrahedron {
public:
    explicit CubeTetrahedron() {;}
    CubeTetrahedron(std::vector<float> &dimensions, std::vector<float> &origin, std::vector<float> &resolution);
    void sample(const int i, const int j, const int k, float &x, float &y, float &z);

    void clearTetras();

    unsigned int numVerts() const  { return _vertices.size(); }
    unsigned int numTetras() const { return _tetras.size(); }
    unsigned int look4NearestVert(const std::vector<float> &p);
    /*unsigned int width() const { return _width; }
    unsigned int height() const { return _height; }
    unsigned int depth() const { return _depth; }*/

    void updateVert(const int i, const float x, const float y, const float z);
    void vert(const int i, float &x, float &y, float &z) const;
    void tetra(const int i, int &v1, int &v2, int &v3, int &v4) const;

    void export2PLY(const std::string &path);
    void export2PLYTetras(const std::string &path);

private:
    //unsigned int _width, _height, _depth;
    std::vector<float> _dimensions;
    std::vector<float> _origin;
    std::vector<float> _resolution;
    std::vector<std::vector<float>> _vertices;
    std::vector<std::vector<unsigned int>> _tetras;
    std::vector<std::vector<unsigned int>> _faces;

};


#endif //COLOR_RESTORATION_CUBETETRAHEDRON_H
