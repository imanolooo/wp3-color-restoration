//
// Created by imanol on 27/5/22.
//

#ifndef COLOR_RESTORATION_CUBETETRAHEDRON_H
#define COLOR_RESTORATION_CUBETETRAHEDRON_H

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
    CubeTetrahedron(const std::vector<float> &dimensions, const std::vector<float> &origin, const std::vector<int> &resolution);
    void sample(const std::vector<float> &p, std::vector<float> &pTransformed);

    void clearTetras();

    unsigned int numVerts() const  { return _vertices.size(); }
    unsigned int numTetras() const { return _tetras.size(); }
    unsigned int look4NearestVert(const std::vector<float> &p);
    /*unsigned int width() const { return _width; }
    unsigned int height() const { return _height; }
    unsigned int depth() const { return _depth; }*/
    const std::vector<int>& res() { return _resolution; }

    void updateVert(const int i, const float x, const float y, const float z);
    void updatedVertices(const Eigen::MatrixXd &V);
    void vert(const int i, float &x, float &y, float &z) const;
    void tetra(const int i, int &v1, int &v2, int &v3, int &v4) const;

    void export2PLY(const std::string &path, const std::string &pathTranf);
    void export2PLYTetras(const std::string &path);

    bool look4BCInCube(const std::vector<float> &p, const int indexCell, int &iv1, int &iv2, int &iv3, int &iv4, float &bc1, float &bc2, float &bc3, float &bc4);
private:

    bool computeBarycentricCoordinates(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Eigen::Vector3d &v4, const Eigen::Vector3d &p,
                                       float &bc1, float &bc2, float &bc3, float &bc4) const;
    bool computeBarycentricCoordinates2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Eigen::Vector3d &v4, const Eigen::Vector3d &p,
                                       float &bc1, float &bc2, float &bc3, float &bc4) const;
    float Determinant4x4(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Eigen::Vector3d &v4) const;

    //unsigned int _width, _height, _depth;
    std::vector<float> _dimensions;
    std::vector<float> _origin;
    std::vector<int> _resolution;
    std::vector<std::vector<float>> _vertices, _updatedVertices;
    std::vector<std::vector<unsigned int>> _tetras;
    std::vector<std::vector<unsigned int>> _faces;

};


#endif //COLOR_RESTORATION_CUBETETRAHEDRON_H
