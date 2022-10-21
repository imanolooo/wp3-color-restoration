//
// Created by imanol on 27/5/22.
//

#include "CubeTetrahedron.h"

#include <cmath>
#include <iostream>
#include <array>

#include "happly.h"

CubeTetrahedron::CubeTetrahedron(const std::vector<float> &dimensions, const std::vector<float> &origin, const std::vector<int> &resolution)
    : _dimensions(dimensions), _origin(origin), _resolution(resolution) {

    //TODO: Use origin and resolution to create the points.

    //std::vector<float> end = {_origin[0]+_dimensions[0], _origin[1]+_dimensions[1], _origin[2]+_dimensions[2]};
    std::vector<float> incr = {_dimensions[0]/_resolution[0], _dimensions[1]/_resolution[1], _dimensions[2]/_resolution[2]};

    //generate vertices - corners
    _vertices.reserve((_resolution[0]+1)*(_resolution[1]+1)*(_resolution[2]+1) + (_resolution[0])*(_resolution[1])*(_resolution[2]));
    auto x = origin[0];
    for(auto i = 0; i <= _resolution[0]; ++i, x+=incr[0]) {
        auto y = origin[1];
        for(auto j = 0; j <= _resolution[1]; ++j, y+=incr[1]) {
            auto z = origin[2];
            for(auto k = 0; k <= _resolution[2]; ++k, z+=incr[2]) {
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

    _updatedVertices = _vertices;

    //generate tetrahedrons
    _tetras.reserve(12*(_resolution[0]*_resolution[1]*_resolution[2]));
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

void CubeTetrahedron::updatedVertices(const Eigen::MatrixXd &V) {
    for(auto i = 0; i < V.rows(); ++i)
        _updatedVertices[i] = {V(i,0), V(i,1), V(i,2)};
}

void CubeTetrahedron::sample(const std::vector<float> &p, std::vector<float> &pTransformed) {
    std::cout << "Sampling" << std::endl;
    std::cout << p[0] << ", " << p[1] << ", " << p[2] << " => " << pTransformed[0] << ", " << pTransformed[1] << ", " << pTransformed[2] << std::endl;

    //find the cell where the point lies
    std::vector<float> incr = {_dimensions[0]/_resolution[0], _dimensions[1]/_resolution[1], _dimensions[2]/_resolution[2]};
    std::vector<float> ijk = {(p[0]-_origin[0])/incr[0], (p[1]-_origin[1])/incr[1], (p[2]-_origin[2])/incr[2]};
    unsigned int indexVLC = (int)ijk[0] * (_resolution[1]+1)*(_resolution[2]+1) + (int)ijk[1] * (_resolution[2]+1) + (int)ijk[2];
    unsigned int indexCell = (int)ijk[0] * (_resolution[1])*(_resolution[2]) + (int)ijk[1] * (_resolution[2]) + (int)ijk[2];

    std::cout << "Incr " << incr[0] << ", " << incr[1] << ", " << incr[2] << std::endl;
    std::cout << "Ijk " << ijk[0] << ", " << ijk[1] << ", " << ijk[2] << std::endl;

    bool found = false;
    int counter = 0;
    while(!found) {
        unsigned int index_X = indexVLC + (_resolution[2]+1)*(_resolution[1]+1);
        unsigned int index_Y = indexVLC + (_resolution[2]+1);
        unsigned int index_Z = indexVLC + 1;

        Eigen::Vector3d VLC(_vertices[indexVLC][0], _vertices[indexVLC][1], _vertices[indexVLC][2]);
        Eigen::Vector3d X(_vertices[index_X][0]-_vertices[indexVLC][0], _vertices[index_X][1]-_vertices[indexVLC][1], _vertices[index_X][2]-_vertices[indexVLC][2]);
        Eigen::Vector3d Y(_vertices[index_Y][0]-_vertices[indexVLC][0], _vertices[index_Y][1]-_vertices[indexVLC][1], _vertices[index_Y][2]-_vertices[indexVLC][2]);
        Eigen::Vector3d Z(_vertices[index_Z][0]-_vertices[indexVLC][0], _vertices[index_Z][1]-_vertices[indexVLC][1], _vertices[index_Z][2]-_vertices[indexVLC][2]);
        Eigen::Matrix3d M;
        M << X, Y, Z;
        Eigen::Vector3d d = M.inverse() * (Eigen::Vector3d(p[0], p[1], p[2])-VLC);
        std::cout << "IndexVLC " << indexVLC << "\t IndexCell " << indexCell << std::endl;
        std::cout << "IndexVLC Coords " << _vertices[indexVLC][0] << ", " << _vertices[indexVLC][1] << ", " << _vertices[indexVLC][2] << "!" << std::endl;
        std::cout << "D " << d.x() << ", " << d.y() << ", " << d.z() << std::endl;
        std::cout << "X " << X.x() << ", " << X.y() << ", " << X.z() << std::endl;
        std::cout << "Y " << Y.x() << ", " << Y.y() << ", " << Y.z() << std::endl;
        std::cout << "Z " << Z.x() << ", " << Z.y() << ", " << Z.z() << std::endl;
        std::cout << "D' " << (p[0]-_vertices[indexVLC][0])/incr[0] << ", " << (p[1]-_vertices[indexVLC][1])/incr[1] << ", " << (p[2]-_vertices[indexVLC][2])/incr[2] << std::endl;

        if(counter == 10){
            std::cout <<_vertices[-1][4] << std::endl;
        }

        if(d.x() >= 0 && d.x() < 1. && d.y() >= 0 && d.y() < 1. && d.z() >= 0 && d.z() < 1.)
            found = true;
        else {
            if(d.x() < 0) {
                indexVLC -= std::abs((int)d.x()+1)*(_resolution[2]+1)*(_resolution[1]+1);
                indexCell -= std::abs((int)d.x()+1)*(_resolution[2])*(_resolution[1]);
            }
            if(d.x() > 1) {
                indexVLC += (int) d.x() * (_resolution[2] + 1) * (_resolution[1] + 1);
                indexCell += (int) d.x() * (_resolution[2]) * (_resolution[1]);
            }
            if(d.y() < 0) {
                indexVLC -= std::abs((int) d.x() + 1) * (_resolution[2] + 1);
                indexCell -= std::abs((int) d.x()) * (_resolution[2]);
            }
            if(d.y() > 1) {
                indexVLC += (int) d.x() * (_resolution[2] + 1);
                indexCell += (int) d.x() * (_resolution[2]);
            }
            if(d.z() < 0) {
                indexVLC -= std::abs((int) d.z() + 1) * (1);
                indexCell -= std::abs((int) d.z() + 1) * (1);
            }
            if(d.z() > 1) {
                indexVLC += (int) d.z() * (1);
                indexCell += (int) d.z() * (1);
            }
        }
        counter++;
    }

    //find the tetra of the cell where the point lies => check if baricentric coords are in range.
    //compute baricentric coordinates in the tetra
    found = false;
    float bc1, bc2, bc3, bc4;

    int iv1, iv2, iv3, iv4;
    for(auto i = 0; i < 12 && !found; ++i) {

        iv1 = _tetras[indexCell*12+i][0];
        iv2 = _tetras[indexCell*12+i][1];
        iv3 = _tetras[indexCell*12+i][2];
        iv4 = _tetras[indexCell*12+i][3];

        Eigen::Vector3d v1, v2, v3, v4;
        v1 = Eigen::Vector3d(_vertices[iv1][0], _vertices[iv1][1], _vertices[iv1][2]);
        v2 = Eigen::Vector3d(_vertices[iv2][0], _vertices[iv2][1], _vertices[iv2][2]);
        v3 = Eigen::Vector3d(_vertices[iv3][0], _vertices[iv3][1], _vertices[iv3][2]);
        v4 = Eigen::Vector3d(_vertices[iv4][0], _vertices[iv4][1], _vertices[iv4][2]);
        Eigen::Vector3d  p(p[0], p[1], p[2]);

        found = computeBarycentricCoordinates(v1, v2, v3, v4, p, bc1, bc2, bc3, bc4);
    }
    if(!found) {
        std::cout << "ERROR: Impossible to sample the point " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    }

    //compute new position
    pTransformed[0] = _updatedVertices[iv1][0]*bc1 + _updatedVertices[iv2][0]*bc2 + _updatedVertices[iv3][0]*bc3 + _updatedVertices[iv4][0]*bc4;
    pTransformed[1] = _updatedVertices[iv1][1]*bc1 + _updatedVertices[iv2][1]*bc2 + _updatedVertices[iv3][1]*bc3 + _updatedVertices[iv4][1]*bc4;
    pTransformed[2] = _updatedVertices[iv1][2]*bc1 + _updatedVertices[iv2][2]*bc2 + _updatedVertices[iv3][2]*bc3 + _updatedVertices[iv4][2]*bc4;

}

void CubeTetrahedron::clearTetras() {
    _tetras.clear();
    std::vector<std::vector<unsigned int>>().swap(_tetras);
}

unsigned int CubeTetrahedron::look4NearestVert(const std::vector<float> &p) {
    //important, only work before changing control points.
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

bool CubeTetrahedron::computeBarycentricCoordinates(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Eigen::Vector3d &v4,
                                                    const Eigen::Vector3d &p, float &bc1, float &bc2, float &bc3, float &bc4) const {
    //https://www.cdsimpson.net/2014/10/barycentric-coordinates.html

    Eigen::Vector3d v_ap = p - v1;
    Eigen::Vector3d v_bp = p - v2;
    Eigen::Vector3d v_ab = v2 - v1;
    Eigen::Vector3d v_ac = v3 - v1;
    Eigen::Vector3d v_ad = v4 - v1;
    Eigen::Vector3d v_bc = v3 - v2;
    Eigen::Vector3d v_bd = v4 - v2;

    float Va = (1./6.)*(v_bp.dot(v_bd.cross(v_bc)));
    float Vb = (1./6.)*(v_ap.dot(v_ac.cross(v_ad)));
    float Vc = (1./6.)*(v_ap.dot(v_ad.cross(v_ab)));
    float Vd = (1./6.)*(v_ap.dot(v_ab.cross(v_ac)));
    float V  = (1./6.)*(v_ab.dot(v_ac.cross(v_ad)));

    bc1 = Va/V;
    bc2 = Vb/V;
    bc3 = Vc/V;
    bc4 = Vd/V;

    return !(bc1 < 0 || bc2 < 0 || bc3 < 0 || bc4 < 0);
}

