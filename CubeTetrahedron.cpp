//
// Created by imanol on 27/5/22.
//

#include "CubeTetrahedron.h"

#include <cmath>
#include <iostream>
#include <array>
#include <igl/colormap.h>

#include "happly.h"

CubeTetrahedron::CubeTetrahedron(const std::vector<float> &dimensions, const std::vector<float> &origin, const std::vector<int> &resolution)
    : _dimensions(dimensions), _origin(origin), _resolution(resolution) {

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
    for(auto i = 0; i < V.rows(); ++i) {
        _updatedVertices[i] = {V(i,0), V(i,1), V(i,2)};
    }

}

void CubeTetrahedron::sample(const std::vector<float> &p, std::vector<float> &pTransformed) {
    //std::cout << "Sampling" << std::endl;
    //std::cout << p[0] << ", " << p[1] << ", " << p[2] << " => " << pTransformed[0] << ", " << pTransformed[1] << ", " << pTransformed[2] << std::endl;

    //put p inside the range using the trick of max-0.01.
    std::vector<float> pNorm = {std::max(_origin[0],std::min(_origin[0]+_dimensions[0]-0.01f,p[0])),
                                std::max(_origin[1],std::min(_origin[1]+_dimensions[1]-0.01f,p[1])),
                                std::max(_origin[2],std::min(_origin[2]+_dimensions[2]-0.01f,p[2]))};
    //std::cout << pNorm[0] << ", " << pNorm[1] << ", " << pNorm[2] << std::endl;

    //find the cell where the point lies
    std::vector<float> incr = {_dimensions[0]/_resolution[0], _dimensions[1]/_resolution[1], _dimensions[2]/_resolution[2]};
    std::vector<float> ijk = {(pNorm[0]-_origin[0])/incr[0], (pNorm[1]-_origin[1])/incr[1], (pNorm[2]-_origin[2])/incr[2]};
    unsigned int indexVLC = (int)ijk[0] * (_resolution[1]+1)*(_resolution[2]+1) + (int)ijk[1] * (_resolution[2]+1) + (int)ijk[2];
    unsigned int indexCell = (int)ijk[0] * (_resolution[1])*(_resolution[2]) + (int)ijk[1] * (_resolution[2]) + (int)ijk[2];

    //std::cout << "Incr " << incr[0] << ", " << incr[1] << ", " << incr[2] << std::endl;
    //std::cout << "Ijk " << ijk[0] << ", " << ijk[1] << ", " << ijk[2] << std::endl;

    int iv1, iv2, iv3, iv4;
    float bc1, bc2, bc3, bc4;
    bool found = look4BCInCube(pNorm, indexCell, iv1, iv2, iv3, iv4, bc1, bc2, bc3, bc4);
    int counter = 1;
    while(!found) {

        int deltaX = (_resolution[2]) * (_resolution[1]);
        int deltaY = (_resolution[2]);
        int deltaZ = 1;
        int deltaVLCX = (_resolution[2]+1) * (_resolution[1]+1);
        int deltaVLCY = (_resolution[2]+1);
        int deltaVLCZ = 1;
        for(auto i = -counter; i < counter && !found; ++i) {
            for(auto j = -counter; j < counter && !found; ++j) {
                for(auto k = -counter; k < counter && !found; ++k) {
                    int currentIndexCell = indexCell + i*deltaX + j*deltaY + k*deltaZ;

                    found = look4BCInCube(pNorm, currentIndexCell, iv1, iv2, iv3, iv4, bc1, bc2, bc3, bc4);
                }
            }
        }

        if(counter == 10){//crash the application if the cell is not suitable to be found
            std::cout << "Impossible to find transform " << p[0] << ", " << p[1] << ", " << p[2] <<std::endl;
            std::cout <<_vertices[-1][4] << std::endl;
        }

        counter++;

    }

    //compute new position
    pTransformed[0] = _updatedVertices[iv1][0]*bc1 + _updatedVertices[iv2][0]*bc2 + _updatedVertices[iv3][0]*bc3 + _updatedVertices[iv4][0]*bc4;
    pTransformed[1] = _updatedVertices[iv1][1]*bc1 + _updatedVertices[iv2][1]*bc2 + _updatedVertices[iv3][1]*bc3 + _updatedVertices[iv4][1]*bc4;
    pTransformed[2] = _updatedVertices[iv1][2]*bc1 + _updatedVertices[iv2][2]*bc2 + _updatedVertices[iv3][2]*bc3 + _updatedVertices[iv4][2]*bc4;
    /*std::cout << "IV1 " << iv1 << " => " << _updatedVertices[iv1][0] << ", " << _updatedVertices[iv1][1] << ", " << _updatedVertices[iv1][2] << std::endl;
    std::cout << "IV1 " << iv2 << " => " << _updatedVertices[iv2][0] << ", " << _updatedVertices[iv2][1] << ", " << _updatedVertices[iv2][2] << std::endl;
    std::cout << "IV1 " << iv3 << " => " << _updatedVertices[iv3][0] << ", " << _updatedVertices[iv3][1] << ", " << _updatedVertices[iv3][2] << std::endl;
    std::cout << "IV1 " << iv4 << " => " << _updatedVertices[iv4][0] << ", " << _updatedVertices[iv4][1] << ", " << _updatedVertices[iv4][2] << std::endl;
    std::cout << "BC's " << bc1 << " " << bc2 << " " << bc3 << " " << bc4 << std::endl;
    std::cout << "PTransformed " << pTransformed[0] << ", " << pTransformed[1] << ", " << pTransformed[2] << std::endl;*/

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

    //std::cout << "Nearest position " << ijk[0]*incr[0] << ", " << ijk[1]*incr[1] << ", " << ijk[2]*incr[2] << std::endl;

    unsigned int index = ijk[0] * (_resolution[1]+1)*(_resolution[2]+1) + ijk[1] * (_resolution[2]+1) + ijk[2];
    //std::cout << "Index " << index << std::endl;

    return index;


}

void CubeTetrahedron::export2PLY(const std::string &path, const std::string &pathTranf) {
    std::vector<std::array<double,3>> positions;
    positions.reserve(_vertices.size());
    for(const auto &v : _vertices)
        positions.push_back({v[0], v[1], v[2]});
    happly::PLYData plyOut;
    plyOut.addVertexPositions(positions);
    //plyOut.addVertexColors(colors);
    plyOut.addFaceIndices(_faces);
    plyOut.write(path, happly::DataFormat::Binary);

    positions.clear();
    for(const auto &v : _updatedVertices)
        positions.push_back({v[0], v[1], v[2]});
    happly::PLYData plyOut2;
    plyOut2.addVertexPositions(positions);
    //plyOut.addVertexColors(colors);
    plyOut2.addFaceIndices(_faces);
    plyOut2.write(pathTranf, happly::DataFormat::Binary);
}

void CubeTetrahedron::export2PLYTetras(const std::string &path, const std::string &pathTranf){
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

    positions.clear();
    positions.reserve(_updatedVertices.size());
    for(const auto &v : _updatedVertices)
    positions.push_back({v[0], v[1], v[2]});
    happly::PLYData plyOutTransf;
    plyOut.addVertexPositions(positions);
    //plyOut.addVertexColors(colors);
    //plyOut.addFaceIndices(_faces);
    faces.clear();
    for(const auto &t : _tetras) {
    faces.push_back({t[0], t[1], t[3]});
    faces.push_back({t[0], t[2], t[1]});
    faces.push_back({t[1], t[2], t[3]});
    faces.push_back({t[0], t[3], t[2]});
    }
    plyOutTransf.addFaceIndices(faces);

    plyOutTransf.write(pathTranf, happly::DataFormat::Binary);
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

bool CubeTetrahedron::look4BCInCube(const std::vector<float> &p, const int indexCell, int &iv1, int &iv2, int &iv3, int &iv4,
                                    float &bc1, float &bc2, float &bc3, float &bc4) {
    //find the tetra of the cell where the point lies => check if baricentric coords are in range.
    //compute baricentric coordinates in the tetra

    bool found = false;
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
        Eigen::Vector3d  pp(p[0], p[1], p[2]);

        found = computeBarycentricCoordinates(v1, v2, v3, v4, pp, bc1, bc2, bc3, bc4);
    }
    //if(!found) {
    //    std::cout << "ERROR: Impossible to sample the point " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    //}

    return found;
}

bool CubeTetrahedron::computeBarycentricCoordinates2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                                                     const Eigen::Vector3d &v3, const Eigen::Vector3d &v4,
                                                     const Eigen::Vector3d &p, float &bc1, float &bc2, float &bc3,
                                                     float &bc4) const {
    //https://dennis2society.de/painless-tetrahedral-barycentric-mapping
    const float det0 = Determinant4x4(v1, v2, v3, v4);
    const float det1 = Determinant4x4(p,  v2, v3, v4);
    const float det2 = Determinant4x4(v1,  p, v3, v4);
    const float det3 = Determinant4x4(v1, v2,  p, v4);
    const float det4 = Determinant4x4(v1, v2, v3,  p);
    bc1 = (det1/det0);
    bc2 = (det2/det0);
    bc3 = (det3/det0);
    bc4 = (det4/det0);

    return !(bc1 < 0 || bc2 < 0 || bc3 < 0 || bc4 < 0);
}

float CubeTetrahedron::Determinant4x4(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3,
                                      const Eigen::Vector3d &v4) const {
    float det = v2.z()*v3.y()*v4.x() - v1.z()*v3.y()*v4.x() -
                v2.y()*v3.z()*v4.x() + v1.y()*v3.z()*v4.x() +

                v1.z()*v2.y()*v4.x() - v1.y()*v2.z()*v4.x() -
                v2.z()*v3.x()*v4.y() + v1.z()*v3.x()*v4.y() +

                v2.x()*v3.z()*v4.y() - v1.x()*v3.z()*v4.y() -
                v1.z()*v2.x()*v4.y() + v1.x()*v2.z()*v4.y() +

                v2.y()*v3.x()*v4.z() - v1.y()*v3.x()*v4.z() -
                v2.x()*v3.y()*v4.z() + v1.x()*v3.y()*v4.z() +

                v1.y()*v2.x()*v4.z() - v1.x()*v2.y()*v4.z() -
                v1.z()*v2.y()*v2.x() + v1.y()*v2.z()*v3.x() +

                v1.z()*v2.x()*v3.y() - v1.x()*v2.z()*v3.y() -
                v1.y()*v2.x()*v3.z() + v1.x()*v2.y()*v3.z();
    return det;
}

void CubeTetrahedron::exportDeformationFactor(const std::string &pathDF, const std::string &pathInv) {
    //create the structures
    std::vector<std::array<double,3>> positions, colors, colorsInv;
    std::vector<float> deformationFactors;
    std::vector<bool> inversions;
    int size = (_resolution[0]+1)*(_resolution[1]+1)*(_resolution[2]+1) + (_resolution[0])*(_resolution[1])*(_resolution[2]);
    positions.reserve(size);
    colors.reserve(size);
    deformationFactors.reserve(size);
    inversions.reserve(size);
    colorsInv.reserve(size);

    //for-each cell of the grid compute the deformation factor
    auto index = 0;
    float sMin = std::numeric_limits<float>::max();
    float sMax = std::numeric_limits<float>::min();
    for(auto i = 0; i < _resolution[0]; ++i) {
        for(auto j = 0; j < _resolution[1]; ++j) {
            for(auto k = 0; k < _resolution[2]; ++k) {
                //compute the indices
                unsigned int v0 = index;
                unsigned int v1 = index+1;
                unsigned int v2 = index+(_resolution[2]+1);
                unsigned int v3 = index+(_resolution[2]+1)*(_resolution[1]+1);
                index++;

                //create the vertices
                Eigen::Vector3f A(_vertices[v0][0], _vertices[v0][1], _vertices[v0][2]);
                Eigen::Vector3f B(_vertices[v1][0], _vertices[v1][1], _vertices[v1][2]);
                Eigen::Vector3f C(_vertices[v2][0], _vertices[v2][1], _vertices[v2][2]);
                Eigen::Vector3f D(_vertices[v3][0], _vertices[v3][1], _vertices[v3][2]);
                Eigen::Vector3f At(_updatedVertices[v0][0], _updatedVertices[v0][1], _updatedVertices[v0][2]);
                Eigen::Vector3f Bt(_updatedVertices[v1][0], _updatedVertices[v1][1], _updatedVertices[v2][2]);
                Eigen::Vector3f Ct(_updatedVertices[v2][0], _updatedVertices[v2][1], _updatedVertices[v2][2]);
                Eigen::Vector3f Dt(_updatedVertices[v3][0], _updatedVertices[v3][1], _updatedVertices[v3][2]);

                //compute the vector bases
                Eigen::Vector3f u = B - A;
                Eigen::Vector3f v = C - A;
                Eigen::Vector3f w = D - A;
                Eigen::Vector3f ut = Bt - At;
                Eigen::Vector3f vt = Ct - At;
                Eigen::Vector3f wt = Dt - At;
                Eigen::Matrix3f M, Mt;
                M << u, v, w;
                Mt << ut, vt, wt;

                //compute the transformation matrix
                Eigen::Matrix3f T = Mt * M.inverse();

                //compute the SVD
                Eigen::JacobiSVD<Eigen::Matrix3f> svd(T);//sorted in decreasing order
                float s1 = svd.singularValues()[0];
                float s2 = svd.singularValues()[1];
                float s3 = svd.singularValues()[2];

                //compute the deformation factor
                float df = s1;//it will depends on the desired metric
                deformationFactors.push_back(df);
                sMax = std::max(sMax, df);
                sMin = std::min(sMin, df);

                //check if there is inversion
                Eigen::Matrix4f Cell, CellT;
                Cell.row(0) << A, 1;
                Cell.row(1) << B, 1;
                Cell.row(2) << C, 1;
                Cell.row(3) << D, 1;
                CellT.row(0) << At, 1;
                CellT.row(1) << Bt, 1;
                CellT.row(2) << Ct, 1;
                CellT.row(3) << Dt, 1;
                float detCell = Cell.determinant();
                float detCellT = CellT.determinant();
                inversions.push_back(detCell*detCellT < 0.f);

                //fill positions structure
                positions.push_back({_vertices[v0][0], _vertices[v0][1], _vertices[v0][2]});
            }
            index++;
        }
        index += _resolution[2] + 1;
    }

    //prepare data and convert deformation factor to color
    for(const auto &df : deformationFactors) {
        //Eigen::MatrixXf color;
        //Eigen::MatrixXf value;
        //value << df;
        float r,g,b;
        igl::colormap(igl::COLOR_MAP_TYPE_JET,(df-sMin)/(sMax-sMin), r, g, b);
        /*float r = color(0,0);
        float g = color(0,1);
        float b = color(0,2);*/
        colors.push_back({r,g,b});
    }
    for(const auto &inv : inversions) {
        if(inv)
            colorsInv.push_back({1,0,0});
        else
            colorsInv.push_back({0,0,1});
    }


    //export
    happly::PLYData ply, plyInv;
    ply.addVertexPositions(positions);
    ply.addVertexColors(colors);
    std::string pathName = pathDF;
    ply.write(pathName, happly::DataFormat::ASCII);
    ply.addVertexColors(colorsInv);
    pathName = pathInv;
    ply.write(pathName, happly::DataFormat::ASCII);
}

