//
// Created by imanol on 21/6/22.
//

#include <iostream>
#include <QElapsedTimer>
#include "ColorTransformation2D.h"
#include <igl/biharmonic_coordinates.h>
#include <igl/readOBJ.h>
#include <color.hpp>
#include "happly.h"

ColorTransformation2D::ColorTransformation2D(const int width, const int height, const float step) :
                        _width(width), _height(height), _step(step), _origin({(float)(-width/2), (float)(-height/2)}) {
    int sizeVerts = (int)(((width-1)/step+1)*((height-1)/step+1));
    std::cout << "Size verts " << sizeVerts << std::endl;
    _vertices = Eigen::MatrixXd(sizeVerts,3);
    auto index = 0;
    for(float j = -height/2; j <= height/2; j+=step) {
        for(float i = -width/2; i <= width/2; i+=step) {
            _vertices(index,0) = 0;
            _vertices(index,1) = i;
            _vertices(index,2) = j;
            index++;
        }
    }

    int sizeFaces = (int)(((width-1)/step)*((height-1)/step))*2;
    std::cout << "Size faces " << sizeFaces << std::endl;
    _faces = Eigen::MatrixXi(sizeFaces,3);
    auto current = 0;
    index = 0;
    for(float j = 0; j < height-1; j += step) {
        for(float i = 0; i < width-1; i += step) {
            _faces(index,0) = current;
            _faces(index,1) = current + ((width-1)/step+1);
            _faces(index,2) = current + 1;
            ++index;
            _faces(index,0) = current + 1;
            _faces(index,1) = current + ((width-1)/step+1);
            _faces(index,2) = current + 1 + ((width-1)/step+1);
            ++index;
            current++;
        }
        current++;
    }

    /*if(!igl::readOBJ("/home/imanol/plane.obj",_vertices,_faces))
    {
        std::cout<<"failed to load mesh"<<std::endl;
    }*/

    /*std::cout << _vertices << std::endl;
    std::cout << std::endl;
    std::cout << _faces << std::endl;*/
}

void ColorTransformation2D::initControlPoints(const std::vector<std::vector<float>> &cp, std::vector<std::vector<float>> &newcp) {
    //Eigen::MatrixXd V, F;
    //std::cout << igl::readOBJ("/home/imanol/planeCP2.obj", V, F) << std::endl;

    //DONE Check that is working with a simple example
    std::cout << "Studying the control points..." << std::endl;
    std::vector<std::vector<int>> S;

    for(auto &p : cp) {
        //look for the nearest vertex
        int i = std::floor((p[1]-_origin[0])/_step);
        int j = std::floor((p[2]-_origin[1])/_step);
        int tl = j * _width + i;
        int tr = tl + 1;
        int bl = tl + _width;
        int br = bl + 1;
        Eigen::Vector3d ep(p[0], p[1],p[2]);
        float dtl = std::numeric_limits<float>::max();
        float dtr = std::numeric_limits<float>::max();
        float dbl = std::numeric_limits<float>::max();
        float dbr = std::numeric_limits<float>::max();
        if(tl >= 0 && tl < _vertices.rows()) dtl = (Eigen::Vector3d(_vertices.row(tl)) - ep).norm();
        if(tr >= 0 && tr < _vertices.rows()) dtr = (Eigen::Vector3d(_vertices.row(tr)) - ep).norm();
        if(bl >= 0 && bl < _vertices.rows())  dbl = (Eigen::Vector3d(_vertices.row(bl)) - ep).norm();
        if(br >= 0 && br < _vertices.rows())  dbr = (Eigen::Vector3d(_vertices.row(br)) - ep).norm();

        std::vector<std::pair<float,int>> distances = {std::make_pair(dtl,tl), std::make_pair(dtr,tr), std::make_pair(dbl,bl), std::make_pair(dbr,br)};
        std::sort(distances.begin(), distances.end());

        //check if that vertex has already been moved
        for(auto &s : S) {
            if(s[0] == distances[0].second) {
                std::cout << "PROBLEM: We need to move an already moved vertex for handle a Control Point."
                          << std::endl;
                return;
            }
        }

        //move the vertex to the control point
        _vertices.row(distances[0].second) = ep;

        //add the vertex to the control points list.
        S.push_back({distances[0].second});
    }

    /*S.push_back({11, 13, 18, 16});//Testing the area control
    std::cout << "Area CP" << std::endl;
    std::cout << _vertices.row(S.back()[0]) << std::endl;
    std::cout << _vertices.row(S.back()[1]) << std::endl;
    std::cout << _vertices.row(S.back()[2]) << std::endl;
    std::cout << _vertices.row(S.back()[3]) << std::endl;*/

    //fixing the boundaries
    auto index = 0;
    /*for(float j = -_height/2; j <= _height/2; j+=_step) {
        for(float i = -_width/2; i <= _width/2; i+=_step) {
            if(j == -_height/2 || j == _height/2) {
                S.push_back({index});
                newcp.push_back({0, i, j});
            } else if(i == -_width/2 || i == _width/2){
                S.push_back({index});
                newcp.push_back({0, i, j});
            }
            index++;
        }
    }*/

    /*for(float j = -15; j <= 50; j+=_step) {
        for(float i = -15; i <= 50; i+=_step) {
            if(j == -15 || j == 50) {
                S.push_back({index});
                newcp.push_back({0, i, j});
            } else if(i == -15 || i == 50){
                S.push_back({index});
                newcp.push_back({0, i, j});
            }
            index++;
        }
    }*/

    /*std::cout << "Vertices " << _vertices.rows() << std::endl;
    for(auto i = 0; i < _vertices.rows(); ++i)
        std::cout << "v " << _vertices(i,0) << " " << _vertices(i,1) << " " << _vertices(i, 2) << std::endl;
    std::cout << "Faces " << _faces.rows() << std::endl;
    for(auto i = 0; i < _faces.rows(); ++i)
        std::cout << "f " << _faces(i,0)+1 << " " << _faces(i,1)+1 << " " << _faces(i, 2)+1 << std::endl;*/

    int k = 2;

    QElapsedTimer timer;
    timer.start();
    std::cout << "Computing biharmonic coordinates... " << std::flush;
    std::cout << igl::biharmonic_coordinates(_vertices,_faces,S,k,_weights) << std::flush;
    std::cout << "Done in " << timer.elapsed() << std::endl;

//    std::cout << "Weights rxc " << _weights.rows() << " x " << _weights.cols() << std::endl;

    QElapsedTimer timerTransf;
    timerTransf.start();
    Eigen::MatrixXd ncp (newcp.size()/*+3*/,3);
    auto i = 0;
    for(; i < newcp.size(); ++i){
        ncp(i,0) = newcp[i][0];
        ncp(i,1) = newcp[i][1];
        ncp(i,2) = newcp[i][2];
    }
   // ncp.block(i, 0, 3, 3).setIdentity();
    /*ncp.row(i++) = _vertices.row(S.back()[0]);
    ncp.row(i++) = _vertices.row(S.back()[1]);
    ncp.row(i++) = _vertices.row(S.back()[2]);
    ncp.row(i++) = _vertices.row(S.back()[3]);*/

    std::cout << "NCP" << std::endl << ncp << std::endl;

    _verticesTransformed = _weights * ncp;
    std::cout << "Transformation in " << timerTransf.elapsed() << std::endl;
    /*std::cout << "Vertices transformed" << std::endl;
    for(auto i = 0; i < _verticesTransformed.rows(); ++i)
        std::cout << "v " << _verticesTransformed(i,0) << " " << _verticesTransformed(i,1) << " " << _verticesTransformed(i, 2) << std::endl;*/
    /*int counter = 0;
    std::cout << "Original control points:" << std::endl;
    for(auto const &p : cp) {
        std::cout << p[0] << " " << p[1] << " " << p[2] << " ---- " << _vertices(S[counter][0],0) << " " << _vertices(S[counter][0],1) << " " << _vertices(S[counter][0],2) << std::endl;
        counter++;
    }
    std::cout << "Transformed control points:" << std::endl;
    for(auto const &p : newcp)
        std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;*/
}

void ColorTransformation2D::sample(const std::vector<float> &p, std::vector<float> &pTransformed) const {
    int i = std::floor((p[1]-_origin[0])/_step);
    int j = std::floor((p[2]-_origin[1])/_step);
    int indexC = j*(_width-1)+i;
    //std::cout << "i " << i << " j " << j << " indexCell " << indexC << std::endl;

    float u, v;
    int iv0, iv1, iv2;
    bool found = false;

    for(auto j = -1; j < 2 && !found; ++j) {
        for(auto i = -1; i < 2 && !found; ++i) {
            for(auto k = 0; k < 2 && !found; ++k) {
                int currentIndexC = indexC + i + j * (_width - 1);
                if (currentIndexC < 0 || currentIndexC >= (_width - 1) * (_height - 1))
                    continue;

                int currentIndexF = currentIndexC * 2 + k;
                iv0 = _faces(currentIndexF, 0);
                iv1 = _faces(currentIndexF, 1);
                iv2 = _faces(currentIndexF, 2);

                found = computeBarycentricCoordinates(_vertices.row(iv0), _vertices.row(iv1), _vertices.row(iv2),
                                                      Eigen::Vector3d(p[0], p[1], p[2]), u, v);
            }
        }
    }
    if(!found) {
        std::cout << "Failed to detect the sampling face!" << std::endl;
        return;
    }
    /*
    //determine in which triangle lies the point. Use the determinant between diagonal and bl-p vectors.
    // >=0 first triangle, <0 second triangle
    int indexBL = j*_width + _width + i;//index bottom left point
    Eigen::Vector2f diagonal(_vertices(index+1,0) - _vertices(indexBL,0),
                             _vertices(index+1,1) - _vertices(indexBL,1));
    Eigen::Vector2f bl_p(p[0]-_vertices(indexBL,0),
                         p[1]-_vertices(indexBL,1));
    Eigen::Matrix2f m;
    m << bl_p , diagonal;
    float u,v;

    int iv0, iv1, iv2;
    if(m.determinant() >= 0) { //upper triangle
        iv0 = indexBL;
        iv1 = index+1;
        iv2 = index;
    } else { //downer triangle
        iv0 = indexBL;
        iv1 = indexBL+1;
        iv2 = index+1;
    }
    computeBarycentricCoordinates(_vertices.row(iv0), _vertices.row(iv1), _vertices.row(iv2),
                                  Eigen::Vector3d(p[0], p[1], 0), u, v);
*/

    //std::cout << "ivo " << iv0 << " iv1 " << iv1 << " iv2 " << iv2 << std::endl;
    Eigen::Vector3d transformed = u*_verticesTransformed.row(iv0) + v*_verticesTransformed.row(iv1) + (1-u-v)*_verticesTransformed.row(iv2);
    //std::cout << "Original point " << std::endl << Eigen::Vector3d(p[0], p[1], p[2]) << std::endl;
    //std::cout << "Transformed point " << std::endl << transformed << std::endl;
    pTransformed = {transformed(0), transformed(1), transformed(2)};

}

void ColorTransformation2D::export2PLY(const std::string pathOri, const std::string pathTransf) {
    std::cout << "Saving original AB space with control points..." << std::endl;
    std::vector<std::array<double,3>> positions;
    std::vector<std::array<double,3>> colors;
    std::vector<std::vector<size_t>> faces;

    positions.reserve(_vertices.rows());
    colors.reserve(_vertices.rows());
    faces.reserve(_faces.rows());

    for(auto i = 0; i < _vertices.rows(); ++i) {

        positions.push_back({_vertices(i,0), _vertices(i,1), _vertices(i,2)});
        color::lab<float> lab( { 100, _vertices(i,1), _vertices(i,2)});
        color::rgb<float> rgb;
        rgb = lab;
        colors.push_back({std::min(1.f,std::max(0.f,(float)rgb[0])),
                          std::min(1.f,std::max(0.f,(float)rgb[1])),
                          std::min(1.f,std::max(0.f,(float)rgb[2]))});
    }

    for(auto i = 0; i < _faces.rows(); ++i)
        faces.push_back({_faces(i,0), _faces(i,1), _faces(i,2)});
        //std::cout << "f " << _faces(i,0)+1 << " " << _faces(i,1)+1 << " " << _faces(i, 2)+1 << std::endl;*/

    happly::PLYData plyOut;
    plyOut.addVertexPositions(positions);
    plyOut.addVertexColors(colors);
    plyOut.addFaceIndices(faces);

    plyOut.write(pathOri, happly::DataFormat::Binary);

    std::cout << "Saving original AB space with control points..." << std::endl;
    positions.clear();
    colors.clear();

    for(auto i = 0; i < _verticesTransformed.rows(); ++i) {

        positions.push_back({_verticesTransformed(i,0), _verticesTransformed(i,1), _verticesTransformed(i,2)});
        color::lab<float> lab( { 100, _verticesTransformed(i,1), _verticesTransformed(i,2)});
        color::rgb<float> rgb;
        rgb = lab;
        colors.push_back({std::min(1.f,std::max(0.f,(float)rgb[0])),
                          std::min(1.f,std::max(0.f,(float)rgb[1])),
                          std::min(1.f,std::max(0.f,(float)rgb[2]))});
    }


    happly::PLYData plyOutTransf;
    plyOutTransf.addVertexPositions(positions);
    plyOutTransf.addVertexColors(colors);
    plyOutTransf.addFaceIndices(faces);

    plyOutTransf.write(pathTransf, happly::DataFormat::Binary);
}

bool ColorTransformation2D::computeBarycentricCoordinates(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                                                          const Eigen::Vector3d &p, float &u, float &v) const {
    //from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates
    Eigen::Vector3d v0v1 = v1 - v0;
    Eigen::Vector3d v0v2 = v2 - v0;
    Eigen::Vector3d v0p  =  p - v0;
    Eigen::Vector3d v1v2 = v2 - v1;
    Eigen::Vector3d v1p  =  p - v1;
    Eigen::Vector3d v2v0 = v0 - v2;
    Eigen::Vector3d v2p  =  p - v2;

    Eigen::Vector3d N = v0v1.cross(v0v2);
    Eigen::Vector3d C = v0v1.cross(v0p);
    if(N.dot(C) < 0)    return false;

    float denom = N.dot(N);

    Eigen::Vector3d C1 = v1v2.cross(v1p);
    u = N.dot(C1);
    if(u < 0)   return false;

    Eigen::Vector3d C2 = v2v0.cross(v2p);
    v = N.dot(C2);
    if(v < 0)   return false;

    u /= denom;
    v/= denom;

    return true;
}


void ColorTransformation2D::removeRow(Eigen::MatrixXi& matrix, unsigned int rowToRemove) {
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.bottomRows(numRows-rowToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void ColorTransformation2D::print() {
    std::cout << "Vertices: " << std::endl;
    std::cout << _vertices << std::endl;
    std::cout << "Transformed Vertices: " << std::endl;
    std::cout << _verticesTransformed << std::endl;
    std::cout << "Weights: " << std::endl;
    std::cout << _weights << std::endl;

}

