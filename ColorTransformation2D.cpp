//
// Created by imanol on 21/6/22.
//

#include <iostream>
#include <QElapsedTimer>
#include "ColorTransformation2D.h"
#include <igl/biharmonic_coordinates.h>
#include <igl/readOBJ.h>

ColorTransformation2D::ColorTransformation2D(const int width, const int height, const float step) :
                        _width(width), _height(height), _step(step), _origin({(float)(-width/2), (float)(-height/2)}) {
    int sizeVerts = (int)(((width-1)/step+1)*((height-1)/step+1));
    std::cout << "Size verts " << sizeVerts << std::endl;
    _vertices = Eigen::MatrixXd(sizeVerts,3);
    auto index = 0;
    for(float j = -height/2; j <= height/2; j+=step) {
        for(float i = -width/2; i <= width/2; i+=step) {
            _vertices(index,0) = i;
            _vertices(index,1) = j;
            _vertices(index,2) = 0;
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

    std::cout << _vertices << std::endl;
    std::cout << std::endl;
    std::cout << _faces << std::endl;
}

void ColorTransformation2D::initControlPoints(const std::vector<std::vector<float>> &cp, const std::vector<std::vector<float>> &newcp) {
    Eigen::MatrixXd V, F;
    std::cout << igl::readOBJ("/home/imanol/planeCP2.obj", V, F) << std::endl;

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
        Eigen::Vector3d ep(p[1],p[2],0);
        float dtl = (Eigen::Vector3d(_vertices.row(tl)) - ep).norm();
        float dtr = (Eigen::Vector3d(_vertices.row(tr)) - ep).norm();
        float dbl = (Eigen::Vector3d(_vertices.row(bl)) - ep).norm();
        float dbr = (Eigen::Vector3d(_vertices.row(br)) - ep).norm();

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

    std::cout << "Vertices " << _vertices.rows() << std::endl;
    for(auto i = 0; i < _vertices.rows(); ++i)
        std::cout << "v " << _vertices(i,0) << " " << _vertices(i,1) << " " << _vertices(i, 2) << std::endl;
    std::cout << "Faces " << _faces.rows() << std::endl;
    for(auto i = 0; i < _faces.rows(); ++i)
        std::cout << "f " << _faces(i,0)+1 << " " << _faces(i,1)+1 << " " << _faces(i, 2)+1 << std::endl;

    int k = 2;

    QElapsedTimer timer;
    timer.start();
    std::cout << "Computing biharmonic coordinates... " << std::endl;
    std::cout << igl::biharmonic_coordinates(_vertices,_faces,S,k,_weights) << std::endl;
    std::cout << "Done in " << timer.elapsed() << std::endl;

    //TODO compute the correct ncp matrix depending on the cp modified
    Eigen::MatrixXd ncp (newcp.size(),3);
    for(auto i = 0; i < newcp.size(); ++i){
        ncp(i,0) = newcp[i][1];
        ncp(i,1) = newcp[i][2];
        ncp(i,2) = 0;
    }

    _verticesTransformed = _weights * ncp;
    std::cout << "Vertices transformed" << std::endl;
    for(auto i = 0; i < _verticesTransformed.rows(); ++i)
        std::cout << "v " << _verticesTransformed(i,0) << " " << _verticesTransformed(i,1) << " " << _verticesTransformed(i, 2) << std::endl;
}

void ColorTransformation2D::sample(const std::vector<float> &p, std::vector<float> &pTransformed) const {
    //TODO Consider the case that we sample a cell that contains a control point.

    int i = std::floor((p[0]-_origin[0])/_step);
    int j = std::floor((p[1]-_origin[1])/_step);
    int indexC = j*(_width-1)+i;

    float u,v;
    int iv0, iv1, iv2;

    for(auto j = -1; j < 2; ++j) {
        for(auto i = -1; i < 2; ++i) {
            int currentIndexC = indexC + i + j*(_width-1);
            if(currentIndexC < 0 || currentIndexC >= (_width-1)*(_height-1))
                continue;

            int currentIndexF = currentIndexC*2;
            iv0 = _faces(currentIndexF,0);
            iv1 = _faces(currentIndexF,1);
            iv2 = _faces(currentIndexF,2);

            if(computeBarycentricCoordinates(_vertices.row(iv0), _vertices.row(iv1), _vertices.row(iv2),
                                             Eigen::Vector3d(p[0], p[1], 0), u, v))
                break;
        }
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

    Eigen::Vector3d transformed = u*_verticesTransformed.row(iv0) + v*_verticesTransformed.row(iv1) + (1-u-v)*_verticesTransformed.row(iv2);
    std::cout << "Original point " << std::endl << Eigen::Vector3d(p[0], p[1], 0) << std::endl;
    std::cout << "Transformed point " << std::endl << transformed << std::endl;
    pTransformed = {transformed(0), transformed(1), transformed(2)};

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