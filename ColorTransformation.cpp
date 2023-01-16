//
// Created by imanol on 27/5/22.
//

#include <cassert>
#include <QElapsedTimer>
#include "ColorTransformation.h"

#include <igl/colormap.h>
#include "happly.h"

template<class Matrix>
void write_matrix(const char* filename, const Matrix& matrix){
    std::ofstream out(filename,std::ios::out /*| std::ios::binary*/ | std::ios::trunc);
    typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
    out.write((char*) (&rows), sizeof(typename Matrix::Index));
    out.write((char*) (&cols), sizeof(typename Matrix::Index));
    out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
    out.close();
}
template<class Matrix>
void read_matrix(const char* filename, Matrix& matrix){
    std::ifstream in(filename,std::ios::in /*| std::ios::binary*/);
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
}

ColorTransformation::ColorTransformation(const std::vector<float> &dim, const std::vector<float> &orig, const std::vector<int> &res) {
    _ct = CubeTetrahedron(dim, orig, res);
    _boundaries = false;
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

void ColorTransformation::prepareControlPoints() {
    //S => list of lists (of dim = 1 per points, dim > 1 per regions) of indexes of control points.
    //Control Points
    _S.clear();
    for(auto &cp : _cp) {
        _S.push_back({cp.first});
    }
    //boundaries

    std::cout << "Boundaries " << _boundaries << std::endl;
    if(_boundaries) {
        int boundRes = 15;
        float incrI = _ct.res()[0]/(boundRes-1.f);
        float incrJ = _ct.res()[1]/(boundRes-1.f);
        float incrK = _ct.res()[2]/(boundRes-1.f);
        //-X
        for(auto k = 0; k < boundRes; ++k) {
            for(auto j = 0; j < boundRes; ++j) {
                _S.push_back({(int)(j*incrJ)*(_ct.res()[2]+1) + (int)(k*incrK)});
            }
        }
        //+X
        for(auto k = 0; k < boundRes; ++k) {
            for(auto j = 0; j < boundRes; ++j) {
                _S.push_back({_ct.res()[0]*(_ct.res()[1]+1)*(_ct.res()[2]+1) + (int)(j*incrJ)*(_ct.res()[2]+1) + (int)(k*incrK)});
            }
        }
        /*Fails because we repeat points.
         * //-Y
        for(auto k = 0; k < boundRes; ++k) {
            for(auto i = 0; i < boundRes; ++i) {
                _S.push_back({i*incrI*(_ct.res()[1]+1)*(_ct.res()[2]+1) + k*incrK});
            }
        }
        //+Y
        for(auto k = 0; k < boundRes; ++k) {
            for(auto i = 0; i < boundRes; ++i) {
                _S.push_back({i*incrI*(_ct.res()[1]+1)*(_ct.res()[2]+1) + (_ct.res()[1])*(_ct.res()[2]+1) + k*incrK});
            }
        }
        //-Z
        for(auto j = 0; j < boundRes; ++j) {
            for(auto i = 0; i < boundRes; ++i) {
                _S.push_back({i*incrI*(_ct.res()[1]+1)*(_ct.res()[2]+1) + j*incrJ*(_ct.res()[2]+1)});
            }
        }
        //+Z
        for(auto j = 0; j < boundRes; ++j) {
            for(auto i = 0; i < boundRes; ++i) {
                _S.push_back({i*incrI*(_ct.res()[1]+1)*(_ct.res()[2]+1) + j*incrJ*(_ct.res()[2]+1) + _ct.res()[2]});
            }
        }*/

        //_S.push_back({0, _ct.res()[2], (_ct.res()[2]+1)*(_ct.res()[1]+1)-1, _ct.res()[2]+1});

    }
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

    //For 3D k needs to be 3.
    int k = 2;

    QElapsedTimer timer;
    timer.start();
    std::cout << "Computing biharmonic coordinates... " << std::endl;
    igl::biharmonic_coordinates(V,T,_S,k,_W);
    std::cout << "Done in " << timer.elapsed() << std::endl;

    write_matrix("weights.txt", _W);

}

void ColorTransformation::loadLastBiharmonicCoordinatesWeights() {
    read_matrix("weights.txt", _W);
}

void ColorTransformation::updateColorTransformation() {
    //V => High Res Verts
    Eigen::MatrixXd V(_ct.numVerts(),3);

    std::cout << "Updating Color Transformation..." << std::endl;

    //L => Low Res Verts
    //first control points
    Eigen::MatrixXd L(_S.size(),3);
    auto i = 0;
    for(i = 0; i < _cp.size(); ++i) {
        L(i,0) = _cp[i].second[0];
        L(i,1) = _cp[i].second[1];
        L(i,2) = _cp[i].second[2];
        //std::cout << L.row(i).x() << ", " << L.row(i).y() << ", " << L.row(i).z() << std::endl;
    }
    //then bounding regions.
    //L.block(i, 0, 4, 3).setIdentity();
    for(; i < _S.size(); ++i) {
        float x,y,z;
        _ct.vert(_S[i][0], x, y ,z);
        L.row(i) = Eigen::Vector3d(x, y, z);
    }

    //std::cout << "W " << _W.rows() << " x " << _W.cols() << std::endl;

    //std::cout << "L " << L.rows() << " x " << L.cols() << std::endl;
    //std::cout << L << std::endl;

    V = _W * L;

    _ct.updatedVertices(V);
}

void ColorTransformation::sample(const std::vector<float> &input, std::vector<float> &output) {
    //std::cout << "Sampling " << std::endl;
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

void ColorTransformation::exportWeights(const std::string path) {

    std::vector<std::array<double,3>> positions, colors;
    positions.reserve(_ct.numVerts());
    colors.reserve(_ct.numVerts());

    //populate the positions
    float x, y, z;
    for(auto i = 0; i < _ct.numVerts(); ++i) {
        _ct.vert(i, x, y, z);
        positions.push_back({x,y,z});
    }

    std::cout << "Positions done" << std::endl;

    for(auto i = 0; i < _S.size(); ++i) {
        std::string file = "Weight" + std::to_string(i) + ".ply";

        //compute the colors
        colors.clear();

        double caxis_min = _W.minCoeff();
        double caxis_max = _W.maxCoeff();
        Eigen::MatrixXd CM;

        double max_abs = std::max(abs(caxis_min), abs(caxis_max));
        Eigen::MatrixXd UV = _W.col(i).array();//(abs(_W.col(i).array())/(max_abs));;//((_W.array()-caxis_min)/(caxis_max-caxis_min));
        igl::colormap(igl::COLOR_MAP_TYPE_JET,UV.eval(),-1,1,CM);

        std::cout << "W " << _W.rows() << " x " << _W.cols() << std::endl;
        std::cout << "Warray " << _W.col(i).array().rows() << " x " << _W.col(i).array().cols() << std::endl;
        std::cout << "UV " << UV.rows() << " x " << UV.cols() << std::endl;
        std::cout << "CM " << CM.rows() << " x " << CM.cols() << std::endl;
        std::cout << caxis_min << " -> " << caxis_max << " => " << max_abs << std::endl;
        std::cout << "colormap done" << std::endl;

        float r, g, b;
        for(auto i = 0; i < _ct.numVerts(); ++i) {
            r = CM(i,0);
            g = CM(i,1);
            b = CM(i,2);
            colors.push_back({r,g,b});
        }

       /* int counter = 0;
        for(auto p : colors) {
            std::cout << counter << "/" << colors.size()  << " pos " << positions.size() << std::endl;
            std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;
            counter++;
        }
        std::cout << "colors printed" << std::endl;*/


        happly::PLYData ply;
        ply.addVertexPositions(positions);
        ply.addVertexColors(colors);
        std::string pathName = path + file;
        ply.write(pathName, happly::DataFormat::ASCII);
    }

}

void ColorTransformation::exportDeformationFactors(const std::string &pathDF, const std::string &pathInv) {
    _ct.exportDeformationFactor(pathDF, pathInv);
}

