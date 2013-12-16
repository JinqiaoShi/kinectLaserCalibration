#ifndef LSSOLVERCALIB_H
#define LSSOLVERCALIB_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "geohelper.h"

using namespace Eigen;
class leastSquareSolverCalib{
public:
    //COMMON PART
    void setInitialGuess(Vector2d params);
    Vector2d optimize(pointpairVec &assoc);
    void setIterationNum(int i);

private:
    //PRIVATE MEMBERS
    int _iterations;
    Vector2d _startingParams;
    Vector2d _result;

    //PRIVATE PART
    void lasercloudToEigenMatrixWithAssociations(MatrixXd &m1, MatrixXd &m2,pointpairVec &assoc);
    Vector2d pointAlignerLoop( Vector2d &x,  MatrixXd &Z, int iterations,  Vector2d &result);
    Vector2d computeError(int i,MatrixXd &Z);
    Matrix2d computeJacobian(int i,MatrixXd &Z);
};

#endif
