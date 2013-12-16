#ifndef LSSOLVER_H
#define LSSOLVER_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include "geohelper.h"

using namespace Eigen;
class leastSquareSolver{
public:
    //COMMON PART
    void setInitialGuess(Vector3d t);
    Vector3d optimize(pointpairVec &assoc);
    void setIterationNum(int i);

private:
    //PRIVATE MEMBERS
    int _iterations;
    Vector3d _startingTrasform;
    Vector3d _result;

    //PRIVATE PART
    void pointcloudToEigenMatrixWithAssociations(MatrixXd &m1, MatrixXd &m2,pointpairVec &assoc);
    Vector3d pointAlignerLoop( Vector3d &x,  MatrixXd &Z, int iterations,  Vector3d &result);
    Vector2d computeError(int i,MatrixXd &X, MatrixXd &Z);
    MatrixXd computeJacobian(int i,Matrix3d X, MatrixXd Z);
};

#endif
