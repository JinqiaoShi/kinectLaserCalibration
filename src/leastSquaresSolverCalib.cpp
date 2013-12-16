#include "leastSquaresSolverCalib.h"


void leastSquareSolverCalib::setInitialGuess(Vector2d params){
    _startingParams = params;
}

Vector2d leastSquareSolverCalib::optimize(pointpairVec &assoc){
    int size=assoc.size();
    MatrixXd P(size,2);
    MatrixXd Pt(size,2);
    MatrixXd Z(4,size);
    lasercloudToEigenMatrixWithAssociations(P,Pt,assoc);
    //UGLY
    //--------------------------------------
    for(int i=0;i<size;i++)
    {
        for(int j=0;j<2;j++)
        {
            Z(j,i)=P(i,j);
        }
        for(int j=2;j<4;j++)
        {
            Z(j,i)=Pt(i,j-2);
        }
    }
    //--------------------------------------

    Vector2d x=_startingParams;
    _result=pointAlignerLoop(x,Z,_iterations,_result);
    return _result;
}
void leastSquareSolverCalib::setIterationNum(int i){
    _iterations=i;
}


void leastSquareSolverCalib::lasercloudToEigenMatrixWithAssociations(MatrixXd &m1, MatrixXd &m2, pointpairVec &assoc)
{
    m1.setOnes();
    m2.setOnes();

    pointpair a;
    geometry_msgs::Point32 p;
    for(unsigned int i=0;i<assoc.size();i++)
    {
        a=assoc.at(i);
        p=a.first;
        m1(i,0)=p.x;
        m1(i,1)=p.y;
        p=a.second;
        m2(i,0)=p.x;
        m2(i,1)=p.y;
    }

}
Vector2d leastSquareSolverCalib::pointAlignerLoop( Vector2d &x,  MatrixXd &Z, int iterations,  Vector2d &result)
{
    result=x;
    MatrixXd H(2,2);
    H.setZero();

    MatrixXd b(2,1);
    b.setZero();

    Vector2d X;
    std::cout << Z.transpose()<< std::endl;
    for(int i = 0; i < iterations; i++){
        std::cout<<"iteration "<<i <<std::endl;
        X=x;
        for(int j=0;j<Z.cols();j++){

            Matrix2d J = computeJacobian(j,Z);
            Vector2d e=computeError(j,Z);
            std:: cout << "error: "<< e<< std::endl<<std::endl;
            H+=J.transpose()*J;
            b+=J.transpose()*e;
        }
    }
    Vector2d dx;
    std::cout << "H: "<<std::endl<<H<<std::endl<<std::endl<<std::endl;
    std::cout << "b: "<<std::endl<<b<<std::endl;
    LDLT<MatrixXd> ldlt(-H);
    dx=ldlt.solve(b); // using a LDLT factorizationldlt;
    return dx;

}
Vector2d leastSquareSolverCalib::computeError(int i,MatrixXd &Z)
{
    laserc pi;
    laserc pj;
    pi.first=Z(0,i);
    pi.second=Z(1,i);
    pj.first=Z(2,i);
    pj.second=Z(3,i);

    Vector2d er;

    er(0) =pi.first-pj.first;
    er(1 )=pi.second-pj.second;


    return er;
}
Matrix2d leastSquareSolverCalib::computeJacobian(int i, MatrixXd &Z)
{
    MatrixXd J(2,2);
    J.setZero();
    double x=Z(2,i);
    double y=Z(3,i);
    double rofake=sqrt(x*x+y*y);
    double alfafake=atan(y/x);

    J(0,0)=-rofake;
    J(0,1)=-rofake*alfafake*alfafake;
    J(1,0)=-rofake;
    J(1,1)=-rofake*alfafake*alfafake;

    //std::cout<< J << std::endl;

    return J;
}



