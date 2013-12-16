#include "leastSquaresSolver.h"


void leastSquareSolver::setInitialGuess(Vector3d t){
    _startingTrasform = t;
}

Vector3d leastSquareSolver::optimize(pointpairVec &assoc){
    int size=assoc.size();
    MatrixXd P(size,3);
    MatrixXd Pt(size,3);
    MatrixXd Z(4,size);
    pointcloudToEigenMatrixWithAssociations(P,Pt,assoc);
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
    Vector3d x=_startingTrasform;
    _result=pointAlignerLoop(x,Z,_iterations,_result);
    return _result;
}
void leastSquareSolver::setIterationNum(int i){
    _iterations=i;
}


void leastSquareSolver::pointcloudToEigenMatrixWithAssociations(MatrixXd &m1, MatrixXd &m2,pointpairVec &assoc)
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
Vector3d leastSquareSolver::pointAlignerLoop( Vector3d &x,  MatrixXd &Z, int iterations,  Vector3d &result)
{
    result=x;
    MatrixXd H(3,3);
    H.setZero();

    MatrixXd b(3,1);
    b.setZero();

    MatrixXd X(3,3);

    for(int i = 0; i < iterations; i++){
        std::cout<<"iteration "<<i <<std::endl;
        X=Malcom::geometry::v2t(x);
        for(int j=0;j<Z.cols();j++){

            Vector2d e=computeError(j,X,Z);
            MatrixXd J=computeJacobian(j,X,Z);
            H+=J.transpose()*J;
            b+=J.transpose()*e;
        }
    }
    MatrixXd id(3,3);
//    id.setIdentity();
//    id*100;
//    H+=id;
    Vector3d dx;
    LDLT<MatrixXd> ldlt(-H);
    dx=ldlt.solve(b); // using a LDLT factorizationldlt;
    MatrixXd dX=Malcom::geometry::v2t(dx);

    Vector3d res;
    res.setZero();
    //std::cout <<"dx:"<<dx<<std::endl<<std::endl;
    //std::cout <<"X: "<<X<<std::endl<<std::endl;
    //MatrixXd transform=dX*X;
    //std::cout <<res<<std::endl<<std::endl;
    res=Malcom::geometry::t2v(dX*X);
    return res;

}
Vector2d leastSquareSolver::computeError(int i,MatrixXd &X, MatrixXd &Z)
{
    Vector3d pi(3);
    pi.setOnes();

    Vector3d pj(3);
    pj.setOnes();

    pi.head(2)=Z.block<2,1>(0,i);
    pj.head(2)=Z.block<2,1>(2,i);
    Vector3d e;
    e=(pi-X*pj);
    Vector2d er;
    er<<e(0),e(1);
    return er;
}
MatrixXd leastSquareSolver::computeJacobian(int i,Matrix3d X, MatrixXd Z)
{
    MatrixXd J(2,3);
    J.setZero();
    J(0,0)=-1;
    J(1,1)=-1;
    //float c=X(0,0);
    //float s=X(0,1);
    Matrix2d Rprime;
    Rprime<<0,-1,1,0;
    //std::cout<<" RPRIME -> "<<Rprime<<std::endl;
    Vector2d pj;
    pj=Z.block<2,1>(2,i);
    //std::cout << "================"<<std::endl;
    //std::cout << X.block<2,2>(0,0)<<std::endl;
    //std::cout << "----------------"<<std::endl;
    //std::cout << pj<<std::endl;
    //std::cout << "----------------"<<std::endl;
    //std::cout << X.block<1,2>(2,0)<<std::endl;
    //std::cout << "================"<<std::endl;

    pj=X.block<2,2>(0,0)*pj+X.block<2,1>(0,2);
    J.block<2,1>(0,2)=-Rprime*pj;
    return J;
}



