#include "spinner.h"
#include <Eigen/LU>

#define SCALING 30

using namespace Eigen;

Spinner::Spinner() : shutdown_required(false),thread(&Spinner::spin, *this){
    this->i= new QImage(QSize(640,480),QImage::Format_RGB16);
    d2=0;
    a2=0;
    tx=0;
    ty=0;
    tz=0;
    rz=0;
    t.setIdentity();
}

Spinner::~Spinner() {
    shutdown_required = true;
    thread.join();
}
//===================================================================0



void Spinner::callback(const LaserScanConstPtr &l1, const LaserScanConstPtr &l2){
    //calibration parameters from the UI


    this->l1=l1;
    this->l2=l2;
    updateData();



}

void Spinner::updateData()
{
    LaserScanCleaner(l1,this->laser1); //
    LaserScanCleaner(l2,this->laser2); // needed to remove crappy kinect noise
    d2=laser->d2;
    a2=laser->a2;
    tx=laser->tx;
    ty=laser->ty;
    tz=laser->tz;
    rz=laser->rz;
    calibrateLaserRanges(d2, a2, this->laser2);
    scanToPointcloud(this->laser1,this->cloud1);
    scanToPointcloud(this->laser2,this->cloud2);



    tf::Transform tnew;
    tnew.setIdentity();
    tf::Quaternion q(tf::Vector3(0,0,1),rz);
    tnew.setRotation(q);
    //tnew.setOrigin(tnew.getBasis()*tf::Vector3(tx,ty,tz));
    tnew.setOrigin(tf::Vector3(tx,ty,tz));
    tranformPointcloud(cloud2,tnew);
    pointcloudToLaserscan(cloud2,this->laser2);

    findScansAssociations(this->laser1,this->laser2,M_PI/(90));

    scanToPointcloud(this->laser1,this->cloud1);
    scanToPointcloud(this->laser2,this->cloud2);
    float err =weightedSquaredErrorEstimation(cloud1,cloud2,assoc);
    //std::cout<< "error is: "<<err<<std::endl;
    laser->errorLabel->setText(QString::number(err));

    laser->f->cloud1.points.clear();
    laser->f->cloud2.points.clear();

    laser->f->cloud1=cloud1;
    laser->f->cloud2=cloud2;
    laser->f->p=assoc;


}

void Spinner::LaserScanCleaner(const sensor_msgs::LaserScanConstPtr &src, myLaserStructure &dst){
    dst.angles.clear();
    dst.ranges.clear();


    float startingAngle=src->angle_min;
    for(unsigned int i=0;i<src->ranges.size();i++)
    {
        if(src->ranges.at(i)!=-1) //-1 is an error in the kinect scans
        {
            dst.angles.push_back(startingAngle+src->angle_increment*i);
            dst.ranges.push_back(src->ranges.at(i));
        }
    }
}

void Spinner::findScansAssociations(myLaserStructure &scan1, myLaserStructure &scan2, float threshold){
    this->assoc.clear();
    for(unsigned int i = 0; i< scan1.angles.size();i++){
        for(unsigned int j = 0; j< scan2.angles.size();j++){
            if(     scan1.angles.at(i)>= (scan2.angles.at(j)-threshold) &&
                    scan1.angles.at(i)<= (scan2.angles.at(j)+threshold) &&
                    abs(scan1.ranges.at(i)*scan1.ranges.at(i)-scan2.ranges.at(j)*scan2.ranges.at(j))<=1.0f){
                assoc.push_back(new pointAssoc(i,j,
                                                   abs(scan1.ranges.at(i)*scan1.ranges.at(i)-scan2.ranges.at(j)*scan2.ranges.at(j))
                                               ));
            }
        }
    }
    cleanAssociations(this->assoc);
}

void Spinner::cleanAssociations(std::vector<pointAssoc*> &assoc)
{
    pointAssoc* p;
    float d;
    for(unsigned int i=0;i<assoc.size();i++){
        d=9999;
        for(unsigned int j=0;j<assoc.size();j++){
            p=assoc.at(j);
            if(p->d<d) d=p->d;
            else assoc.erase (assoc.begin()+j);
        }
    }
}

void Spinner::putAssInTheBag()
{
    for(unsigned int i=0;i<assoc.size();i++){
        this->globalAssoc.push_back(assoc.at(i));
    }
    std::cout<<"GLOBAL ASS SIZE: "<<this->globalAssoc.size()<<std::endl;
}

void Spinner::calibrateLaserRanges(float k1, float k2, myLaserStructure &scan){
    for(unsigned int i=0;i < scan.ranges.size(); i++){
        scan.ranges.at(i)=scan.ranges.at(i)+scan.ranges.at(i)*scan.ranges.at(i)*k1+scan.ranges.at(i)*scan.ranges.at(i)*k1*scan.angles.at(i)*scan.angles.at(i)*k2;

    }
}

void Spinner::scanToPointcloud(myLaserStructure &scan, sensor_msgs::PointCloud & cloud){
    geometry_msgs::Point32 p;
    cloud.points.clear();
    for(unsigned int i=0; i < scan.angles.size(); i++)
    {
        p.x=scan.ranges.at(i)*cos(scan.angles.at(i));
        p.y=scan.ranges.at(i)*sin(scan.angles.at(i));
        p.z=0.0f;
        cloud.points.push_back(p);
    }
}

float Spinner::squaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2, std::vector<pointAssoc*> &assoc){
    float error=0.0f;
    pointAssoc* a;
    for(unsigned int i=0;i<assoc.size();i++)
    {
        a=assoc.at(i);
        error+=compute2DSquaredDistance(cloud1.points.at(a->i),cloud2.points.at(a->j));
    }
    return error;
}

float Spinner::weightedSquaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2, std::vector<pointAssoc*> &assoc){
    float error=squaredErrorEstimation(cloud1,cloud2,assoc);
    error=error/assoc.size();
    return error;
}

float Spinner::compute2DSquaredDistance(geometry_msgs::Point32 src, geometry_msgs::Point32 dst){
    float d=0.0f;
    d=(src.x-dst.x)*(src.x-dst.x)+(src.y-dst.y)*(src.y-dst.y);
    return d;
}


void Spinner::pointcloudToLaserscan(sensor_msgs::PointCloud & cloud,myLaserStructure &scan){

    scan.angles.clear();
    scan.ranges.clear();

    geometry_msgs::Point32 p;
    for(unsigned int i =0;i< cloud.points.size();i++){
        p=cloud.points.at(i);
        scan.angles.push_back(atan2(p.y,p.x));
        scan.ranges.push_back(sqrt(p.x*p.x+p.y*p.y));
    }
}

void Spinner::tranformPointcloud(sensor_msgs::PointCloud & cloud, tf::Transform t){ //UGLY
    //APPLYING A TF TO A POINTCLOUD
    for(unsigned int i=0;i<cloud.points.size();i++){
        geometry_msgs::Point32 p;
        p=cloud.points.at(i);
        tf::Vector3 v;
        v.setX(p.x);
        v.setY(p.y);
        v.setZ(0);
        v=t*v;
        p.x=v.x();
        p.y=v.y();
        cloud.points.at(i)=p;
    }
}

void Spinner::optimization(int iterations,std::vector<pointAssoc*> &assoc){

    int size=assoc.size();
    MatrixXd P(size,3);
    MatrixXd Pt(size,3);
    MatrixXd Z(4,size);
    pointcloudToEigenMatrixWithAssociations(this->cloud1,this->cloud2,P,Pt,assoc);

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

    //DEBUG
    //--------------------------------------
//        IOFormat OctaveFmt( StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
//        std::cout << (P).format(OctaveFmt) <<std::endl;
//        std::cout << (Pt).format(OctaveFmt) <<std::endl;
//        std::cout << (Z).format(OctaveFmt) <<std::endl;
    //--------------------------------------

    Vector3d x(3);
    x<<laser->tx,laser->ty,laser->rz;
    Vector3d result(3);
    result=x;
    std::cout<<"----> Current trasnformation is "<<x<<std::endl;
    pointAlignerLoop(x,Z,iterations,result);
}

void Spinner::pointAlignerLoop( Vector3d &x,  MatrixXd &Z, int iterations,  Vector3d &result)
{
    result=x;
    MatrixXd H(3,3);
    H.setZero();

    MatrixXd b(3,1);
    b.setZero();

    MatrixXd X(3,3);

    for(int i = 0; i < iterations; i++){

        X=v2t(x);
        for(int j=0;j<Z.cols();j++){

            Vector2d e=computeError(j,X,Z);
            MatrixXd J=computeJacobian(j,X,Z);
            //std::cout<<"<><><>"<<j<<"<><><>"<<std::endl<<std::endl;
            //std::cout<<J<<std::endl<<std::endl;
            //std::cout<<J.transpose()*J<<std::endl;
            //std::cout<<"<><><><><><><>"<<std::endl<<std::endl;
            H+=J.transpose()*J;
            b+=J.transpose()*e;
        }
    }
    MatrixXd id(3,3);
    id.setIdentity();
    id*100;
    H+=id;
    Vector3d dx;
    LDLT<MatrixXd> ldlt(-H);
    dx=ldlt.solve(b); // using a LDLT factorizationldlt;
    MatrixXd dX=v2t(dx);
    Vector3d res;
    res.setZero();
    std::cout <<"dx:"<<dx<<std::endl<<std::endl;
    std::cout <<"X: "<<X<<std::endl<<std::endl;
    //MatrixXd transform=dX*X;
    std::cout <<res<<std::endl<<std::endl;
    res=t2v(dX*X);
    std::cout <<res<<std::endl<<std::endl;
    laser->tx=res(0);
    laser->ty=res(1);
    laser->rz=res(2);
    this->laser->setJacobianParameters(res(0),res(1),res(2));
    updateData();
}

Vector2d Spinner::computeError(int i,MatrixXd &X, MatrixXd &Z)
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

MatrixXd Spinner::computeJacobian(int i,Matrix3d X, MatrixXd Z)
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


Matrix3d Spinner::v2t( Vector3d x)
{
    Matrix3d A(3,3);
    double c=cos(x(2));
    double s=sin(x(2));
    A<<c,-s,x(0),s,c,x(1),0,0,1;
    IOFormat OctaveFmt( StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    std::cout << A.format(OctaveFmt)<<std::endl;
    return A;
}

Vector3d Spinner::t2v( Matrix3d x)
{
    Vector3d v;
    v.setZero();
    v(0)=x(0,2);
    v(1)=x(1,2);
    v(2)=atan2(x(1,0),x(0,0));
    return v;
}

void Spinner::pointcloudToEigenMatrix(sensor_msgs::PointCloud &cloud, MatrixXd &m)
{
    //NOT USED
    geometry_msgs::Point32 p;
    m.setOnes();
    for(unsigned int i = 0; i<cloud.points.size();i++){
        p=cloud.points.at(i);
        m(0,i)=p.x;
        m(1,i)=p.y;
    }
}

void Spinner::pointcloudToEigenMatrixWithAssociations(sensor_msgs::PointCloud &cloud1,sensor_msgs::PointCloud &cloud2,  MatrixXd &m1, MatrixXd &m2,std::vector<pointAssoc*> &assoc)
{
    m1.setOnes();
    m2.setOnes();

    pointAssoc* a;
    geometry_msgs::Point32 p;
    for(unsigned int i=0;i<assoc.size();i++)
    {
        a=assoc.at(i);
        p=cloud1.points.at(a->i);
        m1(i,0)=p.x;
        m1(i,1)=p.y;
        p=cloud2.points.at(a->j);
        m2(i,0)=p.x;
        m2(i,1)=p.y;
    }

}

void Spinner::spin() {
    ros::Rate loop(10);
    sleep(1);
    while ( ros::ok() && !shutdown_required ) {
        ros::spinOnce();
        loop.sleep();
    }
}



