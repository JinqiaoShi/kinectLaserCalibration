#ifndef SPINNER_H
#define SPINNER_H

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QApplication>
#include "laser.h"
#include "laserWindow.h"
#include <ecl/threads/thread.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Cholesky>

using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;
class Laser;

class pointAssoc
{
public:
    pointAssoc(unsigned int a,unsigned int b,float c){
        i=a;
        j=b;
        d=c;
    }

    unsigned int i;
    unsigned int j;
    float d;
};

class myLaserStructure
{
    public:
        std::vector<float> angles;
        std::vector<float> ranges;
};

class Spinner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //eigen sciaize

    Spinner();
    ~Spinner();
    void callback(const LaserScanConstPtr &l1, const LaserScanConstPtr &l2);
    void LaserScanCleaner(const sensor_msgs::LaserScanConstPtr &src, myLaserStructure &dst);
    void findScansAssociations(myLaserStructure &scan1, myLaserStructure &scan2, float threshold);
    void cleanAssociations(std::vector<pointAssoc*> &assoc);
    void calibrateLaserRanges(float k1, float k2, myLaserStructure &scan);
    void scanToPointcloud(myLaserStructure &scan, sensor_msgs::PointCloud & cloud);
    float squaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2, std::vector<pointAssoc*> &assoc);
    float weightedSquaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2, std::vector<pointAssoc*> &assoc);
    float compute2DSquaredDistance(geometry_msgs::Point32 src, geometry_msgs::Point32 dst);
    void pointcloudToLaserscan(sensor_msgs::PointCloud & cloud,myLaserStructure &scan);
    void tranformPointcloud(sensor_msgs::PointCloud & cloud, tf::Transform t);
    void updateData();
    void optimization(int iterations,std::vector<pointAssoc*> &assoc);
    void pointcloudToEigenMatrix(sensor_msgs::PointCloud &cloud, MatrixXd &m);
    void pointcloudToEigenMatrixWithAssociations(sensor_msgs::PointCloud &cloud1,sensor_msgs::PointCloud &cloud2, MatrixXd &m1,MatrixXd &m2,std::vector<pointAssoc*> &assoc);
    void pointAlignerLoop(Vector3d &x, MatrixXd &Z, int iterations, Vector3d &result);
    Matrix3d v2t(Vector3d x);
    Vector2d computeError(int i,MatrixXd &X, MatrixXd &Z);
    MatrixXd computeJacobian(int i,Matrix3d X, MatrixXd Z);
    Vector3d t2v( Matrix3d x);
    void putAssInTheBag();
    void spin();

    Laser* laser;
    QImage* i;
    float d2;
    float a2;
    float tx;
    float ty;
    float tz;
    float rz;
    std::vector<pointAssoc*> assoc;
    std::vector<pointAssoc*> globalAssoc;
    tf::Transform t;
    LaserScanConstPtr l1;
    LaserScanConstPtr l2;
private:
    bool shutdown_required;
    ecl::Thread thread;
    sensor_msgs::PointCloud cloud1;
    sensor_msgs::PointCloud cloud2;
    myLaserStructure laser1;
    myLaserStructure laser2;
    tf::TransformListener listener;

};


#endif
