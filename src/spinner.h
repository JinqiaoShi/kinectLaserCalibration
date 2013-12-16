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
#include "geohelper.h"
#include "leastSquaresSolver.h"
#include "fancyMap.h"
#include "solver.h"
#include "leastSquaresSolver.h"
#include "leastSquaresSolverCalib.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace Eigen;
using namespace std;
class Laser;


class myLaserStructure{
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
    void calibrateLaserRanges(float k1, float k2, myLaserStructure &scan);
    void scanToPointcloud(myLaserStructure &scan, sensor_msgs::PointCloud & cloud);
    float squaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2, mymap &fancyMap);
    float weightedSquaredErrorEstimation(sensor_msgs::PointCloud & cloud1, sensor_msgs::PointCloud & cloud2,  mymap &fancyMap);
    float compute2DSquaredDistance(geometry_msgs::Point32 src, geometry_msgs::Point32 dst);
    void pointcloudToLaserscan(sensor_msgs::PointCloud & cloud,myLaserStructure &scan);
    void tranformPointcloud(sensor_msgs::PointCloud & cloud, tf::Transform t);
    void updateData();
    //void optimization(int iterations,pointpairVec &assoc);
    void pointcloudToEigenMatrix(sensor_msgs::PointCloud &cloud, MatrixXd &m);
    void pointcloudToEigenMatrixWithAssociations(MatrixXd &m1,MatrixXd &m2,pointpairVec &assoc);
    //void pointAlignerLoop(Vector3d &x, MatrixXd &Z, int iterations, Vector3d &result);
    //Vector2d computeError(int i,MatrixXd &X, MatrixXd &Z);
    //MatrixXd computeJacobian(int i,Matrix3d X, MatrixXd Z);
    void putAssInTheBag();
    void spin();
    //TEST
    void dumpLaserData();

    Laser* laser;
    QImage* i;
    float d2;
    float a2;
    float tx;
    float ty;
    float tz;
    float rz;
    pointpairVec globalAssoc;
    pointpairVec AllAssoc;
    mymap fancyMap;
    tf::Transform t;
    LaserScanConstPtr l1;
    LaserScanConstPtr l2;
    genericSolver<leastSquareSolver,pointpairVec,Vector3d> solver;
    genericSolver<leastSquareSolverCalib,pointpairVec,Vector2d> solver2;
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
