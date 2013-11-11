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

using namespace sensor_msgs;
using namespace message_filters;
class Laser;

class pointAssoc
{
public:
    pointAssoc(unsigned int a,unsigned int b){
        i=a;
        j=b;
    }

    unsigned int i;
    unsigned int j;
};

class Spinner {
public:
    Spinner();
    ~Spinner();
    void callback(const LaserScanConstPtr &l1, const LaserScanConstPtr &l2);
    void spin();

    Laser* laser;
    QImage* i;
    float d2;
    float a2;
    std::vector<pointAssoc*> assoc;
private:
    bool shutdown_required;
    ecl::Thread thread;
    laser_geometry::LaserProjection projector_laser;
    laser_geometry::LaserProjection projector_kinect;
    tf::TransformListener listener;
    bool t1;
    bool t2;

};


#endif
