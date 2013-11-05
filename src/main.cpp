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
#include "spinner.h"

#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QApplication>
#include "laser.h"
#include "laserWindow.h"
#include <ecl/threads/thread.hpp>

using namespace sensor_msgs;
using namespace message_filters;

int main(int argc, char **argv)
{
    std::cout<<"init"<<std::endl;
    QApplication a(argc, argv);
    Laser w;
    std::cout<<"show"<<std::endl;
    ros::init(argc, argv, "laserFlatter");
    ros::NodeHandle n("laserFlatter");
    Spinner s;
    w._s=&s;
    s.laser=&w;
    message_filters::Subscriber<LaserScan> l1(n, "/scan", 1000);
    message_filters::Subscriber<LaserScan> l2(n, "/laser_0", 1000);
    typedef sync_policies::ApproximateTime<LaserScan, LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),l1, l2);
    sync.registerCallback(boost::bind(&Spinner::callback,&s, _1, _2));
    w.show();

    return a.exec();
}
