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

#define SCALING 30
using namespace sensor_msgs;
using namespace message_filters;
class Spinner {
public:
    // after : there is the initialization list
    Spinner() :
        shutdown_required(false),
        thread(&Spinner::spin, *this)
    {
        this->i= new QImage(QSize(640,480),QImage::Format_RGB16);
        t1=false;
        t2=false;
    }

    ~Spinner() {
        shutdown_required = true;
        thread.join();
    }



    void callback(const LaserScanConstPtr &l1, const LaserScanConstPtr &l2)
    {
        this->i->fill(qRgb(0,0,0));

//        float aMin=l2->angle_min;
//        float aMax=l2->angle_max;
        float aMin=-0.56f;
        float aMax=0.5f;

        LaserScan newScan;
        newScan.angle_min=aMin;
        newScan.angle_max=aMax;
        newScan.angle_increment=l1->angle_increment;

        newScan.range_max=l1->range_max;
        newScan.range_min=l1->range_min;

        newScan.header=l1->header;

        float angle=l1->angle_min;

        for(int j=0;j<l1->ranges.size();j++)
        {
            angle+=l1->angle_increment;
            if(angle>aMin && angle<aMax)
            {
                newScan.ranges.push_back(l1->ranges.at(j));

            }
        }
        printf("new scan %d\n",newScan.ranges.size());
        printf("kinect scan %d\n",l2->ranges.size());
        sensor_msgs::PointCloud cloud1;
        projector_kinect.transformLaserScanToPointCloud("laser",newScan, cloud1,listener);
        sensor_msgs::PointCloud cloud2;
        projector_laser.transformLaserScanToPointCloud("camera_link",*l2, cloud2,listener);


        for(int j=0;j<cloud1.points.size();j++)
        {
            geometry_msgs::Point32 p=cloud1.points.at(j);
            p.x*=SCALING;
            p.y*=SCALING;
            p.x+=320;
            p.y+=240;
            if(p.x<640 && p.y<480 && p.x>0 && p.y>0){
                this->i->setPixel(int(p.x),int(p.y),qRgb(255,255,255));
            }
        }

        for(int j=0;j<cloud2.points.size();j++)
        {
            geometry_msgs::Point32 p=cloud2.points.at(j);
            p.x*=SCALING;
            p.y*=SCALING;
            p.x+=320;
            p.y+=240;
            if(p.x<640 && p.y<480 && p.x>0 && p.y>0){
                this->i->setPixel(int(p.x),int(p.y),qRgb(255,0,0));
            }
        }

        QMetaObject::invokeMethod(laser,"setImage",Qt::QueuedConnection,Q_ARG(QImage,*i));
    }

    void spin() {
        ros::Rate loop(10);
        sleep(1);
        while ( ros::ok() && !shutdown_required ) {
            ros::spinOnce();
            loop.sleep();
        }
    }

    Laser* laser;
    QImage* i;
private:
    bool shutdown_required;
    ecl::Thread thread;
    laser_geometry::LaserProjection projector_laser;
    laser_geometry::LaserProjection projector_kinect;
    tf::TransformListener listener;
    bool t1;
    bool t2;

};

int main(int argc, char **argv)
{
    std::cout<<"init"<<std::endl;
    QApplication a(argc, argv);
    Laser w;
    std::cout<<"show"<<std::endl;
    ros::init(argc, argv, "laserFlatter");
    ros::NodeHandle n("laserFlatter");
    Spinner s;
    s.laser=&w;
    //    ros::Subscriber sub = n.subscribe("/scan", 1000, &Spinner::laserCallback,&s);
    //    ros::Subscriber sub2 = n.subscribe("/laser_0", 1000, &Spinner::kinectLaserCallback,&s);
    message_filters::Subscriber<LaserScan> l1(n, "/scan", 1000);
    message_filters::Subscriber<LaserScan> l2(n, "/laser_0", 1000);
    typedef sync_policies::ApproximateTime<LaserScan, LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),l1, l2);
    sync.registerCallback(boost::bind(&Spinner::callback,&s, _1, _2));
    w.show();

    return a.exec();
}
