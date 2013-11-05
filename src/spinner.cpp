#include "spinner.h"
#define SCALING 30



Spinner::Spinner() :
    shutdown_required(false),
    thread(&Spinner::spin, *this)
{
    this->i= new QImage(QSize(640,480),QImage::Format_RGB16);
    t1=false;
    t2=false;
    d2=0;
    a2=0;
}

Spinner::~Spinner() {
    shutdown_required = true;
    thread.join();
}
//===================================================================0



void Spinner::callback(const LaserScanConstPtr &l1, const LaserScanConstPtr &l2)
{
    this->i->fill(qRgb(0,0,0));

    //      float aMin=l2->angle_min;
    //      float aMax=l2->angle_max;
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

    //TEST
    float k1=0.0;
    float kAngle=l2->angle_min;
    float k2=10;
    LaserScan prova=*l2;
    d2=laser->d2;
    a2=laser->a2;
    printf("d2 %f a2 %f \n",d2,a2);
    for(int k=0;k<l2->ranges.size();k++)
    {
        kAngle +=l2->angle_increment;

        if(l2->ranges.at(k)!=-1) {

            if(kAngle<0.6)
                prova.ranges.at(k)=l2->ranges.at(k)+l2->ranges.at(k)*l2->ranges.at(k)*d2*kAngle*kAngle*a2;
            if(kAngle>0.6)
                prova.ranges.at(k)=l2->ranges.at(k)+l2->ranges.at(k)*l2->ranges.at(k)*d2*(kAngle-6.28)*(kAngle-6.28)*a2;
        }
        else {
            prova.ranges.at(k)=0;
        }
    }


    sensor_msgs::PointCloud cloud1;
    projector_kinect.transformLaserScanToPointCloud("laser",newScan, cloud1,listener);
    sensor_msgs::PointCloud cloud2;
    projector_laser.transformLaserScanToPointCloud("camera_link",prova, cloud2,listener);

    laser->cloud_LASER=&cloud1;
    laser->cloud_KINECT=&cloud2;

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

void Spinner::spin() {
    ros::Rate loop(10);
    sleep(1);
    while ( ros::ok() && !shutdown_required ) {
        ros::spinOnce();
        loop.sleep();
    }
}

