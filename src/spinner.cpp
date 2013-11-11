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
    //    float aMin=-0.56f;
    //    float aMax=0.5f;

    LaserScan newScan=*l1;

    //TEST
    float kAngle=l2->angle_min;

    LaserScan prova=*l2;
    for(unsigned int k=0;k<prova.ranges.size();k++)
    {
        if(prova.ranges.at(k)==-1)
        {
            prova.ranges.at(k)=5.0f;
        }
    }
    d2=laser->d2;
    a2=laser->a2;

    for(unsigned int k=0;k<l2->ranges.size();k++)
    {
        kAngle +=l2->angle_increment;

        if(l2->ranges.at(k)!=-1) {

            if(kAngle<0.6)
                prova.ranges.at(k)=l2->ranges.at(k)+l2->ranges.at(k)*l2->ranges.at(k)*d2*kAngle*kAngle*a2;
            if(kAngle>0.6)
                prova.ranges.at(k)=l2->ranges.at(k)+l2->ranges.at(k)*l2->ranges.at(k)*d2*(kAngle-6.28)*(kAngle-6.28)*a2;
        }
        else {
            prova.ranges.at(k)=0.1f;
        }
    }


    sensor_msgs::PointCloud cloud1;
    projector_kinect.transformLaserScanToPointCloud("laser",newScan, cloud1,listener);
    sensor_msgs::PointCloud cloud2;
    projector_laser.transformLaserScanToPointCloud("camera_link",prova, cloud2,listener);


    laser->f->cloud1=cloud1;
    laser->f->cloud2=cloud2;

    //DRAWING THE POINTCLOUD IN THE QIMAGE PLANAR PLANE
    //============================================================
    for(unsigned int j=0;j<cloud1.points.size();j++)
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

    for(unsigned int j=0;j<cloud2.points.size();j++)
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
    //============================================================

    //FINDING ASSOCIATIONS
    this->assoc.clear();
    laser->f->cloud1=cloud1;
    laser->f->cloud2=cloud2;
    float threshold=0.001f;
    for(unsigned int j=0;j<prova.ranges.size();j++)
    {
        for(unsigned int k=0;k<newScan.ranges.size();k++)
        {
            float kinectScanAngle=prova.angle_min+j*prova.angle_increment;
            float laserScanAngle=newScan.angle_min+k*newScan.angle_increment;
            if(laserScanAngle>=(kinectScanAngle-threshold) && laserScanAngle<=(kinectScanAngle+threshold))
            {
                if(prova.ranges.at(j)!=0.1f)
                    assoc.push_back(new pointAssoc(k,j));
            }
        }
    }
    laser->f->p=assoc;
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

