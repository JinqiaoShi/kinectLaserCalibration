#ifndef FANCYVIEWER_H
#define FANCYVIEWER_H

#include "spinner.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QDialog>
#include <QThread>
#include "spinner.h"
#include <QLabel>
#include <QTime>
#include <QImage>
#include <unistd.h>
#include <iostream>
#include <QLineEdit>
#include <stdio.h>
#include <QGLViewer/qglviewer.h>
#include "fancyMap.h"

class pointAssoc;

class fancyViewer : public QGLViewer
{
public:
    fancyViewer(QWidget *parent);
    sensor_msgs::PointCloud cloud1;
    sensor_msgs::PointCloud cloud2;
    mymap mapPointer;
private :
    virtual void draw();
    virtual void init();
    void drawPointCloud(sensor_msgs::PointCloud &c, float r, float g, float b);
    void drawAssociations(sensor_msgs::PointCloud &sourceCloud, sensor_msgs::PointCloud &destinationCloud, mymap &fancyMap,float r=1.0f,float g=1.0f, float b=0.0f);

};

#endif

