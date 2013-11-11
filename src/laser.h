#ifndef LASER_H
#define LASER_H
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
#include "fancyViewer.h"

class Worker;
class Spinner;
class fancyViewer;

namespace Ui {
    class Laser;
}


class Laser : public QDialog
{
    Q_OBJECT

public:
    explicit Laser(QWidget *parent = 0);
    ~Laser();
    //Worker* _w; //test, delete me
    Spinner* _s;
    float d2;
    float a2;
    sensor_msgs::PointCloud* cloud_LASER;
    sensor_msgs::PointCloud* cloud_KINECT;
    fancyViewer* f;

protected slots:
    void clickme();
    void setImage(QImage *i);
    void setImage(QImage i);
    //void fastSet();
private slots:

    void on_lineEdit_returnPressed();
    void on_lineEdit_2_returnPressed();
    void on_pushButton_3_clicked();
    void on_plus_d_clicked();
    void on_minus_d_clicked();
    void on_minus_a_clicked();
    void on_plus_a_clicked();


private:
    Ui::Laser *ui;



};


//class Worker : public QObject
//{
//    Q_OBJECT
//private:
//    int fd;
//public slots:
//    void doWork() {
//        this->i= new QImage(QSize(640,480),QImage::Format_RGB888);
//        while(1){
//            qsrand(QTime::currentTime().msec());
//            this->i->fill(qRgb(0,0,0));
//            for(int j=0;j<640;j++)
//            {
//                for(int k=0;k<480;k++)
//                {
//                    if(qrand()%5)
//                    {
//                        this->i->setPixel(j,k,qRgb(255,255,255));
//                    }
//                }
//            }

//            //QMetaObject::invokeMethod(laser,"setImage",Qt::QueuedConnection,Q_ARG(QImage,*i));
//            QMetaObject::invokeMethod(laser,"fastSet",Qt::QueuedConnection);
//            usleep(10000);
//        }
//    }
//public:
//    QImage *i;
//    Laser* laser;
//};


#endif
