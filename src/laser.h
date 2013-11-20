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
#include "fancyMap.h"
class Worker;
class Spinner;
class fancyViewer;
class pointAssoc;

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
    float tx;
    float ty;
    float tz;
    float rz;
    QLabel* errorLabel;
    sensor_msgs::PointCloud* cloud_LASER;
    sensor_msgs::PointCloud* cloud_KINECT;
    mymap mapPointer;
    fancyViewer* f;
    void setJacobianParameters(double x,double y, double rz);
protected slots:
    void clickme();

private slots:


    void on_pushButton_3_clicked();
    void on_dial_valueChanged(int value);
    void on_doubleSpinBox_valueChanged(const QString &arg1);
    void on_doubleSpinBox_2_valueChanged(const QString &arg1);
    void on_doubleSpinBox_3_valueChanged(const QString &arg1);
    void on_doubleSpinBox_5_valueChanged(const QString &arg1);
    void on_doubleSpinBox_4_valueChanged(const QString &arg1);

    void on_pushButton_4_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_clicked();

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
