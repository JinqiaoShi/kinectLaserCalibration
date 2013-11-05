#include "laser.h"
#include "ui_laser.h"
#include <iostream>
#include <fstream>
#include <QTime>

using namespace std;


Laser::Laser(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Laser)
{

    ui->setupUi(this);
    QThread *thread = new QThread;
    _w = new Worker;
    _w->laser=this;
    _w->moveToThread(thread);
    thread->start();

    connect(ui->pushButton,SIGNAL(clicked()),SLOT(clickme()));



}

Laser::~Laser()
{
    delete ui;
}

void Laser::clickme()
{
    std::cout<<"click, nothing more, uncomment to see noise"<<std::endl;
    //QMetaObject::invokeMethod(_w, "doWork", Qt::QueuedConnection);
}

void Laser::setImage(QImage *i)
{
    ui->label->setPixmap(QPixmap::fromImage(*i));
}

void Laser::fastSet()
{
    ui->label->setPixmap(QPixmap::fromImage(*(this->_w->i)));
}

void Laser::setImage(QImage i)
{
    ui->label->setPixmap(QPixmap::fromImage(i));
}


void Laser::on_lineEdit_returnPressed()
{
    d2=ui->lineEdit->text().toFloat();
    printf("%f\n",d2);
}

void Laser::on_lineEdit_2_returnPressed()
{
    a2=ui->lineEdit_2->text().toFloat();
    printf("%f\n",a2);
}

void Laser::on_pushButton_3_clicked()
{
    printf("Start dumping to csv\n");
    //LASER FILE
    ofstream myfile;
    myfile.open ("laser.m");
    geometry_msgs::Point32 p;
    for(int i=0;i<cloud_LASER->points.size();i++)
    {
        p=cloud_LASER->points.at(i);
        myfile <<p.x<<" "<<p.y<<endl;
    }
    myfile.close();

    myfile.open ("kinect.m");
    for(int i=0;i<cloud_KINECT->points.size();i++)
    {
        p=cloud_KINECT->points.at(i);
        myfile <<p.x<<" "<<p.y<<endl;
    }
    myfile.close();

}
