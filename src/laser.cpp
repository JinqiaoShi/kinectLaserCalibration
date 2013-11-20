#include "laser.h"
#include "ui_laser.h"
#include <iostream>
#include <fstream>
#include <QTime>

using namespace std;


Laser::Laser(QWidget *parent) : QDialog(parent), ui(new Ui::Laser)
{

    ui->setupUi(this);
    //    QThread *thread = new QThread;
    //    _w = new Worker;
    //    _w->laser=this;
    //    _w->moveToThread(thread);
    //    thread->start();

    connect(ui->pushButton,SIGNAL(clicked()),SLOT(clickme()));
    a2=0;
    d2=0;
    tx=0;
    ty=0;
    tz=0;
    rz=0;
    errorLabel=this->ui->errorLabel;
    f=this->ui->widget;
    this->cloud_LASER=&f->cloud1;
    this->cloud_KINECT=&f->cloud2;
    //this->mapPointer=f->mapPointer;

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



//DUMP TO FILE
//============================================================================================================
void Laser::on_pushButton_3_clicked()
{
//    printf("Start dumping to csv\n");
//    //LASER FILE
//    ofstream myfile;
//    myfile.open ("laser.m");
//    geometry_msgs::Point32 p;
//    for(unsigned int i=0;i<cloud_LASER->points.size();i++)
//    {
//        p=cloud_LASER->points.at(i);
//        myfile <<p.x<<" "<<p.y<<endl;
//    }
//    myfile.close();

//    myfile.open ("kinect.m");
//    for(unsigned int i=0;i<cloud_KINECT->points.size();i++)
//    {
//        p=cloud_KINECT->points.at(i);
//        myfile <<p.x<<" "<<p.y<<endl;
//    }
//    myfile.close();

//    myfile.open ("assoc.m");
//    for(unsigned int i=0;i<this->p->size();i++)
//    {
//        pointAssoc* pa=this->p->at(i);
//        myfile <<pa->i<<" "<<pa->j<<endl;
//    }
//    myfile.close();

}
//============================================================================================================



//TRANSFORM
//============================================================================================================
//RZ
void Laser::on_dial_valueChanged(int value)
{
    rz=((float)(this->ui->dial->value()-3140)/1000);
    this->ui->angleLabel->setText(QString::number(rz));
    this->_s->updateData();

}
//X
void Laser::on_doubleSpinBox_valueChanged(const QString &arg1)
{
    tx=arg1.toFloat();
    this->_s->updateData();
}
//Y
void Laser::on_doubleSpinBox_2_valueChanged(const QString &arg1)
{
    ty=arg1.toFloat();
    this->_s->updateData();
}
//Z
void Laser::on_doubleSpinBox_3_valueChanged(const QString &arg1)
{
    tz=arg1.toFloat();
    this->_s->updateData();
}
//============================================================================================================

//CALIBRATION
//============================================================================================================

void Laser::on_doubleSpinBox_5_valueChanged(const QString &arg1)
{
    a2=arg1.toFloat();
    this->_s->updateData();
}

void Laser::on_doubleSpinBox_4_valueChanged(const QString &arg1)
{
    d2=arg1.toFloat();
    this->_s->updateData();
}
//============================================================================================================


//Optimize
//============================================================================================================
void Laser::on_pushButton_4_clicked()
{
    this->_s->optimization(this->ui->iterationNum->value(),this->_s->globalAssoc);
}
//============================================================================================================

void Laser::setJacobianParameters(double x,double y, double rz)
{

    this->ui->doubleSpinBox->setValue(x);
    this->ui->doubleSpinBox_2->setValue(y);
    this->ui->angleLabel->setText(QString::number(rz));
    this->ui->dial->setValue(int(rz*1000)+3140);

}
//Data accumulation
void Laser::on_pushButton_2_clicked()
{
    this->_s->putAssInTheBag();
    char c[100];
    sprintf(c,"Accumulate (%ud)",(unsigned int)this->_s->globalAssoc.size());
    this->ui->pushButton_2->setText(QString(c));
}

void Laser::on_pushButton_clicked()
{
    this->_s->globalAssoc.clear();
    this->_s->tx=0;
    this->_s->ty=0;
    this->_s->tz=0;
    this->tx=0;
    this->ty=0;
    this->rz=0;
    this->setJacobianParameters(tx,ty,rz);
    char c[100];
    sprintf(c,"Accumulate (%ud)",(unsigned int)this->_s->globalAssoc.size());
    this->ui->pushButton_2->setText(QString(c));
    this->ui->iterationNum->setValue(1);
}
