#include "laser.h"
#include "ui_laser.h"
#include <iostream>
#include <fstream>
#include <QTime>

using namespace std;


Laser::Laser(QWidget *parent) : QDialog(parent), ui(new Ui::Laser)
{

    ui->setupUi(this);
    //parameters
    a2=0;
    d2=0;
    tx=0;
    ty=0;
    tz=0;
    rz=0;
    //----------
    errorLabel=this->ui->errorLabel;
    f=this->ui->widget;
    this->cloud_LASER=&f->cloud1;
    this->cloud_KINECT=&f->cloud2;
    //this->mapPointer=f->mapPointer;

}


Laser::~Laser(){
    delete ui;
}

//void Laser::clickme(){
//    std::cout<<"click, nothing more, uncomment to see noise"<<std::endl;
////    std::cout << "laser data start dump"<<std::endl;
////    this->_s->dumpLaserData();
////    std::cout << "laser data dumped"<<std::endl;
//}



//DUMP TO FILE
//============================================================================================================
void Laser::on_pushButton_3_clicked()
{
    this->_s->dumpLaserData();
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
    std::cout << "DIAL"<<std::endl;
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


//Optimize C
//============================================================================================================
void Laser::on_pushButton_4_clicked()
{
    std::cout << "OPTIMIZE C"<<std::endl;
    this->_s->solver2.setInitialTransform(Eigen::Vector2d(0,0));
    this->_s->solver2.setIterationsNum(1);
    this->_s->solver2.optimize(this->_s->globalAssoc);
    Vector2d res = this->_s->solver2.getOutputData();
    this->d2=res(0);
    this->a2=res(1);
    this->_s->updateData();
    this->ui->doubleSpinBox_4->setValue(this->d2);
    this->ui->doubleSpinBox_5->setValue(this->a2);
    cout << "Solution "<<res<<endl;
}
//============================================================================================================

void Laser::setJacobianParameters(double x,double y, double rz)
{
    std::cout << "SET JACOBIAN TO 0"<<std::endl;
    this->ui->doubleSpinBox->setValue(x);
    this->ui->doubleSpinBox_2->setValue(y);
    this->ui->angleLabel->setText(QString::number(rz));
    this->ui->dial->setValue(int(rz*1000)+3140);

}
//Data accumulation
void Laser::on_pushButton_2_clicked()
{
    std::cout << "ACCOMULATE"<<std::endl;
    this->_s->putAssInTheBag();
    char c[100];
    sprintf(c,"Accumulate (%u)",(unsigned int)this->_s->globalAssoc.size());
    this->ui->pushButton_2->setText(QString(c));
}

void Laser::on_pushButton_clicked()
{
    std::cout << "RESET"<<std::endl;
    this->_s->globalAssoc.clear();
    this->_s->AllAssoc.clear();
    this->_s->tx=0;
    this->_s->ty=0;
    this->_s->tz=0;
    this->_s->a2=0;
    this->_s->d2=0;
    this->a2=0;
    this->d2=0;
    this->tx=0;
    this->ty=0;
    this->rz=0;
    this->setJacobianParameters(tx,ty,rz);
    this->_s->solver.setInitialTransform(Vector3d(0,0,0));
    this->_s->updateData();
    char c[100];
    sprintf(c,"Accumulate (%u)",(unsigned int)this->_s->globalAssoc.size());
    this->ui->pushButton_2->setText(QString(c));
    this->ui->iterationNum->setValue(1);
}
//OPTIMIZE T

void Laser::on_T_Optimizer_clicked()
{
    std::cout << "T optimization"<<std::endl;
    //this->_s->optimization(this->ui->iterationNum->value(),this->_s->globalAssoc)
    this->_s->solver.setInitialTransform(Eigen::Vector3d(tx,ty,rz));
    this->_s->solver.setIterationsNum(1);
    this->_s->solver.optimize(this->_s->globalAssoc);
    Vector3d res = this->_s->solver.getOutputData();
    this->tx=res.x();
    this->ty=res.y();
    this->rz=res.z();
//    this->ui->doubleSpinBox->setValue(this->tx);
//    this->ui->doubleSpinBox_2->setValue(this->ty);
//    this->ui->dial->setValue(this->rz);
    this->_s->updateData();
}
