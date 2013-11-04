#include "laser.h"
#include "ui_laser.h"
#include <iostream>
#include <QTime>
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
