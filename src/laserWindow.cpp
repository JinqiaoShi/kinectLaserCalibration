#include "laserWindow.h"
#include "ui_dialog.h"
#include <iostream>
Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    std::cout<<"setup"<<std::endl;
}

Dialog::~Dialog()
{
    delete ui;
}
