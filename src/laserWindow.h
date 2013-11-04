#ifndef DIALOG_H
#define DIALOG_H
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <QDialog>



namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
     explicit Dialog(QWidget *parent = 0);
    ~Dialog();


private:
    Ui::Dialog *ui;

};


#endif
