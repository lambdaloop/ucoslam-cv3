#include "progresswindow.h"
#include "ui_progresswindow.h"
#include <iostream>
progresswindow::progresswindow(std::string name,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::progresswindow)
{
    ui->setupUi(this);
    _name=name;
    _isExpanded=false;
    ui->progressBar->setStyleSheet(ui->progressBar->property("defaultStyleSheet").toString() +
                                       " QProgressBar::chunk { background: green; border-top-right-radius: 5px; border-bottom-right-radius: 5px; border-bottom-left-radius: 5px;border-top-left-radius: 5px; border: 1px solid black; }");

}

progresswindow::~progresswindow()
{
    delete ui;
}

void progresswindow::notify_action_progress(int v,std::string message){
    ui->progressBar->setValue(v<0?-v:v);
    setToolTip(tr(message.c_str()));
//    if (v>=100){
//        ui->progressBar->setStyleSheet(ui->progressBar->property("defaultStyleSheet").toString() +
//                                           " QProgressBar::chunk { background: green; border-top-right-radius: 5px; border-bottom-right-radius: 5px; border-bottom-left-radius: 5px;border-top-left-radius: 5px; border: 1px solid black; }");
//    }
    if (v<0){
    ui->progressBar->setStyleSheet(ui->progressBar->property("defaultStyleSheet").toString() +
                                       " QProgressBar::chunk { background: red; border-top-right-radius: 5px; border-bottom-right-radius: 5px; border-bottom-left-radius: 5px;border-top-left-radius: 5px; border: 1px solid black; }");
    }
}

/*
 * void progresswindow::on_pushButton_clicked()
{
    if (!_isExpanded){
        ui->label->show();
        ui->pushButton->setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/arrow-left-small.png" ) ));
    }
    else {
        ui->label->hide();
        ui->pushButton->setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/arrow-right-small.png" ) ));
        }

    _isExpanded=!_isExpanded;
}
*/
