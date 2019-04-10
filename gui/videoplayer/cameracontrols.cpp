#include "cameracontrols.h"
#include "ui_cameracontrols.h"
#include <QString>
cameracontrols::cameracontrols(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::cameracontrols)
{
    ui->setupUi(this);
}

cameracontrols::~cameracontrols()
{
    delete ui;
}
void cameracontrols::set(std::shared_ptr<GenericImageReader> reader){
    int h=    reader->get(CV_CAP_PROP_FRAME_HEIGHT);
    int w=    reader->get(CV_CAP_PROP_FRAME_WIDTH);
    int index=reader->getSource().at(0).toInt();
    ui->spinBox->setValue(index);
    string currResolution=std::to_string(w)+"x"+std::to_string(h);
    ui->comboBox->blockSignals(true);
    ui->comboBox->clear();
    ui->comboBox->addItem( "320x240");
    ui->comboBox->addItem( "640x480");
    ui->comboBox->addItem( "800x600");
    ui->comboBox->addItem( "960x720");
    ui->comboBox->addItem( "1024x768");
    ui->comboBox->addItem( "1280x720");
    ui->comboBox->addItem( "1920x1080");

    int idx=ui->comboBox->findText(currResolution.c_str());
    if( idx>=0){
        ui->comboBox->setCurrentIndex(idx);
    }
    else{
        ui->comboBox->addItem(currResolution.c_str());
        ui->comboBox->setCurrentIndex( ui->comboBox->count()-1);
    }

    ui->comboBox->blockSignals(false);

}

void cameracontrols::on_pushButton_clicked()
{
    emit reconnect( ui->spinBox->value(), ui->comboBox->currentText());

}

void cameracontrols::on_comboBox_currentIndexChanged(int index)
{
 //emit resolutionChanged(ui->comboBox->itemText(index));
}
