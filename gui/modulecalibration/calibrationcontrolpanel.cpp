#include "calibrationcontrolpanel.h"
#include "ui_calibrationcontrolpanel.h"
#include <QListWidgetItem>
#include <QMessageBox>
#include <QSettings>
#include <QFileInfo>
#include <QFileDialog>
#include "global_gparams.h"
#include "tools/moduletools/appparams.h"
#include "tools/gparam/paramsetwdgt.h"
#include <QToolBox>
#include <iostream>
calibrationControlPanel::calibrationControlPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::calibrationControlPanel)
{
    ui->setupUi(this);
    shownImage=new QLabel(0);

    //ARUCO PARAMS AND DOCK
    try{
    AppParams::readFromSettings(ArucoGParams::get() );
    }catch(std::exception &ex){
        std::cerr<<ex.what()<<std::endl;
    }

    //        auto _arucoDock=new QDockWidget (  "Aruco Params" );
    //        _arucoDock->setWidget (  _arucoWdgt);
    //        addDockWidget ( Qt::LeftDockWidgetArea, _arucoDock );

}

calibrationControlPanel::~calibrationControlPanel()
{
    delete ui;
}

void calibrationControlPanel::on_pb_images_remove_clicked()
{
    if (ui->listWidget->currentItem()==nullptr) return;
    if ( QMessageBox::question ( this, tr ( "Remove Image" ), tr ( "Remove the selected image?" ), QMessageBox::Yes,QMessageBox::No ) ==QMessageBox::Yes ){
        int ci=ui->listWidget->currentRow();
        ui->listWidget->takeItem(ci);

        if (ImagesMarkers.size()==1) ImagesMarkers.clear();
        else {
            auto it=ImagesMarkers.begin();
            advance(it,ci);
            ImagesMarkers.erase(it);
        }
        shownImage->hide();
      }
}


void calibrationControlPanel::on_pb_images_clear_clicked()
{
    if(ui->listWidget->count()==0) return;
    if ( QMessageBox::question ( this, tr ( "Remove All Images" ), tr ( "Remove all the images?" ), QMessageBox::Yes,QMessageBox::No ) ==QMessageBox::Yes ){
        ImagesMarkers.clear();
        ui->listWidget->clear();
        shownImage->hide();
      }
}

void calibrationControlPanel::add(const cv::Mat &image,const std::vector<aruco::Marker> &markers,QString info)
{

    //check image size as the previous ones
    for(auto &elem:ImagesMarkers){
        if (image.size()!=elem.image.size()){
            QMessageBox::critical ( this,tr ( "Error" ),tr ( "Image size differs from previous ones" ));
            return;
        }
    }

    //if the image is already, replace it
    for(auto &elem:ImagesMarkers){
        if ( elem.info==info){
            elem={image.clone(),markers,info};
            return;
        }
    }


    ui->listWidget->addItem(info);
    ImagesMarkers.push_back( {image.clone(),markers,info});

}

void calibrationControlPanel::on_listWidget_itemDoubleClicked(QListWidgetItem *item)
{
    int ci=ui->listWidget->currentRow();
    if(ci<0)return;
    auto elem=ImagesMarkers.begin();
    std::advance(elem,ci);
    shownImage->setWindowTitle(elem->info);
    shownImage->setScaledContents(true);
    QImage _qimgR ( ( const uchar * ) ( elem->image.ptr<uchar> ( 0 ) ),
                    elem->image.cols,elem->image.rows, QImage::Format_RGB888 ) ;

    shownImage-> setPixmap ( QPixmap::fromImage ( _qimgR.rgbSwapped() ) );
    shownImage->show();


}
aruco::CameraParameters aruco_cameraCalibrate(std::vector<std::vector<aruco::Marker> >  &allMarkers, int imageWidth,int imageHeight,float markerSize,float *currRepjErr,bool isFishEye=false);

void calibrationControlPanel::on_pb_compute_clicked()
{
    if(ImagesMarkers.size()==0)return;

    std::vector<std::vector<aruco::Marker> >  allMarkers;
    for(const auto &elem:ImagesMarkers)
        allMarkers.push_back(elem.markers);
    float currRepjErr=0;
      camParams=aruco_cameraCalibrate(allMarkers, ImagesMarkers.front().image.cols,ImagesMarkers.front().image.rows, 1,&currRepjErr);
    std::stringstream sstr;
    sstr<<tr("Reprojection Error:").toStdString()<<currRepjErr<<"pix"<<std::endl<<std::endl;
    ui->pte_results->clear();
    ui->pte_results->insertPlainText(sstr.str().c_str());
    std::stringstream sstr2;
    sstr2<<camParams;
    ui->pte_results->insertPlainText(sstr2.str().c_str());


}

void calibrationControlPanel::on_pb_saveCalibration_clicked()
{
    if(!camParams.isValid()) return;
    QSettings settings;
    QString filepath = QFileDialog::getSaveFileName(
                this,
                tr ( "Select an output file" ),
                settings.value ( "currDir" ).toString()+"/calibration.yml",
                tr ( "YALM File (*.yml)" ) );
    if ( filepath==QString() ) return;
    settings.setValue ( "currDir",QFileInfo ( filepath ).absolutePath() );
    QFileInfo qfile(filepath);
    if (qfile.completeSuffix()!="yml")
        filepath+=".yml";
    try{
        camParams.saveToFile(filepath.toStdString());
    }catch(std::exception &ex){
        QMessageBox::critical ( this,tr ( "Error" ),tr ( "Could not save file " )+filepath );
    }

}
