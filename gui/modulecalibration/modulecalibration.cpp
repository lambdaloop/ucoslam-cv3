#include <QSplitter>
#include <QTimer>
#include <QFile>
#include "modulecalibration.h"
#include "moduletools/appparams.h"
#include <iostream>
#include "global_gparams.h"

using namespace std;
ModuleCalibration::ModuleCalibration(ModuleSetMainWindow *parent) :AppModule (parent){

    aruco::MarkerDetector::Params params;
    MDetector.setDictionary("ARUCO_MIP_36h12");

    vplayer=new VideoPlayer();
    connect(vplayer,SIGNAL(openedImageOrVideo()),this,SLOT(on_vplayer_opened()));
    connect(vplayer,SIGNAL(newImage(cv::Mat &)),this,SLOT(on_NewImage(cv::Mat &)));

    Btn_addCurImage = new QPushButton(tr("&Add Current..."));
    connect(Btn_addCurImage, &QAbstractButton::clicked, this, &ModuleCalibration::on_addCurrent);
    Btn_addCurImage->setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/plus.png" )));
    Btn_addCurImage->hide();
    vplayer->addButton(Btn_addCurImage);

     act_GetCalibPattern=new QAction ( QIcon ( ":/images/download.png" ), tr ( "&Download Calibration Pattern..." ), this );
     connect(act_GetCalibPattern,&QAction::triggered,this,&ModuleCalibration::on_downloadPattern);

    QToolBar *tbar=new QToolBar("Calibration");
    for(auto a:vplayer->getActions())
        tbar->addAction(a);
    tbar->addAction(act_GetCalibPattern);


    Btn_gotoNextFrame=new QPushButton(tr("&Next Image..."));
    connect(Btn_gotoNextFrame, &QAbstractButton::clicked, vplayer, &VideoPlayer::playNextFrame);
    Btn_gotoNextFrame->setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/arrow-right-small.png" )));
    Btn_gotoNextFrame->hide();
    vplayer->addButton(Btn_gotoNextFrame);

    Btn_addAllImages=new QPushButton(tr("&Add All..."));
    connect(Btn_addAllImages, &QAbstractButton::clicked, this, &ModuleCalibration::on_addAll);
    Btn_addAllImages->setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/done.png" )));
    Btn_addAllImages->hide();
    vplayer->addButton(Btn_addAllImages);




    //register the elements created
    setCentralWidget(vplayer);
    setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/calibration.png" ) ));
    setToolBar(tbar);
    CalibPanel=new calibrationControlPanel();
    setControlPanel(CalibPanel);

}
void ModuleCalibration::on_NewImage(cv::Mat &imIn){

     markers=MDetector.detect(imIn);

    for(auto m:markers)
        m.draw(imIn,cv::Scalar(0,0,255),-1,false);
 }

void ModuleCalibration::on_downloadPattern(){

    QSettings settings;
    QString file = QFileDialog::getSaveFileName (
                0,
                tr ( "Save the Pattern file" ),
                settings.value ( "currDir" ).toString(),
                tr ( "Pattern File (*.pdf)" ) );

    if(file.isEmpty())return;
    settings.setValue ( "currDir",QFileInfo ( file ).absolutePath() );

    if(file.indexOf(".pdf")==-1)
        file+=".pdf";

    QFile filePdf(":/aruco_calibration_grid_board_a4.pdf");
    filePdf.copy(file);

}


void ModuleCalibration::on_addCurrent(){

    if( markers.size()==0)return;
    CalibPanel->add(vplayer->getShownImage(),markers,QString("Frame: #")+vplayer->getCurrentImageInfo().c_str() );
    vplayer->playNextFrame();
}

void ModuleCalibration::on_addAll(){

    if( markers.size()!=0)
        CalibPanel->add(vplayer->getShownImage(),markers,QString("Frame: #")+vplayer->getCurrentImageInfo().c_str() );
    if(    vplayer->playNextFrame())
         QTimer::singleShot(30,this,SLOT(on_addAll()));
}


void ModuleCalibration::on_vplayer_opened(){
    markers.clear();
    Btn_addCurImage->show();
    Btn_gotoNextFrame->show();
    if(vplayer->getType()==VideoPlayer::TYPE_VIDEO)
        Btn_addAllImages->hide();
    if(vplayer->getType()==VideoPlayer::TYPE_LIVE){
        Btn_addAllImages->hide();
        Btn_gotoNextFrame->hide();

    }

    if(vplayer->getType()==VideoPlayer::TYPE_IMAGE)
        Btn_addAllImages->show();
}



void ModuleCalibration::on_deactivate (  ){
    vplayer->on_deactivate();
}
void ModuleCalibration::on_activate (  ){
    vplayer->on_activate();
}


void ModuleCalibration::on_global_action(const gparam::ParamSet &paramset){
    if (paramset.getName()=="arucoParamsChanged")
        vplayer->updateImage();
}


