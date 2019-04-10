#include <QSplitter>
#include "mappingmodule.h"
#include "moduletools/appparams.h"
#include "videoplayer/videoplayer.h"
#include "tools/qtgl/glviewer.h"
#include "tools/gparam/paramsetdlg.h"
#include <iostream>
#include <QDockWidget>
#include <QMainWindow>
#include <QMessageBox>
#include "ucoslam.h"
#include <QInputDialog>
#include <QFileInfo>
#include <QTemporaryFile>
#include "mapviewer/mapdrawer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <QToolBox>
#include <QCoreApplication>
#include <thread>
#include "downloadvocabularydialog.h"
using namespace std;



class MappingFromFileParams: public UcoSlamGParams{
public:
    MappingFromFileParams(){
        vector<gparam::Param>::insert(begin(),gparam::Param("Video file",gparam::Param::EXISTING_FILEPATH));
        find("Video file").setDescription("Video file (*.mp4 *.MP4 *.avi *.AVI *.mov *.MOV *.wmv *.WMV *.flv *.FLV *.webm *.WEBM *.mkv *.MKV *.ogg *.OGG *.3gp *.3GP)");
        vector<gparam::Param>::insert(begin()+1,gparam::Param("Camera Parameters File",gparam::Param::EXISTING_FILEPATH));
        find("Camera Parameters File").setDescription("Camera Parameters Files (*.yml)");
     }

};
void     createMapParams(gparam::ParamSet &PSet){
    PSet.insert(gparam::Param("Show Axis", true));
    PSet.back().setDescription("Hide/show 3D Axis");
    PSet.insert(gparam::Param("Scale Factor", float(1.0),0.01,10.0,0.05));
    PSet.back().setDescription("Scale factor of the map");
    PSet.insert(gparam::Param("KeyFrames", true));
    PSet.back().setDescription("Hide/show KeyFrames");
    PSet.insert(gparam::Param("KeyFrame Scale", float(1.0),0.1,10.0,0.05));
    PSet.back().setDescription("Scale of the KeyFrames");
    PSet.insert(gparam::Param("Graph", false));
    PSet.back().setDescription("Hide/show Graph");
    PSet.insert(gparam::Param("Map Points", true));
    PSet.back().setDescription("Hide/show Map Points");
    PSet.insert(gparam::Param("Markers", true));
    PSet.back().setDescription("Hide/show Map Markers");

}

MappingModule::MappingModule(ModuleSetMainWindow *parent):AppModule(parent) {
    //    centralWidget=new QSplitter(0);
    videoPlayer=new VideoPlayer(0);
    connect(videoPlayer,&VideoPlayer::newImage,this,&MappingModule::onNewImage);

    //centralWidget->addWidget(videoPlayer);

    QMainWindow *mw=(QMainWindow*)parent;
    _3dDock=new QDockWidget("3D Map");
    mapViewer=new qtgl::GlViewer(QSize(320,240));
     mapViewer->enableRotationOnClikedPos(true);
    mapdrawer=std::make_shared<MapDrawer>();
    mapViewer->insert(mapdrawer,"Map");
    float m16[]={-0.999933,-0.00175108,0.0104996,0,0.00599066,-0.907942,0.419036,0,0.00880166,0.419077,0.907896,0,0,0,0,1};

    qtgl::ViewPoint viewPoint;
    viewPoint.setMatrix(m16);
    mapViewer->setViewPoint(viewPoint);

    createMapParams(_MapParams);
    _MapParamsWdgt=new gparam::ParamSetWdgt ( &_MapParams,0,QDialogButtonBox::NoButton );
    connect(_MapParamsWdgt,SIGNAL( paramChanged(int)),this,SLOT(on_MapParamsChanged(int)));
    auto w3d=new QWidget();

    auto l3d=new QHBoxLayout();
    l3d->addWidget(mapViewer);
    l3d->addWidget(_MapParamsWdgt);
    w3d->setLayout(l3d);


    //_3dDock->setLayout(l3d);
    _3dDock->setWidget(w3d);
    mw->addDockWidget ( Qt::TopDockWidgetArea, _3dDock );
    _3dDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);

    _tbar=new QToolBar ( getName().c_str() );
    new_fromvideo_action= new QAction ( QIcon ( ":/images/movie.png" ), tr ( "New from &Video..." ), this );
    connect(new_fromvideo_action,SIGNAL(triggered()),this,SLOT(on_new_fromvideo( )));
    _tbar->addAction(new_fromvideo_action);

    openExisting_action= new QAction ( QIcon ( ":/images/open.png" ), tr ( "&Open map..." ), this );
    connect(openExisting_action,SIGNAL(triggered()),this,SLOT(on_openExistingMap( )));
    _tbar->addAction(openExisting_action);


    resetMap_action= new QAction ( QIcon ( ":/images/reset.png" ), tr ( "&Reset map..." ), this );
    connect(resetMap_action,SIGNAL(triggered()),this,SLOT(on_resetMap( )));
    _tbar->addAction(resetMap_action);

    saveMap_action= new QAction ( QIcon ( ":/images/save.png" ), tr ( "&Save map..." ), this );
    connect(saveMap_action,SIGNAL(triggered()),this,SLOT(on_saveMap( )));
    _tbar->addAction(saveMap_action);
    saveMap_action->setEnabled(false);



    _Map=std::make_shared<ucoslam::Map>();


    //register the elements created
    setCentralWidget( videoPlayer );
    setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/mapping.png" ) ));
    setToolBar(_tbar);

    _tabWidget=new QToolBox((QWidget*)parent );
    _SLAMParamWdgt=new gparam::ParamSetWdgt ( UcoSlamGParamsRunTime( ),0,QDialogButtonBox::NoButton );
    connect(_SLAMParamWdgt,SIGNAL(paramChanged(int)),this,SLOT(on_ControlPanelParamsChanged(int)));
    _tabWidget->addItem(_SLAMParamWdgt,tr("SLAM Params"));

    setControlPanel(_tabWidget);
    getControlPanel()->hide();
}

void MappingModule::on_MapParamsChanged(int){
    updateMapView();
}
void MappingModule::on_resetMap(){
    if(_Map->keyframes.size()!=0){
        if( QMessageBox::question( (QWidget*)parent(),tr ( "Reset Map" ),tr ( "Are you sure you want to reset the current map?" ),QMessageBox::Yes|QMessageBox::No)==QMessageBox::No ) return;
        _UcoSLAM.reset();
        _Map=std::make_shared<ucoslam::Map>();
        videoPlayer->reset();
//        _UcoSLAM->clear();
        _lastPose=cv::Mat();
        updateMapView();
//        videoPlayer->retrieveAndShow();
    }
}
void MappingModule::on_saveMap(){
    if(!_Map)return;

    std::thread waitThread;
    if(_UcoSLAM)
        waitThread= std::thread ([&](){_UcoSLAM->waitForFinished();});


    QSettings settings;
    QString dir=settings.value ( "saveMap" ).toString();
    if( dir.isEmpty())dir=settings.value ( "currDir" ).toString();
    QString file = QFileDialog::getSaveFileName (
                0,
                tr ( "Save the map " ),
                settings.value ( "saveMap" ).toString(),
                tr ( "Map file (*.map)" ) );


    if(waitThread.joinable())
        waitThread.join();

    if ( file==QString() ) return;

    settings.setValue ( "saveMap",QFileInfo ( file ).absolutePath() );

    try{
        _Map->saveToFile(file.toStdString());
    }catch(std::exception &ex){
        QMessageBox::critical ( (QWidget*)parent(),tr ( "Error" ),tr ( "Could not save to file:" )+file);
        return;
    }
}
void  MappingModule::updateMapView( ){


    mapdrawer->setScaleFactor(_MapParams["Scale Factor"].get<float>());
    mapdrawer->showKeyFrames( _MapParams["KeyFrames"].get<bool>());
    mapdrawer->showPoints( _MapParams["Map Points"].get<bool>());
    mapdrawer->showGraph(  _MapParams["Graph"].get<bool>());
    mapdrawer->setKeyFrameScaleFactor(_MapParams["KeyFrame Scale"].get<float>());


    mapViewer->showAxis( _MapParams["Show Axis"].get<bool>());
    mapdrawer->set(_Map,_lastPose);
    mapViewer->updateScene();
}

void MappingModule::on_openExistingMap()
{

    if(_UcoSLAM)_UcoSLAM->waitForFinished();

    {
        QSettings settings;
        QString file = QFileDialog::getOpenFileName (
                    0,
                    tr ( "Select a map " ),
                    settings.value ( "saveMap" ).toString(),
                    tr ( "Open Map file (*.map)" ) );
        if ( file==QString() ) return;

        settings.setValue ( "saveMap",QFileInfo ( file ).absolutePath() );



        try{
            _Map=std::make_shared<ucoslam::Map>();
            _Map->readFromFile(file.toStdString());
            _UcoSLAM.reset();
            updateMapView();

        }catch(std::exception &ex){
            QMessageBox::critical ( parent(),tr ( "Error" ),tr ( "Could not load  map from file:" )+file);
            return;
        }
    }
    return;

}
QString MappingModule::getVocabularyPath(){
    QSettings settings;
    QString path=settings.value("vocabularypath").toString();

    if(!path.isEmpty()) return path;

#ifdef WIN32
      return QCoreApplication::applicationDirPath()+"/orb.fbow";
#else
      return "/usr/share/ucoslam/orb.fbow";
#endif
}

void MappingModule::on_new_fromvideo(){

    if(_Map->keyframes.size()!=0){
           _inputScaleFactor=1.;
            gparam::ParamSet params("Open Video");
            params.insert(gparam::Param("Video file",gparam::Param::EXISTING_FILEPATH));
            params.back().setDescription("Video file (*.mp4 *.MP4 *.avi *.AVI *.mov *.MOV *.wmv *.WMV *.flv *.FLV *.webm *.WEBM *.mkv *.MKV *.ogg *.OGG *.3gp *.3GP)");

            params.insert(gparam::Param("Camera Parameters File",gparam::Param::EXISTING_FILEPATH));
            params.back().setDescription("Camera Parameters Files (*.yml)");
            gparam::ParamSetDialog PSDlg(&params,QDialogButtonBox::Ok | QDialogButtonBox::Cancel );
            QSettings settings;
            PSDlg.setFileDialogPath(settings.value ( "currDir" ).toString());
            if(  PSDlg.exec()!=QDialog::Accepted) return;
            if(params.find("Video file").asString().empty() || params.find("Camera Parameters File").asString().empty())  {
                QMessageBox::information(parent(),tr("Missing Information"),tr("Video file or camera file not specified"));
                return;
            }
            settings.setValue ( "currDir",QFileInfo (  params.find("Video file").asString().c_str() ).absolutePath() );


            ucoslam::ImageParams imageParams;
            try {
                _imageParams.readFromXMLFile(params["Camera Parameters File"].asString());
            } catch (std::exception &ex) {
                QMessageBox::critical((QWidget*)parent(),tr("Load Error"),tr("Unable to load camera parameters file"));
                return ;
            }

            _UcoSLAM=std::make_shared<ucoslam::UcoSlam>();
            ucoslam::Params ucoslamparams;
            UcoSlamGParamsRunTime rtp;
            rtp.update(ucoslamparams);
            rtp["Mode"]="Track Only";
            _UcoSLAM->setMode(ucoslam::MODE_LOCALIZATION);
            if(_Map->map_markers.size()==0) ucoslamparams.detectMarkers=false;
            else
                ucoslamparams.aruco_Dictionary= _Map->map_markers.begin()->second.dict_info;

            _UcoSLAM->setParams(_Map,ucoslamparams);
            auto videoFile=params["Video file"].asString();
            if(!videoPlayer->openVideoFilePath(videoFile.c_str())){
                _UcoSLAM.reset();
                return;
            }
            videoPlayer->setMaxSpeed(true);
            videoPlayer->setProcessImageWithArucoDetector(false);
            _SLAMParamWdgt->setParamSet(rtp);
            getControlPanel()->show();

    }
 else{

    //show the params
    MappingFromFileParams mfp;
    mfp.setName("Mapping from Video");
    gparam::ParamSetDialog PSDlg(&mfp,QDialogButtonBox::Ok | QDialogButtonBox::Cancel );
    QSettings settings;
    PSDlg.setFileDialogPath(settings.value ( "currDir" ).toString());


    if(  PSDlg.exec()!=QDialog::Accepted) return;
    if(mfp.find("Video file").asString().empty() || mfp.find("Camera Parameters File").asString().empty())  {
        QMessageBox::information(parent(),tr("Missing Information"),tr("Video file or camera file not specified"));
        return;
    }

    settings.setValue ( "currDir",QFileInfo (  mfp.find("Video file").asString().c_str() ).absolutePath() );

    _inputScaleFactor=mfp["Img Resize Fact"].get<float>();



    //creates the vocabulary temp  file
    auto voc_tmp=QDir::tempPath();
    QFileInfo voc_filepath=getVocabularyPath();

    if(!voc_filepath.isFile()   ){
        if( QMessageBox::question(parent(),tr("Missing File"),tr("Vocabulary file has not been found. In order to do relozalization in the map, you need a vocabulary file. Do you want to download it from internet now?"))==QMessageBox::Yes)
        {
            auto path=QStandardPaths::standardLocations(QStandardPaths::AppDataLocation);
            QFileInfo fsave=path.at(0)+"/orb.fbow";
            QDir dirpath(path.at(0));
            if(!dirpath.exists()){
                if( !dirpath.mkpath(path.at(0))){
                    QMessageBox::critical(parent(),tr("Access error"),tr("The path where to save the vocabulary is not writable:")+path.at(0));
                    QString file = QFileDialog::getSaveFileName (
                                parent(),
                                tr ( "Select a path for the vocabulary" ),
                                settings.value ( "currDir" ).toString(),
                                tr ( "Vocabulary file (*.voc)" ) );
                    if (!file.isEmpty()) fsave=QFileInfo(file);
                }
            }

            auto Dialog=new DownloadVocabularyDialog(QUrl("http://rabinf24.uco.es/in1musar/ucoslam/vocabularies/orb.fbow"),fsave.absoluteFilePath());
            Dialog->setWindowTitle(tr("Download"));
            if(Dialog->exec()==QDialog::Accepted) {
                if(Dialog->succeeded()){
                    settings.setValue ( "vocabularypath",fsave.absoluteFilePath() );
                    voc_filepath=fsave.absoluteFilePath();
                }
                else{
                    QMessageBox::critical(parent(),tr("Download Error"),Dialog->errorMsg());
                }
            }
        }
    }

    ucoslam::Params params;
    mfp.update(params);
    load(mfp.find("Video file").asString(),mfp.find("Camera Parameters File").asString(),params,  voc_filepath.absoluteFilePath().toStdString() );


   _SLAMParamWdgt->setParamSet(UcoSlamGParamsRunTime(params));
   getControlPanel()->show();
    }
    _lastPose=cv::Mat();
//   _ControlPanel->show();

}


void MappingModule::onNewImage(cv::Mat &image){
    if(!_UcoSLAM || !_Map)return;
    if(_Map->keyframes.size()!=0)
        saveMap_action->setEnabled(true);
    if(_UcoSLAM){
        if( fabs(_inputScaleFactor-1)>1e-3){
            cv::Mat resizedImage;
            ucoslam::ImageParams imageParams=_imageParams;
            cv::Size newSize=cv::Size(image.cols*_inputScaleFactor,image.rows*_inputScaleFactor);
            cv::resize(image,resizedImage,newSize);
            imageParams.resize(newSize);
            _lastPose= _UcoSLAM->process(resizedImage,imageParams,videoPlayer->getFrameIndex());
            resizedImage.copyTo(image);
        }
        else{//no resize
            _lastPose= _UcoSLAM->process(image,_imageParams,videoPlayer->getFrameIndex());
        }
        updateMapView();
    }
}

bool MappingModule::load(const std::string &videoFile,const std::string &cameraFile,    const ucoslam::Params &params,const std::string &vocfile){
    ucoslam::ImageParams imageParams;
    try {
        _imageParams.readFromXMLFile(cameraFile);
    } catch (std::exception &ex) {
        QMessageBox::critical((QWidget*)parent(),tr("Load Error"),tr("Unable to load camera parameters file"));
        return false;
    }

    _UcoSLAM=std::make_shared<ucoslam::UcoSlam>();
    _UcoSLAM->setParams(_Map,params,vocfile);
    if(!videoPlayer->openVideoFilePath(videoFile.c_str())){
        _UcoSLAM.reset();
        return false;
    }
    videoPlayer->setMaxSpeed(true);
    videoPlayer->setProcessImageWithArucoDetector(false);

    return true;
}


void MappingModule::onParamsOkPressed(){

}
void MappingModule::on_ControlPanelParamsChanged(int idx){
    if(!_UcoSLAM)return;
    ucoslam::Params ucoslamparams=_UcoSLAM->getParams();
    UcoSlamGParamsRunTime gparams( _SLAMParamWdgt->getParamSet());
    gparams.update(ucoslamparams);

     _UcoSLAM->updateParams(ucoslamparams);
     ucoslam::MODES SLAMMODE;
     if(gparams["Mode"].asString()=="SLAM")
         SLAMMODE=ucoslam::MODE_SLAM;
     else
         SLAMMODE=ucoslam::MODE_LOCALIZATION;
     _UcoSLAM->setMode(SLAMMODE);

     videoPlayer->retrieveAndShow();
 }
void MappingModule::on_activate (  ) {
    if(_UcoSLAM)
       getControlPanel()->show();
    else
        getControlPanel()->hide();
    getToolBar()->show();
    _3dDock->show();

}


void MappingModule::on_deactivate (  ){
    _3dDock->hide();
     getControlPanel()->hide();
  //  _ParamsDock->hide();

}


void MappingModule::on_globalaction(const gparam::ParamSet &paramset){
}
