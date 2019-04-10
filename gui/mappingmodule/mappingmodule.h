#ifndef MAPPING_MODULE_H
#define MAPPING_MODULE_H



#include <QToolBox>
#include <QWidget>
#include <QThread>
#include <QSplitter>
#include <memory>
#include "moduletools/appmodule.h"
#include "gparam/paramsetwdgt.h"
#include <iostream>
#include "mappingmodule_exports.h"
#include "global_gparams.h"
#include "imageparams.h"
#include "tools/qtgl/glviewer.h"
namespace ucoslam{
class  UcoSlam;
class  Map;
struct  Params;

};
class MapDrawer;
class VideoPlayer;
class APP_MAPPING_MODULE_TOOLS_API  MappingModule: public AppModule {
    Q_OBJECT
public:
    MappingModule(ModuleSetMainWindow *parent);
    std::string getName() const {return "MappingModule";}
    std::string getToolBoxTitle() const {return "";}


public slots:
    void on_activate (  );
    void on_deactivate (  );
    void on_globalaction(const gparam::ParamSet &paramset);
    void on_ControlPanelParamsChanged(int);
    void on_MapParamsChanged(int);
    void updateMapView();

private slots:
    void onParamsOkPressed();
    void on_new_fromvideo();
    void on_openExistingMap();
    void on_saveMap();
    void onNewImage(cv::Mat &image);
    void on_resetMap();

private:

    bool load(const std::string &videoFile,const std::string &cameraFile,  const ucoslam::Params &params,const std::string &vocfile);
    QString getVocabularyPath();


    QDockWidget *_3dDock,*_ParamsDock;
    QSplitter *centralWidget;
    QLabel *_labelImage=0;
     QToolBar *_tbar=0;
    QAction *new_fromvideo_action,*openExisting_action,*saveMap_action,*resetMap_action;
    QToolBox *_tabWidget;
    gparam::ParamSetWdgt *_SLAMParamWdgt,*_MapParamsWdgt;
    gparam::ParamSet _MapParams;
    VideoPlayer *videoPlayer;
    qtgl::GlViewer *mapViewer;
    std::shared_ptr<MapDrawer> mapdrawer;


    std::shared_ptr<ucoslam::UcoSlam> _UcoSLAM;
    std::shared_ptr<ucoslam::Map> _Map;
    ucoslam::ImageParams _imageParams;
    cv::Mat _lastPose;

    float _inputScaleFactor=1;

};









#endif // TSDFTOOLBOX_H
