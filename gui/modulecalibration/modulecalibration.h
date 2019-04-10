#ifndef ModuleCalibration_MODULE_H
#define ModuleCalibration_MODULE_H



#include <QToolBox>
#include <QWidget>
#include <QThread>
#include <memory>
#include "moduletools/appmodule.h"
#include "gparam/paramsetwdgt.h"
#include "videoplayer/videoplayer.h"
#include "aruco/aruco.h"
#include "modulecalibration_exports.h"

#include "calibrationcontrolpanel.h"
#include <iostream>
 class APP_MODULE_CALIBRATION_TOOLS_API  ModuleCalibration: public AppModule {
    Q_OBJECT
public:
    ModuleCalibration(ModuleSetMainWindow *parent=nullptr);
    std::string getName() const {return "Calibration";}
    std::string getToolBoxTitle() const {return "Calibration";}


public slots:
    void on_global_action(const gparam::ParamSet &paramset);
    void on_NewImage(cv::Mat &img);

private slots:
    void on_deactivate (  );
    void on_activate();
     void on_vplayer_opened();
    void on_addCurrent();
    void on_addAll();
    void on_downloadPattern();
private:


    QAbstractButton *Btn_addCurImage,*Btn_addAllImages,*Btn_gotoNextFrame;
    QAction *act_GetCalibPattern;
    VideoPlayer *vplayer=0;
    calibrationControlPanel *CalibPanel;
    aruco::MarkerDetector MDetector;
     vector<aruco::Marker> markers;

};


#endif // TSDFTOOLBOX_H
