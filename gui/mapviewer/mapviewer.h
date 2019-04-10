#ifndef QTMODULEEXAMPLE_MODULE_H
#define QTMODULEEXAMPLE_MODULE_H



#include <QToolBox>
#include <QWidget>
#include <QThread>
#include <memory>
#include "moduletools/appmodule.h"
#include "gparam/paramsetwdgt.h"
#include <iostream>
#include "mapviewer_exports.h"
#include "tools/qtgl/glviewer.h"


class MapDrawer;
 class APP_MODULEEXAMPLE_TOOLS_API  MapViewer: public AppModule {
    Q_OBJECT
public:
    MapViewer(ModuleSetMainWindow *parent=nullptr);
    std::string getName() const {return "3DModuleExample";}
    std::string getToolBoxTitle() const {return "3DQtExample";}


public slots:
    void on_activate (  );
    void on_deactivate (  );
    void on_globalaction(const gparam::ParamSet &paramset);
    void on_showkf_action();
    void on_showgraph_action();
    void on_showpoints_action();
    void on_openmap_action();
    void mousePressed(QMouseEvent *);

private slots:
    void onParamsOkPressed();
private:
    qtgl::GlViewer *_glWindow;
    QAction *openMapAction,*showKFAction,*showGraphAction,*showPointsAction;
    std::shared_ptr<MapDrawer> mapdrawer;


    void readParamSet();


};


#endif // TSDFTOOLBOX_H
