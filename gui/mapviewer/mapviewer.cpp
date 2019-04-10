#include <QSplitter>
#include "mapviewer.h"
#include "moduletools/appparams.h"
#include <iostream>
#include <ucoslam/map.h>
#include <QtGui/QOpenGLFunctions>
#include <QFileDialog>
#include <QMessageBox>
#include <QMouseEvent>
#include "mapdrawer.h"
using namespace std;

MapViewer::MapViewer(ModuleSetMainWindow *parent ):AppModule (parent) {


    _glWindow=new qtgl::GlViewer();
    _glWindow->enableRotationOnClikedPos(true);
    connect(_glWindow,SIGNAL(mousePressed(QMouseEvent *)),this,SLOT(mousePressed(QMouseEvent *)));

    //register the elements created
    setCentralWidget(_glWindow);
    setIcon(QPixmap ( QString:: fromUtf8 ( ":/images/program_icon.ico" ) ));
    auto toolbar=_glWindow->getToolBar();




    openMapAction= new QAction ( QIcon ( ":/images/open.png" ), tr ( "&Load..." ), this );
    connect(openMapAction,SIGNAL(triggered()),this,SLOT(on_openmap_action( )));
    toolbar->addAction(openMapAction);

    showPointsAction= new QAction ( QIcon ( ":/images/points.png" ), tr ( "&Points..." ), this );
    connect(showPointsAction,SIGNAL(triggered()),this,SLOT(on_showpoints_action( )));
    toolbar->addAction(showPointsAction);
    showPointsAction->setCheckable(true);
    showPointsAction->setChecked(true);

    showKFAction= new QAction ( QIcon ( ":/images/camera.png" ), tr ( "&KeyFrames..." ), this );
    connect(showKFAction,SIGNAL(triggered()),this,SLOT(on_showkf_action( )));
    toolbar->addAction(showKFAction);
    showKFAction->setCheckable(true);
    showKFAction->setChecked(true);

    showGraphAction= new QAction ( QIcon ( ":/images/graph.png" ), tr ( "&Graph..." ), this );
    connect(showGraphAction,SIGNAL(triggered()),this,SLOT(on_showgraph_action( )));
    toolbar->addAction(showGraphAction);
    showGraphAction->setCheckable(true);
    showGraphAction->setChecked(true);



    setToolBar(toolbar);
    //    setControlPanel(_tbox);

    std::shared_ptr< ucoslam::Map > map=std::make_shared<ucoslam::Map>();
    mapdrawer= std::make_shared<MapDrawer>( );
    mapdrawer->set(map);
    _glWindow->insert(mapdrawer,"Map");

}

void MapViewer::mousePressed(QMouseEvent *mev){
}

void MapViewer::on_openmap_action(){
        QSettings settings;
        QString file = QFileDialog::getOpenFileName (
                    0,
                    tr ( "Select a map " ),
                    settings.value ( "saveMap" ).toString(),
                    tr ( "Open Map file (*.map)" ) );
        if ( file==QString() ) return;

        settings.setValue ( "saveMap",QFileInfo ( file ).absolutePath() );



        try{
            std::shared_ptr< ucoslam::Map > map=std::make_shared<ucoslam::Map>();
            map->readFromFile(file.toStdString());
            mapdrawer->set(map);
            _glWindow->updateScene();

        }catch(std::exception &ex){
            QMessageBox::critical ( _glWindow,tr ( "Error" ),tr ( "Could not load  map from file:" )+file);
            return;
        }
}



void MapViewer::onParamsOkPressed(){

}

void MapViewer::on_showkf_action(){
    mapdrawer->showKeyFrames(showKFAction->isChecked());
    _glWindow->updateScene();
}
void MapViewer::on_showgraph_action(){
    mapdrawer->showGraph(showGraphAction->isChecked());
    _glWindow->updateScene();
}
void MapViewer::on_showpoints_action(){
    mapdrawer->showPoints(showPointsAction->isChecked());
    _glWindow->updateScene();
}

void MapViewer::on_activate (  ) {
    // getControlPanel()->show();
    getToolBar()->show();

}


void MapViewer::on_deactivate (  ){

}


void MapViewer::on_globalaction(const gparam::ParamSet &paramset){
}


