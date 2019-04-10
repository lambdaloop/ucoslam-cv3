#include "glviewer.h"
#include "glviewerscene.h"
#include <QGLWidget>
#include <QGraphicsEllipseItem>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QComboBox>
#include <fstream>
using namespace std;
namespace qtgl{



#define VIEWPOINT_SEND_SIGNALS_MS 500
/**
  */
GlViewer::GlViewer(QSize size, QWidget *parent)
{
    // create gl viewport for the viewer
    QGLWidget *widget = new QGLWidget(QGLFormat(QGL::SampleBuffers));
    setViewport(widget);
    widget->makeCurrent();
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    setMouseTracking(true);
    // create scene
    if( size.width()==-1|| size.height()==-1)size=QSize(2048,1024);
    _viewportGL = new GlViewerScene(0,0,size.width(),size.height());
    setScene(_viewportGL);
    // set some graphical parameters
    setRenderHints( QPainter::Antialiasing ); // antialising makes the program slower
    this->setOptimizationFlag(QGraphicsView::DontClipPainter, true);
    this->setOptimizationFlag(QGraphicsView::DontSavePainterState, true);
    this->scene()->setItemIndexMethod(QGraphicsScene::NoIndex);
    this->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    ZOOMINC= 0.05f;
     // toolbar actions
    view_menu_actions.push_back(std::make_shared<QAction>(QIcon ( ":/qtgl/icons/axis3d.png" ),"show/hide axis",(QObject*)(0)));
    view_menu_actions.back().get()->setCheckable(true);
    view_menu_actions.back().get()->setChecked(true);
    connect(view_menu_actions.back().get(),SIGNAL(toggled(bool)),this,SLOT(slot_toggleAxis(bool)));
    _viewportGL->showAxis(true);


    _combo=new QComboBox();
    _combo->addItem(QIcon ( ":/qtgl/icons/top.png" ),"top");
//    _combo->addItem(QIcon ( ":/qtgl/icons/bottom.png" ),"bottom");
    _combo->addItem(QIcon ( ":/qtgl/icons/front.png" ),"front");
    _combo->addItem(QIcon ( ":/qtgl/icons/leftlateral.png" ),"left side");
    _combo->addItem(QIcon ( ":/qtgl/icons/rightlateral.png" ),"right side");
    connect(_combo,SIGNAL(activated(QString)),this,SLOT(on_combo_item_activated(QString)));
    _tbar=new QToolBar();


    for(size_t i=0;i<view_menu_actions.size();i++)
        _tbar->addAction(view_menu_actions[i].get());
    _tbar->addWidget(_combo);
 }

void	GlViewer::keyPressEvent ( QKeyEvent * event ){
    if (event->key()==Qt::Key_L){
        _combo->setCurrentIndex(_combo->findText("left side"));
        on_combo_item_activated("left side");
    }
    else if (event->key()==Qt::Key_T){
        _combo->setCurrentIndex(_combo->findText("top"));
        on_combo_item_activated("top");
    }
    else if (event->key()==Qt::Key_F){
        _combo->setCurrentIndex(_combo->findText("front"));
        on_combo_item_activated("front");
    }
    else if (event->key()==Qt::Key_R){
        _combo->setCurrentIndex(_combo->findText("right side"));
        on_combo_item_activated("right side");
    }
    //cerr<<"GlViewer::keyPressEvent "<<endl;
}

void GlViewer::setViewPoint(ViewPoint vp){
    _viewportGL->setViewPoint(vp);
    _viewportGL->updateScene();

}
void GlViewer::on_combo_item_activated(QString str){
    ViewPoint vp=_viewportGL->getViewPoint();
    if (str=="top"){
        vp.eye = qtgl::Point3f(0,0,-3);
        float m[16]={0, 0, 1, 0,    0, 1, 0, 0,   -1 ,0, 0, 0 ,0, 0, 0, 1};
        memcpy(vp.rotationMatrix,m,16*sizeof(float));
    }
    else   if (str=="front"){
        vp.eye=Point3f (0, 0, -4 );
        float m[16]={0, -1, 0, 0, 1, 0, 0, 0,  0 ,0, 1, 0 ,0, 0, 0, 1};
        memcpy(vp.rotationMatrix,m,16*sizeof(float));
    }else if (str=="left side") {
        vp.eye = qtgl::Point3f(0,0,-4);
        float m[16]={0, -1, 0, 0,  0,0,1,0,  -1, 0, 0, 0,      0, 0, 0, 1};
        memcpy(vp.rotationMatrix,m,16*sizeof(float));
    }
    else if (str=="right side") {
        vp.eye = qtgl::Point3f(0,0,-4);
        float m[16]={0, -1, 0, 0,  0,0,-1,0,  1, 0, 0, 0,      0, 0, 0, 1};
        memcpy(vp.rotationMatrix,m,16*sizeof(float));
    }
    _viewportGL->setViewPoint(vp);
    _viewportGL->updateScene();
}

void GlViewer::slot_toggleAxis(bool state)
{

    _viewportGL->showAxis(state);

}
void GlViewer::showAxis(bool showAxis){

    _viewportGL->showAxis(showAxis);
}

/**
  */
//cv::Mat GlViewer::getViewPointMatrix() {

//    ViewPoint vp=_viewportGL->getViewPoint();
//    cv::Mat m(4,4,CV_32F);
//    for(int i=0; i<16; i++) m.ptr<float>(0)[i] = vp.rotationMatrix[i];
//    return m;
//}


/**
  */
void GlViewer::resizeEvent(QResizeEvent *event) {
    // if there is a scene assigned, resize the scene too
    if (scene())  scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
    //_viewportGL->resizeGL(this->width(), this->height()); // resize the opengl part of the scene
     // place items according to current point of view (as size has changed, point view has changed too)
    QGraphicsView::resizeEvent(event);
    emit resized();
}

void GlViewer::wheelEvent(QWheelEvent *event) {
    // update items and viewpoint
    int x = event->delta() /120;
    _viewportGL->setViewPoint( _viewportGL->getViewPoint() .translate ( 0,0, x*ZOOMINC   ));
 //   _viewportGL->viewPoint.translate ( t );

    this->scene()->update(); // make sure the scene is rendered
    emit viewPointChanged();

 }


/**
  */
void GlViewer::mousePressEvent(QMouseEvent *event) {
//err<<"Must check this 131klk"<<endl;
//    // if press over a item, use the standard qgraphicsview event handler to handle the qgraphicsitems
//    if(this->scene()->itemAt(this->mapToScene(event->pos()))!=NULL)
//        QGraphicsView::mousePressEvent(event);
//    // else, apply to opengl
//    else {
//        lastPos = event->pos();
//        emit mousePressed(event);
//    }
    if(_enableRotationInClickedPos)
       _viewportGL->setCenterOfRotation(event->pos().x(),event->pos().y(),30);

    lastPos = event->pos();
    emit mousePressed(event);
}


/**
  */
void GlViewer::mouseReleaseEvent(QMouseEvent *event) {
  //  scenePress = false; // unset flag
    QGraphicsView::mouseReleaseEvent(event);
}


/**
  */
void GlViewer::mouseMoveEvent(QMouseEvent *event) {
     if (event->modifiers()==Qt::NoModifier) {
         float dx = ( float ) ( event->pos().x() - lastPos.x() );
         float dy = ( float ) ( event->pos().y() - lastPos.y() );

         // if this flag is true, we are interacting with the gl scene
         if ( event->buttons() & Qt::LeftButton ) {
             _viewportGL->setViewPoint( _viewportGL->getViewPoint().doXRot ( -dy*0.1 ));
             _viewportGL->setViewPoint( _viewportGL->getViewPoint().doYRot ( dx*0.1 ));
         }
         else if ( event->buttons() & Qt::RightButton ) // zoom scene
         {
             _viewportGL->setViewPoint(_viewportGL->getViewPoint().goForward ( -dy*0.0025 ));
         }
         else if ( event->buttons() & Qt::MidButton ) // move scene
         {
             //translate
             _viewportGL->setViewPoint( _viewportGL->getViewPoint().translate (   dx*0.0025,dy*0.0025 ,0 ));
         }

         this->scene()->update();
         emit viewPointChanged();
     }
    //methods to move the objects
//     else if (event->modifiers()==Qt::ShiftModifier &&    getObjects().size()!=0){
//         float dy = ( float ) ( event->pos().y() - lastPos.y() );
//         float dx = ( float ) ( event->pos().x() - lastPos.x() );
//         ViewPoint vp=_viewportGL->getViewPoint();
//         if ( event->buttons() & Qt::LeftButton ) {
//             //move the first object if any
//             for(auto it:getObjects()){
//                 it.second->setTransform( it.second->getTransform().doXRot(vp,-dy*0.1));
//                 it.second->setTransform( it.second->getTransform().doYRot(vp,dx*0.1 ));
//             }
//         }
//         else if(event->buttons() & Qt::MidButton ){
//             for(auto it:getObjects())
//                 it.second->setTransform(it.second->getTransform().translate(vp, -dx*0.0025,-dy*0.0025 ,0));
//         }

//         else if ( event->buttons() & Qt::RightButton ) // zoom scene
//         {
//             for(auto it:getObjects())
//                 it.second->setTransform(it.second->getTransform().goForward(vp,-dy*0.0025));
//         }
//         this->scene()->update();
//     }
// else just call the normal QGraphicsView method to handle the qgraphicsitems
     else QGraphicsView::mouseMoveEvent(event);

     lastPos = event->pos(); // update
     emit mouseMoved(event);
}

//cv::Mat GlViewer::getDepthBuffer(){
//    glFlush();



//    cv::Mat im(height(),width(),CV_32FC1);
//    glReadPixels(0, 0, width(), height(), GL_DEPTH_COMPONENT, GL_FLOAT, im.ptr<uchar>(0));

//      cv::Mat im2;
//    cv::flip(im,im2,0);
//    return im2;

//}

//cv::Mat GlViewer::getFrameBuffer(  ){
//    glFlush();



//    cv::Mat im(height(),width(),CV_8UC4);
//    glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_BYTE, im.ptr<uchar>(0));

//    cv::Mat im2;
//    cv::cvtColor(im,im2,CV_RGBA2BGR);
//    cv::Mat im3;
//    cv::flip(im2,im3,0);
//    // return success
//    return im3;
//}

void GlViewer::getGlMatrices(float modelview[16],  float projection[16] ,  int viewport[4],int *width ,int *height ){
    // get modelview,projection and viewport data from gl
    _viewportGL->getViewPoint().set(); // set current viewpoint matrixes

    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
    _viewportGL->getViewPoint().unset(); // unset current viewpoint
    if (width!=0)
    *width=this->width();
    if (height!=0)
    *height=this->height();

}



/////////

//object manipulation
void  GlViewer::insert ( std::shared_ptr<Object> objs ,std::string name  ) {_viewportGL->insert(objs,name);}
std::shared_ptr<Object> GlViewer::find ( std::string name ) {return _viewportGL->find( name);}
void  GlViewer::erase( std::string name ) { _viewportGL->erase(name);}
std::unordered_map<std::string,std::shared_ptr<Object> >  & GlViewer::getObjects(){return _viewportGL->getObjects();}
void  GlViewer::clear ( bool repaint ){_viewportGL->clear(repaint);} //clears all data}
void  GlViewer::updateScene(){_viewportGL->updateScene();}
bool  GlViewer::unproject(int xscreen,int yscreen,float &x,float &y,float &z){return _viewportGL->unproject(xscreen,yscreen,x,y,z);}

bool  GlViewer::getNearestProjection(int xscreen,int yscreen, float &x,float &y,float &z,int maxWsize){return _viewportGL->getNearestProjection(xscreen,yscreen,x,y,z,maxWsize);}
bool GlViewer::setCenterOfRotation(int x,int y,int maxWsize) {    return _viewportGL->setCenterOfRotation(x,y,maxWsize);}


bool  GlViewer::project(  float X, float  Y, float  Z,  int  &x, int &y) {return _viewportGL->project(X,Y,Z,x,y);}
qtgl::ViewPoint GlViewer::getViewPoint(){return _viewportGL->getViewPoint();}

}
