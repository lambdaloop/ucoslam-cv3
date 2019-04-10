#ifndef GLVIEWER_H
#define GLVIEWER_H
#include <QMenu>
#include <QGraphicsView>
#include <unordered_map>
#include <iostream>
#include <memory>
#include "object.h"
#include "qtgl_exports.h"
#include <QToolBar>
#include <QComboBox>
#include <memory>
namespace qtgl{
class GlViewerScene;

/**
  * Viewer for the opengl scene. It handles the qgraphicsitems, while the GlViewerScene handles the gl background
  */
class QTGL_API GlViewer : public QGraphicsView
{
    Q_OBJECT
public:

public:
    GlViewer(QSize size=QSize(-1,-1),QWidget *parent = nullptr);

    //object manipulation
    void  insert ( std::shared_ptr<Object> objs ,std::string name="" );
    std::shared_ptr<Object> find ( std::string name ) ;
    void  erase( std::string name ) ;
    std::unordered_map<std::string,std::shared_ptr<Object> >  & getObjects();
    void  clear ( bool repaint=true ); //clears all data
    void  updateScene();
    qtgl::ViewPoint getViewPoint();

    // get matrix from opengl
//    cv::Mat getViewPointMatrix( );
    //copy to the passed paramters the main gl matrices
    //    void getGlMatrices(GLdouble modelview[16], d projection[16] , GLint viewport[4],int *width=0,int *height=0);
    void getGlMatrices(float modelview[16], float projection[16] , int viewport[4],int *width=0,int *height=0);
    bool unproject(int xscreen,int yscreen, float &x,float &y,float &z);
    bool project(float X,float Y,float Z, int &x,int &y);
    bool getNearestProjection(int xscreen,int yscreen, float &x,float &y,float &z,int maxWsize=std::numeric_limits<int>::max());
    void enableRotationOnClikedPos(bool v){_enableRotationInClickedPos=v;}
    //returns a basic menu with options to set the view(top,front, etc)
    std::vector<std::shared_ptr< QAction> > getViewActions(){return view_menu_actions;}

    QToolBar *getToolBar(){return _tbar;}
    void setViewPoint(ViewPoint vp);
    void showAxis(bool showAxis);
    //grabs the current image buffer

//    cv::Mat getFrameBuffer();
//    cv::Mat getDepthBuffer();
public slots:
    void on_combo_item_activated(QString);
    void slot_toggleAxis(bool);

signals:
    void mousePressed(QMouseEvent *);
    void mouseMoved(QMouseEvent *);
    void viewPointChanged();
    void resized();
protected:
    void resizeEvent(QResizeEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent ( QKeyEvent * event );




private:

    QComboBox *_combo;
    QToolBar *_tbar;
    float ZOOMINC;
    GlViewerScene *_viewportGL; // scene
    // auxiliar variables for mouse handling
    QPoint lastPos;
    bool scenePress; // flag indicating if the mouse has been pressed on the scene
    bool _enableRotationInClickedPos=false;
    std::vector<std::shared_ptr< QAction> > view_menu_actions;
    bool  setCenterOfRotation(int x,int y,int maxWsize);


};
}
#endif // GLVIEWERWRAPPER_H
