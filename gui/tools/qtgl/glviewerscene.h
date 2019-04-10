/*! \file GlViewer.h
    \brief Qt OpenGL Widget to view 3D RGB Point Clouds. Modification for AnatomicPoints Software
 */
#ifndef _IHS_GLVIEWERSCENE
#define _IHS_GLVIEWERSCENE

#include <QGraphicsScene>
#include <unordered_map>
#include "viewpoint.h"
#include "object.h"
#include "transform.h"
#include <memory>
#include "qtgl_exports.h"

namespace qtgl{


/**
  * QGraphicsScene that display the OpenGL viewer in its background
  */
class  GlViewerScene : public QGraphicsScene
{
    Q_OBJECT

public:

    GlViewerScene ( int x, int y, int w, int h );

 //object manipulation
    void  insert ( std::shared_ptr<Object> objs ,std::string name="" ) ;
    std::shared_ptr<Object> find ( std::string name ) ;
    void  erase( std::string name ) ;
    std::unordered_map<std::string,std::shared_ptr<Object> >  & getObjects(){return _vObjects;}
    void  clear ( bool repaint=true ); //clears all data
    void  updateScene(){update();}


    ViewPoint getViewPoint(){return viewPoint;}
    void setViewPoint(ViewPoint v){viewPoint=v; }
    bool project(float X,float Y,float Z, int &x,int &y);

    bool getNearestProjection(uint32_t px,uint32_t py,float &x,float &y,float &z ,int maxWsize);

    bool unproject(int xscreen,int yscreen, float &x, float &y, float &z);
    void showAxis(bool s){_showAxis=s;updateScene();}

    bool setCenterOfRotation(int x,int y,int maxWsize);
  private:


    ViewPoint viewPoint;
    void drawAxis();     /// Axis Rendering
    void drawBackground(QPainter *painter, const QRectF &);

    std::unordered_map<std::string,std::shared_ptr<Object> > _vObjects;
    std::string getRandomString ( size_t size );

    bool _showAxis;

};
};

#endif
