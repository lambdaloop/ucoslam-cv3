/*! \file GlViewer.cpp
    \brief Qt OpenGL Widget to view 3D RGB Point Clouds
 */



#include "glviewerscene.h"


#ifdef WIN32
//#include <GL/glew.h>
#else
#include "glextensions.h"
#include <QtOpenGL>
#endif
#include <QGLWidget>
#include <iostream>
#include "utils.h"
namespace qtgl{
/**
  */
GlViewerScene::GlViewerScene ( int x, int y, int w, int h ) {
     setSceneRect(x, y, w, h); // scene size

     _showAxis=true;

//#ifdef WIN32
//        GLenum err =  glewInit();
//        if ( GLEW_OK != err )
//            cerr<<"Error: "<< glewGetErrorString ( err ) <<endl;
//#else
//     getGLExtensionFunctions().resolve (  );
//#endif
}


void  GlViewerScene::insert ( std::shared_ptr<Object> objs ,std::string name ) {

     if ( name.empty() ) name=getRandomString ( 30 );
    _vObjects.erase(name);
    _vObjects.insert ( std::make_pair ( name,objs ) );
    updateScene();
}

std::shared_ptr<Object> GlViewerScene::find ( std::string name ) {
    std::unordered_map<std::string,std::shared_ptr<Object> >::iterator obj=_vObjects.find(name);
    if (obj==_vObjects.end()) return std::shared_ptr<Object>();
    else return obj->second;
}

void  GlViewerScene::erase( std::string name ) {
     _vObjects.erase(name);
      updateScene();
}


void GlViewerScene:: clear ( bool doRepaint ) {
    _vObjects.clear();
     if ( doRepaint )   updateScene();
}


/**
  */
void GlViewerScene::drawBackground(QPainter *painter, const QRectF &)
{

    auto Perspective=[]( GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar )
    {
        GLdouble xmin, xmax, ymin, ymax;

        ymax = zNear * tan( fovy * 3.14159 / 360.0 );
        ymin = -ymax;
        xmin = ymin * aspect;
        xmax = ymax * aspect;

        glFrustum( xmin, xmax, ymin, ymax, zNear, zFar );
    };

   // cerr<<"drawBackground"<<endl;
    if (painter->paintEngine()->type() != QPaintEngine::OpenGL
            && painter->paintEngine()->type() != QPaintEngine::OpenGL2)
    {
        qWarning("OpenGLScene: drawBackground needs a QGLWidget to be set as viewport on the graphics view");
        return;
    }

    float width = float(painter->device()->width());
    float height = float(painter->device()->height());

    painter->beginNativePainting();



//    glClearDepth(1.0f);
    glViewport ( 0, 0, ( GLint ) width, ( GLint ) height );
    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity();
    float fov=30.0;
    float zNear=0.001,zFar=100;
    float ratio=double ( width ) /float(height);


    Perspective(fov, ratio, zNear, zFar);


    glMatrixMode ( GL_MODELVIEW );
    glLoadIdentity();


  //  glClearColor(0.2, 0.2, 0.2, 1.0f);
    glClearColor(1., 1.0, 1.0, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    viewPoint.set();

    // before drawing grids and axis, disable depthmask
    // since we dont want to pick these elements when using gluUnProject
    //show grids

    glEnable(GL_DEPTH_TEST);
    glDepthMask( GL_TRUE );

    vector<std::shared_ptr<Object> > obj_list;
   // cerr<<"................."<<endl;
    for ( std::unordered_map<std::string,std::shared_ptr<Object> >::iterator obj=_vObjects.begin(); obj!=_vObjects.end(); obj++ ){
        obj_list.push_back(obj->second);
     }

    //sort by priority
    std::sort(obj_list.begin(),obj_list.end(),[](std::shared_ptr<Object> a,std::shared_ptr<Object> b)->bool{return a->getPriority()>b->getPriority();});
    for (size_t i=0;i<obj_list.size();i++){
        obj_list[i]->draw();

    }

    ///////////////////////////

   if(_showAxis) drawAxis();
   glDepthMask ( GL_FALSE ); // finally disable it
    glDisable(GL_DEPTH_TEST);
    viewPoint.unset();
   // glDisable(GL_NORMALIZE);
   // glDisable(GL_BLEND);
    //glFlush();

//viewPoint.print();
    painter->endNativePainting();
}






/**
  */
void GlViewerScene::drawAxis()
{
  float limit = 10.0f;
  float limitAxe = 0.3f;

    glLineWidth ( 1.0 );
    glBegin ( GL_LINES );
    {

      glColor4f ( 0.10, 0.10, 0.10, 0.5 );

      glVertex3f ( -limit, 0, 0 );
      glVertex3f ( 0, 0, 0 );
      glVertex3f ( limitAxe, 0, 0 );
      glVertex3f ( limit, 0, 0 );

      glVertex3f ( 0, -limit,  0 );
      glVertex3f ( 0, 0, 0 );
      glVertex3f ( 0, limitAxe, 0 );
      glVertex3f ( 0, limit,  0 );

      glVertex3f ( 0, 0, -limit );
      glVertex3f ( 0, 0, 0 );
      glVertex3f ( 0, 0, limitAxe );
      glVertex3f ( 0, 0, limit );
    }
    glEnd();

  glLineWidth ( 3.0 );
  glBegin ( GL_LINES );
  {

    glColor3f ( 1.0, 0, 0  );
    glVertex3f ( 0, 0, 0 );
    glVertex3f ( limitAxe, 0, 0 );

    glColor3f ( 0, 1.0, 0 );
    glVertex3f ( 0, 0, 0 );
    glVertex3f ( 0, limitAxe, 0 );

    glColor3f ( 0, 0, 1.0  );
    glVertex3f ( 0, 0, 0 );
    glVertex3f ( 0, 0, limitAxe );
  }
  glEnd();
}

std::string GlViewerScene::getRandomString ( size_t size ) {
    std::string ret;
    ret.resize ( size );
    for ( size_t i=0; i<size; i++ )
        ret[i]=48+ ( rand() %40 );
    return ret;
}

//given a 2d buffer position, searches for the nearest projection in th zbuffer and returns its 3d position.
//If empty, returns nan in all components
bool GlViewerScene::getNearestProjection(uint32_t px, uint32_t py, float &outx, float &outy, float &outz , int maxWsize){

    auto readDepthBufferPixel=[](int x,int y,GLint *viewport){
        float z;
        glReadPixels( x, (float)viewport[3] - (float)y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z );
        return z;
    };

    auto readLine=[](int x,int y,int w,int h,GLint *viewport,vector<float> &data){

        data.resize(w*h);
        glReadPixels( x, viewport[3] - y, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, &data[0] );

    };

    auto getNearestValidElement=[](int x,int y,int w,int h,GLint *viewport,vector<float> &data){

        data.resize(w*h);
        glReadPixels( x, viewport[3] - y, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, &data[0] );
        int center=data.size()/2;
        int minDist=std::numeric_limits<int>::max();
        int nearest=-1;
        for(size_t i=0;i<data.size();i++){
            if(data[i]!=1){
                int dist=std::abs(int(center-i));
                if(dist<minDist){
                    dist=minDist;
                    nearest= i;
                }
            }
        }
        return nearest;
    };
    GLint viewport[4];
    GLfloat modelview[16];
    GLfloat projection[16];

    int _width=this->width();
    int _height=this->height();


     viewPoint.set();

    glGetFloatv( GL_MODELVIEW_MATRIX, modelview );
    glGetFloatv( GL_PROJECTION_MATRIX, projection );

    glGetIntegerv( GL_VIEWPORT, viewport );
    viewPoint.unset();

    std::pair<int,int> nearest;
    float zVal=1;

    if ( (zVal=readDepthBufferPixel( px,py,viewport))!=1){
         nearest.first=px;
         nearest.second=py;
    }
    else{
        int w=1;
        vector<float> zdata;
        int minDist=std::numeric_limits<int>::max();
        while(zVal==1 && w<=maxWsize){
             int xleft=px-w;
            int xright=px+w;
            int ytop=py-w;
            int ybottom=py+w;
            int nout=0;
            if ( xleft<0) {xleft=0;nout++;}
            if ( xright>=_width) {xright=_width-1;nout++;}
            if (ytop<0){ytop=0;nout++;}
            if (ybottom>=_height){ybottom=_height-1;nout++;}
            if(nout==4) break;//all pixels checked
            //analyzed top line if required

            if ( py-w>=0 ){

                int bestIndex=getNearestValidElement(xleft,ytop,xright -xleft+1,1,viewport,zdata);
                if(bestIndex!=-1){
                    nearest={ bestIndex+xleft,ytop};
                    zVal=zdata[bestIndex];
                }
            }
            //bottom line
            if (py+w<uint32_t(_height) && (zVal==1)){
                int bestIndex=getNearestValidElement(xleft,ybottom,xright -xleft+1,1,viewport,zdata);
                if(bestIndex!=-1){
                    nearest={ bestIndex+xleft,ytop};
                    zVal=zdata[bestIndex];
                }
            }

            //now, left column
            if ( px-w>=0 && (zVal==1)){

                int height= (ybottom - ytop-1);
                int bestIndex=getNearestValidElement(xleft,ytop+1,height,1,viewport,zdata);
                if(bestIndex!=-1){
                    nearest={ xleft,ytop+1+bestIndex};
                    zVal=zdata[bestIndex];
                }
            }
            //finally, right column
            if ( px-w<uint32_t(_width)  && (zVal==1)){

                int height= (ybottom - ytop-1);
                int bestIndex=getNearestValidElement(xright,ytop+1,height,1,viewport,zdata);
                if(bestIndex!=-1){
                    nearest={ xright,ytop+1+bestIndex};
                    zVal=zdata[bestIndex];
                }
            }
            w++;
        }
    }

    if(zVal==1) return false;

    else {
         glhUnProjectf( nearest.first,nearest.second, zVal, modelview, projection, viewport, &outx, &outy, &outz);
        return true;
    }
}


// get 3d point from 2d point coordinate using gluUnProject
bool GlViewerScene::unproject(int xscreen,int yscreen, float &xout,float &yout,float &zout){

    GLfloat winX, winY, winZ;
    GLfloat posX, posY, posZ;
    GLint viewport[4];
    GLfloat modelview[16];
    GLfloat projection[16];

    float x = xscreen;
    float y = yscreen;

     viewPoint.set();

    glGetFloatv( GL_MODELVIEW_MATRIX, modelview );
    glGetFloatv( GL_PROJECTION_MATRIX, projection );

    glGetIntegerv( GL_VIEWPORT, viewport );
    viewPoint.unset();

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    if (winZ==1) return false;
    glhUnProjectf( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
    xout=posX;
    yout=posY;
    zout=posZ;

    return true;
}
bool GlViewerScene::project(float X,float Y,float Z, int &wx,int &wy){
    GLint viewport[4];
    GLfloat modelview[16];
    GLfloat projection[16];

     viewPoint.set();

    glGetFloatv( GL_MODELVIEW_MATRIX, modelview );
    glGetFloatv( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    viewPoint.unset();

    GLfloat x,y,z;

    if (glhProject( X, Y, Z, modelview, projection, viewport, &x, &y, &z)==0)return false;

    wx=x/z;
    wy=y/z;
      wy = (float)viewport[3] - (float)wy;

    return true;
}

bool GlViewerScene::setCenterOfRotation(int x,int y,int maxWsize)
{
    qtgl::Point3f center;
    if( getNearestProjection(x,y,center.x,center.y,center.z,maxWsize)){

        viewPoint.setCenterOfRotation(center);
            return true;
    }
    return false;
}

}

