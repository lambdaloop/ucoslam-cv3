#include "glo_pointcloud.h"
#include <depthmaps/depthmaputils.h>
#include <iostream>
#ifdef WIN32
#include <GL/glew.h>
#else
#include <GL/glu.h>
#endif

using namespace std;
namespace qtgl{
Glo_PointCloud::Glo_PointCloud() {
    _visibilityTest=false;
    _isSet=false;
    _ptSize=2;
    _list=-1;
    _isobject3dinfovalid=false;
}

Glo_PointCloud::Glo_PointCloud (const depthmaps::DepthMap  dm ,int ptSize ) {
    _list=-1;
    _visibilityTest=false;
    _isSet=false;
    _ptSize=ptSize;
    _isobject3dinfovalid=false;
    set ( dm );
}
Glo_PointCloud::~Glo_PointCloud() {
    release();
}

void Glo_PointCloud::release ( ) {
    if(_list!=-1)
        glDeleteLists(_list,_list);
    _isSet=false;
    _list=-1;
    _isobject3dinfovalid=false;

}


void Glo_PointCloud::set (const depthmaps::DepthMap  dm ,int ptSize) {
    release();
    if ( dm.total() ==0 ) return;
    if (ptSize!=-1) _ptSize=ptSize;
    _dm=dm;
    calculateObject3dinfo(_dm);
    _isSet=true;
    _normals=cv::Mat();

}

void Glo_PointCloud::internal_set(){


    if (_list!=-1) return;
    if ( _dm.total() ==0 ) return;
    _list = glGenLists ( 1 );
//    depthmaps::DepthMapUtils::computeNormals(_dm,_normals,true,1);

    glNewList ( _list, GL_COMPILE );


//    glEnable ( GL_LIGHTING );
//    glEnable ( GL_LIGHT0 );
//    GLfloat mat_amb[4]= {0,0,0,1};
//    GLfloat mat_dif[4]= {0,0,0,1};
//    GLfloat mat_spec[4]= {0,0,0,1};
//    float shiness=1;
//        mat_amb[0]=mat_amb[1]=mat_amb[2]=0.19225;
//        mat_dif[0]=mat_dif[1]=mat_dif[2]=0.50754;
//        mat_spec[0]=mat_spec[1]=mat_spec[2]=0.508273;
//        shiness=0.4*128.0;
//    glMaterialfv ( GL_FRONT, GL_AMBIENT, mat_amb );
//    glMaterialfv ( GL_FRONT, GL_DIFFUSE, mat_dif );
//    glMaterialfv ( GL_FRONT, GL_SPECULAR, mat_spec );
//    glMaterialf ( GL_FRONT, GL_SHININESS, shiness );
//    glShadeModel ( GL_SMOOTH );
//    glEnable ( GL_NORMALIZE ) ;


    glPointSize(_ptSize);
    glBegin ( GL_POINTS );
    int npoints=0;
    for ( size_t y=0; y<_dm.rows; y++ ) {
        const depthmaps::DepthPixel *dp=_dm.ptr<depthmaps::DepthPixel> ( y );
       // cv::Point3f *nm=_normals.ptr<cv::Point3f>(y);
        for ( size_t x=0; x<_dm.cols; x++ )
            if ( dp[x].isValid() ) {
                glColor3ub ( dp[x].b(),dp[x].g(),dp[x].r() );
               // glNormal3f(nm[x].x,nm[x].y,nm[x].z);

                glVertex3f ( dp[x][0],dp[x][1],dp[x][2] );
                npoints++;
            }
    }
    glEnd();

//    glDisable ( GL_LIGHT0 );
//    glDisable ( GL_LIGHTING );
//    glDisable ( GL_NORMALIZE );

    glEndList();


}

void Glo_PointCloud::_draw_impl() {
    if (!_isSet) return;
    if (_visibilityTest)
        _draw_visibilityTest();
    else{
        internal_set();
        glCallList ( _list );
 }
}

void Glo_PointCloud::_draw_visibilityTest()
{
    if (_normals.total()==0)
    depthmaps::DepthMapUtils::computeNormals(_dm,_normals,true,1);
    float modelview[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    cv::Mat m(4,4,CV_32F);
    int idx=0;
    for(int j=0;j<4;j++)
        for(int i=0;i<4;i++)
            m.at<float>(i,j)=modelview[idx++];
    cv::Mat minv=m.inv();

    cv::Point3f center( minv.at<float>(0,3),minv.at<float>(1,3),minv.at<float>(2,3));
    //    center*=1./cv::norm(center);

    glPointSize(_ptSize);
    glBegin ( GL_POINTS );
    for ( size_t y=0; y<_dm.rows; y++ ) {
        const depthmaps::DepthPixel *dp=_dm.ptr<depthmaps::DepthPixel> ( y );
        cv::Point3f *normal=_normals.ptr<cv::Point3f>(y);
        for ( size_t x=0; x<_dm.cols; x++ )
            if ( dp[x].isValid()){
                cv::Point3f p=dp[x].toPoint3f();
                //ray from center to point , and then the dot product of normal and the ray
                float ray_dot = normal[x].x*(p.x-center.x)+ normal[x].y*  (p.y-center.y)+normal[x].z* (p.z-center.z);
                if (ray_dot<=0){
                    glColor3ub ( dp[x].b(),dp[x].g(),dp[x].r() );
                    glVertex3f ( p.x,p.y,p.z );
                }
            }
    }
    glEnd();
}
bool Glo_PointCloud::getBoundingBox(Point3f&min,Point3f&max){

if (_isobject3dinfovalid){
   min=_min;max=_max;
}
return _isobject3dinfovalid;

}
void Glo_PointCloud::calculateObject3dinfo(depthmaps::DepthMap dm){
    _isobject3dinfovalid=true;
    cv::Point3f min,max;
    depthmaps::DepthMapUtils::calculateLimits(dm,min,max);
    _min.x=min.x;
    _min.y=min.y;
    _min.z=min.z;
    _max.x=max.x;
    _max.y=max.y;
    _max.z=max.z;

}

}

