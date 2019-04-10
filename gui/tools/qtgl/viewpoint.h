#ifndef _QTGL_VIEWPOINT_H
#define _QTGL_VIEWPOINT_H
#include "utils.h"
#include "qtgl_exports.h"
namespace qtgl{

// viewpoint auxiliar class to make viewpoint changes easier
struct  QTGL_API ViewPoint {
public:
    float rotationMatrix [16];
    Point3f eye,forward,up,centerOfRotation;
    float scale;

public:
    ViewPoint(bool fromzero=false);
    void reset();//sets tou nity
    ViewPoint & goForward ( float dist );
    ViewPoint & translate (  float tx,float ty,float tz );
    ViewPoint & doXRot ( float angle );
    ViewPoint & doYRot ( float angle );
    ViewPoint & doZRot ( float angle );
    ViewPoint & scaleInc ( float inc );
    bool operator==(const ViewPoint &)const;
    bool operator!=(const ViewPoint &v)const;


    void set ();
    void unset ();
    void print();

    void getRotationMatrix( float rmatrix[16])const;
    Point3f getForwardVector()const{return forward;}

    void setCenterOfRotation(Point3f p){centerOfRotation=p;}
    void setMatrix(float *m16);
private:
    float norm(Point3f p){return sqrt(p.x*p.x+p.y*p.y+p.z*p.z);}

    Point3f normalize ( Point3f in ) {
        return  in /norm ( in ) ;
    }

};
}
#endif
