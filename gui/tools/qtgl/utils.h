#ifndef _QTGL_UTILS_H_XX
#define _QTGL_UTILS_H_XX
#include <cmath>
#include <ostream>
#include "qtgl_exports.h"

namespace qtgl
{
struct QTGL_API Point3f{
    float x,y,z;

    Point3f(){}
    Point3f(float X,float Y,float Z){x=X;y=Y;z=Z;}
    Point3f operator+(const Point3f p){return Point3f(x+p.x,y+p.y,z+p.z);}
    Point3f operator+=(const Point3f p){  x+=p.x;y+=p.y;z+=p.z; return *this;}
    Point3f operator-(const Point3f p){return Point3f(x-p.x,y-p.y,z-p.z);}
    Point3f operator/(float v){return Point3f(x/v,y/v,z/v);}
    Point3f operator*(float v){return Point3f(x*v,y*v,z*v);}

    float operator[](int i){if (i==0) return x;if (i==1) return y;return z;}
    bool operator==(const Point3f p)const{  return x==p.x && y==p.y && z==p.z;}

    friend std::ostream&operator<<(std::ostream&str,const Point3f &p){str<<"["<<p.x<<","<<p.y<<","<<p.z<<"]";return str;}
};



QTGL_API
 int glhProject(float objx, float objy, float objz, float *modelview, float *projection, int *viewport, float *x,float *y,float *z);
QTGL_API
 void glhLookAtf2( float *matrix, float *eyePosition3D,  float *center3D, float *upVector3D );

class QTGL_API  UnProjector
{
public:
    UnProjector();
    UnProjector( float *modelview, float *projection, int *viewport);
    void set( float *modelview, float *projection, int *viewport);
    bool unproject(float winx, float winy, float depth, float *objectCoordinate);
private:

    bool _isValid,_isInverseValid;
    float _modelview[16],_projection[16];
    int _viewport[16];
     float _m[16];
};

QTGL_API int glhUnProjectf(float winx, float winy, float winz, float *modelview, float *projection, int *viewport, float *objectCoordinate);
QTGL_API int glhUnProjectf(float winx, float winy, float winz, float *modelview, float *projection, int *viewport, float *posX,float *posY,float *posZ);
//glhUnProjectf( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

}
#endif

