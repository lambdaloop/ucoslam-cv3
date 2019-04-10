#ifndef qtgl_H_Transform_
#define qtgl_H_Transform_
#include "qtgl_exports.h"
#include "viewpoint.h"
namespace qtgl{



class QTGL_API Transform{

public:

    Transform();
    Transform&  doXRot (  ViewPoint  vp, float angle ) ;
    Transform&  doYRot (   ViewPoint  vp,float angle ) ;
    Transform&  translate(  ViewPoint  vp,float x,float y,float z);
    Transform& goForward(  ViewPoint  vp,float dist);


    void setRotationCenter(Point3f t){_rotcenter=t;}
    //applies
    void set();
    void unset();

    void getTransformMatrix(float m[16]);
private:
    float TMatrix[16];
    Point3f _rotcenter;
    void  applyTransform( ViewPoint vp,float tm[16]);

    bool invertMatrix(const float m[16], float invOut[16]);

};

}
#endif
