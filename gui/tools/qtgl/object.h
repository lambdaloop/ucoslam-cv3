#ifndef _QTGL_OBJECT_
#define _QTGL_OBJECT_
#include <string>
#include "qtgl_exports.h"
#include "transform.h"
#include <iostream>
using namespace std;
namespace qtgl{


class QTGL_API Object{
protected:
    bool _isVisible;
    unsigned int _priority;//in range 0,10 (0 min, 10 max)
    Transform _transform;

public:
    Object() ;
    virtual ~Object();
    void  draw();
    void setVisible(bool visible){_isVisible=visible;}
    bool isVisible()const{return _isVisible;}
    unsigned int getPriority(){return _priority;}
    virtual void setTransform(Transform t){_transform=t;}
    Transform getTransform( ){return _transform;}
    //sets the priority of this object in drawing
    void setPriority(unsigned int p){_priority=p;}


    //has to be reimplemented
    virtual std::string getType()const=0;//return an string indicating the type of the subclass
    //some object needs this
    virtual bool getBoundingBox(Point3f&min,Point3f&max){return false;}

protected:
    //you have to reimplement
    virtual void _draw_impl() =0;
};
};
#endif
