#include "object.h"
namespace qtgl{


Object::Object() {
    _isVisible=true;
    _priority=5;
}
Object::~Object() {}

void  Object::draw() {
    if (_isVisible){
        Point3f min,max,center;
        if ( getBoundingBox(min,max) ){
            center=min+(max-min)/2.;
            _transform.setRotationCenter(center);
        }
        _transform.set();
        _draw_impl();
        _transform.unset();
    }
}

}
