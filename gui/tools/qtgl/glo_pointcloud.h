#ifndef  GlObject_PointCloud_H
#define  GlObject_PointCloud_H

#include <depthmaps/depthmap.h>
#include <iostream>
#include "object.h"
#include "exports.h"

namespace qtgl{
//uncomment if you are using a single file

class QTGL_API Glo_PointCloud  :public qtgl::Object {

public:
    Glo_PointCloud();
    Glo_PointCloud ( const depthmaps::DepthMap  dm,int ptSize=2 );
    virtual ~Glo_PointCloud();
    void set (const depthmaps::DepthMap  dm ,int ptSize=-1);
    bool isSet( ) const {return _isSet;}
    string getType( ) const {return "glo_pointcloud";}
    void enableVisibilityTest(bool enable){_visibilityTest=enable;}
    bool getBoundingBox(Point3f&min,Point3f&max);

private:
    void release ( );
    void _draw_impl();
    void _draw_visibilityTest();
    void calculateObject3dinfo(depthmaps::DepthMap dm);


    cv::Mat _normals;
    unsigned int _list;
    bool _isSet;
    int _ptSize;
    bool _isVisible;
    bool _visibilityTest;
    depthmaps::DepthMap  _dm;
    void internal_set();
    Point3f _min,_max;
    bool _isobject3dinfovalid;
};

}
#endif



