#ifndef  GlObject_IndexedPolygonMesh_H
#define  GlObject_IndexedPolygonMesh_H
#include <depthmaps/indexedpolygonmesh.h>
#include <iostream>
#include <cstdint>
#include "object.h"
#include "exports.h"

namespace qtgl{


class QTGL_API Glo_Ipm  :public Object {

public:
    Glo_Ipm();
    Glo_Ipm ( depthmaps::Indexed_PolygonMesh  ipm );
    virtual ~Glo_Ipm();
    void set ( depthmaps::Indexed_PolygonMesh   ipm );
    bool isSet( ) const {return _ipm_IndexVBOID!=-1;}
    string getType( ) const {return "glo_imp";}

    void setShowTexture ( bool val ) {
        _showTexture=val;
    }

    bool showTexture() const {return _showTexture;}


    enum Materials {SILVER,BRONZE};
    void setMaterial ( Materials m ) {
        _material=m;
    }
    Materials getMaterial() const { return _material;}
     bool getBoundingBox(Point3f&min,Point3f&max);
 private:

    void release ( );
    void _draw_impl();
    void internal_set();

    uint32_t  loadTexture ( const cv::Mat &im ) ;

    void setMaterialValues ( Materials m );

    bool _showTexture;
    uint32_t TheTextureId,_ipm_IndexVBOID,_ipm_VertexVBOID;
    int _nVertices;
    bool _perVertexColor;
    Materials _material;
    depthmaps::Indexed_PolygonMesh  _ipm;

    void calculateObject3dinfo(depthmaps::Indexed_PolygonMesh dm);

    Point3f _min,_max;//center of object
    bool _isobject3dinfovalid;
};


}
#endif


