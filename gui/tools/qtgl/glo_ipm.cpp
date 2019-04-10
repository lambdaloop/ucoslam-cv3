#include "glo_ipm.h"
#ifdef WIN32
#include <GL/glew.h>
#else

#include <GL/glu.h>
#include "glextensions.h"
#endif
namespace qtgl{


Glo_Ipm::Glo_Ipm() {
    TheTextureId=_ipm_VertexVBOID=_ipm_IndexVBOID=-1;
    _showTexture=true;
    _material=SILVER;
    _isobject3dinfovalid=false;
}

Glo_Ipm::Glo_Ipm ( depthmaps::Indexed_PolygonMesh   ipm ) {
    TheTextureId=_ipm_VertexVBOID=_ipm_IndexVBOID=-1;
    _showTexture=true;
    _material=SILVER;
    _isobject3dinfovalid=false;
    set ( ipm );
}

Glo_Ipm::~Glo_Ipm() {
    release();
}

void Glo_Ipm::release ( ) {

    _perVertexColor=false;
    _isobject3dinfovalid=false;
    if ( TheTextureId!=-1 )
        glDeleteTextures ( 1,&TheTextureId );


    if ( _ipm_IndexVBOID!=-1 )
        glDeleteBuffers ( 1, &_ipm_VertexVBOID );
    if ( _ipm_IndexVBOID!=-1 )
        glDeleteBuffers ( 1, &_ipm_IndexVBOID );


    TheTextureId=_ipm_VertexVBOID=_ipm_IndexVBOID=-1;
}


void Glo_Ipm::set ( depthmaps::Indexed_PolygonMesh   _ipmesh ) {
    //  cerr<<__func__<<" "<<_ipmesh.getVertices().size()<<endl;
    release();
    _ipm=_ipmesh;
    calculateObject3dinfo(_ipm);

}
void Glo_Ipm::calculateObject3dinfo(depthmaps::Indexed_PolygonMesh ipm){
    _isobject3dinfovalid=true;
    _min=Point3f( ipm.getVertices().front().point[0],ipm.getVertices().front().point[1],ipm.getVertices().front().point[2]);
    _max=_min;
    for ( size_t i=0; i<ipm.getVertices().size(); i++ ) {

        _max.x=std::max(_max.x,ipm.getVertices()[i].point[0] );
        _min.x=std::min(_min.x,ipm.getVertices()[i].point[0] );
        _max.y=std::max(_max.y,ipm.getVertices()[i].point[1] );
        _min.y=std::min(_min.y,ipm.getVertices()[i].point[1] );
        _max.z=std::max(_max.z,ipm.getVertices()[i].point[2] );
        _min.z=std::min(_min.z,ipm.getVertices()[i].point[2] );
    }
}

bool Glo_Ipm::getBoundingBox(Point3f&min,Point3f&max){
    if (_isobject3dinfovalid){
        min=_min;max=_max;
    }
    return _isobject3dinfovalid;
}


uint32_t  Glo_Ipm::loadTexture ( const cv::Mat &im ) {
    //  cerr<<"Load tex"<<endl;
    //   cv::imwrite("out.jpg",im);
    GLuint tId;
    glGenTextures ( 1, &tId );
    std::cerr<<"Tex id="<<tId<<endl;
    glBindTexture ( GL_TEXTURE_2D, tId );
    glPixelStorei ( GL_UNPACK_ALIGNMENT, 1 );
    glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
    glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
    glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexEnvf ( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    assert ( im.isContinuous() );
    gluBuild2DMipmaps ( GL_TEXTURE_2D, 3, im.cols,im.rows,GL_BGR_EXT, GL_UNSIGNED_BYTE, im.ptr<uchar> ( 0 ) );
    return(uint32_t) tId;
}


void Glo_Ipm::setMaterialValues ( Materials m ) {
    GLfloat mat_amb[4]= {0,0,0,1};
    GLfloat mat_dif[4]= {0,0,0,1};
    GLfloat mat_spec[4]= {0,0,0,1};
    float shiness=1;
    if ( m==SILVER ) {
        mat_amb[0]=mat_amb[1]=mat_amb[2]=0.19225;
        mat_dif[0]=mat_dif[1]=mat_dif[2]=0.50754;
        mat_spec[0]=mat_spec[1]=mat_spec[2]=0.508273;
        shiness=0.4*128.0;
    } else if ( m==BRONZE ) {
        mat_amb[0]=0.2125;
        mat_amb[1]=0.1275;
        mat_amb[2]=0.054;
        mat_dif[0]=0.714;
        mat_dif[1]=0.4284;
        mat_dif[2]=0.18144;
        mat_spec[0]=0.393548;
        mat_spec[1]=0.271906;
        mat_spec[2]= 0.166721;
        shiness=0.2*128.0;

    }
    glMaterialfv ( GL_FRONT, GL_AMBIENT, mat_amb );
    glMaterialfv ( GL_FRONT, GL_DIFFUSE, mat_dif );
    glMaterialfv ( GL_FRONT, GL_SPECULAR, mat_spec );
    glMaterialf ( GL_FRONT, GL_SHININESS, shiness );
}
void Glo_Ipm::internal_set(){

    //clear previous data
    if (_ipm_VertexVBOID!=-1 ) return;
    if (_ipm.getVertices().size()==0) return;
    if ( _ipm.getTexture().total() !=0 ){
        TheTextureId= loadTexture ( _ipm.getTexture() );
        _perVertexColor=false;
    }
    else{
        _perVertexColor=_ipm.areColorVerticesValid();
//        if (_perVertexColor){//swap colors since they are going to be extracted from here
//            for(size_t i=0;i<_ipm.getVertices().size();i++)
//                swap(_ipm.getVertices()[i].color[0],_ipm.getVertices()[i].color[2]);
//        }
    }
    glGenBuffers ( 1, &_ipm_VertexVBOID );
    glBindBuffer ( GL_ARRAY_BUFFER, _ipm_VertexVBOID );
    glBufferData ( GL_ARRAY_BUFFER, sizeof ( depthmaps::Indexed_PolygonMesh::Vertex ) *_ipm.getVertices().size(), & ( _ipm.getVertices() [0] ), GL_STATIC_DRAW );
    glGenBuffers ( 1, &_ipm_IndexVBOID );
    glBindBuffer ( GL_ELEMENT_ARRAY_BUFFER, _ipm_IndexVBOID );
    glBufferData ( GL_ELEMENT_ARRAY_BUFFER, sizeof ( uint32_t ) * _ipm.getIndices().size(), & ( _ipm.getIndices() [0] ) , GL_STATIC_DRAW );
    _nVertices=_ipm.getIndices().size();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

//    if (_perVertexColor)//swap colors back
//        for(size_t i=0;i<_ipm.getVertices().size();i++)
//            swap(_ipm.getVertices()[i].color[0],_ipm.getVertices()[i].color[2]);

}

void Glo_Ipm::_draw_impl() {
    internal_set();//Create if not yet

    //must use color or material??
    bool _applyColor= ( _showTexture && ( TheTextureId!=-1 ||_perVertexColor));
    if ( _applyColor  ) {
        if (TheTextureId!=-1){
            glEnable ( GL_TEXTURE_2D );
            if ( TheTextureId!=-1) glBindTexture ( GL_TEXTURE_2D, TheTextureId );
            glColor3f(1,1,1);
        }
    }
    else{
        glEnable ( GL_LIGHTING );
        glEnable ( GL_LIGHT0 );
        setMaterialValues ( _material );
        glShadeModel ( GL_SMOOTH );
        glEnable ( GL_NORMALIZE ) ;
    }

    ///////////////////////////////////////////
    //Set the array of vertices
    glBindBuffer ( GL_ARRAY_BUFFER, _ipm_VertexVBOID );
    glEnableClientState ( GL_VERTEX_ARRAY );
    glVertexPointer ( 3, GL_FLOAT, sizeof ( depthmaps::Indexed_PolygonMesh::Vertex ), ( void* ) ( 0 ) ); //The starting point of the VBO, for the vertices

    ///////////////////////////////////////////
    //set color if enabled
    if ( _applyColor ) {
        if ( TheTextureId!=-1){//texture mode
            glEnableClientState ( GL_TEXTURE_COORD_ARRAY );
            glTexCoordPointer ( 2, GL_FLOAT, sizeof ( depthmaps::Indexed_PolygonMesh::Vertex ), ( void* ) ( 2*sizeof ( cv::Vec3f ) ) ); //The starting point of texcoords, 24 bytes away
        }
        else if(_perVertexColor){//per vetex mode
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3,GL_UNSIGNED_BYTE,sizeof ( depthmaps::Indexed_PolygonMesh::Vertex ),( void* ) ( 2*sizeof ( cv::Vec3f )+sizeof(cv::Point2f ) ));

        }
    }
    else {//no color, use material. Then, we are setting normals
        glEnableClientState ( GL_NORMAL_ARRAY );
        glNormalPointer ( GL_FLOAT, sizeof ( depthmaps::Indexed_PolygonMesh::Vertex ), ( void* ) ( sizeof ( cv::Vec3f ) ) ); //The starting point of normals, 12 bytes away

    }
    //////////////////////////////
    //Set the triangle indices
    glBindBuffer ( GL_ELEMENT_ARRAY_BUFFER, _ipm_IndexVBOID );


    ////////////////////////////////////////
    /// Draw
    glDrawElements ( GL_TRIANGLES, _nVertices, GL_UNSIGNED_INT, ( void* ) ( 0 ) ); //The starting point of the IBO

    ///Clean up

    glDisableClientState ( GL_VERTEX_ARRAY );
    if ( _applyColor ) {
        if (TheTextureId!=-1){
            glDisableClientState ( GL_TEXTURE_COORD_ARRAY );
            glDisable ( GL_TEXTURE_2D );
        }
        else if(_perVertexColor){
            glDisableClientState ( GL_COLOR_ARRAY );
        }
    } else {
        glDisableClientState ( GL_NORMAL_ARRAY );

        glDisable ( GL_LIGHT0 );
        glDisable ( GL_LIGHTING );
        glDisable ( GL_NORMALIZE );

    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

}


}
