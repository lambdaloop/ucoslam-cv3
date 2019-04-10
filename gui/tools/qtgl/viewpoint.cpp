#include "viewpoint.h"
#include <iostream>
#include <cstring>
#include "utils.h"
#include <QtOpenGL>
#ifdef WIN32
    #include <GL/GLU.h>
#else
 #include <GL/glu.h>
#endif
using namespace std;
namespace qtgl{


ViewPoint::ViewPoint(bool fromzero) {

    centerOfRotation=Point3f(0,0,0);
    if (!fromzero){
    float initpos[16]= {   -0.695502 ,-0.307819 ,-0.649242, 0, -0.00438319, 0.905384, -0.424565 ,0 ,0.718507, -0.292441,-0.631046 ,0 ,0 ,0, 0, 1 };
    memcpy(rotationMatrix,initpos,16*sizeof(float));
    eye=Point3f (0.015, 0.045, -2 );
    forward=Point3f ( 0,0,1 );
    up=Point3f ( 0,1,0 );
    scale=1;
    }
    else{
        float initpos[16]= {   1,0,0,0 , 0,1,0,0,  0,0,1,0 , 0,0,0,1  };
        memcpy(rotationMatrix,initpos,16*sizeof(float));
        eye=Point3f (0,0,0);
        forward=Point3f ( 0,0,1 );
        up=Point3f ( 0,1,0 );
        scale=1;
    }
}
void ViewPoint::reset(){
    //eye = qtgl::Point3f(0,0,-3);
    float m[16]={0, 0, 1, 0,    0, 1, 0, 0,   1 ,0, 0, 0 ,0, 0, 0, 1};
    memcpy(rotationMatrix,m,16*sizeof(float));
}


ViewPoint & ViewPoint::goForward ( float dist ) {
    eye+=forward*dist;
    return *this;
}

ViewPoint & ViewPoint::translate ( float tx,float ty,float tz) {
    eye+=Point3f(tx,ty,tz);
    return *this;
}

ViewPoint& ViewPoint::doXRot ( float angle ) {



    glPushMatrix();
    glLoadIdentity();
    glTranslatef(centerOfRotation.x,centerOfRotation.y,centerOfRotation.z);
    glRotatef ( angle, 1.0, 0, 0 );
    glTranslatef(-centerOfRotation.x,-centerOfRotation.y,-centerOfRotation.z);
    glMultMatrixf ( rotationMatrix );
    glGetFloatv ( GL_MODELVIEW_MATRIX, rotationMatrix );
    glPopMatrix();
    return *this;
}
ViewPoint& ViewPoint::doZRot ( float angle ) {

    glPushMatrix();
    glLoadIdentity();
    glTranslatef(centerOfRotation.x,centerOfRotation.y,centerOfRotation.z);
    glRotatef ( angle, 0.0, 0.0, 1. );
    glTranslatef(-centerOfRotation.x,-centerOfRotation.y,-centerOfRotation.z);
    glMultMatrixf ( rotationMatrix );
    glGetFloatv ( GL_MODELVIEW_MATRIX, rotationMatrix );
    glPopMatrix();

    return *this;
}


ViewPoint& ViewPoint::doYRot ( float angle ) {

    glPushMatrix();
    glLoadIdentity();
    glTranslatef(centerOfRotation.x,centerOfRotation.y,centerOfRotation.z);
    glRotatef ( angle, 0.0, 1.0, 0 );
    glTranslatef(-centerOfRotation.x,-centerOfRotation.y,-centerOfRotation.z);
    glMultMatrixf ( rotationMatrix );
    glGetFloatv ( GL_MODELVIEW_MATRIX, rotationMatrix );
    glPopMatrix();

    return *this;
}
ViewPoint& ViewPoint::scaleInc ( float inc ) {
    scale*=inc;
    return *this;
}
void ViewPoint::setMatrix(float *m16){
    memcpy(rotationMatrix,m16,sizeof (float)*16);
}

void ViewPoint::set () {
    glPushMatrix();
    glLoadIdentity();
    Point3f center=eye+forward;
    gluLookAt ( eye[0],eye[1],eye[2],center[0],center[1],center[2],up[0],up[1],up[2] );

//    float matrix[16];
//    glhLookAtf2( matrix,(float*)&eye ,(float*)&center,(float*)&up );
//    glMatrixMode ( GL_MODELVIEW );
//    glMultMatrixf(matrix);
//    glTranslated(-eye.x, -eye.y, -eye.z);

    glMultMatrixf ( rotationMatrix );

}
void ViewPoint:: unset () {
    glPopMatrix();
}

bool ViewPoint::operator==(const ViewPoint &v)const{
    bool eq=true;
    for(int i=0;i<16;i++) eq&=rotationMatrix [i]==v.rotationMatrix[i];
    if (!eq) return false;
    eq=(eye==v.eye && forward==v.forward && up==v.up);
   return eq;

}
bool ViewPoint::operator!=(const ViewPoint &v)const{
    return !((*this)==v);
}
void ViewPoint::print(){
std::cerr<<eye<<" "<<forward<<" "<<up<<endl;
for(size_t i=0;i<16;i++) cerr<<rotationMatrix[i]<<" ";
cerr<<endl;
 }
void ViewPoint::getRotationMatrix( float rmatrix[16])const{
    memcpy(rmatrix,rotationMatrix,16*sizeof(float));
}


}
