#include <iostream>
#include "transform.h"
#include <cstring>
#include <QtOpenGL>
using namespace std;
namespace qtgl{

Transform::Transform(){
    _rotcenter=Point3f(0,0,0);
    float id[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    memcpy(TMatrix,id,16*sizeof(float));
}

Transform& Transform::doXRot (ViewPoint vp, float angle ) {


    float rotationMatrix[16]={1,0,0,0 , 0,cos(angle*1.7453292e-2f),sin(angle*1.7453292e-2f),0,  0,-sin(angle*1.7453292e-2f),cos(angle*1.7453292e-2f),0 ,0,0,0,1};
    applyTransform(vp,rotationMatrix);
    return *this;

}


Transform& Transform::doYRot (ViewPoint vp, float angle ) {


    float rotationMatrix[16]={cos(angle*1.7453292e-2f),0,-sin(angle*1.7453292e-2f),0 , 0,1,0,0,  sin(angle*1.7453292e-2f),0,cos(angle*1.7453292e-2f),0 ,0,0,0,1};
    applyTransform(vp,rotationMatrix);
    return *this;
}


Transform & Transform::goForward (ViewPoint vp, float dist ) {
    Point3f t=vp.getForwardVector()*dist;
    translate(vp,t[0],t[1],t[2]);
    return *this;
}


Transform&  Transform::translate(ViewPoint vp,float x,float y,float z)
{
    float trmatrix[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, x,y,z,1};
    applyTransform(vp,trmatrix);
    return *this;
}


void Transform::set(){

    glPushMatrix();
    glMultMatrixf ( TMatrix );
}

void Transform::getTransformMatrix(float m[16]){
memcpy(m,TMatrix,16*sizeof(float));

}

void Transform::unset(){

    glPopMatrix();
}

void Transform::applyTransform (ViewPoint vp, float rm[16] ) {

    float _global_view_rot_inv[16],_global_view_rot[16];
    vp.getRotationMatrix(_global_view_rot);
    invertMatrix(_global_view_rot,_global_view_rot_inv);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(_rotcenter.x+TMatrix[12],_rotcenter.y+TMatrix[13],_rotcenter.z+TMatrix[14]);
    glMultMatrixf ( _global_view_rot_inv );
    glMultMatrixf ( rm );
    glMultMatrixf ( _global_view_rot );
    glTranslatef(-_rotcenter.x-TMatrix[12],-_rotcenter.y-TMatrix[13],-_rotcenter.z-TMatrix[14]);
    glMultMatrixf ( TMatrix);
    glGetFloatv ( GL_MODELVIEW_MATRIX, TMatrix );
    glPopMatrix();
}
bool Transform::invertMatrix(const float mf[16], float invOutf[16])
{
    double m[16],invOut[16];
    for(int i=0;i<16;i++) m[i]=mf[i];
    double inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;


    for(int i=0;i<16;i++) invOutf[i]=invOut[i];

    return true;
}
}
