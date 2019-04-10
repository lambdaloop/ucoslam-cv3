/*****************************
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
//IF YOU USE THIS CODE, PLEASE CITE
//"Automatic generation and detection of highly reliable fiducial markers under occlusion"
//http://www.sciencedirect.com/science/article/pii/S0031320314000235
//AND
//"Generation of fiducial marker dictionaries using mixed integer linear programming"
//http://www.sciencedirect.com/science/article/pii/S0031320315003544

#ifndef _SGL_H
#define _SGL_H
#include <memory>
#include <cstring>
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
//Simplest graphics library
//Version 1.0.4
//Author. Rafael Muñoz Salinas (rmsalinas@uco.es) 2017

//Extremmely simple yet efficient device independent graphic library for points and lines
namespace sgl{
struct Point3{
    inline Point3( ){ }
    inline Point3(float X,float Y,float Z){x=X;y=Y;z=Z;}
    inline float norm()const{return sqrt( (x*x)+(y*y)+(z*z));}
    inline Point3 operator-(const Point3 &p)const{return Point3( x-p.x, y-p.y,z-p.z);}
    inline Point3 operator+(const Point3 &p)const{return Point3( x+p.x, y+p.y,z+p.z);}
    inline Point3 operator*(float v)const{return Point3( x*v,y*v,z*v);}
    float x,y,z;
    friend std::ostream & operator<<(std::ostream &str, const Point3 &p){
        str<<"["<<p.x<<" "<<p.y<<" "<<p.z<<"]";
        return str;
    }
};
struct Point2{
    Point2( ){ }
    Point2(float X,float Y){x=X;y=Y;}
    float x,y;

    friend std::ostream & operator<<(std::ostream &str, const Point2 &p){
        str<<"["<<p.x<<" "<<p.y<<"]";
        return str;
    }
};
struct Matrix44{

    inline Matrix44 (){float m[16]={1,0,0,0, 0,1,0,0 ,0,0,1,0 ,0,0,0,1};std::memcpy(m44,m,16*sizeof(float));}
    inline Matrix44 (float m[16]){ std::memcpy(m44,m,16*sizeof(float));}
    inline const Matrix44&operator=(const Matrix44&M){std::memcpy(m44,M.m44,16*sizeof(float));return *this;}
    inline   void  rotateX(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={1,0,0,0, 0,c,-s,0, 0,s,c,0, 0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }
    inline   void  rotateY(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={c,0,s,0, 0,1,0,0 ,-s,0,c,0 ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }

    inline   void  rotateZ(float rads){
        float c=cos(rads),s=sin(rads);
        float m[16]={c,-s,0,0, s,c,0,0 ,0,0,1,0 ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
    }
    inline   void  translate(const Point3 &p){
        float m[16]={1,0,0,p.x, 0,1,0,p.y ,0,0,1,p.z ,0,0,0,1};
        (*this)= Matrix44(m)*(*this);
     }
    inline   Point3 operator*(const Point3 &p){    return Point3( p.x*m44[0]+ p.y*m44[1]+p.z*m44[2]+m44[3], p.x*m44[4]+ p.y*m44[5]+p.z*m44[6]+m44[7],p.x*m44[8]+ p.y*m44[9]+p.z*m44[10]+m44[11] );}
    inline  Matrix44 operator*(const Matrix44 &M){
        Matrix44 res;
        for(int i=0;i<3;i++)
            for(int j=0;j<4;j++){
                res(i,j)=0;
                for(int k=0;k<4;k++)
                    res(i,j)+= at(i,k)*M(k,j);
            }
        return res;
    }


    //access to elements
    inline float & operator()(int r,int c){return m44[r*4+c];}
    inline float   operator()(int r,int c)const{return m44[r*4+c];}
    inline float & at(int r,int c){return m44[r*4+c];}


    friend std::ostream & operator<<(std::ostream &str, const Matrix44 &m){
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++)
                str<<m(i,j)<<" ";
            str<<std::endl;
        }
        return str;
    }

    Matrix44 inv()const{

        Matrix44  Inv;
        memcpy(Inv.m44,m44,16*sizeof(float));

        Inv.m44[3] = -  ( m44[3]*Inv.m44[0]+m44[7]*Inv.m44[1]+m44[11]*Inv.m44[2]);
        Inv.m44[7] = -  ( m44[3]*Inv.m44[4]+m44[7]*Inv.m44[5]+m44[11]*Inv.m44[6]);
        Inv.m44[11]= -  ( m44[3]*Inv.m44[8]+m44[7]*Inv.m44[9]+m44[11]*Inv.m44[10]);
        Inv.m44[12]=Inv.m44[13]=Inv.m44[14]=0;
        Inv.m44[15]=1;
        return Inv;
    }

    float m44[16];
};


struct Color{

    Color(){
    }
    Color(unsigned char  r,unsigned char  g,unsigned char b,unsigned char a=255){
        rgb[0]=r;
        rgb[1]=g;
        rgb[2]=b;
        rgb[3]=a;
    }
    inline unsigned char &operator[](int i){return rgb[i];}
    unsigned char rgb[4];
};


class Scene{

    struct ColorBuffer{


        ~ColorBuffer(){deallocate();}

        void create(int w,int h,int nchannels){
            if ( ( _w!=w || _h!=h || _nchannels!=nchannels) || !mustFreeBuffer ){
                deallocate();
                ptr=new uint8_t[w*h*nchannels];
                mustFreeBuffer=true;
            }
            _w=w;
            _h=h;
            _nchannels=nchannels;
        }
        void set(int w,int h,int nchannels, uint8_t *externalbuffer ){
            ptr=externalbuffer;
            mustFreeBuffer=false;
            _w=w;
            _h=h;
            _nchannels=nchannels;
        }

        void deallocate(){
            if (ptr!=0 && mustFreeBuffer)
                delete ptr;
            ptr=0;
            mustFreeBuffer=false;
        }

        inline uint8_t operator[](uint32_t i){return *(ptr+i*_nchannels);}
        inline void set(uint32_t i,const Color &c){memcpy(ptr+i*_nchannels,&c,_nchannels);}
        inline void set(uint32_t x,uint32_t y,const Color &c){memcpy(ptr+ (y*_w+x)*_nchannels,&c,_nchannels);}

        int _w=-1,_h=-1,_nchannels=3;
        uint8_t *ptr=0;
        bool mustFreeBuffer=false;

    };
    struct ZBuffer{

        ~ZBuffer(){if (_zbuf)delete _zbuf;}

        void create(int w,int h){
            if (_zbuf)delete _zbuf;
            _w=w;_h=h;
            _zbuf=new float[_w*_h];
        }
        inline void clear(){
            float mv=std::numeric_limits<float>::max() ;
            for(int i=0;i<_w*_h;i++) _zbuf[i]=mv;
        }
        inline bool set(int x,int y,float z ){
            float &p=_zbuf[y*_w+x];
            if (p>z){p=z;return true;}
            return false;
        }
        inline float get(int x,int y){
            return _zbuf[y*_w+x];
        }

        int _w,_h;
        float *_zbuf=0;
    };

    Matrix44 _viewMatrix;
    std::vector<Matrix44> modelMatrices;
    Matrix44 _cameraMatrix;
    Matrix44 TM;//transform from global coordinates to camera ones
    ColorBuffer colorbuffer;
    ZBuffer zBuffer;

    int pointSize=1,lineSize=1;
    float _focal;
    int _width,_height;

    inline void set(void *ptr,Color c,int n){memcpy(ptr,&c,n);}
public:
    Scene(){
        modelMatrices.resize(1);
        modelMatrices[0]= Matrix44();//set identity
        TM=_viewMatrix;
    }

    ~Scene(){ }

    void setPointSize(int s){pointSize=s;}
    void setLineSize(int s){lineSize=s;}
    //sets external buffer

    inline void setCameraParams(float focal,int width,int height,int nchannels,void *external_buffer=0){
        _focal=focal;
        _width=width;
        _height=height;
        _cameraMatrix(0,0)=_focal*float(_width);
        _cameraMatrix(1,1)=_focal*float(_width);
        _cameraMatrix(0,2)=float(_width)/2.;
        _cameraMatrix(1,2)=float(_height)/2.;
        if(external_buffer!=0) colorbuffer.set(width,height,nchannels,(uint8_t*)external_buffer);
        else colorbuffer.create(width,height,nchannels);
        zBuffer.create(width,height);
     }



    //this will erase the transform matrix
    inline void setViewMatrix(const Matrix44&vp){_viewMatrix=vp; TM=_viewMatrix*modelMatrices.back();}

    inline void setModelMatrix(const Matrix44 &M=Matrix44()){
        modelMatrices.resize(1);
        modelMatrices[0]=M;
        TM=_viewMatrix*modelMatrices.back();
    }
    inline void pushModelMatrix(const Matrix44 &M=Matrix44()){
        modelMatrices.push_back(modelMatrices.back()*M);
        TM=_viewMatrix*modelMatrices.back();
    }
    inline void popModelMatrix(){
        if (modelMatrices.size()>1){
            modelMatrices.pop_back();
            TM=_viewMatrix*modelMatrices.back();
        }
    }



    inline void clear(const Color &backgroundColor){

        const int size=_width*_height;
        for(int i=0;i<size;i++) colorbuffer.set(i,backgroundColor);
        zBuffer.clear();
      }
    //returns current view point
    inline Matrix44& getViewMatrix(){return _viewMatrix;}
    inline Matrix44& getModelMatrix(){return modelMatrices.back();}



    //draws a 3d point
    inline void drawPoint(const Point3 &p,const Color &c){  drawPoint(&p,c);}

    inline void drawPoint(const Point3 *p,const Color &c){
        Point2 pres;
        Point3 p3d=TM*(*p);
        if ( p3d.z<0 ) return;
        if ( project(p3d,pres)){
            int ix=pres.x+0.5;
            int iy=pres.y+0.5;
             switch (pointSize) {
                case 1:
                if ( ix>=0 && ix<_width && iy>=0 && iy<_height){
                    setPixel(ix,iy,p3d.z,c);
                    }
                break;
                case 2:
                 if ( ix> 0 && ix<_width-1 && iy>0 && iy<_height-1){
                     setPixel(ix,iy-1,p3d.z,c);
                     setPixel(ix,iy,p3d.z,c);setPixel(ix-1,iy,p3d.z,c);setPixel(ix+1,iy,p3d.z,c);
                     setPixel(ix,iy+1,p3d.z,c);
//                     setPixel(ix,iy+1,p3d.z,c);setPixel(ix-1,iy+1,p3d.z,c);setPixel(ix+1,iy+1,p3d.z,c);
                 }break;
                case 3:
                if ( ix> 0 && ix<_width-1 && iy>0 && iy<_height-1){
                    setPixel(ix,iy-1,p3d.z,c);setPixel(ix-1,iy-1,p3d.z,c);setPixel(ix+1,iy-1,p3d.z,c);
                    setPixel(ix,iy,p3d.z,c);setPixel(ix-1,iy,p3d.z,c);setPixel(ix+1,iy,p3d.z,c);
                    setPixel(ix,iy+1,p3d.z,c);setPixel(ix-1,iy+1,p3d.z,c);setPixel(ix+1,iy+1,p3d.z,c);
                }break;
             default: drawPoint(ix,iy,p3d.z,c,pointSize);

            }
        }
    }

    inline void drawLine(const Point3 &p1,const Point3 &p2,const Color &color){drawLine(&p1,&p2,color);}


    inline void drawLine(const Point3 *p1,const Point3 *p2,const Color &color){
        Point3 p1t=TM*(*p1);  if (p1t.z<0 ) return;
        Point3 p2t=TM*(*p2);  if (p2t.z<0) return;//check that both are in front of camera(otherwise, do not draw)
        Point2 pr1,pr2;
        if(! project(p1t,pr1))return;
        if(! project(p2t,pr2))return;

        drawline(Point3(pr1.x,pr1.y,p1t.z),Point3(pr2.x,pr2.y,p2t.z),color,lineSize);//project line bweten projected points
    }

    //returns the internal frame buffer
    inline unsigned char* getBuffer()const{return (unsigned char*)colorbuffer.ptr;}

    inline int getWidth()const{return _width;}
    inline int getHeight()const{return _height;}


    //zooms the current view
     inline void zoom(float value){
        auto vp=getViewMatrix();
        vp.translate({0,0,  value });
        setViewMatrix(vp);
    }

    //rotates the current view
    inline void rotate(float x,float z){
        sgl::Matrix44 tm;
        tm.rotateX(x);
        tm.rotateZ(z);
        sgl::Matrix44 res= tm*getModelMatrix() ;
        setModelMatrix(res);
    }

    //rotates the current view
    inline void rotate(float rx,float rz,float x,float y,float z){
        sgl::Matrix44 rot;
        rot.rotateX(rx);
        rot.rotateZ(rz);
        sgl::Matrix44 t,tinv;
        t.translate(sgl::Point3(x,y,z));
        tinv.translate(sgl::Point3(-x,-y,-z));
        sgl::Matrix44 res= tinv*rot*t*getModelMatrix() ;
        setModelMatrix(res);
    }

    //translates the current view
    inline void translate(float x,float y){
        auto vp=getViewMatrix();
        vp.translate(sgl::Point3(x,y,0.f));
        setViewMatrix(vp);
    }

    //given a 2d buffer position, searches for the nearest projection in th zbuffer and returns its 3d position.
    //If empty, returns nan in all components
    inline Point3 getNearestProjection(uint32_t px,uint32_t py ){

        if (px>=uint32_t(_width) || py>=uint32_t(_height))//out of limits
            return Point3(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());

        Point2 nearest;int minDist=std::numeric_limits<int>::max();

        if (zBuffer.get(px,py)!=std::numeric_limits<float>::max() ){
            minDist=0;nearest.x=px;nearest.y=py;
        }
        else{
            int w=1;
            while(minDist==std::numeric_limits<int>::max()){
                int xleft=px-w;
                int xright=px+w;
                int ytop=py-w;
                int ybottom=py+w;
                int nout=0;
                if ( xleft<0) {xleft=0;nout++;}
                if ( xright>=_width) {xright=_width-1;nout++;}
                if (ytop<0){ytop=0;nout++;}
                if (ybottom>=_height){ybottom=_height-1;nout++;}
                if(nout==4) break;//all pixels checked
                //analyzed top line if required

                if ( py-w>=0){
                    int y=ytop;
                    for(int x=xleft;x<=xright;x++)
                        if (zBuffer.get(x,y)!=std::numeric_limits<float>::max()){
                            int dist= (x-px)*(x-px) + (y-py)*(y-py);
                            if(dist<minDist){
                                minDist=dist;
                                nearest=Point2(x,y);
                            }
                        }
                }
                //bottom line
                if (py+w<uint32_t(_height)){
                    int y=ybottom;
                    for(int x=xleft;x<=xright;x++)
                        if (zBuffer.get(x,y)!=std::numeric_limits<float>::max()){
                            int dist= (x-px)*(x-px) + (y-py)*(y-py);
                            if(dist<minDist){
                                minDist=dist;
                                nearest=Point2(x,y);
                            }
                        }
                }

                //now, left column
                if ( px-w>=0){
                    int x=xleft;
                    for(int y=ytop+1;y<=ybottom-1;y++)
                        if (zBuffer.get(x,y)!=std::numeric_limits<float>::max()){
                            int dist= (x-px)*(x-px) + (y-py)*(y-py);
                            if(dist<minDist){
                                minDist=dist;
                                nearest=Point2(x,y);
                            }
                        }
                }
                //finally, right column
                if ( px-w<uint32_t(_width)){
                    int x=xright;
                    for(int y=ytop+1;y<=ybottom-1;y++)
                        if (zBuffer.get(x,y)!=std::numeric_limits<float>::max()){
                            int dist= (x-px)*(x-px) + (y-py)*(y-py);
                            if(dist<minDist){
                                minDist=dist;
                                nearest=Point2(x,y);
                            }
                        }


                }
                w++;
            }
        }

        if(minDist==std::numeric_limits<int>::max())
            return Point3(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
        else {
            float z=zBuffer.get(nearest.x,nearest.y);
            float x=z*(float(nearest.x)- _cameraMatrix(0,2))  /_cameraMatrix(0,0) ;
            float y=z*(float(nearest.y)- _cameraMatrix(1,2))  / _cameraMatrix(1,1) ;
            //now, move the point to global coordinates
            return getViewMatrix().inv() *Point3(x,y,z);
        }

    }
private:
    inline void setPixel(int x,int y,float z,sgl::Color c){
        if (zBuffer.set(x,y,z)) colorbuffer.set(x,y,c);
    }

    inline bool project(const Point3 &p,Point2 &res){
        Point3 pres=_cameraMatrix*(p);
        if(pres.z==0) return false;
        float invz=1./pres.z;
        res.x=invz*pres.x+0.5; res.y=invz*pres.y+0.5;
        return true;
    }

    //Bresenham's algorithm

    inline bool inLimits(int x,int y){
        if (x<0)return false;
        if (y<0)return false;
        if(x>=_width)return false;
        if(y>=_height)return false;
        return true;
    }


    void drawline(Point3 start, Point3 end,  const Color& color ,int size=1)
    {


        (void)size;
        int x0=start.x,y0=start.y,x1=end.x,y1=end.y;

        if (!inLimits(x0,y0) && !inLimits(x1,y1) ) return;



        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        float zcurr=start.z;
        float zinc=(end.z-start.z)/ std::max(dx,dy);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        while (true)
        {
            if (size==1){
                if(y0>=0 && x0>=0 && y0<_height && x0<_width)   setPixel(x0,y0,zcurr,color);
            }
            else{
                drawPoint(x0,y0,zcurr,color,size);
            }
            if (x0 == x1 && y0 == y1) return;
            int e2 = (err << 1);
            if (e2 > -dy){err -= dy;x0 += sx;}
            if (e2 < dx){err += dx;y0 += sy;}
            zcurr+=zinc;
        }
    }
    inline void drawPoint(int ix,int iy,float depth,sgl::Color c,int size){
        int ws=size/2;
        int minx=std::max(ix-ws,0);
        int miny=std::max(iy-ws,0);
        int maxx=std::min(_width,ix+ws);
        int maxy=std::min(_height,iy+ws);
        for(int ix=minx;ix<maxx;ix++)
            for(int iy=miny;iy<maxy;iy++)
                setPixel(ix,iy,depth,c);
    }
};


}


#endif
