/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#ifndef s3_arucomm_h
#define s3_arucomm_h
#include <opencv2/calib3d/calib3d.hpp>
#include <cmath>
#include <iostream>
#include <vector>
namespace ucoslam{
/**Represents a transform in the SE3 space. Rotation + Translation
 * The class allows a seamless integration with opencv routines while preserving the minimal compacted memory of 6 components
 */
struct  se3{
    float rt[6];
    se3(){  for(int i=0;i<6;i++) rt[i]=std::numeric_limits<float>::quiet_NaN();}
    se3(float rx,float ry,float rz,float tx,float ty,float tz){   rt[0]=rx;rt[1]=ry;rt[2]=rz;rt[3]=tx;rt[4]=ty;rt[5]=tz;}
    se3(float v){  for(int i=0;i<6;i++) rt[i]=v;}
    se3(const float * v){ for(int i=0;i<6;i++) rt[i]=v[i];}
    se3(const cv::Mat &rt){  *this=convert(rt);}
    se3(const cv::Mat &r,const cv::Mat &t){  *this=convert(r,t);}
    bool isValid()const{for(int i=0;i<6;i++)  if ( std::isnan(rt[i]))return false;  return true;}
    void operator+=(const se3 &s){for(int i=0;i<6;i++) rt[i]+=s.rt[i];}
    float &operator()(int i){ return rt[i];}
    void operator/=(float val){for(int i=0;i<6;i++) rt[i]/=val;}
    se3 & operator*(float val){for(int i=0;i<6;i++) rt[i]*=val;return *this;}
    se3   operator*(const se3  &b)const{ return se3(convert()*b.convert());}
    se3 & operator=(const cv::Mat &rt){ *this=convert(rt);return *this;}
    bool operator==(const se3 &p)const{ for(int i=0;i<6;i++)if ( rt[i]!=p.rt[i])return false;return true;}
    se3   operator-(const se3 &p)const{return se3 ( rt[0]-p.rt[0],rt[1]-p.rt[1],rt[2]-p.rt[2],rt[3]-p.rt[3],rt[4]-p.rt[4], rt[5]-p.rt[5]);}

    //applies the operation on the 3d point(very slow)
    cv::Point3f   operator*(const cv::Point3f  &val)const{
        cv::Mat RT=convert();
        float *m16=RT.ptr<float>(0);
        cv::Point3f res;
        res.x= val.x*m16[0]+val.y*m16[1]+val.z*m16[2]+m16[3];
        res.y= val.x*m16[4]+val.y*m16[5]+val.z*m16[6]+m16[7];
        res.z= val.x*m16[8]+val.y*m16[9]+val.z*m16[10]+m16[11];
        return res;
    }

    //returns the 3d translation
    cv::Point3f getTasPoint3f()const{return cv::Point3f(rt[3],rt[4],rt[5]);}
    void setTo(float v){for(int i=0;i<6;i++) rt[i]=v;}
    float operator[](size_t d)const {return rt[d];}
    float &operator[](size_t d){return rt[d];}
    friend std::ostream & operator<<(std::ostream &str,const se3 &s){for(int i=0;i<6;i++)str<<s.rt[i]<<" ";return str;}
    inline operator cv::Mat ()const{return convert();}


    float r_dot(const se3 &b)const{
        cv::Vec3f ra(rt[0],rt[1],rt[2]);
        cv::Vec3f rb(b.rt[0],b.rt[1],b.rt[2]);
        ra*=1./cv::norm(ra);
        rb*=1./cv::norm(rb);
        return ra.dot(rb);
    }
    inline float t_dist(const se3 &b)const{
        double s=0;
        for(int i=3;i<6;i++) s+=(rt[i]-b.rt[i] )*(rt[i]-b.rt[i] );
        return sqrt(s);
    }

    float tnorm()const{
        return sqrt( rt[3]*rt[3]+ rt[4]*rt[4]+rt[5]*rt[5]);
    }

    void setRotation(float rx,float ry,float rz){
        rt[0]=rx;
        rt[1]=ry;
        rt[2]=rz;
    }
    void setRT(const cv::Mat &rvec ,const cv::Mat &tvec){
        assert(rvec.type()==tvec.type());
        if (tvec.type()==CV_32F){
            memcpy( rt,rvec.ptr<float>(0),3*sizeof(float) );
            memcpy( rt+3,tvec.ptr<float>(0),3*sizeof(float) );
        }
        else if ( tvec.type()==CV_64F){
            rt[0]=rvec.ptr<double>(0)[0];
            rt[1]=rvec.ptr<double>(0)[1];
            rt[2]=rvec.ptr<double>(0)[2];
            rt[3]=tvec.ptr<double>(0)[0];
            rt[4]=tvec.ptr<double>(0)[1];
            rt[5]=tvec.ptr<double>(0)[2];
        }
        else{
            assert(0);
        }
    }

    void setTranslation(const cv::Mat &tvec){
        if (tvec.type()==CV_32F)
            memcpy( rt+3,tvec.ptr<float>(0),3*sizeof(float) );
        else if ( tvec.type()==CV_64F){
            rt[3]=tvec.ptr<double>(0)[0];
            rt[4]=tvec.ptr<double>(0)[1];
            rt[5]=tvec.ptr<double>(0)[2];
        }
        else{
            assert(0);
        }
    }

    void setRotation(const cv::Mat &rvec){
        if (rvec.type()==CV_32F)
            memcpy( rt,rvec.ptr<float>(0),3*sizeof(float) );
        else if ( rvec.type()==CV_64F){
            rt[0]=rvec.ptr<double>(0)[0];
            rt[1]=rvec.ptr<double>(0)[1];
            rt[2]=rvec.ptr<double>(0)[2];
        }
        else{
            assert(0);
        }
    }

    cv::Mat getRotation3x3() {//as a 3x3 matrix
        cv::Mat rot;
        cv::Mat r3(1,3,CV_32F,rt);
        cv::Rodrigues(r3,rot);
        return rot;

    }
    cv::Mat getTvec() const{//as a 3x3 matrix
        cv::Mat m(1,3,CV_32F);
        memcpy(m.ptr<float>(0),rt+3,3*sizeof(float));
        return m;
    }
    cv::Mat getRvec() const{//as a 3x3 matrix
        cv::Mat m(1,3,CV_32F);
        memcpy(m.ptr<float>(0),rt,3*sizeof(float));
        return m;
    }
    se3  convert(const  cv::Mat &r,const cv::Mat &t) {
        assert(r.type()==CV_32F || r.type()==CV_64F);
        se3 res;
        if(r.type()==CV_32F){
        for(int i=0;i<3;i++){
            res.rt[i]=r.ptr<float>(0)[i];
            res.rt[i+3]=t.ptr<float>(0)[i];
        }
        }
        else{
            for(int i=0;i<3;i++){
                res.rt[i]=r.ptr<double>(0)[i];
                res.rt[i+3]=t.ptr<double>(0)[i];
            }

        }
        return res;
    }




    static se3  convert(const  cv::Mat &RT){

        se3 res;
        if (RT.empty())return res;
        cv::Mat r,t;
          getRTfromMatrix44(RT,r,t);
        if ( RT.type()==CV_32F){
            for(int i=0;i<3;i++){
                res.rt[i]=r.ptr<float>(0)[i];
                res.rt[i+3]=t.ptr<float>(0)[i];
            }
        }
        else{
            for(int i=0;i<3;i++){
                res.rt[i]=r.ptr<double>(0)[i];
                res.rt[i+3]=t.ptr<double>(0)[i];
            }

        }
        return res;
    }

    inline cv::Mat   convert( )const{
        if(!isValid())return cv::Mat();
        cv::Mat m(4,4,CV_32F);
        convert(m.ptr<float>(0));
        return m;
    }

    //returns the inverse transform
    se3 inverse( )const {return convert(this->convert().inv());}

    static float norm(const se3 &a){return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3]+a[4]*a[4]+a[5]*a[5]);}

    void toStream(std::ostream &str)const{str.write((char*)rt,6*sizeof(float));}
    void fromStream(std::istream &str){str.read((char*)rt,6*sizeof(float));}

    //makes a fast conversion to the 4x4 array passed
    inline void convert(float*rt_44)const{
        float rx=rt[0];
        float ry=rt[1];
        float rz=rt[2];
        float tx=rt[3];
        float ty=rt[4];
        float tz=rt[5];
        float nsqa=rx*rx + ry*ry + rz*rz;
        float a=std::sqrt(nsqa);
        float i_a=a?1./a:0;
        float rnx=rx*i_a;
        float rny=ry*i_a;
        float rnz=rz*i_a;
        float cos_a=cos(a);
        float sin_a=sin(a);
        float _1_cos_a=1.-cos_a;
        rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
        rt_44[1]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt_44[2]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[3]=tx;
        rt_44[4]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt_44[5]=cos_a+rny*rny*_1_cos_a;
        rt_44[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
        rt_44[7]=ty;
        rt_44[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[9]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
        rt_44[11]=tz;
        rt_44[12]=rt_44[13]=rt_44[14]=0;
        rt_44[15]=1;
    }
    //makes a fast conversion to the 4x4 array passed in transposed mode
    inline void convert_t(float*rt_44)const{
        float rx=rt[0];
        float ry=rt[1];
        float rz=rt[2];
        float tx=rt[3];
        float ty=rt[4];
        float tz=rt[5];
        float nsqa=rx*rx + ry*ry + rz*rz;
        float a=std::sqrt(nsqa);
        float i_a=a?1./a:0;
        float rnx=rx*i_a;
        float rny=ry*i_a;
        float rnz=rz*i_a;
        float cos_a=cos(a);
        float sin_a=sin(a);
        float _1_cos_a=1.-cos_a;

        rt_44[0] =cos_a+rnx*rnx*_1_cos_a;
        rt_44[4]=rnx*rny*_1_cos_a- rnz*sin_a;
        rt_44[8]=rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[12]=tx;
        rt_44[1]=rnz*sin_a +rnx*rny*_1_cos_a;
        rt_44[5]=cos_a+rny*rny*_1_cos_a;
        rt_44[9]= -rnx*sin_a+ rny*rnz*_1_cos_a;
        rt_44[13]=ty;
        rt_44[2]= -rny*sin_a + rnx*rnz*_1_cos_a;
        rt_44[6]= rnx*sin_a + rny*rnz*_1_cos_a;
        rt_44[10]=cos_a+rnz*rnz*_1_cos_a;
        rt_44[14]=tz;
        rt_44[3]=rt_44[7]=rt_44[11]=0;
        rt_44[15]=1;
    }

private:
    /**
       * Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
       */
    static cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType=-1 ) {
        cv::Mat M;
        cv::Mat R,T;
        R_.copyTo ( R );
        T_.copyTo ( T );
        if ( R.type() ==CV_64F ) {
            assert ( T.type() ==CV_64F );
            cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );

            cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
            if ( R.total() ==3 ) {
                cv::Rodrigues ( R,R33 );
            } else if ( R.total() ==9 ) {
                cv::Mat R64;
                R.convertTo ( R64,CV_64F );
                R.copyTo ( R33 );
            }
            for ( int i=0; i<3; i++ )
                Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
            M=Matrix;
        } else if ( R.depth() ==CV_32F ) {
            cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
            cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
            if ( R.total() ==3 ) {
                cv::Rodrigues ( R,R33 );
            } else if ( R.total() ==9 ) {
                cv::Mat R32;
                R.convertTo ( R32,CV_32F );
                R.copyTo ( R33 );
            }

            for ( int i=0; i<3; i++ )
                Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
            M=Matrix;
        }

        if ( forceType==-1 ) return M;
        else {
            cv::Mat MTyped;
            M.convertTo ( MTyped,forceType );
            return MTyped;
        }
    }

   static  void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T,bool useSVD=false ) {

        assert ( M.cols==M.rows && M.cols==4 );
        assert ( M.type() ==CV_32F || M.type() ==CV_64F );
//extract the rotation part
        cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
        if (useSVD){
        cv::SVD svd ( r33 );
        cv::Mat Rpure=svd.u*svd.vt;
        cv::Rodrigues ( Rpure,R );
        }else
            cv::Rodrigues ( r33,R );

        T.create ( 1,3,M.type() );
        if ( M.type() ==CV_32F )
            for ( int i=0; i<3; i++ )
                T.ptr<float> ( 0 ) [i]=M.at<float> ( i,3 );
        else
            for ( int i=0; i<3; i++ )
                T.ptr<double> ( 0 ) [i]=M.at<double> ( i,3 );
    }



};
}
#endif
