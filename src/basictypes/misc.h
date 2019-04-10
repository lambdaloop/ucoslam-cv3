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
#ifndef _UCOSLAM_UTILS_H
#define _UCOSLAM_UTILS_H
#include "map_types/frame.h"

#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include "debug.h"

namespace ucoslam{
//Returns the matches between two frames. Basic method using first a brute force, and then computing ratio to second best match
//[queryIdx=f1; train =f2]
std::vector<cv::DMatch> match_frames(const Frame &f1, const Frame &f2, double nn_match_ratio=0.8f);
std::vector<cv::DMatch> match_desc(const cv::Mat & desc1,const cv::Mat & desc2, double nn_match_ratio=0.8f);

  //resolve matching ambiguity. It happens when multiple elemnts of one set are matched with one of the other set
//  ambiguos occurs when multiple elements of query are assinged to the same element of train. This migh happen when using opencv matcher
 void filter_ambiguous_train(  std::vector<cv::DMatch> &matches_io );
//  ambiguos occurs when multiple elements of query are assinged to the same element of query. This might happen in our map to frame matching
void filter_ambiguous_query(std::vector<cv::DMatch> &matches_io );
//resizes the matches vector efficiently removing the with -1 value either in queryIdx or trainIdx
void remove_unused_matches(std::vector<cv::DMatch> &matches_io );
void remove_bad_matches(std::vector<cv::DMatch> &matches_io ,const vector<bool> &vBadMatches);
 //given a 4x4 transform amtrix, computes the r and t vectors. If the rotation 3x3 matrix is not correct, use the useSVD param to correct the error if possile
void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T,bool useSVD=false ) ;
//returns the 4x4 matrix given r and t vectors
//the output type is the same as the input ones, if you want, use the last paramter to specify the output type
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType =-1);

//last version which is the most strict one.
/**
 * @brief triangulate_
 * @param RT_21: 4x4 matrix moving points from camera 1 to camera 2
 * @param kp1
 * @param kp2
 * @param CameraMatrix
 * @param vP3D
 * @param scaleFactors
 * @param vbGood
 * @param maxChi2
 * @return
 */
int   triangulate_(const cv::Mat &RT_21, std::vector<cv::KeyPoint> &kp1,std::vector<cv::KeyPoint> &kp2,  const cv::Mat &CameraMatrix1,const cv::Mat &CameraMatrix2, vector<cv::Point3f> &vP3D,  const vector<float> &scaleFactors,vector<bool> &vbGood,float maxChi2=5.998 );

//triangulates the matches of the given frames.
//returns as many points as matches. Invalid points (because of bad triangulation) will be nan numbers
//RT_Q2T is matrix moving from Train to Query
vector<cv::Point3f> Triangulate(const Frame &Train, const Frame &Query,const cv::Mat &RT_Q2T, const vector<cv::DMatch> &matches, float maxChi2=5.998 );
//computes the reprojection error
double  reprj_error( const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp,const cv::Mat &rv,const cv::Mat &tv);
double  reprj_error(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp, const cv::Mat &rt44, vector<cv::Point2f> *projections=0,vector<float> *errors=0);

//computes the Fundamental matrix from 1
cv::Mat computeF12(const cv::Mat &RT1,const cv::Mat &CameraMatrix1, const cv::Mat &RT2,const cv::Mat &CameraMatrix2=cv::Mat());
inline float epipolarLineSqDist(const cv::Point2f &kp1,const cv::Point2f &kp2,const cv::Mat &F12){
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.x*F12.at<float>(0,0)+kp1.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.x*F12.at<float>(0,1)+kp1.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float den = a*a+b*b;
    if(den==0) return std::numeric_limits<float>::max();
    const float c = kp1.x*F12.at<float>(0,2)+kp1.y*F12.at<float>(1,2)+F12.at<float>(2,2);
    const float num = a*kp2.x+b*kp2.y+c;
    return num*num/den;
}


//computes mean and dev of the given distribution and returns the elements that are outside ntimes the standard dev
vector<int> outlierFiltering(const vector<float> &data, int ntimes, float *mean=0, float *stddev_out=0);
//-------------------------------------------------------------------------------

//computes the reprojection error of the 3d points computed from previous function
double  reprj_error(const ImageParams &ip,const Frame &f1,const Frame &f2, const std::vector<cv::DMatch> &matches,const std::vector<cv::Point3f> &objPoints,std::vector<double> *err=0,const cv::Mat &t21=cv::Mat());

//basic triangulation



double  triangulate ( const vector<cv::Point2f> &points0_und, const cv::Mat & camMatrix0 ,const cv::Mat &T0_,
                      const vector<cv::Point2f> &points1_und, const cv::Mat & camMatrix1 ,const cv::Mat &T1_,
                      vector<cv::Point3f > &objectPoints,vector<double> *repj_err =0 );


double  triangulate (const vector<cv::Point2f> &points0, const ImageParams &ip0 , const cv::Mat &T0_,
                     const vector<cv::Point2f> &points1, const ImageParams &ip1, const cv::Mat &T1_,
                     vector<cv::Point3f > &objectPoints, vector<double> *repj_err=0);
//Shawn error minimization method
cv::Point3f angular_error_minimization(cv::Point3f point, const vector<se3> &poses_f2g);

//projects assuming no distortion
cv::Point2f project(const cv::Point3f &p3d,const cv::Mat &cameraMatrix_32f,const cv::Mat &RT44__32f);


//projects a point using the projection matrix (CameraMatrix*RT)
inline cv::Point2f fast__project__(const float *K,const float* p3d) {
    float Z_inv=1./ (p3d[0]*K[8]+p3d[1]*K[9]+p3d[2]*K[10]+K[11]);
    return cv::Point2f (Z_inv*(p3d[0]*K[0]+p3d[1]*K[1]+p3d[2]*K[2]+K[3]),Z_inv*(p3d[0]*K[4]+p3d[1]*K[5]+p3d[2]*K[6]+K[7]) );
}

inline cv::Point2f fast__project__(const float *K,const cv::Point3f &p3d) {
    float Z_inv=1./ (p3d.x*K[8]+p3d.y*K[9]+p3d.z*K[10]+K[11]);
    return cv::Point2f (Z_inv*(p3d.x*K[0]+p3d.y*K[1]+p3d.z*K[2]+K[3]),Z_inv*(p3d.x*K[4]+p3d.y*K[5]+p3d.z*K[6]+K[7]) );
}
cv::Mat getFastProjectK( const  cv::Mat &CameraMatrix,const cv::Mat &RT);


//projects points using no distortion model
void project(const vector<cv::Point3f> &p3d, const cv::Mat &cameraMatrix_32f, const cv::Mat &RT44__32f, vector<cv::Point2f> &p2d);

//removes distortion of points_io. If pout!=0, results are set there. Otherwise, points_io is modified with the new points
void undistortPoints(   vector<cv::Point2f> &points_io,const ImageParams &ip,vector<cv::Point2f> *pout=0);

//Multiplication of m by p. m is a 4x4 matrix
template <typename T,typename Point3dType>
Point3dType mult ( const cv::Mat &m,  Point3dType  p ) {
    assert ( m.isContinuous() );
    assert ( ( m.type() ==CV_32F && sizeof ( T ) ==4 ) || ( m.type() ==CV_64F && sizeof ( T ) ==8 ) );

    const T *ptr=m.ptr<T> ( 0 );
    Point3dType res;
    res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
    res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
    res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
    return res;
}

inline cv::Point3f operator* ( const cv::Mat &m, const  cv::Point3f  & p ){

    assert ( m.isContinuous()  && m.type() ==CV_32F  &&m.total()>=12 );
    const float *ptr=m.ptr<float> ( 0 );
    return cv::Point3f(  ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3],
            ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7],
            ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11]);
}

inline cv::Point3f mult ( const cv::Mat &m, const cv::Point3f & p ){
    assert ( m.isContinuous() );
    assert ( ( m.type() ==CV_32F  ) );
    cv::Point3f  res;
        const float *ptr=m.ptr<float> ( 0 );
        res.x= ptr[0]*p.x +ptr[1]*p.y +ptr[2]*p.z+ptr[3];
        res.y= ptr[4]*p.x +ptr[5]*p.y +ptr[6]*p.z+ptr[7];
        res.z= ptr[8]*p.x +ptr[9]*p.y +ptr[10]*p.z+ptr[11];
    return res;
}
/**A solution computed from homography or fundamental matrices
  */
struct RTSolution{
    std::vector<cv::Point3f> points3d;
    cv::Mat Rt;
    double repr_error;
    double positivePointsRatio;
};
enum FILTER_METHOD{FM_HOMOGRAPHY=0,FM_FUNDAMENTAL=1};



//given the vector (rx,ry,rz,tx,ty,tz) obtains the resulting matrix
template<typename T>
inline void createRTMatrix(const T *RxyzTxyz,T *matres){

    T nsqa=RxyzTxyz[0]*RxyzTxyz[0] + RxyzTxyz[1]*RxyzTxyz[1] + RxyzTxyz[2]*RxyzTxyz[2];
    T a=std::sqrt(nsqa);
    T i_a=a?1./a:0;
    T rnx=RxyzTxyz[0]*i_a;
    T rny=RxyzTxyz[1]*i_a;
    T rnz=RxyzTxyz[2]*i_a;
    T cos_a=cos(a);
    T sin_a=sin(a);
    T _1_cos_a=1.-cos_a;
    matres[0]=cos_a+rnx*rnx*_1_cos_a;
    matres[1]=rnx*rny*_1_cos_a- rnz*sin_a;
    matres[2]=rny*sin_a + rnx*rnz*_1_cos_a;
    matres[3]=RxyzTxyz[3];

    matres[4]=rnz*sin_a +rnx*rny*_1_cos_a;
    matres[5]= cos_a+rny*rny*_1_cos_a;
    matres[6]= -rnx*sin_a+ rny*rnz*_1_cos_a;
    matres[7]=RxyzTxyz[4];

    matres[8]= -rny*sin_a + rnx*rnz*_1_cos_a;
    matres[9]= rnx*sin_a + rny*rnz*_1_cos_a;
    matres[10]=cos_a+rnz*rnz*_1_cos_a;
    matres[11]=RxyzTxyz[5];

    matres[12]=matres[13]=matres[14]=0;
    matres[15]=1;

}

//100x faster

template<typename T>
void invTMatrix(T *M,T *Minv){

    Minv[0]=M[0];
    Minv[1]=M[4];
    Minv[2]=M[8];
    Minv[4]=M[1];
    Minv[5]=M[5];
    Minv[6]=M[9];
    Minv[8]=M[2];
    Minv[9]=M[6];
    Minv[10]=M[10];

    Minv[3] = -  ( M[3]*Minv[0]+M[7]*Minv[1]+M[11]*Minv[2]);
    Minv[7] = -  ( M[3]*Minv[4]+M[7]*Minv[5]+M[11]*Minv[6]);
    Minv[11]= -  ( M[3]*Minv[8]+M[7]*Minv[9]+M[11]*Minv[10]);
    Minv[12]=Minv[13]=Minv[14]=0;
    Minv[15]=1;

}
//10x faster
inline void matmul(const float *a,const float *b,float *c){

    c[0]= a[0]*b[0]+ a[1]*b[4]+a[2]*b[8];
    c[1]= a[0]*b[1]+ a[1]*b[5]+a[2]*b[9];
    c[2]= a[0]*b[2]+ a[1]*b[6]+a[2]*b[10];
    c[3]= a[0]*b[3]+ a[1]*b[7]+a[2]*b[11]+a[3];

    c[4]= a[4]*b[0]+ a[5]*b[4]+a[6]*b[8];
    c[5]= a[4]*b[1]+ a[5]*b[5]+a[6]*b[9];
    c[6]= a[4]*b[2]+ a[5]*b[6]+a[6]*b[10];
    c[7]= a[4]*b[3]+ a[5]*b[7]+a[6]*b[11]+a[7];

    c[8]=  a[8]*b[0]+ a[9]*b[4]+a[10]*b[8];
    c[9]=  a[8]*b[1]+ a[9]*b[5]+a[10]*b[9];
    c[10]= a[8]*b[2]+ a[9]*b[6]+a[10]*b[10];
    c[11]= a[8]*b[3]+ a[9]*b[7]+a[10]*b[11]+a[11];
    c[12]=c[13]=c[14]=0;
    c[15]=1;

}
//64x faster than using opencv method
//converts a 4x4 transform matrix to rogrigues and translation vector rx,ry,rz,tx,ty,tz

inline void Mat44ToRTVec(const float *M44,float *rvec){

        double theta, s, c;
        cv::Point3d r( M44[9] - M44[6], M44[2] - M44[8] , M44[4]  - M44[1]);

        s = std::sqrt((r.x*r.x + r.y*r.y + r.z*r.z)*0.25);
        c = (M44[0]  +M44[5]  +M44[10] - 1)*0.5;
        c = c > 1. ? 1. : c < -1. ? -1. : c;
        theta = acos(c);

        if( s < 1e-5 )
        {
            double t;

            if( c > 0 ) r.x=r.y=r.z=0;
            else
            {
                t = (M44[0]  + 1)*0.5;
                r.x = std::sqrt(std::max(t,0.));
                t = (M44[5] + 1)*0.5;
                r.y = std::sqrt(std::max(t,0.))*(M44[1] < 0 ? -1. : 1.);
                t = (M44[10] + 1)*0.5;
                r.z = std::sqrt(std::max(t,0.))*(M44[2]  < 0 ? -1. : 1.);
                if( fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (M44[6]  > 0) != (r.y*r.z > 0) )
                    r.z = -r.z;
                theta /=  sqrt(r.x*r.x+r.y*r.y+r.z*r.z);
                r *= theta;
            }


        }
        else
        {
            double vth = 1/(2*s);
            vth *= theta;
            r *= vth;
        }

        rvec[0]=r.x;
        rvec[1]=r.y;
        rvec[2]=r.z;
        rvec[3]=M44[3];
        rvec[4]=M44[7];
        rvec[5]=M44[11];
}


//returns the matrix moving points from org to dst
cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst, bool mbFixScale,double *err=nullptr);
std::pair<cv::Mat,cv::Mat> rigidBodyTransformation_Horn1987_2(const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst, bool mbFixScale, double *err=nullptr);


uint countCommonMapPoints(const Frame & f1, const Frame & f2);

//Old code
//filter the input matches method=0(homography), 1 fundamental matrix
//filters matches using either homography or fundamental matrix
std::vector<cv::DMatch> filter_matches(const vector<cv::KeyPoint> &f1, const vector<cv::KeyPoint> &f2, const vector<cv::DMatch> &matches, FILTER_METHOD method, cv::Mat &result, float ransacReprojThreshold=2.f);
std::vector<cv::DMatch> filter_matches(const vector<cv::Point2f> &p1, const vector<cv::Point2f> &p2, const vector<cv::DMatch> &matches,  FILTER_METHOD method, cv::Mat &result,float ransacReprojThreshold);



//--------------
void savePointsToPCD(const std::vector<cv::Point3f> &points, std::string filename, cv::Scalar color=cv::Scalar(255,0,0));



//---------------- io utils


//------------- aruco
//given a set of markers in two views, returns the most likely location of the camera
cv::Mat ARUCO_initialize(const std::vector<ucoslam::MarkerObservation> &markers1,
                                      const std::vector<ucoslam::MarkerObservation> &markers2,
                                      const ucoslam::ImageParams &cp, float markerSize,
                                      float  minCornerPixDist, float repj_err_Thres, float minDistance,
                                      std::map<uint32_t,se3> &_marker_poses);    //in the global reference system of each marker. Computed in  getRtFromMarkerSet

//estimates the best marker pose when observed from multiple views
//if can estimate a reliable pose returns the marker pose g2m
cv::Mat ARUCO_bestMarkerPose(const vector<ucoslam::MarkerObservation> &marker_views, const vector<se3> &frameposes_f2g, const ucoslam::ImageParams &cp,float markerSize);
//given a marker observed in multiple locations, returns the best pose in each view by analyzing pair
//obtains the position that minimizes the reprojection errors
//vector<cv::Mat> ARUCO_correctMarkerPoses(const vector<aruco::Marker> &marker_views, const ucoslam::ImageParams &cp,float markerSize);




template<typename T,typename T2>
void removeFromVector(vector<T> &elements,const vector<T2> &indicesToRemove){
    vector<bool> indices(elements.size(),true);
    for(auto i:indicesToRemove) indices[i]=false;
    vector<T> elements2;elements2.reserve(elements.size());
    for(size_t i=0;i<indices.size();i++){
        if ( indices[i])
            elements2.push_back(elements[i]);
    }
    elements=elements2;
//    memcpy(&<elements[0],&elements2[0],elements2.size()*sizeof(elements2[0]));
}

//computes mean and stddev of a vector
template<typename T>
pair<double,double> getMeanAndStdDev(const vector<T> &data){

    double sum=0,sq_sum=0,nvalid=0;
    for(auto datum:data){
        if (!isnan(datum)){
            sum+=datum;
            sq_sum+= datum*datum;
            nvalid++;
        }
    }
    double mean = sum / nvalid;
    double variance = sq_sum / nvalid - mean * mean;
    double stddev=sqrt(variance);
    return {mean,stddev};

}
template<typename T>
struct zeroinitvar{
    operator T& (){return val;}
    operator T ()const{return val;}
    void operator++(int){val++;}
    T val=0;
};

}




#endif
