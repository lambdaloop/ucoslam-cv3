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
#include "misc.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "debug.h"
#include "map_types/marker.h"
#include "map_types/frame.h"
#include "optimization/ippe.h"
using namespace std;
namespace ucoslam{

/**
   * Given a Rotation and a Translation expressed both as a vector, returns the corresponding 4x4 matrix
   */
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
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

/**
    * @brief getRTfromMatrix44
    * @param M
    * @param R
    * @param T
    * @param useSVD
    */
void getRTfromMatrix44 ( const cv::Mat &M,  cv::Mat &R,cv::Mat &T,bool useSVD) {

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

void remove_unused_matches(std::vector<cv::DMatch> &matches ){
    matches.erase(std::remove_if(matches.begin(),matches.end(), [](const cv::DMatch &m){return m.trainIdx==-1 || m.queryIdx==-1;}), matches.end());
}
void remove_bad_matches(std::vector<cv::DMatch> &matches ,const vector<bool> &vBadMatches){
    assert(matches.size()==vBadMatches.size());
    for(size_t i=0;i<matches.size();i++)
        if (vBadMatches[i]) matches[i].trainIdx=-1;
    matches.erase(std::remove_if(matches.begin(),matches.end(), [](const cv::DMatch &m){return m.trainIdx==-1 || m.queryIdx==-1;}), matches.end());

}


void filter_ambiguous_query(  std::vector<cv::DMatch> &matches ){
    if (matches.size()==0)return;
    //determine maximum values of queryIdx
    int maxT=-1;
    for(auto m:matches)   maxT=std::max(maxT,m.queryIdx);

    //now, create the vector with the elements
    vector<int> used(maxT+1,-1);
    vector<cv::DMatch> best_matches(maxT);
    int idx=0;
    bool needRemove=false;

    for(auto &match:matches ){
        if (used[match.queryIdx]==-1){
            used[match.queryIdx]=idx;
        }
        else{
            if ( matches[ used[match.queryIdx] ].distance>match.distance){
                matches[ used[match.queryIdx] ].queryIdx=-1;//annulate the other match
                used[match.queryIdx]=idx;
                needRemove=true;
            }
            else{
                match.queryIdx=-1;//annulate this match
                needRemove=true;
            }
        }
        idx++;
    }


    if (needRemove) remove_unused_matches(matches);

}


void filter_ambiguous_train(  std::vector<cv::DMatch> &matches ){
    if (matches.size()==0)return;
    //determine maximum values of train
    int maxT=-1;
    for(auto m:matches)
        maxT=std::max(maxT,m.trainIdx);

    //now, create the vector with the elements
    vector<int> used(maxT+1,-1);
    vector<cv::DMatch> best_matches(maxT);
    int idx=0;
    bool needRemove=false;

    for(auto &match:matches ){
        if (used[match.trainIdx]==-1){
            used[match.trainIdx]=idx;
        }
        else{
            if ( matches[ used[match.trainIdx] ].distance>match.distance){
                matches[ used[match.trainIdx] ].trainIdx=-1;//annulate the other match
                used[match.trainIdx]=idx;
                needRemove=true;
            }
            else{
                match.trainIdx=-1;//annulate this match
                needRemove=true;

            }
        }
        idx++;
    }

    if (needRemove) remove_unused_matches(matches);

}

/**
 * @brief filter_matches
 * @param f1
 * @param f2
 * @param matches
 * @param method
 * @param result
 * @param ransacReprojThreshold
 * @param ip
 * @return
 */


vector<cv::DMatch> filter_matches(const vector<cv::Point2f> &p1, const vector<cv::Point2f> &p2, const vector<cv::DMatch> &matches,  FILTER_METHOD method, cv::Mat &result,float ransacReprojThreshold){
    cv::Mat inlier_mask;
    vector<cv::DMatch> inlier_matches;
    if (method==FM_HOMOGRAPHY)
        result=cv::findHomography ( p1, p2, cv::RANSAC, ransacReprojThreshold, inlier_mask);
    else      {
         result=cv::findFundamentalMat ( p1,p2,cv::FM_RANSAC,ransacReprojThreshold,0.99 ,inlier_mask );
    }

    if (!result.empty()){
        for(unsigned i = 0; i < matches.size(); i++)
            if(inlier_mask.at<uchar>(i))
                inlier_matches.push_back(matches[i]);

    }
    return inlier_matches;
}


vector<cv::DMatch> filter_matches(const vector<cv::KeyPoint> &f1, const vector<cv::KeyPoint> &f2, const vector<cv::DMatch> &matches,  FILTER_METHOD method, cv::Mat &result,float ransacReprojThreshold  ){
    //remove outliers by finding homography
    std::vector<cv::Point2f> p1, p2;
    p1.reserve(matches.size());
    p2.reserve(matches.size());
    for(auto m:matches){
        p1.push_back(  f1[ m.queryIdx].pt  );
        p2.push_back(  f2[ m.trainIdx].pt  );
    }
   return filter_matches(p1,p2,matches,method,result,ransacReprojThreshold);
}


double  reprj_error(const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp, const cv::Mat &rt44, vector<cv::Point2f> *projections, vector<float> *errors){
    std::vector<cv::Point2f> prepj_;
    if (projections==0) projections=&prepj_;
    project(objPoints,imp.CameraMatrix,rt44,*projections);

    if (errors!=0) errors->resize(projections->size());
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<projections->size();i++){
        if ( !isnan(objPoints[i].x)){
            float err=cv::norm( points2d[i]-(*projections)[i]);
             if (errors) (*errors)[i]=err;
             sum+= err;
             nvalid++;
        }
    }
    return sum/double(nvalid);

}

double  reprj_error( const std::vector<cv::Point3f> &objPoints, const std::vector<cv::Point2f>points2d, const ImageParams &imp,const cv::Mat &rv,const cv::Mat &tv){

    std::vector<cv::Point2f> prepj;
    project(objPoints,imp.CameraMatrix,getRTMatrix(rv,tv),prepj);
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<prepj.size();i++){
        if ( !isnan(objPoints[i].x)){
             sum+= cv::norm( points2d[i]-prepj[i]);
            nvalid++;
        }
    }
    return sum/double(nvalid);
}

void undistortPoints(   vector<cv::Point2f> &points_io,const ImageParams &ip,vector<cv::Point2f> *out){
    std::vector<cv::Point2f> pout;
    if (out==0) out=&pout;

    assert(ip.CameraMatrix.type()==CV_32F);
    cv::undistortPoints ( points_io, *out,ip.CameraMatrix, ip.Distorsion);//results are here normalized. i.e., in range [-1,1]

    float fx=ip.CameraMatrix.at<float> ( 0,0 );
    float fy=ip.CameraMatrix.at<float> ( 1,1 );
    float cx=ip.CameraMatrix.at<float> ( 0,2 );
    float cy=ip.CameraMatrix.at<float> ( 1,2 );

    if (out==&pout){
        for ( size_t i=0; i<pout.size(); i++ ) {
            points_io[i].x=pout[i].x*fx+cx;
            points_io[i].y=pout[i].y*fy+cy;
        }
    }
    else{
        for ( size_t i=0; i<pout.size(); i++ ) {
            (*out)[i].x=(*out)[i].x*fx+cx;
            (*out)[i].y=(*out)[i].y*fy+cy;
        }
    }
}

double  reprj_error( const ImageParams &ip,const Frame &f1,const Frame &f2, const std::vector<cv::DMatch> &matches, const std::vector<cv::Point3f> &objPoints,std::vector<double> *repj_err,const cv::Mat &t21){
    if (f1.und_kpts.size()==0 || f2.und_kpts.size()==0) throw std::runtime_error("No undistorted key points");
    if (objPoints.size()!= matches.size()) throw std::runtime_error("object points and matches must have equal size");


    //compute the reproj err
    std::vector<cv::Point2f> prepj_0,prepj_1;
    cv::Mat rv[2],tv[2];
    if (!t21.empty()){
        rv[0]=cv::Mat::zeros(1,3,CV_32F);
        tv[0]=cv::Mat::zeros(1,3,CV_32F);
        getRTfromMatrix44(t21,rv[1],tv[1]);
    }
    else{
        rv[0]=f1.pose_f2g.getRvec();
        tv[0]=f1.pose_f2g.getTvec();
        rv[1]=f2.pose_f2g.getRvec();
        tv[1]=f2.pose_f2g.getTvec();
    }


    project(objPoints,ip.CameraMatrix,getRTMatrix(rv[0],tv[0]) ,prepj_0);
    project(objPoints,ip.CameraMatrix,getRTMatrix(rv[1],tv[1]) ,prepj_1);

    if (repj_err!=0) repj_err->resize(objPoints.size());
    double sum=0;
    int nvalid=0;
    for(size_t i=0;i<matches.size();i++){
        if (!isnan(objPoints[i].x)){
            const auto &m=matches[i];
            auto err1=cv::norm( f1.und_kpts[m.queryIdx].pt-prepj_0[i]);
            auto err2=cv::norm( f2.und_kpts[m.trainIdx].pt-prepj_1[i]);
            sum+=err1+err2 ;
            //cout<<err1<<" "<<err2<<endl;
            if(repj_err!=0)  repj_err->at(i)=(0.5*(err1+err2));
            nvalid+=2;
        }
        else repj_err->at(i)=std::numeric_limits<float>::max();
    }
    return sum/double(nvalid);

}




cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst,bool mbFixScale,double *err){
    return rigidBodyTransformation_Horn1987_2(org,dst,mbFixScale,err).first;
}

std::pair<cv::Mat,cv::Mat> rigidBodyTransformation_Horn1987_2(const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst, bool mbFixScale,double *err){

    auto ComputeCentroid=[](cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,cv::REDUCE_SUM);
        C = C/P.cols;
        for(int i=0; i<P.cols; i++)
            Pr.col(i)=P.col(i)-C;
    };

    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    //Create the P1 an P2 matrices
    cv::Mat P1(3,org.size(),CV_32F);
    cv::Mat P2(3,org.size(),CV_32F);
    for(size_t i=0;i<org.size();i++){
        P1.at<float>(0,i)=org[i].x;
        P1.at<float>(1,i)=org[i].y;
        P1.at<float>(2,i)=org[i].z;
        P2.at<float>(0,i)=dst[i].x;
        P2.at<float>(1,i)=dst[i].y;
        P2.at<float>(2,i)=dst[i].z;
    }


        // Step 1: Centroid and relative coordinates

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        if (norm(vec)<1e-7)return {cv::Mat::eye(4,4,CV_32F),cv::Mat::eye(4,4,CV_32F)};

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        cv::Mat mR12i(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale
        float ms12i;

        if(!mbFixScale)
        {
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            ms12i = nom/den;
        }
        else
            ms12i = 1.0f;

        // Step 7: Translation

        cv::Mat  mt12i(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
//        return mT12i;

//        // Step 8.2 T21

        cv::Mat mT21i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        cv::Mat tinv = -sRinv*mt12i;
        tinv.copyTo(mT21i.rowRange(0,3).col(3));

        if(err!=nullptr){
            *err=0;
            for(size_t i=0;i<org.size();i++){
                    auto res=mT21i*org[i];
                    *err+= cv::norm( dst[i]-res);
            }
            *err/=double(org.size());
        }

        return { mT21i,mT12i};
}


uint countCommonMapPoints(const Frame & f1, const Frame & f2)
{
   uint ncommon = 0;
   auto idsF = f1.ids;

   if (f1.idx != f2.idx)
   {
       auto idsKF = f2.ids;
       for (uint i = 0; i < idsF.size(); i++)
           for (uint j = 0; j < idsKF.size(); j++)
           {
              if (idsF[i] != std::numeric_limits<uint32_t>::max() && idsF[i] == idsKF[j])
                  ncommon++;
           }
   }
   else
       ncommon = f1.ids.size();

   return ncommon;
}



void savePointsToPCD(const std::vector<cv::Point3f> &points, string filename, cv::Scalar color)
{
    auto convertP3DtoV4=[](cv::Point3f p,cv::Scalar color){

        float fcolor;uchar *c=(uchar*)&fcolor;

        for(int i=0;i<3;i++)c[i]=color[i];
        return cv::Vec4f(p.x,p.y,p.z,fcolor);
    };

   vector<cv::Vec4f> pcdpoints;

   std::ofstream filePCD ( filename, std::ios::binary );
   if (!filePCD.is_open())
   {
      std::cerr << "Could not open file: "  << filename << std::endl;
   }

   // Prepare points
   pcdpoints.reserve(points.size());
   for (uint i = 0; i < points.size(); i++)
   {
      pcdpoints.push_back(convertP3DtoV4(points[i], color));
   }


   // Start PCD file
   filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<pcdpoints.size()<<"\nHEIGHT 1\nPOINTS "<<pcdpoints.size()<<"\nDATA binary\n";


   filePCD.write((char*)&pcdpoints[0],pcdpoints.size()*sizeof(pcdpoints[0]));

   filePCD.close();
}

cv::Point2f project(const cv::Point3f &p3d,const cv::Mat &cameraMatrix_32f,const cv::Mat &RT44__32f){
    assert(RT44__32f.type()==CV_32F);
    assert(cameraMatrix_32f.type()==CV_32F);
    cv::Point3f res;
    const float *rt=RT44__32f.ptr<float>(0);
    res.x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
    res.y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
    res.z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
    //now, project
    const float *cam=cameraMatrix_32f.ptr<float>(0);
    cv::Point2f r2d;
    r2d.x= (cam[0]*res.x/res.z)+cam[2];
    r2d.y= (cam[4]*res.y/res.z)+cam[5];
    return r2d;
}

void project(const vector<cv::Point3f> &vp3d,const cv::Mat &cameraMatrix_32f,const cv::Mat &RT44__32f,vector<cv::Point2f> &p2dv){
    assert(RT44__32f.type()==CV_32F);
    assert(RT44__32f.total()==16);
    assert(cameraMatrix_32f.type()==CV_32F);
    assert(cameraMatrix_32f.total()==9);
    p2dv.resize(vp3d.size());
    const float *rt=RT44__32f.ptr<float>(0);
    const float *cam=cameraMatrix_32f.ptr<float>(0);
    for(size_t i=0;i<vp3d.size();i++){
        const auto &p3d=vp3d[i];
        float x,y,z;
        x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
        y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
        z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
        //now, project
        auto &p2d=p2dv[i];
        p2d.x= (cam[0]*x/z)+cam[2];
        p2d.y= (cam[4]*y/z)+cam[5];

    }
}
//triangulation method in
//@INPROCEEDINGS{Recker2012,
//author={Recker, Shawn and Hess-Flores, Mauricio and Duchaineau, Mark A. and Joy, Kenneth I.},
//booktitle={Applied Imagery Pattern Recognition Workshop (AIPR), 2012 IEEE},
//title={Visualization of scene structure uncertainty in multi-view reconstruction},
//year={2012},
//pages={1-7},
//doi={10.1109/AIPR.2012.6528216},
//ISSN={1550-5219},}

cv::Point3f angular_error_minimization(cv::Point3f point, const vector<se3> &poses_f2g){
    struct cam{
        cv::Point3f direction,position;
    };

    auto angular_gradient=[]( const vector<cam>& cameras , const cv::Point3f &point)
    {
        cv::Point3f g = cv::Point3f(0,0,0);
        for(auto const &c:cameras){
            cv::Point3f v = point - c.position;

            double denom2 = v.dot(v);
            double denom = sqrt(denom2);
            double denom15 = pow(denom2, 1.5);
            double vdotw = v.dot(c.direction);
            g.x += (-c.direction.x/denom) + ((v.x*vdotw)/denom15);
            g.y += (-c.direction.y/denom) + ((v.y*vdotw)/denom15);
            g.z += (-c.direction.z/denom) + ((v.z*vdotw)/denom15);
        }
        return g;
    };

    //create cameras
    vector<cam> cameras;
    for(auto p:poses_f2g){
        cv::Mat mPose=p.convert().inv();
        cam c;
        c.position =mPose*cv::Point3f(0,0,0);  //obtain camera center in global reference system
        auto v1=mPose*cv::Point3f(0,0,1);
        auto vd=v1-c.position;
        c.direction=  vd * (1./cv::norm(vd));
         cameras.push_back(c);
      }


    double precision = 1e-25;
        cv::Point3f g_old;
      cv::Point3f x_old;
      cv::Point3f x_new =point;;
      cv::Point3f grad = angular_gradient(cameras,x_new);
      double epsilon = .001;
      double diff;
      int count = 150;
      do {
        x_old = x_new;
        g_old = grad;
        x_new = x_old - epsilon * g_old;
        grad = angular_gradient(cameras, x_new);
        cv::Point3f sk = x_new - x_old;
        cv::Point3f yk = grad - g_old;
        double skx = sk.x;
        double sky = sk.y;
        double skz = sk.z;
        diff = skx*skx+sky*sky+skz*skz;
        //Compute adaptive step size (sometimes get a divide by zero hence
        //the subsequent check)
        epsilon = diff/(skx*yk.x+sky*yk.y+skz*yk.z);
        epsilon = (epsilon != epsilon) ||
          (epsilon == numeric_limits<double>::infinity()) ? .001 : epsilon;
        --count;
      } while(diff > precision && count-- > 0);
      if(isnan(x_new.x ) || isnan(x_new.y ) || isnan(x_new.z )) {
        return point;
      }
      return  x_new;
}


cv::Mat  ARUCO_initialize(const std::vector<ucoslam::MarkerObservation> &markers1,
                                      const std::vector<ucoslam::MarkerObservation> &markers2,
                                      const ucoslam::ImageParams &cp, float markerSize,
                                      float  minCornerPixDist,float repj_err_Thres, float minDistance,
                                      std::map<uint32_t,se3> &_marker_poses){

    const std::vector<ucoslam::MarkerObservation> &ms1_in=markers1;
    const std::vector<ucoslam::MarkerObservation> &ms2_in=markers2;

    //find matches of markers in both images
    vector< std::pair<uint32_t,uint32_t> > matches;

    for(size_t i=0;i<ms1_in.size();i++){
        for(size_t j=0;j<ms2_in.size();j++)
            if ( ms1_in[i].id==ms2_in[j].id) matches.push_back(make_pair(i,j));
    }
    if (matches.size()==0)return cv::Mat();

    //create vectors with matched elements so that the elements are in order in both ms1 and ms2
    std::vector<ucoslam::MarkerObservation> ms1,ms2;
    for(auto m:matches){
        ms1.push_back( ms1_in[m.first]);
        ms2.push_back( ms2_in[m.second]);
    }
    std::vector<cv::Point2f> p2d_v1,p2d_v2;
    for(auto m: ms1)p2d_v1.insert(p2d_v1.end(),m.und_corners.begin(),m.und_corners.end());
    for(auto m: ms2)p2d_v2.insert(p2d_v2.end(),m.und_corners.begin(),m.und_corners.end());

    //compute average pixel distance to avoid computing from too near views
    float avrgPixDist=0;//std::numeric_limits<float>::max();
    for(size_t i=0;i<p2d_v1.size();i++)
        avrgPixDist+= float(cv::norm(p2d_v1[i]-p2d_v2[i]));
    avrgPixDist/=float(p2d_v1.size());
  _debug_msg("avrgPixDist=" << avrgPixDist,10);
  //too near?
  if (avrgPixDist< minCornerPixDist*cp.CamSize.width ) return cv::Mat();




    //compte the marker poses (both the good and other one)
     std::vector<std::vector<std::pair<cv::Mat,double>>> marker_poses_v1,marker_poses_v2;
     _debug_msg(cp.Distorsion,10);
    for(size_t i=0;i<ms1.size();i++){
        marker_poses_v1.push_back(IPPE::solvePnP_(markerSize,ms1[i].und_corners,cp.CameraMatrix,cp.Distorsion));
        marker_poses_v2.push_back(IPPE::solvePnP_(markerSize,ms2[i].und_corners,cp.CameraMatrix,cp.Distorsion));
    }


    _debug_exec(10,for(size_t i=0;i<marker_poses_v1.size();i++)cout<<marker_poses_v1[i][1].second/marker_poses_v1[i][0].second<< "-" <<marker_poses_v2[i][1].second/marker_poses_v2[i][0].second<<endl;);

   auto get_marker_points=[](float ms){
        return vector<cv::Point3f>({ cv::Point3f(-ms/2.,ms/2.,0), cv::Point3f(ms/2.,ms/2.,0),
                    cv::Point3f(ms/2.,-ms/2.,0),cv::Point3f(-ms/2.,-ms/2.,0)});
    };




//     auto repj_err=[&](const cv::Mat &rt_totest){
//        vector<cv::Point3f> p3d;
//        vector<double> errv;
//        auto err=triangulate(p2d_v1,cp,cv::Mat::eye(4,4,CV_32F), p2d_v2,cp,rt_totest,p3d,&errv);
//        return err;

//    };

  auto repj_err=[&](const cv::Mat &rt_totest){
      //for each marker, get the 3d points in camera 2, and project to camera 1 using rt_totest
     double sumerr=0;
     for(size_t i=0;i<ms1.size();i++){
         vector<cv::Point3f> p3d=get_marker_points(markerSize);//3d points in marker ref system
         //move points to v2
         for(auto &p:p3d) p=marker_poses_v1[i][0].first*p; //move to v1
         for(auto &p:p3d) p=rt_totest*p; //move to v2
         //now, project and get the error
         sumerr+=reprj_error( p3d, ms2[i].und_corners,cp,cv::Mat::eye(4,4,CV_32F));
     }
     return sumerr/float(ms1.size());

 };

    //now, calculate all possible solutions and evaluate them
    struct pinfo{
        cv::Mat v1_c2m,v2_c2m;
        cv::Mat rt; // from 1 -> 2, i.e., from global -> 2
        double err;
    };
    vector<pinfo> sol_err;
    for(size_t i=0;i<ms1.size();i++){
        for(size_t j=0;j<2;j++){
            for(size_t k=0;k<2;k++){
                pinfo pi;
                pi.v1_c2m=marker_poses_v1[i][k].first;
                pi.v2_c2m=marker_poses_v2[i][j].first;
                pi.rt=   pi.v2_c2m*pi.v1_c2m.inv();

                pi.err=repj_err(pi.rt);
                if(!std::isnan(pi.err) && !std::isinf(pi.err)) sol_err.push_back(pi);
            }
        }
    }

    if (sol_err.size()==0)return cv::Mat();

    std::sort(sol_err.begin(),sol_err.end(),[](const pinfo&a,const pinfo &b){return a.err<b.err;});
     _debug_exec(10, for(auto pi:sol_err)cout<<"pi.err="<<pi.err<<" ";cout<<endl;);
     //has a low repoj error?
     auto best_sol=sol_err.front();
     if( best_sol.err < repj_err_Thres){
         //has enough distance between the views??
         auto curBaseLine=cv::norm(best_sol.rt.rowRange(0,3).colRange(3,4));
         if (curBaseLine>=minDistance){

             for(size_t i=0;i<ms1.size();i++){
                 //solution a, project and get error
                 auto p3d=get_marker_points(markerSize);
                 //move points to v2 and then back to v1
                 cv::Mat pose1 = best_sol.rt.inv()*marker_poses_v2[i][0].first;
                 for(auto &p:p3d) p=pose1*p;
                 //finall, get repr err in v1
                 auto err1= reprj_error( p3d, ms1[i].und_corners,cp,cv::Mat::eye(4,4,CV_32F));
                 //repeat in inverse order
                 p3d=get_marker_points(markerSize);
                 //move points to v1 and then back to v2
                 cv::Mat pose2=best_sol.rt*marker_poses_v1[i][0].first;
                 for(auto &p:p3d) p=pose2*p;
                 //finall, get repr err in v1
                 auto err2= reprj_error( p3d, ms2[i].und_corners,cp,cv::Mat::eye(4,4,CV_32F));
                 _debug_msg("err1_2:"<<err1<<" "<<err2,10);
                 //get the best solution
                 if (err1<err2){
                      _marker_poses.insert({ms1[i].id,  se3(pose1) });
                 }
                 else
                     _marker_poses.insert({ms1[i].id, se3(marker_poses_v1[i][0].first)});


             }
             return best_sol.rt;
         }

     }
     return   cv::Mat();
}

cv::Mat ARUCO_bestMarkerPose(const vector<ucoslam::MarkerObservation> &marker_views, const vector<se3> &frameposes_f2g, const ucoslam::ImageParams &cp, float markerSize){
    
    assert( marker_views.size()>=2);
    
    //create all possible solutions from frame to global
    struct poseinfo{
        double err=0;
        cv::Mat pose_g2m;
    };
    vector<poseinfo> solutions;
    
    for(size_t i=0;i<marker_views.size();i++){
        auto ss=  IPPE::solvePnP_(markerSize,marker_views[i].und_corners,cp.CameraMatrix,cp.Distorsion);
        poseinfo pi;
        
        pi.pose_g2m= frameposes_f2g[i].convert().inv() * ss[0].first; // g2f  * f2m
        solutions.push_back(pi);
        //            pi.pose_g2m= frameposes_f2g[i].convert().inv() * ss[1].first; // g2f  * f2m
        //            solutions.push_back(pi);
    }
    //now, compute the reproj error and get the best
    auto p3d=ucoslam::Marker::get3DPoints(Se3Transform(),markerSize,false);
    for( auto &sol:solutions)
    {
        //        sol.err=std::numeric_limits<float>::min();
        for(size_t i=0;i<frameposes_f2g.size();i++)
            sol.err+= reprj_error(p3d,marker_views[i].und_corners,cp,frameposes_f2g[i].convert()*sol.pose_g2m);
        //sol.err=std::max( sol.err,  reprj_error(p3d,marker_views[i],cp,frameposes_f2g[i].convert()*sol.pose_g2m));
        sol.err/=float(frameposes_f2g.size());
    }
    std::sort(solutions.begin(),solutions.end(),[](const poseinfo&a,const poseinfo&b){return a.err<b.err;});
    _debug_msg("repj  err="<<solutions.front().err<<" "<<solutions.back().err,10);
    return solutions.front().pose_g2m;
}

vector<int> outlierFiltering(const vector<float> &data,int ntimes,float *mean_out,float *stddev_out){
    //calculate mean and dev
    float sum=0,sq_sum=0,nvalid=0;
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


    float thres=mean+2*stddev;

    vector<int> outliers;
    outliers.reserve(float(data.size())*0.2);
    for(size_t i=0;i<data.size();i++){
        if (!isnan(data[i])){
            if (data[i]>thres)
                outliers.push_back( i);
        }
        else  outliers.push_back( i);
    }
    if (mean_out!=0)*mean_out=mean;
    if (stddev_out!=0) *stddev_out=stddev;
    return outliers;
}
  cv::Mat getFastProjectK( const  cv::Mat &CameraMatrix,const cv::Mat &RT){
    assert(CameraMatrix.type()== RT.type() &&  RT.type() ==CV_32F);
    //precomputed projection matrix lhs
    //  |fx 0   cx |   |1 0 0 0|
    //  |0  fy  cy | * |0 1 0 0|
    //  |0  0   1  |   |0 0 1 0|
    float m[16]={CameraMatrix.at<float>(0,0),0,CameraMatrix.at<float>(0,2),0,
                 0,CameraMatrix.at<float>(1,1),CameraMatrix.at<float>(1,2),0,
                 0,0,1,0,
                 0,0,0,1};
    cv::Mat   LPm (4,4,CV_32F,m);
    return LPm*RT;
}

  cv::Mat computeF12(const cv::Mat &RT1,const cv::Mat &CameraMatrix1, const cv::Mat &RT2,const cv::Mat &_CameraMatrix2)
   {

      cv::Mat K2;

      if (_CameraMatrix2.empty())
          K2=CameraMatrix1;
      else K2=_CameraMatrix2;
      cv::Mat R1w = RT1(cv::Range(0,3),cv::Range(0,3));//pKF1->GetRotation();
      cv::Mat t1w = RT1(cv::Range(0,3),cv::Range(3,4));// ;pKF1->GetTranslation();
      cv::Mat R2w = RT2(cv::Range(0,3),cv::Range(0,3));//pKF2->GetRotation();
      cv::Mat t2w = RT2(cv::Range(0,3),cv::Range(3,4));//pKF2->GetTranslation();

      cv::Mat R12 = R1w*R2w.t();
      cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

      float d[9]={0, -t12.at<float>(2), t12.at<float>(1),
                  t12.at<float>(2),               0,-t12.at<float>(0),
                  -t12.at<float>(1),  t12.at<float>(0),              0};

      cv::Mat t12x(3,3,CV_32F,d);// = SkewSymmetricMatrix(t12);




      cv::Mat F12=CameraMatrix1.t().inv()*t12x*R12*K2.inv();
      return F12;
  }


  vector<cv::Point3f> Triangulate(const Frame &Train, const Frame &Query,const cv::Mat &RT_Q2T, const vector<cv::DMatch> &matches, float maxChi2){
      auto  Triangulate=[](const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
      {
          cv::Mat A(4,4,CV_32F);

          A.row(0) = kp1.x*P1.row(2)-P1.row(0);
          A.row(1) = kp1.y*P1.row(2)-P1.row(1);
          A.row(2) = kp2.x*P2.row(2)-P2.row(0);
          A.row(3) = kp2.y*P2.row(2)-P2.row(1);

          cv::Mat u,w,vt;
          cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
          x3D = vt.row(3).t();
          if(x3D.at<float>(3)==0) return false;

          x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
          return true;
      };

      vector<cv::Point3f> vP3D(matches.size(),cv::Point3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN()));

      const auto &CameraMatrix1=Train.imageParams.CameraMatrix;
      const auto &CameraMatrix2=Query.imageParams.CameraMatrix;

      vector<float> invScaleFactorsQuery,invScaleFactorsTrain;
      for(auto f:Train.scaleFactors)           invScaleFactorsQuery.push_back(1.f/(f*f));
      for(auto f:Query.scaleFactors)           invScaleFactorsTrain.push_back(1.f/(f*f));
      cv::Mat R=RT_Q2T(cv::Range(0,3),cv::Range(0,3));
      cv::Mat R_t=R.t();
      cv::Mat t=RT_Q2T(cv::Range(0,3),cv::Range(3,4));

      // Calibration parameters
      const float fx1 = CameraMatrix1.at<float>(0,0);
      const float fy1 = CameraMatrix1.at<float>(1,1);
      float invfy1=1.f/fy1;
      float invfx1=1.f/fx1;
      const float cx1 = CameraMatrix1.at<float>(0,2);
      const float cy1 = CameraMatrix1.at<float>(1,2);

      const float fx2 = CameraMatrix2.at<float>(0,0);
      const float fy2 = CameraMatrix2.at<float>(1,1);
      const float cx2 = CameraMatrix2.at<float>(0,2);
      const float cy2 = CameraMatrix2.at<float>(1,2);
      float invfy2=1.f/fy2;
      float invfx2=1.f/fx2;


      // Camera 1 Projection Matrix K[I|0]
      cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
      CameraMatrix1.copyTo(P1.rowRange(0,3).colRange(0,3));

      // Camera 2 Projection Matrix K[R|t]
      cv::Mat P2(3,4,CV_32F);
      R.copyTo(P2.rowRange(0,3).colRange(0,3));
      t.copyTo(P2.rowRange(0,3).col(3));
      P2 = CameraMatrix2*P2;

      int nGood=0;

       for(  size_t i=0;i<matches.size();i++)
      {
          assert(matches[i].queryIdx>=0 && matches[i].trainIdx>=0);
          const auto&kp1=Train.und_kpts[matches[i].trainIdx];
          const auto&kp2=Query.und_kpts[matches[i].queryIdx];

            // Check parallax between rays
          cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
          cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

          cv::Mat ray1 = xn1/cv::norm(xn1);
          cv::Mat ray2 = R_t*(xn2/cv::norm(xn2));
          const double cosParallaxRays = ray1.dot(ray2);

          if (cosParallaxRays<0 ||  cosParallaxRays>0.9998 )
              continue;

          cv::Mat p3dC1;

          if (!Triangulate(kp1.pt,kp2.pt,P1,P2,p3dC1)) continue;

          if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
              continue;

          // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          if(p3dC1.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          cv::Mat p3dC2 = R*p3dC1+t;

          if(p3dC2.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check reprojection error in first image
          float invZ1 = 1.f/p3dC1.at<float>(2);
          cv::Point2f queryReprj(  fx1*p3dC1.at<float>(0)*invZ1+cx1,fy1*p3dC1.at<float>(1)*invZ1+cy1);
          float chi2 =  invScaleFactorsQuery[kp1.octave] *   ((queryReprj.x-kp1.pt.x)*(queryReprj.x-kp1.pt.x)+(queryReprj.y-kp1.pt.y)*(queryReprj.y-kp1.pt.y));

          if(chi2>maxChi2)
              continue;

          // Check reprojection error in second image
          float invZ2 = 1.f/p3dC2.at<float>(2);
          cv::Point2f trainReprj(fx2*p3dC2.at<float>(0)*invZ2+cx2,fy2*p3dC2.at<float>(1)*invZ2+cy2);

          chi2 = invScaleFactorsTrain[kp2.octave]  * ((trainReprj.x-kp2.pt.x)*(trainReprj.x-kp2.pt.x)+( trainReprj.y-kp2.pt.y)*( trainReprj.y-kp2.pt.y));

          if(chi2>maxChi2)
              continue;

          vP3D[i] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
          nGood++;
      }


      return vP3D;


  }

  int   triangulate_(const cv::Mat &RT, std::vector<cv::KeyPoint> &kp1,std::vector<cv::KeyPoint> &kp2,  const cv::Mat &CameraMatrix1,const cv::Mat &CameraMatrix2, vector<cv::Point3f> &vP3D,  const vector<float> &scaleFactors,vector<bool> &vbGood,float maxChi2 )
  {
      auto  Triangulate=[](const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
      {
          cv::Mat A(4,4,CV_32F);

          A.row(0) = kp1.x*P1.row(2)-P1.row(0);
          A.row(1) = kp1.y*P1.row(2)-P1.row(1);
          A.row(2) = kp2.x*P2.row(2)-P2.row(0);
          A.row(3) = kp2.y*P2.row(2)-P2.row(1);

          cv::Mat u,w,vt;
          cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
          x3D = vt.row(3).t();
          if(x3D.at<float>(3)==0) return false;

          x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
          return true;
      };

      vector<float> invScaleFactors2;
      for(auto f:scaleFactors)           invScaleFactors2.push_back(1.f/(f*f));
      cv::Mat R=RT(cv::Range(0,3),cv::Range(0,3));
      cv::Mat R_t=R.t();
      cv::Mat t=RT(cv::Range(0,3),cv::Range(3,4));

      // Calibration parameters
      const float fx1 = CameraMatrix1.at<float>(0,0);
      const float fy1 = CameraMatrix1.at<float>(1,1);
      float invfy1=1.f/fy1;
      float invfx1=1.f/fx1;
      const float cx1 = CameraMatrix1.at<float>(0,2);
      const float cy1 = CameraMatrix1.at<float>(1,2);

      const float fx2 = CameraMatrix2.at<float>(0,0);
      const float fy2 = CameraMatrix2.at<float>(1,1);
      const float cx2 = CameraMatrix2.at<float>(0,2);
      const float cy2 = CameraMatrix2.at<float>(1,2);
      float invfy2=1.f/fy2;
      float invfx2=1.f/fx2;

      vP3D.resize(kp1.size());
      vbGood=vector<bool>(kp1.size(),false);

      // Camera 1 Projection Matrix K[I|0]
      cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
      CameraMatrix1.copyTo(P1.rowRange(0,3).colRange(0,3));

      // Camera 2 Projection Matrix K[R|t]
      cv::Mat P2(3,4,CV_32F);
      R.copyTo(P2.rowRange(0,3).colRange(0,3));
      t.copyTo(P2.rowRange(0,3).col(3));
      P2 = CameraMatrix2*P2;

      int nGood=0;

      cv::Point3f invalid3d(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
      for(size_t i=0;i<kp1.size();i++)
      {
           vP3D[i]=invalid3d;
          // Check parallax between rays
          cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1[i].pt.x-cx1)*invfx1, (kp1[i].pt.y-cy1)*invfy1, 1.0);
          cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2[i].pt.x-cx2)*invfx2, (kp2[i].pt.y-cy2)*invfy2, 1.0);

          cv::Mat ray1 = xn1/cv::norm(xn1);
          cv::Mat ray2 = R_t*(xn2/cv::norm(xn2));
          const double cosParallaxRays = ray1.dot(ray2);

          if (cosParallaxRays<0 ||  cosParallaxRays>0.9998 )
              continue;

          cv::Mat p3dC1;

          if (!Triangulate(kp1[i].pt,kp2[i].pt,P1,P2,p3dC1)) continue;
           if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
              continue;

          // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          if(p3dC1.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
          cv::Mat p3dC2 = R*p3dC1+t;

          if(p3dC2.at<float>(2)<=0 /*&& cosParallax<0.99998*/)
              continue;

          // Check reprojection error in first image
          float im1x, im1y;
          float invZ1 = 1.0/p3dC1.at<float>(2);
          im1x = fx1*p3dC1.at<float>(0)*invZ1+cx1;
          im1y = fy1*p3dC1.at<float>(1)*invZ1+cy1;

          float chi2 =  invScaleFactors2[kp1[i].octave] * ((im1x-kp1[i].pt.x)*(im1x-kp1[i].pt.x)+(im1y-kp1[i].pt.y)*(im1y-kp1[i].pt.y));
          //          float chi2 =  (im1x-p1[i].x)*(im1x-p1[i].x)+(im1y-p1[i].y)*(im1y-p1[i].y);

          if(chi2>maxChi2)
              continue;

          // Check reprojection error in second image
          float im2x, im2y;
          float invZ2 = 1.0/p3dC2.at<float>(2);
          im2x = fx2*p3dC2.at<float>(0)*invZ2+cx2;
          im2y = fy2*p3dC2.at<float>(1)*invZ2+cy2;

          chi2 = invScaleFactors2[kp2[i].octave]  *  ((im2x-kp2[i].pt.x)*(im2x-kp2[i].pt.x)+(im2y-kp2[i].pt.y)*(im2y-kp2[i].pt.y));

          if(chi2>maxChi2)
              continue;


          vbGood[i]=true;
          vP3D[i] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
          nGood++;
      }


      return nGood;
  }


}
