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
*/#ifndef ucoslam_ImageParmas_H
#define ucoslam_ImageParmas_H
#include <opencv2/core/core.hpp>
#include "ucoslam_exports.h"
namespace ucoslam{

/**
 * @brief The ImageParams class represents the parameters of the employed camera
 */
class  UCOSLAM_API ImageParams {

 public:

     // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Mat CameraMatrix;
    //  distortion matrix
    cv::Mat Distorsion;
    // size of the image
    cv::Size CamSize;


    float bl=0;//stereo camera base line
    float rgb_depthscale=1;//scale to obtain depth from the rgbd values

    ImageParams();
    ImageParams(const ImageParams &IP);
    ImageParams& operator=(const ImageParams& ip) ;

    /**Reads from a YAML file generated with the opencv2.2 calibration utility
     */
    void readFromXMLFile(std::string filePath);
    void saveToXMLFile(std::string path );

    //accessor to the main parameters of the camera model
    inline float fx()const{return  CameraMatrix.ptr<float>(0)[0];}
    inline float cx()const{return  CameraMatrix.ptr<float>(0)[2];}
    inline float fy()const{return  CameraMatrix.ptr<float>(0)[4];}
    inline float cy()const{return  CameraMatrix.ptr<float>(0)[5];}
    inline float k1()const{return (Distorsion.total()>=1?Distorsion.ptr<float>(0)[0]:0);}
    inline float k2()const{return (Distorsion.total()>=2? Distorsion.ptr<float>(0)[1]:0);}
    inline float p1()const{return (Distorsion.total()>=3? Distorsion.ptr<float>(0)[2]:0);}
    inline float p2()const{return (Distorsion.total()>=4? Distorsion.ptr<float>(0)[3]:0);}
    inline float k3()const{return (Distorsion.total()>=5?  Distorsion.ptr<float>(0)[4]:0);}
    /**Indicates whether this object is valid
     */
    bool isValid() const;
    /**Adjust the parameters to the size of the image indicated
     */
    void resize(cv::Size size);
    /**OPerator ==
     */
    inline bool operator==(const ImageParams& ip)const{
        return getSignature()==ip.getSignature();
    }
    /**OPerator !=
     */
    inline bool operator!=(const ImageParams& ip)const{
        return getSignature()!=ip.getSignature();
    }

    inline bool isIntoImage(cv::Point2f &p2){
        if ( p2.x>=0 && p2.y>=0 && p2.x<CamSize.width &&  p2.y<CamSize.height) return true;
        return false;
    }

    void toStream(std::ostream &str) const ;
     void fromStream(std::istream &str) ;

     //apply distortion to a undistorted point
   cv::Point2f distortPoint(const cv::Point2f &p) const;
   std::vector<cv::Point2f> distortPoints(const std::vector<cv::Point2f> &p) const;


   //returns a version of this without distortion
    ImageParams undistorted()const{
        ImageParams ip=*this;
        ip.Distorsion.setTo(cv::Scalar::all(0));
        return ip;
    }

    void clear(){
        CameraMatrix=cv::Mat();
        Distorsion=cv::Mat();
        CamSize=cv::Size(-1,-1);
    }


    bool isStereoCamera()const{return bl!=0;}
    //indicates if a depth point is close or far
    inline bool isClosePoint(float z)const{
        return z<40*bl;
    }


    uint64_t getSignature()const;


    friend std::ostream &operator<<(std::ostream &str,const ImageParams &ip);


private:

 };
}
#endif
