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
#ifndef _StereoRectify_H
#define _StereoRectify_H
#include <string>
#ifdef STANDALONE
    #define UCOSLAM_API
#else
    #include "imageparams.h"
    #include "ucoslam_exports.h"
#endif
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
namespace ucoslam{

/** Stereo image rectification
 *
 * Reads the stereo config from a YML and then rectifies the images so that they can be passed to UcoSLAM.
 *
 * The file must have been generated with with the  program in utils/ucoslam_stereocalibrate
 *
 * The file must have the matrices for stereo rectification ( M1,M2,D1...) and image dimensions (image_width,image_height)
 */
class UCOSLAM_API StereoRectify{
public:


    //reads the data from XML file.
    void readFromXMLFile(const std::string &path){ readFromXMLFile_(path);}

    //does the work.
    void rectify( const cv::Mat &left,const cv::Mat &right){rectify_(left,right);}
    cv::Mat rectify( const cv::Mat &img,int idx){return rectify_(img,idx);}
    //returns the image params (of the rectified images) needed to call UcoSLAM
#ifndef STANDALONE
    inline ImageParams getImageParams()const{return imgParams;}
#endif
    //returns the left rectified image
    inline cv::Mat & getLeft() {return leftRect;}
    //returns the right rectified image
    inline cv::Mat &getRight() {return rightRect;}

private:
    cv::Mat leftRect,rightRect;
    cv::Mat mapx[2],mapy[2];
#ifndef STANDALONE
    ImageParams imgParams;
#endif


    //reads the data from XML file.
    void  readFromXMLFile_(const std::string &filePath){
        cv::FileStorage fs(filePath, cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+" could not open file:"+filePath);

        for(int i=0;i<2;i++){
            mapx[i]=cv::Mat();
            mapy[i]=cv::Mat();
        }
        cv::Mat M1,M2,D1,D2,R,T,R1,R2,P1,P2 ;
        int w = -1, h = -1;
        fs["image_width"]>>w;
        fs["image_height"]>>h;
        fs["M1"]>>M1;
        fs["M2"]>>M2;
        fs["D1"]>>D1;
        fs["D2"]>>D2;
        fs["T"]>>T;
        fs["R1"]>>R1;
        fs["R2"]>>R2;
        fs["P1"]>>P1;
        fs["P2"]>>P2;
        //consider only the 3x3 part of P1 and P2 as the new camera matrix
        cv::Mat ncm[2];
        ncm[0]=P1.rowRange(0,3).colRange(0,3);
        ncm[1]=P2.rowRange(0,3).colRange(0,3);


        if(w==-1 || h==-1)  throw std::runtime_error("StereoRectify::readFromXMLFile_ image_width or image_height not found in YML file");


        cv::initUndistortRectifyMap(M1,D1,R1,ncm[0],cv::Size( w,h),CV_16SC2,mapx[0],mapy[0]);
        cv::initUndistortRectifyMap(M2,D2,R2,ncm[1],cv::Size( w,h),CV_16SC2,mapx[1],mapy[1]);
#ifndef STANDALONE
        imgParams.CamSize=cv::Size(w,h);
        ncm[0].convertTo(imgParams.CameraMatrix,CV_32F);
        imgParams.Distorsion=cv::Mat::zeros(1,5,CV_32F);
        imgParams.bl=cv::norm(T);
#endif
    }


    void  rectify_( const cv::Mat &left,const cv::Mat &right){

        std::thread LeftThread=std::thread([&](){cv::remap(left,leftRect,mapx[0],mapy[0],cv::INTER_LINEAR);});
        std::thread RightThread=std::thread([&](){cv::remap(right,rightRect,mapx[1],mapy[1],cv::INTER_LINEAR);});
        LeftThread.join();
        RightThread.join();


    }
    cv::Mat rectify_( const cv::Mat &img,int idx){
        if(idx==0){
            cv::remap(img,leftRect,mapx[0],mapy[0],cv::INTER_LINEAR);
            return leftRect;
        }
        else  {
            cv::remap(img,rightRect,mapx[1],mapy[1],cv::INTER_LINEAR);
            return rightRect;
        }
    }

};



};

#endif
