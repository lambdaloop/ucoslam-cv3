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

/**Reads a map and the video sequence it was created with, and generates the required data for PMVS 3D reconstruction software
 * https://github.com/pmoulon/CMVS-PMVS
 */
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "map.h"
using namespace  std;
class Undistorter{

public:
inline void operator()(const cv::Mat &in,cv::Mat &out)    {

    assert(!mapx.empty());
    cv::remap(in,out,mapx,mapy,cv::INTER_LINEAR);

}
    void setParams(ucoslam::ImageParams &ip){
        assert(ip.isValid());
        bool do_=false;
        if(!imgP.isValid()  || imgP!=ip ){
            imgP=ip;
            cv::initUndistortRectifyMap(imgP.CameraMatrix,imgP.Distorsion,cv::Mat(),cv::Mat(),imgP.CamSize,CV_32FC1,mapx,mapy);
        }
    }
private:
    cv::Mat mapx,mapy;
    ucoslam::ImageParams imgP;

};

#ifdef WIN32
void mkdir(string path){

}
#else
void mkdir(string path){
    string cmd="mkdir -p "+path;
    std::system(cmd.c_str());
    cmd="rm  "+path+"/* -rf";
    std::system(cmd.c_str());
}
#endif

string getImageFileName(int number){

    stringstream fname;
    fname<<number<<".jpg";
    string sn=fname.str();
    while(sn.size()!=12) sn="0"+sn;
    return sn;
}
string getTxtFileName(int number){

    stringstream fname;
    fname<<number<<".txt";
    string sn=fname.str();
    while(sn.size()!=12) sn="0"+sn;
    return sn;
}
int main(int argc,char **argv){
    try {
        if(argc!=4)throw std::runtime_error("Usage: map video outDir");

        cout<<"Opening Map"<<endl;
        ucoslam::Map Map;
        Map.readFromFile(argv[1]);

        cout<<"Opening Video"<<endl;
        cv::VideoCapture vcap;
        vcap.open(argv[2]);
        if(!vcap.isOpened()) throw std::runtime_error("Could not open:"+string(argv[2]));

        string outDir=argv[3];
        mkdir(outDir);
        mkdir(outDir+"/visualize");
        mkdir(outDir+"/txt");
        mkdir(outDir+"/models");

        std::map<uint32_t,cv::Mat> images;
        cv::Mat image,undImage;
        int imgIdx=0;

        Undistorter undist;

        std::set<uint32_t> usedKeyFrames;
        for(auto &kf:Map.keyframes){
            cout<<"processing "<<imgIdx<<"/"<<Map.keyframes.size()<<" ";
            cout.flush();
            usedKeyFrames.insert(kf.idx);
            vcap.set(cv::CAP_PROP_POS_FRAMES,kf.fseq_idx);//go to image
            vcap.grab();
            vcap.retrieve(image);
            undist.setParams(kf.imageParams);
            undist(image,undImage);
            cv::imwrite(outDir+"/visualize/"+getImageFileName(imgIdx),undImage);
            //write the camera projection matrix
            ofstream txtFile( outDir+string("/txt/")+getTxtFileName(imgIdx));
            //lets create the projection matrix

            cv::Mat camExt=cv::Mat::zeros(3,4,CV_32F);
            for(int i=0;i<3;i++)
                for(int j=0;j<3;j++)
                    camExt.at<float>(i,j)=kf.imageParams.CameraMatrix.at<float>(i,j);




             camExt= camExt* kf.pose_f2g;


             txtFile<<"CONTOUR"<<endl;
             for(int i=0;i<3;i++){
                 for(int j=0;j<4;j++)
                     txtFile<<camExt.at<float>(i,j)<<" ";
                 txtFile<<endl;
             }


             imgIdx++;/*
             if(imgIdx>=maxImages) break;*/
        }


        ofstream visFile( outDir+string("/vis.dat"));
        visFile<<"VISDATA "<< usedKeyFrames.size()<<endl;//Map.keyframes.size()<<endl;
        for(auto kf:usedKeyFrames){
            visFile<<imgIdx<<" ";
            for(auto neigh:Map.getNeighborKeyFrames(kf,false))
                if( usedKeyFrames.count( neigh))
                    visFile<<neigh<<" ";
            visFile<<endl;
        }
        ofstream optionFile(outDir+string("/option.txt"));
        optionFile<<"level 2"<<endl;
        optionFile<<"csize 2"<<endl;
        optionFile<<"threshold 0.7"<<endl;
        optionFile<<"wsize 7"<<endl;
        optionFile<<"minImageNum 3"<<endl;
        optionFile<<"CPU 4"<<endl;
        optionFile<<"useVisData 1"<<endl;
        optionFile<<"sequence 1"<<endl;
        optionFile<<"timages -1 0 "<<usedKeyFrames.size()<<endl;
        optionFile<<"oimages 0"<<endl;

    } catch (std::exception &ex) {
        cout<<ex.what()<<endl;
    }
}
