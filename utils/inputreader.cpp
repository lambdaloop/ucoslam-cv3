#include "inputreader.h"
#include "dirreader.h"
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <chrono>
#include <iostream>
InputReader::InputReader( )
{
}
void InputReader::open(std::string path,bool emulateRealTime){
    _emulateRealTime=emulateRealTime;
    vcap.open(path);
    if(!vcap.isOpened()){//try dir
        files=DirReader::read(path,"",DirReader::Params(true));
        imgIdx=0;
    }
    isLive=false;
}
void InputReader::open(int cameraIdx){
    vcap.open(cameraIdx);
    isLive=true;
    imgIdx=0;
    videoTimeBetweenFrame=std::chrono::milliseconds(0);
}

bool InputReader::isOpened()const{
    if(vcap.isOpened())return true;
    if(files.size()!=0)return true;
    return false;
}
void InputReader:: set(int v,double val){
    if( vcap.isOpened()){
            vcap.set(v,val);
        }
    else{//images
        if(v==cv::CAP_PROP_POS_FRAMES && val<files.size()) imgIdx=val;
    }
}
double InputReader:: get(int v){

    if( vcap.isOpened()){
        if( isLive && v==cv::CAP_PROP_POS_FRAMES){
            return imgIdx;
        }

             else return vcap.get(cv::CAP_PROP_POS_FRAMES);
        }
    else{//images
        if(v==cv::CAP_PROP_POS_FRAMES) return imgIdx;
    }

    throw std::runtime_error("InputReader::get Should not get here");
}
int InputReader::getCurrentFrameIndex(  ){

    if(!isOpened()) return -1;
    if(vcap.isOpened()){
        if (isLive) return imgIdx;
        else return  int(vcap.get(cv::CAP_PROP_POS_FRAMES));
    }
    else{
        return imgIdx;
    }
}

InputReader & 	InputReader::operator>> (cv::Mat &image){
    if ( grab())
        retrieve(image);
    else image=cv::Mat();
    return *this;
}

bool InputReader::grab(){
    if(vcap.isOpened()){
        if(isLive)imgIdx++;
        else if(_emulateRealTime){
            //wait to deliver at the appropriate speed
            std::chrono::milliseconds waitTime=std::chrono::milliseconds(0);
            if(videoTimeBetweenFrame== std::chrono::milliseconds(0)){//first time here
                float fps=vcap.get(cv::CAP_PROP_FPS);
                videoTimeBetweenFrame =  std::chrono::milliseconds(  int(1000./fps) );
                lastCapture=std::chrono::high_resolution_clock::now();
            }
            else{
                auto now=std::chrono::high_resolution_clock::now();
                auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>( now-lastCapture);
                if( elapsedTime< videoTimeBetweenFrame)
                    waitTime=videoTimeBetweenFrame-elapsedTime;
                lastCapture=now;
            }
            std::this_thread::sleep_for(waitTime);
        }
        return vcap.grab();
    }

    do{
     fileImage=cv::imread(files[imgIdx++]);
    }while(fileImage.empty() &&imgIdx<files.size());

    return !fileImage.empty();
}

void InputReader::retrieve(cv::Mat &im){
    if(vcap.isOpened()){
        vcap.retrieve(im);
    }
    else fileImage.copyTo(im);

}
