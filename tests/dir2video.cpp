#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "dirreader.h"
#include <iostream>
using namespace std;


int main(int argc,char **argv){


    if(argc<3){cerr<<"Usage : indir outpath.mp4 [startFrame]"<<endl;return -1;}

    cv::VideoWriter videoOut;
    string path=argv[2];
    cout<<"saving to "<<path<<endl;

    int startFrame=0;
    if(argc>=4) startFrame=stoi(argv[3]);
    bool first=true;
    int curFrame=0;
    for( auto file: DirReader::read(argv[1],"",DirReader::Params(true))){
        if(curFrame>=startFrame) {
        cv::Mat image=cv::imread(file);
        if(first && startFrame!=0){
            cv::imshow("image",image);
            cv::waitKey(0);
            first=false;
        }
        if(image.empty())continue;
        if (!videoOut.isOpened())
            videoOut.open(path, CV_FOURCC('X', '2', '6', '4'), 30,image.size()  , image.channels()!=1);
        videoOut<<image;
        cerr<<file<<endl;
        }
        curFrame++;
    }
    videoOut.release();

}
