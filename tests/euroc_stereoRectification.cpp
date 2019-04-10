#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "dirreader.h"
#include "stereorectify.h"
#include <iostream>
#include <map>
#include <fstream>
using namespace std;

template<typename T>
T get(cv::FileStorage &fs,string name){
    T aux;
 fs[name]>>   aux;
    return aux;
}

int main(int argc,char **argv){


    if(argc<3){cerr<<"Usage : orbslam2_stereo_cameraparams  out_stereo.yml"<<endl;return -1;}

    cv::FileStorage in_orb(argv[1],cv::FileStorage::READ);
    cv::FileStorage out_ucoslam(argv[2],cv::FileStorage::WRITE);


    out_ucoslam<<"image_width"<<get<int>(in_orb,"Camera.width");
    out_ucoslam<<"image_height"<<get<int>(in_orb,"Camera.height");
    cv::Mat T=cv::Mat::zeros(1,3,CV_32F);
    float fx,baseLine;
    in_orb["Camera.fx"]>>fx;
    in_orb["Camera.bf"]>>baseLine;
    baseLine/=fx;
    cout<<"baseLine="<<baseLine<<endl;
    T.at<float>(0,0)=baseLine;

    out_ucoslam<<"M1"<<get<cv::Mat>(in_orb,"LEFT.K");
    out_ucoslam<<"D1"<<get<cv::Mat>(in_orb,"LEFT.D");
    out_ucoslam<<"R1"<<get<cv::Mat>(in_orb,"LEFT.R");
    out_ucoslam<<"P1"<<get<cv::Mat>(in_orb,"LEFT.P");

    out_ucoslam<<"M2"<<get<cv::Mat>(in_orb,"RIGHT.K");
    out_ucoslam<<"D2"<<get<cv::Mat>(in_orb,"RIGHT.D");
    out_ucoslam<<"R2"<<get<cv::Mat>(in_orb,"RIGHT.R");
    out_ucoslam<<"P2"<<get<cv::Mat>(in_orb,"RIGHT.P");
    out_ucoslam<<"T"<<T;

}
