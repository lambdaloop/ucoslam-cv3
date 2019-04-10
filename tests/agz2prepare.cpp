//prepares the dataset Zurich Urban Micro Aerial Vehicle Dataset to be included in out dataset
#include "dirreader.h"
#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <Eigen/Geometry>
using namespace std;


void Dir2Video(string dir,string videoPath){
    cv::VideoWriter videoOut,videoOutSmall;
    cout<<"saving to "<<videoPath<<endl;
    for( auto file: DirReader::read(dir)){
        cv::Mat image=cv::imread(file);

        if(image.empty())continue;
        cv::Size ims;
        ims.width=float(image.size().width)*0.4;
        ims.height=float(image.size().height)*0.4;
        if (ims.width%2!=0) ims.width++;
        if (ims.height%2!=0) ims.height++;
        cv::Mat imageSmall;
        cv::resize(image,imageSmall,ims);
        if (!videoOut.isOpened())
            videoOut.open(videoPath+".mp4", CV_FOURCC('X', '2', '6', '4'), 30,image.size()  , image.channels()!=1);
        if (!videoOutSmall.isOpened())
            videoOutSmall.open(videoPath+"-small.mp4", CV_FOURCC('X', '2', '6', '4'), 30,imageSmall.size()  , imageSmall.channels()!=1);

        videoOut<<image;
        videoOutSmall<<imageSmall;
        cerr<<file<<endl;
    }
    videoOut.release();
}


void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;
    M_in.convertTo(M,CV_64F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<double>(0,3);
    ty=M.at<double>(1,3);
    tz=M.at<double>(2,3);
}

struct Pose{
    double tx,ty,tz,yaw,pitch,roll;
    cv::Mat getMatrix(){
        cv::Mat m=cv::Mat::eye(4,4,CV_32F);
        m.at<float>(0,3)=tx;
        m.at<float>(1,3)=ty;
        m.at<float>(2,3)=tz;
        return m;
    }
};
std::pair<double,Pose> lineRead(string line){
    stringstream sstr;
    sstr<<line;
    double v;
    sstr>>v;
    Pose p;
    if (sstr>>p.tx>>p.ty>>p.tz>>p.yaw>>p.pitch>>p.roll)    return {v,p};
    else return {-1,p};
}


std::map<int,string> readAndConvertGTFile(string path){
    ifstream   fileIn(path);
    if(!fileIn)throw std::runtime_error("Could not open file");
    std::map<int,string> result;
    string line;
    std::getline(fileIn,line);
    while(!fileIn.eof()){
        std::getline(fileIn,line);
        if(line.find("#")!=std::string::npos)continue;
        std::replace( line.begin(), line.end(), ',', ' '); // replace all 'x' to 'y'
        auto linedata=lineRead(line);
        if (linedata.first!=-1){
            double tx,ty,tz,qx,qy,qz,qw;
            getQuaternionAndTranslationfromMatrix44(linedata.second.getMatrix(),qx,qy,qz,qw,tx,ty,tz);
            stringstream sres;sres<<" "<<tx<<" "<<ty<<" " <<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw;
            result.insert({linedata.first, sres.str()});
        }
    }
    return result;
}
void calibrateCamera(string calibImagesPath){

}

int main(int argc,char **argv){
    try{
        if (argc!=3)throw std::runtime_error(" Usage: inDir outDir");
        string cmd="mkdir -p "+string(argv[2]);
        system(cmd.c_str());
        Dir2Video(string(argv[1])+"/MAV Images/",string(argv[2])+"/cam0");
        auto lines=readAndConvertGTFile(string(argv[1])+"/Log Files/GroundTruthAGL.csv");

        ofstream fileOut(string(argv[2])+"/cam0_gt.log");
        for(auto line:lines)
            fileOut<<line.first<<" "<<line.second<<endl;

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
