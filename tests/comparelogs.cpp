#include <map>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iomanip>
#include "logtools.h"
using namespace  std;

cv::Point3f getT(cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point3f(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3));
    else
        return cv::Point3f(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3));
}

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

cv::Scalar parseColor(string str){
    for(auto &s:str)if(s==',') s=' ';
    stringstream sstr;sstr<<str;
    cv::Scalar color;
    sstr>>color[0]>>color[1]>>color[2];
    return color;
}


vector<cv::Vec4f>  toPcd(  map<string,cv::Mat> &data,cv::Scalar color){
   auto toVec=[](   map<string,cv::Mat> &data){
        map<int,cv::Point3f> datai;
        for(auto d:data) datai.insert( { std::stoi(d.first),getT(d.second)});
        vector<pair<int,cv::Point3f> > res;
        for(auto d:datai) res.push_back({d.first,d.second});
        return res;
    };
   vector<cv::Vec4f> points;
   auto gtPos=toVec(data);
   int start=1,end=gtPos.size();
   for(size_t i=start;i<end;i++){
        auto pp=ucoslam::logtools::getLine(gtPos[i-1].second,gtPos[i].second,color,1);
       points.insert(points.end(),pp.begin(),pp.end());
   }
   return points;
}
int main(int argc,char **argv){

    CmdLineParser cml(argc,argv);
    if(argc<4){
        cerr<<"Usage: groundtruth other out_other.pcd [-c r,g,b]"<<endl;return -1;
    }
    auto gt=ucoslam::logtools::loadFile(argv[1]);
    auto other=ucoslam::logtools::loadFile(argv[2]);
    auto gt_other_=ucoslam::logtools::getMatchedLocations(gt,other);

    auto T=ucoslam::logtools::alignAndScaleToGroundTruth(gt_other_);

    //get the ATE error
    double e=0;
    for(auto p:gt_other_)
        e+= cv::norm(getT(p.first)-getT(p.second));
    cout<<"ATE="<<e/double(gt_other_.size())<<" Perct="<< float(gt_other_.size())/float(gt.size())<< endl;

    vector<cv::Vec4f> pcdpoints_gt,pcdpoints_est,joinlines;
    cv::Scalar otherColor=parseColor(cml("-c","255,0,0"));



    for(auto &t:other)
             t.second = T* t.second;



    ucoslam::logtools::savePcd("gt.pcd",toPcd(gt,cv::Scalar(0,0,255)));
    ucoslam::logtools::savePcd(argv[3],toPcd(other,otherColor));
    //    ucoslam::logtools::savePcd("joinlines.pcd",joinlines);

}
