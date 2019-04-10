#ifndef UCOSLAM_LOGTOOLS_H
#define UCOSLAM_LOGTOOLS_H
#include <opencv2/core/core.hpp>
#include <map>
#include <string>
namespace ucoslam{
namespace logtools{





struct TestInfo{
    std::string seq_name,cam,method,iteration;
    std::string fullPath;
    std::string gt_file;
    std::string time_file;

    std::string getKey()const{
        return seq_name+cam+method;
    }
    std::string getDesc()const{
        return seq_name+"|"+cam;
    }
};
std::vector<TestInfo> getDataSetTestInfo(std::string resultsPath_dataset, std::string TheDataSetPath_dataset );


struct FrameMatchLocation{
     cv::Mat first;
     cv::Mat second;
     std::string frame;
     double error;

};

struct TestResult{
    TestInfo ti;
    double ATE=-1;
     double perctFramesTracked=0;

     std::vector< FrameMatchLocation > matchedFrames;//errors in each of the frames
     bool isFrame(std::string frame){
         for(auto &f:matchedFrames)
             if (f.frame==frame)return true;
         return false;
     }
     std::map<std::string,cv::Mat> gt,poses;
     int64_t timeMapping=-1,timeTracking=-1;
     float getMappingFPS(){if (timeMapping<=1 )return -1; return (gt.size()*perctFramesTracked/100.)/double(timeMapping);}
     float getTrackingFPS(){if (timeTracking<=1)return -1; return (gt.size()*perctFramesTracked/100.)/double(timeTracking);}

};

std::map<std::string,cv::Mat>   loadFile(std::string fp,bool invert=false);
cv::Mat  alignAndScaleToGroundTruth( std::vector<FrameMatchLocation> &gt_other );
std::vector<FrameMatchLocation> getMatchedLocations( std::map<std::string,cv::Mat>   &gt, std::map<std::string,cv::Mat>  &other);
void getMatchedLocations_io( std::map<std::string,cv::Mat>   &gt_IO, std::map<std::string,cv::Mat>  &other_IO);
void savePcd(std::string filepath,const std::vector<cv::Vec4f> & points);
std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints);


}
}
#endif

