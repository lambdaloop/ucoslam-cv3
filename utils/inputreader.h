#ifndef INPUTREADER_H
#define INPUTREADER_H
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
/**Class to read either from video, camera or a directory of images
 */

class InputReader
{
public:
    InputReader( );

    void open(std::string path,bool emulateRealTime=false);
    void open(int cameraIdx);
    bool isOpened()const;
    bool grab();
    void retrieve(cv::Mat &im);
    void set(int v,double val);
    double get(int v);
    virtual InputReader & 	operator>> (cv::Mat &image);
    int cIndexLive=0;
    int getCurrentFrameIndex( ) ;
    private:
    cv::VideoCapture vcap;
    std::vector<std::string> files;
    int imgIdx=0;
    cv::Mat fileImage;
    bool isLive=false;

private:
    bool _emulateRealTime=false;
    std::chrono::high_resolution_clock::time_point lastCapture;
     std::chrono::milliseconds videoTimeBetweenFrame= std::chrono::milliseconds(0);

};

#endif // INPUTREADER_H
