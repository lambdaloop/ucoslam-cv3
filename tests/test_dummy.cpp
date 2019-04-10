#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ucoslam.h"
#include "optimization/ippe.h"
using namespace std;

float getArea(const vector<cv::Point2f> &m)
{
    assert(size() == 4);
    // use the cross products
    cv::Point2f v01 = m[1] - m[0];
    cv::Point2f v03 = m[3] - m[0];
    float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
    cv::Point2f v21 = m[1] - m[2];
    cv::Point2f v23 = m[3] - m[2];
    float area2 = fabs(v21.x * v23.y - v21.y * v23.x);
    return (area2 + area1) / 2.f;
}
int main(int argc,char **argv){

    srand(time(0));
    if(argc!=10)  {
        cerr<<"USage : camparam.yml msize errorLevel rx ry rz rx ty tz "<<endl;
    }
    ucoslam::ImageParams imparams;
    imparams.readFromXMLFile(argv[1]);

    float msize=stof(argv[2]);
    float noise=stof(argv[3]);
    cv::Mat rv(1,3,CV_32F);
    cv::Mat tv(1,3,CV_32F);
    rv.ptr<float>(0)[0]=stof(argv[4]);
    rv.ptr<float>(0)[1]=stof(argv[5]);
    rv.ptr<float>(0)[2]=stof(argv[6]);
    tv.ptr<float>(0)[0]=stof(argv[7]);
    tv.ptr<float>(0)[1]=stof(argv[8]);
    tv.ptr<float>(0)[2]=stof(argv[9]);

    ucoslam::Se3Transform transform;
     transform.set(rv,tv);
     ucoslam::Marker marker(0,transform,msize);

    vector<cv::Point2f> proj;
    cv::projectPoints(marker.get3DPoints(true),cv::Mat::zeros(1,3,CV_32F),cv::Mat::zeros(1,3,CV_32F),imparams.CameraMatrix,imparams.Distorsion,proj);
    double sumerrT=0,sumerrAngle= 0;


    int ntimes=1000;
    for(int i=0;i<ntimes;i++){
        vector<cv::Point2f> noise_proj=proj;
        for(auto &p:noise_proj){
            p.x += noise * double(rand())/double(RAND_MAX);
            p.y += noise * double(rand())/double(RAND_MAX);
        }
//        for(int i=0;i<4;i++)
//            cout<<proj[i]<<"-"<< noise_proj[i]<<endl;
        auto sols=IPPE::solvePnP(msize,noise_proj, imparams.CameraMatrix,imparams.Distorsion);

        //cout<<sols[0]<<endl;
        ucoslam::Se3Transform tres; tres=(sols[0]);
        double errT= cv::norm(tv-tres.getTvec());
        double errAngle= cv::norm(rv-tres.getTvec());
        sumerrT+=errT;
        sumerrAngle+=errAngle;
        //cout<<"Err="<<1000*errT<<"mm errangle="<<errAngle <<"- area:"<< getArea(proj)/ double(imparams.CamSize.area())<<endl;
    }
    sumerrT/=double(ntimes);
    sumerrAngle/=double(ntimes);
    cout<<"T="<< sumerrT<<"meters  relative="<< 100*sumerrT/cv::norm(tv)<<"%"<<endl;

}

