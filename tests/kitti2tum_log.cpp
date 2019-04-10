//converts kitti log to tum format
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
using namespace std;



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
void savePcd(string filepath,const vector<cv::Vec4f> & points){
    std::ofstream filePCD (filepath, std::ios::binary );
   filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
   filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));
}
vector<cv::Vec4f> vp3d;
std::vector<string > readFile(string path){

    ifstream   fileIn(path);
    if(!fileIn)throw std::runtime_error("Could not open file");
    std::vector<string> result;
    string line;
    cv::Mat  M(4,4,CV_32F);
    int idex=0;
    while(!fileIn.eof()){
        std::getline(fileIn,line);
        stringstream sstr;sstr<<line;
        bool readed=true;
        for(int i=0;i<12;i++)
            if (!(sstr>>M.at<float>(i))) {
                    readed=false;
                    break;
            }
        if (readed){
            double tx,ty,tz,qw,qx,qy,qz;
            getQuaternionAndTranslationfromMatrix44(M,qx,qy,qz,qw,tx,ty,tz);
            stringstream res;
            res<<idex<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw;
            result.push_back(res.str());

            cv::Scalar color(0,244,0);
            float fcolor;uchar *c=(uchar*)&fcolor;
            for(int i=0;i<3;i++)c[i]=color[i];
            vp3d.push_back(cv::Vec4f(tx,ty,tz,fcolor));
        }
        idex++;
    }
    return result;
}


int main(int argc,char **argv){


    try{
        if (argc!=3)throw std::runtime_error(" Usage: inGt out");

        auto gt=readFile(argv[1]);

        ofstream outFile(argv[3]);
        outFile<<"#frame tx ty tz qw qx qy qz ..."<<endl;
        for(auto line:gt )
            outFile<< line<<endl;

       // savePcd("auxlog.pcd",vp3d);

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
