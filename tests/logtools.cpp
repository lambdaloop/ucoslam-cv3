#include <map>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>
#include "logtools.h"
#include "dirreader.h"
using namespace  std;

namespace ucoslam{
namespace logtools{


std::vector<TestInfo> getDataSetTestInfo(string resultsPath_dataset, string TheDataSetPath_dataset ){

    auto dateToSeconds=[](std::string date){
        stringstream sstr;sstr<<date;
        vector<string > parts(4);
        if( sstr>>parts[0]>>parts[1]>>parts[2]>>parts[3]){
            //remove :
            for(auto &c:parts[3])if(c==':')c=' ';
            //divide
            int day=stoi(parts[2]);
            int hours,minutes,secs;
            stringstream sstr2;sstr2<<parts[3];
            sstr2>>hours>>minutes>>secs;
            return secs+minutes*60+hours*60*60+ day*60*60*24;
        }
        else return -1;

        };

    auto  parseFn=[](string fileName,TestInfo &ti){
                ti.fullPath=fileName;
                auto pos=fileName.find_last_of('/')+1;
                string baseName=std::string(fileName.begin()+pos,fileName.end());

                std::replace( baseName.begin(), baseName.end(), '@', ' '); // replace all 'x' to 'y'
                std::replace( baseName.begin(), baseName.end(), '.', ' '); // replace all 'x' to 'y'

                stringstream sstr;sstr<<baseName;


                if ( sstr>>ti.seq_name>>ti.cam>> ti.iteration)
                    return true;
                else return false;
                //mh_01@cam0@2.log

            };
    auto files=DirReader::read(resultsPath_dataset,".log",DirReader::Params(true));
    auto filesTime=DirReader::read(resultsPath_dataset,".logtime",DirReader::Params(true));

    std::vector<TestInfo> tests;
    for(auto file:files){
        if(file.find("@")==std::string::npos)continue;
        TestInfo ti;
        if(parseFn(file,ti)){
            //set the gt file path
            ti.gt_file=TheDataSetPath_dataset+"/"+ti.seq_name+"/"+ti.cam+"_gt.txt";
            if( std::find( filesTime.begin(),filesTime.end(),file+"time")!=filesTime.end()){
                ti.time_file=file+"time";
            }
            tests.push_back(ti);
        }
    }
return tests;
}

cv::Mat  getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){


    double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_64F);

    m.at<double>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<double>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<double>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<double>(0,3)=tx;

    m.at<double>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<double>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<double>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<double>(1,3)=ty;

    m.at<double>(2,0)=2*qx*qz - 2*qy*qw	;
    m.at<double>(2,1)=2*qy*qz + 2*qx*qw	;
    m.at<double>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<double>(2,3)=tz;
    return m;
}





 map<string,cv::Mat>   loadFile(std::string fp,bool invert){
   map<string,cv::Mat>   fmap;
    ifstream file(fp);
    if(!file)throw std::runtime_error("Could not open file:"+fp);
    string stamp;
    float tx,ty,tz,qx,qy,qz,qw;
    cv::Mat firstFrameT;
    while(!file.eof()){
        string line;
        std::getline(file,line);
        stringstream sline;sline<<line;
         if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
             auto m=getMatrix(qx,qy,qz,qw,tx,ty,tz);
             if (invert)m=m.inv();
             if (firstFrameT.empty())
                 firstFrameT=m.inv();
            fmap.insert( {stamp, firstFrameT*m});//refers everything to the first frame
        }
     }

    //now, find the transform from every frame to the first
    return fmap;
}

vector<FrameMatchLocation> getMatchedLocations( map<string,cv::Mat>   &gt, map<string,cv::Mat>  &other){
   vector<FrameMatchLocation> res;

   for(auto frame_est:other){
       auto pos=gt.find(frame_est.first);
       if(pos!=gt.end())
           res.push_back({pos->second.clone(),frame_est.second.clone(),frame_est.first,std::numeric_limits<float>::quiet_NaN()});
   }
   return res;

}

void getMatchedLocations_io( map<string,cv::Mat>   &gt_io, map<string,cv::Mat>  &other_io){

    map<string,cv::Mat>   gt_res,other_res;
   for(auto frame_est:other_io){
       auto pos=gt_io.find(frame_est.first);
       if(pos!=gt_io.end()){
           gt_res.insert(*pos);
           other_res.insert(frame_est);
       }
   }
   gt_io=gt_res;
   other_io=other_res;

}

cv::Mat rigidBodyTransformation_Horn1987 (const std::vector<cv::Point3f> &org, const std::vector<cv::Point3f> &dst,bool mbFixScale){
    auto ComputeCentroid=[](cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
    {
        cv::reduce(P,C,1,cv::REDUCE_SUM);
        C = C/P.cols;
        for(int i=0; i<P.cols; i++)
            Pr.col(i)=P.col(i)-C;
    };

    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    //Create the P1 an P2 matrices
    cv::Mat P1(3,org.size(),CV_32F);
    cv::Mat P2(3,org.size(),CV_32F);
    for(size_t i=0;i<org.size();i++){
        P1.at<float>(0,i)=org[i].x;
        P1.at<float>(1,i)=org[i].y;
        P1.at<float>(2,i)=org[i].z;
        P2.at<float>(0,i)=dst[i].x;
        P2.at<float>(1,i)=dst[i].y;
        P2.at<float>(2,i)=dst[i].z;
    }


        // Step 1: Centroid and relative coordinates

        cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        ComputeCentroid(P1,Pr1,O1);
        ComputeCentroid(P2,Pr2,O2);

        // Step 2: Compute M matrix

        cv::Mat M = Pr2*Pr1.t();

        // Step 3: Compute N matrix

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,P1.type());

        N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        N12 = M.at<float>(1,2)-M.at<float>(2,1);
        N13 = M.at<float>(2,0)-M.at<float>(0,2);
        N14 = M.at<float>(0,1)-M.at<float>(1,0);
        N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        N23 = M.at<float>(0,1)+M.at<float>(1,0);
        N24 = M.at<float>(2,0)+M.at<float>(0,2);
        N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        N34 = M.at<float>(1,2)+M.at<float>(2,1);
        N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        // Step 4: Eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        cv::Mat vec(1,3,evec.type());
        (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        double ang=atan2(norm(vec),evec.at<float>(0,0));

        if (norm(vec)<1e-7)return cv::Mat::eye(4,4,CV_32F);

        vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        cv::Mat mR12i(3,3,P1.type());

        cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2

        cv::Mat P3 = mR12i*Pr2;

        // Step 6: Scale
        float ms12i;

        if(!mbFixScale)
        {
            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            ms12i = nom/den;
        }
        else
            ms12i = 1.0f;

        // Step 7: Translation

        cv::Mat  mt12i(1,3,P1.type());
        mt12i = O1 - ms12i*mR12i*O2;

        // Step 8: Transformation

        // Step 8.1 T12
        cv::Mat mT12i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sR = ms12i*mR12i;

        sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        mt12i.copyTo(mT12i.rowRange(0,3).col(3));
//        return mT12i;

//        // Step 8.2 T21

        cv::Mat mT21i = cv::Mat::eye(4,4,P1.type());

        cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        cv::Mat tinv = -sRinv*mt12i;
        tinv.copyTo(mT21i.rowRange(0,3).col(3));
        return mT21i;
}

cv::Mat  alignAndScaleToGroundTruth(std::vector<FrameMatchLocation> &gt_other ){

    vector<cv::Point3f> points_other,points_gt;
    for(auto gto:gt_other){
        assert(gto.first.type()==CV_64F);
        points_gt.push_back(cv::Point3f(gto.first.at<double>(0,3),gto.first.at<double>(1,3),gto.first.at<double>(2,3)));
        points_other.push_back(cv::Point3f(gto.second.at<double>(0,3),gto.second.at<double>(1,3),gto.second.at<double>(2,3)));
    }

    cv::Mat best_T= rigidBodyTransformation_Horn1987(points_other,points_gt,false);
     cv::Mat best_T64;
    if( best_T.type()!=CV_64F) best_T.convertTo(best_T64,CV_64F);
    else best_T64=best_T;
    //cout<<best_T64<<endl;
    for(auto &gto:gt_other){
         gto.second = best_T64* gto.second;
    }
return best_T64;
}

std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints);
void savePcd(string filepath,const vector<cv::Vec4f> & points){
    std::ofstream filePCD (filepath, std::ios::binary );
   filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
   filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));
}
std::vector<cv::Vec4f> getLine(const  cv::Point3f &a,const  cv::Point3f &b,cv::Scalar color,int npoints){
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    std::vector<cv::Vec4f>  points;
    cv::Vec4f pa(a.x,a.y,a.z,fcolor);
    cv::Vec4f pb(b.x,b.y,b.z,fcolor);
    cv::Vec4f d =pb-pa;
    if (npoints==1)return {pa};
    d*=1./cv::norm(d);
    double step=  cv::norm(pb-pa)*( 1./ double(npoints-1));
    //cout<<"step="<<step<<endl;
//    cout<<pa<<"-"<<pb<<endl;
    for(int i=0;i<npoints;i++){
//        cout<<"d="<<d<<" "<<step<<" "<<(d*step*double(i))<<" "<<pa+ (d*step*double(i))<<endl;
        points.push_back(pa+ (d*step*double(i)));
    }
  //  cout<<pa<<" "<<pb<<" "<<pa+ (d*step*double(npoints))<<endl;
    return points;

}
}
}
