/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#include "ucoslamtypes.h"
#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "cvni2.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret;
    cv::resize(in,ret,size);
    return ret;
}



ucoslam::ImageParams getASUSImageParams() {

    ucoslam::ImageParams  IP;
    IP.CameraMatrix= (cv::Mat_<float>(3,3) <<  517.306408,0,318.643040,0,516.469215,255.313989,0,0,1);
    IP.Distorsion= (cv::Mat_<float>(1,5) <<  0.262383,-0.953104, -0.005358,0.002628,1.163314);
//    IP.Distorsion= (cv::Mat_<float>(1,5) <<  0,0,0,0,0);
    IP.CamSize=cv::Size(640,480);
    IP.bl=0.07;//camera base line
    //IP.cameraType=ucoslam::ImageParams::CAMTYPE_STEREO;
    IP.rgb_depthscale=1e-3;//scale factor to convert into the desired scale

    return IP;
}
inline cv::Vec4f mult(cv::Vec4f point,const cv::Mat &T){
    const float *mat_ptr=T.ptr<float>(0);
    float x=mat_ptr[0]*point[0]+ mat_ptr[1]*point[1] + mat_ptr[2]*point[2]+mat_ptr[3];
    float y=mat_ptr[4]*point[0]+ mat_ptr[5]*point[1] + mat_ptr[6]*point[2]+mat_ptr[7];
    float z=mat_ptr[8]*point[0]+ mat_ptr[9]*point[1] + mat_ptr[10]*point[2]+mat_ptr[11];
    return cv::Vec4f(x,y,z,point[3]);
}
void savePcd(const cv::Mat &rgb,const cv::Mat &depth,const cv::Mat &C2G,string filepathpcd);



int main(int argc,char **argv){
    try {
        CmdLineParser cml(argc, argv);
        if (argc < 2 || cml["-h"]) {
            cerr << "Usage: (onifile|live)    [-in world]  [-out name] [-loc_only: use if only needs tracking (no SLAM)] [-d descriptor 0:orb 1:AKAZE] [-size markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  [-nomarkers] [-vsize w:h] [-debug level] [-voc bow_volcabulary_file] [-fdt n:number of threads of the feature detector] [-st starts the processing stopped ] [-nokeypoints] [-marker_minsize <val_[0,1]>]  [-noX disabled windows] [-fps X: set video sequence frames per second] [-skip <n>:skips n frames before starting] [-outavi <filename>:saves to an avi file] [-KFMinConfidence <val>]"   << endl; return -1;
        }
        CvNI2 vcap;
        cv::VideoWriter vwrite;
        string videoPath=cml("-outavi","");
        string openString;
        if (string(argv[1])!="live") openString=argv[1];
        vcap.open(openString);
        if (!vcap.isOpen())
            throw std::runtime_error("Video not opened");

        ucoslam::UcoSlam Slam;
        Slam.setDebugLevel(stoi(cml("-debug", "0")));
        cv::Mat in_image,in_depth;


        ucoslam::ImageParams image_params=getASUSImageParams();
        ucoslam::Params params;

         params.aruco_markerSize = stof(cml("-size", "1"));
        params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
        params.detectMarkers = !cml["-nomarkers"]; //work only with keypoints!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        params.detectKeyPoints = !cml["-nokeypoints"];
        params.runSequential = cml["-sequential"];
        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
         params.kpDescriptorType = ucoslam::DescriptorTypes::fromString(cml("-d", "orb"));
         if(cml["-KFMinConfidence"])
             params.KFMinConfidence=stof(cml("-KFMinConfidence"));


         cout<<"PPP="<<params.KFMinConfidence<<endl;
        //create an empty map and give it to the class

        std::shared_ptr<ucoslam::Map> TheMap=std::make_shared<ucoslam::Map>();
        if (cml["-in"])
            TheMap->readFromFile(cml("-in"));

        Slam.setParams( TheMap,params,cml("-voc"));

        int currentFrameIndex=0;
        if (cml["-in_debug"]) {//if read debug input, then forget about the rest
            Slam.readFromFile(cml("-in"));
            params = Slam.getParams();
            //read until the last processed frame
                cerr << "skipping frames" << endl;
                for(size_t i=0;i<Slam.getLastProcessedFrame();i++) vcap.grab();
                currentFrameIndex=Slam.getLastProcessedFrame();
        }
        else{
            currentFrameIndex=stoi(cml("-skip","0"));
            cerr << "skipping frames" << endl;
            for(int i=0;i<currentFrameIndex ;i++) {
                if (i%10==0)cerr<<i<<"/"<<currentFrameIndex<<" ";
                vcap.grab();
            }
            cerr<<endl;

        }

        if (cml["-loc_only"]) Slam.setMode(ucoslam::MODE_LOCALIZATION);


        ucoslam::TimerAvrg Fps;
        int waitTimeWindow = cml["-st"] ? 0 : 10;
        bool finish = false;
        ucoslam::MapViewer TheViewer;
        while (!finish && vcap.grab()) {
            vcap.retrieve(in_image,in_depth);
            if (in_image.empty())continue;

            Fps.start();
//            Slam.process(in_image, image_params,currentFrameIndex++);
            cv::Mat pose_f2gT=Slam.processRGBD(in_image, in_depth, image_params,currentFrameIndex);
            Fps.stop();



            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;
        //    Slam.drawMatches(in_image);
            char k = TheViewer.show( TheMap, in_image ,pose_f2gT,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            if (int(k) == 27)finish = true;//pressed ESC

            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
//                Slam.saveToFile("world-"+number+ ".ucs");
                TheMap->saveToFile("world-"+number+ ".map");
            }
            if (k=='p'){
                cout<<"T="<<pose_f2gT<<endl;
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                savePcd(in_image,in_depth,pose_f2gT,"depth-"+number+".pcd");
                cout<<"Saved:"<<"depth-"+number+".pcd"<<endl;
            }
            if (k=='r')
                Slam.clear();

            if (!videoPath.empty()){
                cv::Mat imwrite;
                TheViewer.getImage(imwrite);
                if (!vwrite.isOpened())
                    vwrite.open(videoPath, CV_FOURCC('X', '2', '6', '4'),stoi(cml("-fps","30")),imwrite.size()  , imwrite.channels()!=1);
                vwrite<<imwrite;
            }

            currentFrameIndex++;
        }



        TheMap->saveToFile(cml("-out","world") + ".map");
    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
    }
}
void savePcd(const cv::Mat &rgb,const cv::Mat &depth,const cv::Mat &C2G,string filepathpcd){

    auto getRGBfloat=[](cv::Vec3b color){
        float f;
        uchar *fc=(uchar*)&f;
        fc[0]=color[0];
        fc[1]=color[1];
        fc[2]=color[2];
        return f;
    };
    int nvalid=0;
    for(int y=0;y<depth.rows;y++){
        const uint16_t *_depth=depth.ptr<uint16_t>(y);
        for(int x=0;x<depth.cols;x++)
            if ( _depth[x]*1e-3<4 && _depth[x]!=0) nvalid++;
    }

    cv::Mat G2C=C2G.inv();
                ofstream file(filepathpcd);
file<<"# .PCD v.7 - Point Cloud Data file format"
<<endl<<"VERSION .7"
<<endl<<"FIELDS x y z rgb"
<<endl<<"SIZE 4 4 4 4"
<<endl<<"TYPE F F F F"
<<endl<<"COUNT 1 1 1 1"
<<endl<<"VIEWPOINT 0 0 0 1 0 0 0"
<<endl<<"WIDTH "<<nvalid
<<endl<<"HEIGHT "<<1
<<endl<<"POINTS "<<nvalid
<<endl<<"DATA binary"<<endl;
float fx_inv=1./517.306408;
float fy_inv=1./516.469215;
float cx=318.643040;
float cy=255.313989;
cv::Vec4f p;

for(int y=0;y<depth.rows;y++){
    const cv::Vec3b *_rgb=rgb.ptr<cv::Vec3b>(y);
    const uint16_t *_depth=depth.ptr<uint16_t>(y);
    for(int x=0;x<depth.cols;x++){
        if ( _depth[x]*1e-3<4 && _depth[x]!=0){
            //get the 3d
            p[2]= float(_depth[x])*1e-3;
            p[0]= p[2] * float( x - cx ) * fx_inv;
            p[1]= p[2] * float( y - cy ) * fy_inv;
            p[3]=getRGBfloat(_rgb[x]);
            p=mult(p,G2C);
           // cout<<_depth[x]<<" "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
            file.write((char*)&p,sizeof(float)*4);
        }
    }
}

}
