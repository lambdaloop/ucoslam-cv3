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
#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "inputreader.h"


cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    ucoslam::Frame frame;
    vcap.set(cv::CAP_PROP_POS_FRAMES,frameIdx);
    vcap.grab();
    vcap.set(cv::CAP_PROP_POS_FRAMES,frameIdx);
    vcap.retrieve(im);
    return im;
}


class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret,ret2;
    cv::resize(in,ret,size);  return ret;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  int(vcap.get(cv::CAP_PROP_POS_FRAMES));
}


void overwriteParamsByCommandLine(CmdLineParser &cml,ucoslam::Params &params){
    if ( cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if ( cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["s"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
     if (cml["-desc"])       params.kpDescriptorType = ucoslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])  params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])  params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if(cml["s"])    params.KPNonMaximaSuppresion=true;

    if(cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if(cml["-extra_params"])    params.extraParams=cml("-extra_params");

    if(cml["-scale"]) params.kptImageScaleFactor=stof(cml("-scale"));

    if(cml["-nokploopclosure"]) params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) params.inPlaneMarkers=true;
    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if (cml["-dbg_str"])
        ucoslam::debug::Debug::addString(cml("-dbg_str"),"");
}

int main(int argc,char **argv){
	try {
		CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"]) {
            cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml [-params ucoslam_params.yml] [-map world]  [-out name] [-scale <float>:video resize factor]"
                    "[-loc_only do not update map, only do localization. Requires -in]"
                    "\n"
                    "[-desc descriptor orb,akaze,brisk,freak] "
                    "[-aruco-markerSize markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  "
                    "[-nomarkers]  [-debug level] [-voc bow_volcabulary_file] "
                    "[-t_fe n:number of threads of the feature detector] [-st starts the processing stopped ] "
                    "[-nokeypoints] [-marker_minsize <val_[0,1]>] [-em . Uses enclosed markers] [-noX disabled windows] "
                    "[-fps X: set video sequence frames per second] [-outvideo filename]"
                    "[-featDensity <float>:features density]"
                    "[-nOct <int>:number of octave layers]"
                    "[-noMapUpdate]"
                    "[-tfocus <float>: target focus employed to create the map. Replaces the one of the camera] "
                    "[-undistort] will undistort image before processing it"
                    "[-extra_params \"param1=value1 param2=value2...\"]"
                 << endl; return -1;
		}

		bool liveVideo = false;
        InputReader vcap;
        cv::VideoWriter videoout;
		string TheInputVideo = string(argv[1]);
        string TheOutputVideo=cml("-outvideo");
		if (TheInputVideo.find("live") != std::string::npos)
		{
			int vIdx = 0;
			// check if the :idx is here
            char cad[100];
			if (TheInputVideo.find(":") != string::npos)
			{
				std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
				sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
			}
			cout << "Opening camera index " << vIdx << endl;
			vcap.open(vIdx);
            //vcap.set(cv::CAP_PROP_AUTOFOCUS, 0);
            liveVideo = true;

		}
        else vcap.open(argv[1],!cml["-sequential"]);

		if (!vcap.isOpened())
			throw std::runtime_error("Video not opened");

        ucoslam::UcoSlam Slam;
		int debugLevel = stoi(cml("-debug", "0"));
        Slam.setDebugLevel(debugLevel);
        Slam.showTimers(true);
        ucoslam::ImageParams image_params;
        ucoslam::Params params;
        cv::Mat in_image;

        image_params.readFromXMLFile(argv[2]);

        if( cml["-params"])        params.readFromYMLFile(cml("-params"));
        overwriteParamsByCommandLine(cml,params);

        auto TheMap=std::make_shared<ucoslam::Map>();
        //read the map from file?
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));

        Slam.setParams(TheMap, params,cml("-voc"));


        if (cml["-loc_only"]) Slam.setMode(ucoslam::MODE_LOCALIZATION);

        //need to skip frames?
        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            vcap.set(cv::CAP_PROP_POS_FRAMES,n);
            cerr<<endl;
        }

        //read the first frame if not yet
        while (in_image.empty())
            vcap >> in_image;
        //need to resize input image?
        cv::Size vsize(0,0);
        //need undistortion

        bool undistort=cml["-undistort"];
        vector<cv::Mat > undistMap;
        if(undistort ){
            if( undistMap.size()==0){
                undistMap.resize(2);
                cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);
            }
            image_params.Distorsion.setTo(cv::Scalar::all(0));
        }
        //Create the viewer to see the images and the 3D
        ucoslam::MapViewer TheViewer;

        if (cml["-slam"]){
            Slam.readFromFile(cml("-slam"));
             vcap.set(cv::CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            vcap.set(cv::CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            TheMap=Slam.getMap();
            overwriteParamsByCommandLine(cml,params);
            Slam.updateParams(params);

        }

        if (cml["-noMapUpdate"])
            Slam.setMode(ucoslam::MODE_LOCALIZATION);

        cv::Mat auxImage;
        //Ok, lets start
        ucoslam::TimerAvrg Fps;
        bool finish = false;
        cv::Mat camPose_c2g;
        while (!finish && !in_image.empty()) {
            //image resize (if required)
            in_image = resize(in_image, vsize);


            //image undistortion (if required)
            if(undistort ){               
                cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
                in_image=auxImage;
                image_params.Distorsion.setTo(cv::Scalar::all(0));
            }


            int currentFrameIndex = vcap.getCurrentFrameIndex();
            Fps.start();
            camPose_c2g=Slam.process(in_image, image_params,currentFrameIndex);
            Fps.stop();


            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;
            //            Slam.drawMatches(in_image);
            //    char k = TheViewer.show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            char k = TheViewer.show(TheMap,   in_image, camPose_c2g,"#" + std::to_string(currentFrameIndex)/* + " fps=" + to_string(1./Fps.getAvrg())*/ ,Slam.getCurrentKeyFrameIndex());
            if (int(k) == 27)finish = true;//pressed ESC

            //save to output video?
            if (!TheOutputVideo.empty()){
                auto image=TheViewer.getImage();
                if(!videoout.isOpened())
                    videoout.open(TheOutputVideo, cv::VideoWriter::fourcc('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
                if(videoout.isOpened())  videoout.write(image);
            }

            //draw cube



            //reset?
            if (k=='r') Slam.clear();
            //write the current map
            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                TheMap->saveToFile("world-"+number+".map");
            }
            if (k=='v'){
                Slam.saveToFile("slam.slm");
            }
            //read next
            vcap >> in_image;
        }

        //release the video output if required
        if(videoout.isOpened()) videoout.release();

        //save the output

        TheMap->saveToFile(cml("-out","world") +".map");
        //save also the parameters finally employed
        params.saveToYMLFile("ucoslam_params_"+cml("-out","world") +".yml");
        if (debugLevel >=10){
            Slam.saveToFile("slam.slm");
        }
        TheMap->saveToMarkerMap("markermap.yml");


    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
    }
}
