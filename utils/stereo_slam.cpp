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
#include "map.h"
#include "mapviewer.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "basictypes/debug.h"
#include "stereorectify.h"
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

cv::Mat composedStereo(cv::Mat left ,cv::Mat &right){
    cv::Mat comp(left.rows,left.cols*2,left.type());
    auto aux=comp.colRange(0,left.cols);
    left.copyTo(aux);
    aux=comp.colRange(left.cols,2*left.cols);
    right.copyTo(aux);
    for(int i=0;i<comp.rows;i+=20)
        cv::line(comp,cv::Point(0,i),cv::Point(comp.cols,i),cv::Scalar(0,255,0),1);
    return comp;
}
int main(int argc, char* argv[]){
    CmdLineParser cml(argc,argv);
    if(argc<4 || cml["-h"]){
        cout<<"Usage: <video1> <video2> <stereo_calibration_file> [-voc path] [-params ucoslamparams.yml]"<<endl;
        return -1;
    }
    ucoslam::StereoRectify Rectifier;
    ucoslam::ImageParams imp;
     Rectifier.readFromXMLFile(argv[3]);

    cv::Mat inImage[2];
    cv::VideoCapture video[2];
    for(int i=0;i<2;i++){
        video[i].open(argv[i+1]);
        if(!video[i].isOpened())
            throw runtime_error(string("Cannot open video file at:")+argv[i+1]);
    }

    ucoslam::Params sparams;
    sparams.detectMarkers=false;
    sparams.KFMinConfidence=0.8;
    sparams.runSequential=cml["-sequential"];

    if(cml["-params"])
        sparams.readFromYMLFile(cml("-params"));

    auto smap=make_shared<ucoslam::Map>();
    ucoslam::UcoSlam system;
    ucoslam::MapViewer mv;
    system.setParams(smap,sparams,cml("-voc",""));
    char key=0;
    while( video[0].grab() && video[1].grab() && key!=27){
        int frameNumber=video[0].get(cv::CAP_PROP_POS_FRAMES);
        video[0].retrieve(inImage[0]);
        video[1].retrieve(inImage[1]);
        Rectifier.rectify(inImage[0],inImage[1]);

//        cv::imshow("composed",composedStereo(Rectifier.getLeft(),Rectifier.getRight()));
        cv::Mat pose=system.processStereo(Rectifier.getLeft(),Rectifier.getRight(),Rectifier.getImageParams(),frameNumber );
        key=mv.show(smap,Rectifier.getLeft(),pose,"");
    }
    smap->saveToFile("world.map");

    return 0;
}
