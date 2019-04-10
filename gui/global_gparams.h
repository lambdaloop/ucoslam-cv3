#ifndef ARUCOGPARAM_H
#define ARUCOGPARAM_H
#include "gparam/gparam.h"
#include "aruco/markerdetector.h"
#include "aruco/dictionary.h"
#include "ucoslamtypes.h"
#include <iostream>
class ArucoGParams: public gparam::ParamSet {
public:
    ArucoGParams(){
        setName("ArucoParams");
        insert(gparam::Param("Dictionary", aruco::Dictionary::getDicTypes(),3));
        insert(gparam::Param("DetectionMode", { "DM_NORMAL","DM_FAST","DM_VIDEO_FAST"},0));
        insert(gparam::Param("CornerMethod", { "CORNER_SUBPIX","CORNER_LINES","CORNER_NONE"},0));
        insert(gparam::Param("MinMarkerSize", 0.02,0.,0.5,0.01));
        insert(gparam::Param("Threshold", 7,1,30,2));
        insert(gparam::Param("Enclosed", false));
    }

    //used as singleton
    static ArucoGParams & get(){
        static ArucoGParams    instance; // Guaranteed to be destroyed.                                         // Instantiated on first use.
        return instance;
    }

    //loads the detector from the params
    static void loadFromParams(aruco::MarkerDetector &mdetector){

       const ArucoGParams & ap= get();
       mdetector.setDictionary(ap["Dictionary"].asString());
       mdetector.getParameters().setDetectionMode( aruco::MarkerDetector::Params::getDetectionModeFromString( ap["DetectionMode"].asString()),ap["MinMarkerSize"].asDouble());
       mdetector.getParameters().setCornerRefinementMethod( aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(ap["CornerMethod"].asString()) );
       mdetector.getParameters().detectEnclosedMarkers(   ap["Enclosed"].asInt());
       mdetector.getParameters().ThresHold=ap["Threshold"].asInt();
    }
private:
};

class ArucoMarkerDetector{


public:
    static aruco::MarkerDetector & get(bool updateParams=true){
            static aruco::MarkerDetector mdetector;
            if (updateParams) ArucoGParams::loadFromParams(mdetector);
            return mdetector;
    }
};



class UcoSlamGParamsBasic: public gparam::ParamSet {
public:
    UcoSlamGParamsBasic(const gparam::ParamSet &ps):gparam::ParamSet (ps){

    }

    UcoSlamGParamsBasic(){
        create();
    }
    UcoSlamGParamsBasic(const ucoslam::Params &params){
        create();
        set(params);
    }


    void set(const ucoslam::Params &params){
        find("Sequential")=bool(params.runSequential);
        if(!params.detectMarkers)
            find("Detect Markers")="No";
        else
            find("Detect Markers")=params.aruco_Dictionary;
        find("Detect KeyPoints")=bool(params.detectKeyPoints);
        find("Min Confidence")=double(params.KFMinConfidence);
        find("Culling Factor")=double(params.KFCulling);
        find("Max Features")=int(params.maxFeatures);
        find("Non-Max Suppr")=bool(params.KPNonMaximaSuppresion);
        find("#Threads")=int(params.nthreads_feature_detector);
        find("Marker DetectionMode")= params.aruco_DetectionMode;
        find("Marker CornerMethod")=params.aruco_CornerRefimentMethod;
        find("Marker MinMarkerSize")=float(params.aruco_minMarkerSize==-1)?0.0:params.aruco_minMarkerSize;
    }

    void update(ucoslam::Params &params){
        params.runSequential=find("Sequential").get<bool>();
        if(find("Detect Markers").asString()=="No")
            params.detectMarkers=false;
        else
            params.aruco_Dictionary=find("Detect Markers").asString();
         params.detectKeyPoints=find("Detect KeyPoints").get<bool>();
        params.KFMinConfidence=find("Min Confidence").get<float>();
        params.KFCulling=find("Culling Factor").get<float>();
        params.maxFeatures=find("Max Features").get<int>();
        params.KPNonMaximaSuppresion=find("Non-Max Suppr").get<bool>();
        params.nthreads_feature_detector=find("#Threads").get<int>();

        params.aruco_DetectionMode=aruco::MarkerDetector::Params::getDetectionModeFromString( find("Marker DetectionMode").asString());
        params.aruco_CornerRefimentMethod=  aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(find("Marker CornerMethod").asString());
        params.aruco_minMarkerSize=find("Marker MinMarkerSize").get<float>();
    }
private:
    void create(){



        auto aruco_types= aruco::Dictionary::getDicTypes();
        aruco_types.insert(aruco_types.begin(),"No");
        insert(gparam::Param("Detect Markers",aruco_types,0));


        back().setDescription("(dis/en)ables marker detection");


        insert(gparam::Param("Detect KeyPoints",true));
        back().setDescription("(dis/en)ables keypoint detection");

        insert(gparam::Param("Sequential",true));
        back().setDescription("(dis/en)ables sequential processing.");
        insertAdvanced();
          insert(gparam::Param("Min Confidence",double(0.6),0.1,1.2,0.1));
        back().setDescription("value that regulates when a keyframe is added. Range (0,inf). low values will include less keyframes. high value will include more keyframes");

        insert(gparam::Param("Culling Factor",double(0.8),0.1,1.0,0.1));
        back().setDescription("Value indicating how many redundant keypoints must be in a keyframe to remove it. Range [0,1]. If low, few keyframes will survive. If 1, no keyframe will be removed once added.");


        insert(gparam::Param("Max Features",int(4000),1000,10000,100));
        back().setDescription("number of features to be detected in the image");

        insert(gparam::Param("Non-Max Suppr",false));
        back().setDescription("(De)/Activatesit keypoint non maxima suppresion. It will create maps with less points (faster)");

        insert(gparam::Param("#Threads",int(2),1,3));
        back().setDescription("Number of threads employed in the KeyPoint extractor");


         insert(gparam::Param("Marker DetectionMode", { "DM_NORMAL","DM_FAST","DM_VIDEO_FAST"},0));
         insert(gparam::Param("Marker CornerMethod", { "CORNER_SUBPIX","CORNER_LINES","CORNER_NONE"},0));
         insert(gparam::Param("Marker MinMarkerSize", 0.0,0.,0.5,0.01));
    }

};
class UcoSlamGParamsRunTime: public UcoSlamGParamsBasic{
public:
    UcoSlamGParamsRunTime(){
        std::vector<gparam::Param>::insert(begin(),gparam::Param("Mode", {"SLAM","Track Only"},0) );
        find("Marker Size").setDescription("Size of the markers");

    }
    UcoSlamGParamsRunTime(const ucoslam::Params &params):UcoSlamGParamsBasic(params){
        std::vector<gparam::Param>::insert(begin(),gparam::Param("Mode", {"SLAM","Track Only"},0) );
        find("Marker Size").setDescription("Size of the markers");

    }
    UcoSlamGParamsRunTime(const gparam::ParamSet &ps):UcoSlamGParamsBasic (ps){

    }
};

class UcoSlamGParams: public UcoSlamGParamsBasic{
public:
    UcoSlamGParams(){
         std::vector<gparam::Param>::insert(begin()+2,gparam::Param("Marker Size",double(1),1e-3,1e10) );
         find("Marker Size").setDescription("Size of the markers");

         std::vector<gparam::Param>::insert(begin(),gparam::Param("Img Resize Fact",double(1),0.1,1.,0.05) );
         find("Img Resize Fact").setDescription("Resize factor for the input image. Small values will speed up computation by reducing the original image size");


         insert(gparam::Param("Num Octaves",int(8),1,16));
         back().setDescription("Number of scales in the KeyPoint Pyramid");
         insert(gparam::Param("Scale Factor",double(1.2),1.05,3.,0.1));
         back().setDescription("Scale Factor for KeyPoint Pyramid");
    }
    void set(const ucoslam::Params &params){
        UcoSlamGParamsBasic::set(params);
        find("Marker Size")=double(params.aruco_markerSize);
        find("Num Octaves")=int(params.nOctaveLevels);
        find("Scale Factor")=double(params.scaleFactor);
    }

    void update(  ucoslam::Params &params){
        UcoSlamGParamsBasic::update(params);
        params.aruco_markerSize=find("Marker Size").get<float>();
        params.nOctaveLevels=find("Num Octaves").get<int>();
        params.scaleFactor=find("Scale Factor").get<float>();

    }

};

#endif // ARUCOGPARAM_H
