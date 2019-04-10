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
#include "basictypes/io_utils.h"
#include "basictypes/hash.h"
using namespace std;
namespace  ucoslam {
Params::Params(){
      global_optimizer= "g2o";//which global optimizer to use
      detectMarkers=true;//(dis/e)nables marker detection
      detectKeyPoints=true;//(dis/e)nables kp detection
      kpDescriptorType=DescriptorTypes::Type::DESC_ORB;//keypoint descriptor employed
      KPNonMaximaSuppresion=false;//activating it will create maps with less points (faster).
      KFMinConfidence=0.6;//value that regulates when a keyframe is added. Range (0,inf). low values will include less keyframes. high value will include more keyframes
      maxFeatures=4000;//number of features to be detected
      nOctaveLevels=8;//number of octaves for keypoint detection
      scaleFactor=1.2;//scale factor
      KFCulling=0.8;// Value indicating how many redundant keypoints must be in a keyframe to remove it. Range [0,1]. If low, few keyframes will survive. If 1, no keyframe will be removed once added.
      aruco_markerSize=1;//Size of markers in meters
      maxNewPoints=350;//maximum number of new points created when a new keyframe is added


       forceInitializationFromMarkers=false;//If true, the system will not initialize until a good initialization from markers is obtained
      nthreads_feature_detector=2;//number of threads employed in keypoint detection




    //////////////////////////////////////////
    ///Params not mean to be modified by users
    //////////////////////////////////////////
     maxVisibleFramesPerMarker=10;
     projDistThr=15;//when searching for points by projection, maximum 2d distance for search radius
     maxDescDistance=std::numeric_limits<float>::max();//minimum distance between descriptors to consider a possible match. It is internally rewritten according to the descriptor employed


}
void Params::setParams( bool sequential, DescriptorTypes::Type desc){
    if(desc==DescriptorTypes::DESC_ORB){
        kpDescriptorType=DescriptorTypes::Type::DESC_ORB;//keypoint descriptor employed
        nOctaveLevels=8;//number of octaves for keypoint detection
        scaleFactor=1.2;//scale factor
        runSequential=sequential;
    }
}

void Params::toStream(ostream &str)const{

    uint64_t start_sig=9837138769928;
    str.write((char*)&start_sig,sizeof(start_sig));

    str.write((char*)&detectMarkers,sizeof(detectMarkers));
    str.write((char*)&detectKeyPoints,sizeof(detectKeyPoints));
    str.write((char*)&targetFocus,sizeof(targetFocus));
    str.write((char*)&KFMinConfidence,sizeof(KFMinConfidence));
    str.write((char*)&KPNonMaximaSuppresion,sizeof(KPNonMaximaSuppresion));
    str.write((char*)&maxNewPoints,sizeof(maxNewPoints));


    str.write((char*)&forceInitializationFromMarkers,sizeof(forceInitializationFromMarkers));

    str.write((char*)&removeKeyPointsIntoMarkers,sizeof(removeKeyPointsIntoMarkers));
    str.write((char*)&maxDescDistance,sizeof(maxDescDistance));
    str.write((char*)&baseline_medianDepth_ratio_min,sizeof(baseline_medianDepth_ratio_min));
    str.write((char*)&aruco_markerSize,sizeof(aruco_markerSize));
    str.write((char*)&projDistThr,sizeof(projDistThr));
    str.write((char*)&nthreads_feature_detector,sizeof(nthreads_feature_detector));
    str.write((char*)&maxVisibleFramesPerMarker,sizeof(maxVisibleFramesPerMarker));
    str.write((char*)&minNumProjPoints,sizeof(minNumProjPoints));
    str.write((char*)&KFCulling,sizeof(KFCulling));
     str.write((char*)&thRefRatio,sizeof(thRefRatio));
    str.write((char*)&maxFeatures,sizeof(maxFeatures));
    str.write((char*)&nOctaveLevels,sizeof(nOctaveLevels));
    str.write((char*)&scaleFactor,sizeof(scaleFactor));
    str.write((char*)&kpDescriptorType,sizeof(kpDescriptorType));

    str.write((char*)&aruco_minerrratio_valid,sizeof(aruco_minerrratio_valid));
    str.write((char*)&aruco_minNumFramesRequired,sizeof(aruco_minNumFramesRequired));
    str.write((char*)&aruco_allowOneFrameInitialization,sizeof(aruco_allowOneFrameInitialization));

    str.write((char*)&minBaseLine,sizeof(minBaseLine));
    str.write((char*)&runSequential,sizeof(runSequential));

    str.write((char*)&markersOptWeight,sizeof(markersOptWeight));
    str.write((char*)&minMarkersForMaxWeight,sizeof(minMarkersForMaxWeight));

    toStream__(global_optimizer,str);
    toStream__(aruco_Dictionary,str);
    toStream__(aruco_DetectionMode,str);
    toStream__(aruco_CornerRefimentMethod,str);
    str.write((char*)&aruco_minMarkerSize,sizeof(aruco_minMarkerSize));
    str.write((char*)&kptImageScaleFactor,sizeof(kptImageScaleFactor));
    str.write((char*)&autoAdjustKpSensitivity,sizeof(autoAdjustKpSensitivity));
    str.write((char*)&reLocalizationWithKeyPoints,sizeof(reLocalizationWithKeyPoints));
    str.write((char*)&reLocalizationWithMarkers,sizeof(reLocalizationWithMarkers));
    str.write((char*)&inPlaneMarkers,sizeof(inPlaneMarkers));




    toStream__(extraParams,str);
    uint64_t end_sig=1837138769921;
    str.write((char*)&end_sig,sizeof(start_sig));

}


void Params::fromStream(istream &str){
    uint64_t start_sig=0;
    str.read((char*)&start_sig,sizeof(start_sig));
    if(start_sig!=9837138769928) throw std::runtime_error("Invalid signature");

    str.read((char*)&detectMarkers,sizeof(detectMarkers));
    str.read((char*)&detectKeyPoints,sizeof(detectKeyPoints));
    str.read((char*)&targetFocus,sizeof(targetFocus));
    str.read((char*)&KFMinConfidence,sizeof(KFMinConfidence));
    str.read((char*)&KPNonMaximaSuppresion,sizeof(KPNonMaximaSuppresion));
    str.read((char*)&maxNewPoints,sizeof(maxNewPoints));

    str.read((char*)&forceInitializationFromMarkers,sizeof(forceInitializationFromMarkers));
    str.read((char*)&removeKeyPointsIntoMarkers,sizeof(removeKeyPointsIntoMarkers));
    str.read((char*)&maxDescDistance,sizeof(maxDescDistance));
    str.read((char*)&baseline_medianDepth_ratio_min,sizeof(baseline_medianDepth_ratio_min));
    str.read((char*)&aruco_markerSize,sizeof(aruco_markerSize));
    str.read((char*)&projDistThr,sizeof(projDistThr));
    str.read((char*)&nthreads_feature_detector,sizeof(nthreads_feature_detector));
    str.read((char*)&maxVisibleFramesPerMarker,sizeof(maxVisibleFramesPerMarker));
    str.read((char*)&minNumProjPoints,sizeof(minNumProjPoints));
    str.read((char*)&KFCulling,sizeof(KFCulling));
    str.read((char*)&thRefRatio,sizeof(thRefRatio));
    str.read((char*)&maxFeatures,sizeof(maxFeatures));
    str.read((char*)&nOctaveLevels,sizeof(nOctaveLevels));
    str.read((char*)&scaleFactor,sizeof(scaleFactor));
    str.read((char*)&kpDescriptorType,sizeof(kpDescriptorType));


    str.read((char*)&aruco_minerrratio_valid,sizeof(aruco_minerrratio_valid));
    str.read((char*)&aruco_minNumFramesRequired,sizeof(aruco_minNumFramesRequired));
    str.read((char*)&aruco_allowOneFrameInitialization,sizeof(aruco_allowOneFrameInitialization));
      str.read((char*)&minBaseLine,sizeof(minBaseLine));
    str.read((char*)&runSequential,sizeof(runSequential));


    str.read((char*)&markersOptWeight,sizeof(markersOptWeight));
    str.read((char*)&minMarkersForMaxWeight,sizeof(minMarkersForMaxWeight));

    fromStream__(global_optimizer,str);

    fromStream__(aruco_Dictionary,str);
    fromStream__(aruco_DetectionMode,str);
    fromStream__(aruco_CornerRefimentMethod,str);
    str.read((char*)&aruco_minMarkerSize,sizeof(aruco_minMarkerSize));
    str.read((char*)&kptImageScaleFactor,sizeof(kptImageScaleFactor));
    str.read((char*)&autoAdjustKpSensitivity,sizeof(autoAdjustKpSensitivity));
    str.read((char*)&reLocalizationWithKeyPoints,sizeof(reLocalizationWithKeyPoints));
    str.read((char*)&reLocalizationWithMarkers,sizeof(reLocalizationWithMarkers));
    str.read((char*)&inPlaneMarkers,sizeof(inPlaneMarkers));
    fromStream__(extraParams,str);

    //read until end signature

    uint64_t end_sig=0;
    while(end_sig!=1837138769921 && !str.eof())
        str.read((char*)&end_sig,sizeof(start_sig));
    if(str.eof())throw std::runtime_error("Reached EOF without finding end signature");

}

uint64_t  Params::getSignature()const{
    Hash sig;
    sig+=detectMarkers;
    sig+=detectKeyPoints;
    sig+=kpDescriptorType;
    sig+=KPNonMaximaSuppresion;
    sig+=KFMinConfidence;
    sig+=maxFeatures;
    sig+=nOctaveLevels;
    sig+=scaleFactor;
    sig+=KFCulling;
    sig+=aruco_markerSize;
    sig+=maxNewPoints;
//    sig+=aruco_DetectorParams.;
    sig+=forceInitializationFromMarkers;
    sig+=nthreads_feature_detector;
    sig+=maxDescDistance;
    sig+=baseline_medianDepth_ratio_min;
    sig+=global_optimizer;
    sig+=minNumProjPoints;
    sig+=projDistThr;
    sig+=maxVisibleFramesPerMarker;
    sig+=aruco_minNumFramesRequired;
    sig+=aruco_minerrratio_valid;
    sig+=aruco_allowOneFrameInitialization;
    sig+=targetFocus;
    sig+=thRefRatio;
    sig+=minBaseLine;
    sig+=removeKeyPointsIntoMarkers;
    sig+=aruco_Dictionary;
    sig+=aruco_DetectionMode;
    sig+=aruco_CornerRefimentMethod;
    sig+=aruco_minMarkerSize;
    sig+=kptImageScaleFactor;
    sig+=autoAdjustKpSensitivity;
    sig+=reLocalizationWithKeyPoints;
    sig+=reLocalizationWithMarkers;
    sig+=inPlaneMarkers;

  return sig;
}

void Params::saveToYMLFile(const string &path){
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened())throw std::runtime_error("Could not open "+path);
    fs<<"kpDescriptorType" << DescriptorTypes::toString( kpDescriptorType);
    fs<<"KPNonMaximaSuppresion"<<KPNonMaximaSuppresion;
    fs<<"targetFocus"<<targetFocus;
    fs<<"minKFConfidence"<<KFMinConfidence;
    fs<<"maxNewPoints"<<maxNewPoints;
    fs<<"targetWidth" <<kptImageScaleFactor;

    fs<<"maxFeatures" <<maxFeatures;
    fs<<"nthreads_feature_detector" <<nthreads_feature_detector;
    fs<<"nOctaveLevels" <<nOctaveLevels;
    fs<<"scaleFactor" <<scaleFactor;
    fs<<"runSequential" <<runSequential;
    fs<<"detectMarkers" <<detectMarkers;
    fs<<"forceInitializationFromMarkers"<<forceInitializationFromMarkers;
    fs<<"aruco_allowOneFrameInitialization" <<aruco_allowOneFrameInitialization;
    fs<<"reLocalizationWithKeyPoints" <<reLocalizationWithKeyPoints;
    fs<<"reLocalizationWithMarkers" <<reLocalizationWithMarkers;
    fs<<"inPlaneMarkers" <<inPlaneMarkers;



    fs<<"minBaseLine" <<minBaseLine;
    fs<<"detectKeyPoints" <<detectKeyPoints;
    fs<<"removeKeyPointsIntoMarkers" <<removeKeyPointsIntoMarkers;
    fs<<"minDescDistance" <<maxDescDistance;
    fs<<"baseline_medianDepth_ratio_min" <<baseline_medianDepth_ratio_min;
    fs<<"aruco_markerSize" <<aruco_markerSize;
    fs<<"projDistThr" <<projDistThr;
    fs<<"maxVisibleFramesPerMarker" <<maxVisibleFramesPerMarker;
    fs<<"minNumProjPoints" <<minNumProjPoints;
    fs<<"keyFrameCullingPercentage" <<KFCulling;
     fs<<"thRefRatio" <<thRefRatio;
    fs<<"aruco_minerrratio_valid" <<aruco_minerrratio_valid;

    fs<<"aruco_minNumFramesRequired" <<aruco_minNumFramesRequired;
    fs<<"markersOptWeight" <<markersOptWeight;
    fs<<"minMarkersForMaxWeight" <<minMarkersForMaxWeight;
    fs<<"autoAdjustKpSensitivity" <<autoAdjustKpSensitivity;



    fs<<"global_optimizer" <<global_optimizer;

    fs<<"aruco-dictionary" <<aruco_Dictionary;
    fs<<"aruco-detectMode" <<aruco_DetectionMode;
    fs<<"aruco-cornerRefinementM" <<aruco_CornerRefimentMethod;
    fs<<"aruco-minSize" <<aruco_minMarkerSize;
    fs<<"extraParams" <<extraParams;




}


void Params:: readFromYMLFile(const string &filePath){



    //first
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error("CameraParameters::readFromXMLFile could not open file:"+filePath);
    attemtpRead("detectMarkers",detectMarkers,fs);
    attemtpRead("targetFocus",targetFocus,fs);
    attemtpRead("minKFConfidence",KFMinConfidence,fs);
    attemtpRead("KPNonMaximaSuppresion",KPNonMaximaSuppresion,fs);
    attemtpRead("maxNewPoints",maxNewPoints,fs);
    attemtpRead("targetWidth",kptImageScaleFactor,fs);

    attemtpRead("maxFeatures",maxFeatures,fs);
    attemtpRead("detectKeyPoints",detectKeyPoints,fs);
    attemtpRead("removeKeyPointsIntoMarkers",removeKeyPointsIntoMarkers,fs);
    attemtpRead("minDescDistance",maxDescDistance,fs);
    attemtpRead("baseline_medianDepth_ratio_min",baseline_medianDepth_ratio_min,fs);
    attemtpRead("aruco_markerSize",aruco_markerSize,fs);
    attemtpRead("projDistThr",projDistThr,fs);
    attemtpRead("nthreads_feature_detector",nthreads_feature_detector,fs);
    attemtpRead("maxVisibleFramesPerMarker",maxVisibleFramesPerMarker,fs);
    attemtpRead("minNumProjPoints",minNumProjPoints,fs);
    attemtpRead("keyFrameCullingPercentage",KFCulling,fs);
     attemtpRead("thRefRatio",thRefRatio,fs);
    attemtpRead("nOctaveLevels",nOctaveLevels,fs);
    attemtpRead("scaleFactor",scaleFactor,fs);
    attemtpRead("aruco_minerrratio_valid",aruco_minerrratio_valid,fs);
    attemtpRead("aruco_minNumFramesRequired",aruco_minNumFramesRequired,fs);
    attemtpRead("aruco_allowOneFrameInitialization",aruco_allowOneFrameInitialization,fs);
    attemtpRead("minBaseLine",minBaseLine,fs);
    attemtpRead("runSequential",runSequential,fs);
    attemtpRead("markersOptWeight",markersOptWeight,fs);
    attemtpRead("minMarkersForMaxWeight",minMarkersForMaxWeight,fs);

    attemtpRead("autoAdjustKpSensitivity",autoAdjustKpSensitivity,fs);
    attemtpRead("reLocalizationWithKeyPoints",reLocalizationWithKeyPoints,fs);
    attemtpRead("reLocalizationWithMarkers",reLocalizationWithMarkers,fs);
    attemtpRead("inPlaneMarkers",inPlaneMarkers,fs);

    attemtpRead("global_optimizer",global_optimizer,fs);

    attemtpRead("aruco-dictionary",aruco_Dictionary,fs);
    attemtpRead("aruco-detectMode",aruco_DetectionMode,fs);
    attemtpRead("aruco-cornerRefinementM",aruco_CornerRefimentMethod,fs);
    attemtpRead("aruco-minSize",aruco_minMarkerSize,fs);
    attemtpRead("extraParams",extraParams,fs);


    string auxStr="orb";
    attemtpRead("kpDescriptorType",auxStr,fs);
    kpDescriptorType= DescriptorTypes::fromString(auxStr);


}



}
