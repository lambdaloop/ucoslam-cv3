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
#include "framematcher.h"
#include "basictypes/misc.h"
#include "basictypes/timers.h"
#include "map_types/mappoint.h"
#include "basictypes/hash.h"
#include "basictypes/io_utils.h"
#include <xflann/xflann.h>
#include <fbow/fbow.h>
namespace ucoslam
{
namespace _impl {

class FrameMatcher_impl{
public:

    virtual void setParams(const Frame &trainFrame,FrameMatcher::Mode mode=FrameMatcher::MODE_ALL,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1)=0;



    //normal match.
    //maxSearch number of comparisons in the  approximated search
    //only searchs for best and  second best
    virtual std::vector<cv::DMatch> match(const Frame &queryFrame, FrameMatcher::Mode mode )=0;

    //matches considering epipolar constrains
    //F12 is the SE3 matrix moving points from train 2 query
    //nn : number nearest neighbor searched
    //maxSearch number of comparisons in the  approximated search
    virtual std::vector<cv::DMatch> matchEpipolar(const Frame &queryFrame, FrameMatcher::Mode mode, const cv::Mat &FQ2T )=0;


//    virtual uint64_t getSignature()const=0;
//    virtual void toStream(ostream &str)const=0;
//    virtual void fromStream(istream &str)=0 ;

    ImageParams _trainImageParams;
    cv::Mat getFund12(const cv::Mat &TrainCamMatrix,const cv::Mat &QueryCamMatrix,const cv::Mat &FQ2T);
};

cv::Mat FrameMatcher_impl::getFund12(const cv::Mat &TrainCamMatrix,const cv::Mat &QueryCamMatrix,const cv::Mat &FQ2T){
    cv::Mat FQ2T_32f;
    if(FQ2T.empty()) return cv::Mat();
    FQ2T.convertTo(FQ2T_32f,CV_32F);
    cv::Mat F12=   computeF12(cv::Mat::eye(4,4,CV_32F),TrainCamMatrix,FQ2T_32f,QueryCamMatrix);
    return F12;
}


void   ucoslam_FM___computeThreeMaxima(const vector<vector<int>> & histo,   int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(size_t i=0; i<histo.size(); i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


//CLass to analyzed matches between two frames using FLANN

class FrameMatcher_Flann:public FrameMatcher_impl{
    std::vector<uint32_t> map_idx_trainkp;
    xflann::Index trainIndex;
    std::vector<cv::KeyPoint> train_kp;
    float _minDescDist=std::numeric_limits<float>::max();
    float _nn_match_ratio=0.8;
    bool _checkOrientation=true;
    int _maxOctaveDiff=1;
    int maxSearch=16;
    int nn=10;
public:




    FrameMatcher_Flann();
    FrameMatcher_Flann(const Frame &trainFrame,FrameMatcher::Mode mode=FrameMatcher::MODE_ALL,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);
    void setParams(const Frame &trainFrame,FrameMatcher::Mode mode=FrameMatcher::MODE_ALL,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);
    bool isValid()const{return train_kp.size()!=0;}
    //normal match.
    //maxSearch number of comparisons in the  approximated search
    //only searchs for best and  second best
    std::vector<cv::DMatch> match(const Frame &queryFrame, FrameMatcher::Mode mode  );

    //matches considering epipolar constrains
    //F12 is the SE3 matrix moving points from train 2 query
    //nn : number nearest neighbor searched
    //maxSearch number of comparisons in the  approximated search
    std::vector<cv::DMatch> matchEpipolar(const Frame &queryFrame, FrameMatcher::Mode mode, const cv::Mat &FQ2T );


    uint64_t getSignature()const;
    void toStream(ostream &str)const;
    void fromStream(istream &str) ;


private:

    void  manageMode(FrameMatcher::Mode mode,const Frame &frame,vector<uint32_t> &map_idx,cv::Mat &desc);


};



FrameMatcher_Flann::FrameMatcher_Flann(){

}
FrameMatcher_Flann::FrameMatcher_Flann(const Frame &trainFrame,FrameMatcher::Mode mode,float minDescDist, float nn_match_ratio, bool checkOrientation, int maxOctaveDiff){
    setParams(trainFrame,mode,minDescDist,nn_match_ratio,checkOrientation,maxOctaveDiff);
}

//saves to map_idx and desc the elements that will be used depending on the mode
void FrameMatcher_Flann::manageMode(FrameMatcher::Mode mode,const Frame &frame,vector<uint32_t> &map_idx,cv::Mat &desc){
    map_idx.clear();
    if(mode==FrameMatcher::MODE_ALL){//use all keypoints
        desc=frame.desc;
        map_idx.resize(frame.ids.size());
        for(size_t i=0;i<map_idx.size();i++)
            map_idx[i]=i;
    }
    else if(mode==FrameMatcher::MODE_ASSIGNED){ //save only assigned ones
        map_idx.reserve(frame.ids.size());
        desc=cv::Mat(frame.ids.size(),frame.desc.cols,frame.desc.type());

        for(size_t i=0;i<frame.ids.size();i++)
            if(frame.ids[i]!=std::numeric_limits<uint32_t>::max() && !frame.flags[i].is(Frame::FLAG_NONMAXIMA)){
                frame.desc.row(i).copyTo(desc.row(map_idx.size()));
                map_idx.push_back(i);
            }
        desc.resize(map_idx.size());
    }
    else if(mode==FrameMatcher::MODE_UNASSIGNED){

        map_idx.reserve(frame.ids.size());
        desc=cv::Mat(frame.ids.size(),frame.desc.cols,frame.desc.type());

        for(size_t i=0;i<frame.ids.size();i++)
            if(frame.ids[i]==std::numeric_limits<uint32_t>::max() && !frame.flags[i].is(Frame::FLAG_NONMAXIMA)){
                frame.desc.row(i).copyTo(desc.row(map_idx.size()));
                map_idx.push_back(i);
            }
        desc.resize(map_idx.size());
    }


}
void FrameMatcher_Flann::setParams(const Frame &trainFrame, FrameMatcher::Mode  mode,float minDescDist, float nn_match_ratio, bool checkOrientation , int maxOctaveDiff){
    _minDescDist=minDescDist;
    _nn_match_ratio=nn_match_ratio;
    _checkOrientation=checkOrientation;
    _maxOctaveDiff=maxOctaveDiff;
    _trainImageParams=trainFrame.imageParams;
    //determine which are used.


    cv::Mat desc;
    manageMode(mode,trainFrame,map_idx_trainkp,desc);
    train_kp=trainFrame.und_kpts;
    trainIndex.clear();
    trainIndex.build(desc,xflann::HKMeansParams(32,0));
    train_kp=trainFrame.und_kpts;
}
std::vector<cv::DMatch> FrameMatcher_Flann::match(const Frame &queryFrame ,FrameMatcher::Mode  mode ){
return matchEpipolar(queryFrame,mode,cv::Mat());
}









std::vector<cv::DMatch> FrameMatcher_Flann::matchEpipolar(const Frame &queryFrame,FrameMatcher::Mode mode,const cv::Mat &FQ2T  ){
    __UCOSLAM_ADDTIMER__

    std::vector<uint32_t> map_idx_query;
    cv::Mat QueryDesc;
    manageMode(mode,queryFrame,map_idx_query,QueryDesc);
    cv::Mat F12=getFund12(_trainImageParams.CameraMatrix,queryFrame.imageParams.CameraMatrix,FQ2T);


    __UCOSLAM_TIMER_EVENT__("manage mode");
    cv::Mat indices,distances;
     if(!trainIndex.search(QueryDesc,nn,indices,distances ,xflann::KnnSearchParams(maxSearch,false)) )
        return{};
    __UCOSLAM_TIMER_EVENT__("search");

    vector<cv::DMatch> matches;
    if ( distances.type()==CV_32S)
        distances.convertTo(distances,CV_32F);
    vector<float> scaleFactor2(queryFrame.scaleFactors);
    for(auto &v:scaleFactor2) v=v*v;
    for(int i = 0; i < indices.rows; i++) {
        float bestDist=_minDescDist,bestDist2=std::numeric_limits<float>::max();
        int64_t bestQuery=-1,bestTrain=-1;
        int octaveBest2=-1;
        int queryIndex=map_idx_query[i];
        const auto &queryKp=queryFrame.und_kpts[queryIndex];
        for(int j=0;j<indices.cols;j++){
            if ( distances.at<float>(i,j)>_minDescDist) continue;
            if ( distances.at<float>(i,j)<bestDist2){
                int trainIdex=map_idx_trainkp[ indices.at<int>(i,j)];
                const auto &trainKp=train_kp[trainIdex];
                if ( std::abs(trainKp.octave-queryKp.octave)>_maxOctaveDiff) continue;
                if ( !F12.empty())
                if (epipolarLineSqDist(trainKp.pt ,queryKp.pt,F12 ) >=3.84*scaleFactor2[queryKp.octave])continue;
                    if (distances.at<float>(i,j)<bestDist){
                        bestDist=distances.at<float>(i,j);
                        bestQuery=queryIndex;
                        bestTrain=trainIdex;
                    }
                    else{
                        bestDist2=distances.at<float>(i,j);
                        octaveBest2=trainKp.octave;
                    }
                }

        }

        if ( bestQuery!=-1){
            if(!( octaveBest2==queryFrame.und_kpts[bestQuery].octave &&  bestDist > bestDist2*_nn_match_ratio  ))
            {
                cv::DMatch  match;
                match.queryIdx=bestQuery;
                match.trainIdx=bestTrain;
                match.distance=bestDist;
                matches.push_back(match);
            }
        }

    }
    filter_ambiguous_train(matches);


    if(_checkOrientation)
    {
        vector<vector<int>> rotHist(30);
        for(auto &v:rotHist) v.reserve(500);
        const float factor = 1.0f/float(rotHist.size());
        for(size_t midx=0;midx<matches.size();midx++){
            const auto &match=matches[midx];
            float rot =train_kp[match.trainIdx].angle-queryFrame.und_kpts[match.queryIdx].angle;
            if(rot<0.0)
                rot+=360.0f;
            size_t bin = round(rot*factor);
            if(bin==rotHist.size())
                bin=0;
            assert(bin>=0 && bin<rotHist.size());
            rotHist[bin].push_back(midx);
        }

        int ind1=-1,ind2=-1,ind3=-1;
        ucoslam_FM___computeThreeMaxima(rotHist,ind1,ind2,ind3);
        for(int i=0; i<int(rotHist.size()); i++)
        {
            if(i==ind1 || i==ind2 || i==ind3) continue;
            for(auto midx : rotHist[i] )
                matches[midx].queryIdx=matches[midx].trainIdx=-1;//mark as unused
        }
        remove_unused_matches(matches);
    }

    return matches;
}




uint64_t FrameMatcher_Flann::getSignature()const{
    Hash sig;
    sig.add(map_idx_trainkp.begin(),map_idx_trainkp.end());
    sig.add(train_kp.begin(),train_kp.end());
    sig+=trainIndex.hash();
    sig+=_minDescDist;
    sig+=_nn_match_ratio;
    sig+=_checkOrientation;
    sig+=_maxOctaveDiff;

    return sig;
}
void FrameMatcher_Flann::toStream(ostream &str)const{
    toStream__(map_idx_trainkp,str);
    toStream__(train_kp,str);
    trainIndex.toStream(str);
    str.write((char*)&_minDescDist,sizeof(_minDescDist));
    str.write((char*)&_nn_match_ratio,sizeof(_nn_match_ratio));
    str.write((char*)&_checkOrientation,sizeof(_checkOrientation));
    str.write((char*)&_maxOctaveDiff,sizeof(_maxOctaveDiff));
}
void FrameMatcher_Flann::fromStream(istream &str) {
    fromStream__(map_idx_trainkp,str);
    fromStream__(train_kp,str);
    trainIndex.fromStream(str);

    str.read((char*)&_minDescDist,sizeof(_minDescDist));
    str.read((char*)&_nn_match_ratio,sizeof(_nn_match_ratio));
    str.read((char*)&_checkOrientation,sizeof(_checkOrientation));
    str.read((char*)&_maxOctaveDiff,sizeof(_maxOctaveDiff));
}



////////////////////77



class FrameMatcher_BoW:public FrameMatcher_impl{



public:
     FrameMatcher_BoW();
    FrameMatcher_BoW(const Frame &trainFrame, FrameMatcher::Mode trainMode,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);
    void setParams( const Frame &trainFrame,FrameMatcher:: Mode trainMode,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);
    std::vector<cv::DMatch> matchEpipolar( const Frame &queryFrame, FrameMatcher::Mode queryMode,const cv::Mat &FQ2T );
    std::vector<cv::DMatch> match(const Frame &queryFrame, FrameMatcher::Mode mode ){return matchEpipolar(queryFrame,mode,cv::Mat());}
private:
    inline bool isUsed(const Frame &frame,int idx,FrameMatcher::Mode mode){
        if (frame.flags[idx].is(Frame::FLAG_NONMAXIMA) )return false;
        if (mode==FrameMatcher::MODE_ALL)return true;
        if (mode==FrameMatcher::MODE_ASSIGNED && frame.ids[idx]==std::numeric_limits<uint32_t>::max() ) return false;
        if (mode==FrameMatcher::MODE_UNASSIGNED && frame.ids[idx]!=std::numeric_limits<uint32_t>::max()) return false;
        return true;
    }
    float _minDescDist=std::numeric_limits<float>::max();
    float _nn_match_ratio=0.8;
    bool _checkOrientation=true;
    int _maxOctaveDiff=1;
    FrameMatcher::Mode _trainMode;
    Frame *_trainKF=nullptr;//maintains only a reference!!!
};

FrameMatcher_BoW::FrameMatcher_BoW(){}

FrameMatcher_BoW::FrameMatcher_BoW(const Frame &trainFrame, FrameMatcher::Mode trainMode,float minDescDist, float nn_match_ratio, bool checkOrientation , int maxOctaveDiff){
    setParams( trainFrame,trainMode, minDescDist,   nn_match_ratio,   checkOrientation ,   maxOctaveDiff);
}


void FrameMatcher_BoW::setParams(const Frame &trainFrame, FrameMatcher::Mode trainMode,float minDescDist, float nn_match_ratio, bool checkOrientation , int maxOctaveDiff){
    _minDescDist=minDescDist;
    _nn_match_ratio=nn_match_ratio;
    _checkOrientation=checkOrientation;
    _maxOctaveDiff=maxOctaveDiff;
    _trainMode=trainMode;
    _trainKF=(Frame*)&trainFrame;
    _trainImageParams=trainFrame.imageParams;
}



std::vector<cv::DMatch> FrameMatcher_BoW::matchEpipolar(  const Frame &queryFrame, FrameMatcher::Mode queryMode,const cv::Mat &FQ2T){

    assert(_trainKF!=nullptr);
    const Frame &trainFrame=*_trainKF;
    assert(!trainFrame.bowvector_level->empty());
    assert(!queryFrame.bowvector_level->empty());


    std::vector<cv::DMatch>  matches;
    vector<float> scaleFactor2(queryFrame.scaleFactors);
    for(auto &v:scaleFactor2) v=v*v;


    fbow::fBow2::const_iterator query_it, train_it;
    const fbow::fBow2::const_iterator query_end = queryFrame.bowvector_level->end();
    const fbow::fBow2::const_iterator train_end = trainFrame.bowvector_level->end();

    query_it = queryFrame.bowvector_level->begin();
    train_it = trainFrame.bowvector_level->begin();
    cv::Mat F12=getFund12(_trainImageParams.CameraMatrix,queryFrame.imageParams.CameraMatrix,FQ2T);


    while(query_it != query_end && train_it != train_end)
    {

        if(query_it->first == train_it->first)//same word
        {

            //check here the elements
            const auto &query_vidx=query_it->second;//indices of the keypoints in the trainframe
            const auto &train_vidx=train_it->second;//indices of the keypoints in the queryframe
            if ( query_vidx.size()==0 || train_vidx.size()==0)continue;
            //match query to train

            for(auto qidx:query_vidx){
                //can be used?
                if(!isUsed(queryFrame,qidx,queryMode)) continue;

                const auto &queryKp=queryFrame.und_kpts[qidx];
                float bestDist=_minDescDist,bestDist2=std::numeric_limits<float>::max();
                int64_t bestQuery=-1,bestTrain=-1;
                int octaveBest2=-1;

                for(auto tidx:train_vidx){
                    if(!isUsed(trainFrame,tidx,_trainMode)) continue;
                    const auto &trainKp=trainFrame.und_kpts[tidx];
                    if ( std::abs(trainKp.octave-queryKp.octave)>_maxOctaveDiff) continue;

                    if(!F12.empty())
                     if (epipolarLineSqDist(trainKp.pt ,queryKp.pt,F12 ) >=3.84*scaleFactor2[queryKp.octave])continue;
                    //compute actual distace
                    assert( tidx<trainFrame.desc.rows);
                    assert( qidx<queryFrame.desc.rows);
                    float dist=MapPoint::getDescDistance(trainFrame.desc.row(tidx),queryFrame.desc.row(qidx));

                    if (dist<bestDist){
                        bestDist=dist;
                        bestQuery=qidx;
                        bestTrain=tidx;
                    }
                    else{
                        bestDist2=dist;
                        octaveBest2=trainKp.octave;
                    }
                }
                if ( bestQuery!=-1){
                    if(!( octaveBest2==queryFrame.und_kpts[bestQuery].octave &&  bestDist > bestDist2*_nn_match_ratio  ))
                    {
                        cv::DMatch  match;
                        match.queryIdx=bestQuery;
                        match.trainIdx=bestTrain;
                        match.distance=bestDist;
                        matches.push_back(match);
                    }
                }


            }

            // move v1 and v2 forward
            ++query_it;
            ++train_it;
        }
        else if(query_it->first < train_it->first)// move v1 forward
        {
            while(query_it!=query_end&& query_it->first<train_it->first) ++query_it;
        }
        else// move v2 forward

        {
            while(train_it!=train_end && train_it->first<query_it->first) ++train_it;
        }
    }

    filter_ambiguous_train(matches);



    if(_checkOrientation)
    {
        vector<vector<int>> rotHist(30);
        for(auto &v:rotHist) v.reserve(500);
        const float factor = 1.0f/float(rotHist.size());
        for(size_t midx=0;midx<matches.size();midx++){
            const auto &match=matches[midx];
            float rot =trainFrame.und_kpts[match.trainIdx].angle-queryFrame.und_kpts[match.queryIdx].angle;
            if(rot<0.0)
                rot+=360.0f;
            size_t bin = round(rot*factor);
            if(bin==rotHist.size())
                bin=0;
            assert(bin>=0 && bin<rotHist.size());
            rotHist[bin].push_back(midx);
        }

        int ind1=-1,ind2=-1,ind3=-1;
        ucoslam_FM___computeThreeMaxima(rotHist,ind1,ind2,ind3);
        for(int i=0; i<int(rotHist.size()); i++)
        {
            if(i==ind1 || i==ind2 || i==ind3) continue;
            for(auto midx : rotHist[i] )
                matches[midx].queryIdx=matches[midx].trainIdx=-1;//mark as unused
        }
        remove_unused_matches(matches);
    }

    return matches;

}
}
////////////////////////////////////////777////
///////////////////////////////////////////////
///
///
///



FrameMatcher::    FrameMatcher(Type t){
    _type=t;
}
void FrameMatcher::setParams(const Frame &trainFrame,Mode mode,float minDescDist , float nn_match_ratio , bool checkOrientation , int maxOctaveDiff){

    switch (_type) {
    case TYPE_AUTO:
    {
        if (trainFrame.bowvector_level->size()==0)
            _impl=std::make_shared<_impl::FrameMatcher_Flann>();
        else
            _impl=std::make_shared<_impl::FrameMatcher_BoW>();

    }break;
    case TYPE_BOW:
        _impl=std::make_shared<_impl::FrameMatcher_BoW>();
        break;
    case TYPE_FLANN:
        _impl=std::make_shared<_impl::FrameMatcher_Flann>();
        break;

    };

    _impl->setParams(trainFrame,mode,minDescDist,nn_match_ratio,checkOrientation,maxOctaveDiff);
}



//normal match.
//maxSearch number of comparisons in the  approximated search
//only searchs for best and  second best
std::vector<cv::DMatch> FrameMatcher::match(const Frame &queryFrame, Mode mode ){
    assert(_impl);
    return _impl->match(queryFrame,mode);
}

//matches considering epipolar constrains
//F12 is the SE3 matrix moving points from train 2 query
//nn : number nearest neighbor searched
//maxSearch number of comparisons in the  approximated search
std::vector<cv::DMatch> FrameMatcher::matchEpipolar(const Frame &queryFrame, Mode mode, const cv::Mat &FQ2T ){
    assert(_impl);
    return _impl->matchEpipolar(queryFrame,mode,FQ2T);
}


//uint64_t FrameMatcher::getSignature()const{
//    assert(_impl);
//    return _impl->getSignature();
//}
//void FrameMatcher::toStream(ostream &str)const{
//    assert(_impl);
//    return _impl->toStream(str);
//}
//void FrameMatcher::fromStream(istream &str) {
//    assert(_impl);
//    return _impl->fromStream(str);
//}

}




