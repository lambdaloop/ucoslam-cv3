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
#ifndef ucoslam_MapPoint_H
#define ucoslam_MapPoint_H
#include "ucoslam_exports.h"
#include <opencv2/core/core.hpp>
#include <map>
#include <vector>
#include <cmath>
#include <bitset>
#include "basictypes/safemap.h"
#include "basictypes/flag.h"
namespace ucoslam {

class Frame;
/**Represents a 3d point of the environment
 */
class UCOSLAM_API MapPoint
{
    friend class Map;
 public:
    enum FlagsTypes: uint8_t{FLAG_BAD=0x01,FLAG_STABLE=0x02,FLAG_STEREO=0x04};
    MapPoint();


    inline cv::Point3f getCoordinates(void) const {return pos3d;}

    //returns the set of keyframes observing this 3Dpoint
    //it returns pairs idx:pos, where idx is the keyframe identifier and pos is the
    // position in the keyframe vector. Eg. if the funtion returns{ (3,39),(1,3)}
    // it means that the 3d point is observed in frames 3 and 1. In frame 3, the point is observed
    //in the  keypoint und_kpt[39].pt, and in the frame 1, in und_kpt[3].
    //You could access to these elements as Map->keyframes[3].und_kpts[39] and
    // Map->keyframes[1].und_kpts[3]
    std::vector<std::pair<uint32_t,uint32_t> > getObservingFrames()const;


    //do not use below here

    inline void setCoordinates(const cv::Point3f &p){ pos3d=p;}
    inline cv::Point3f  getNormal(void) const{return normal;}

    inline float getMinDistanceInvariance()const{    return  mfMinDistance;}
    inline float getMaxDistanceInvariance()const{     return  mfMaxDistance;}


    //---------------------
    inline bool isValid()const{return !std::isnan(normal.x);}

    /////////////////////////////////////////////////////////////////////
   // void addKeyFrameObservation(const Frame & frame, uint32_t kptIdx);

    inline void setNormal(cv::Point3f n){normal=n;}
    inline void setSeen() { if(nTimesSeen<std::numeric_limits<uint16_t>::max()) nTimesSeen++;}
    inline void setVisible(  ){  if(nTimesVisible<std::numeric_limits<uint16_t>::max())  nTimesVisible++;}
    inline int geNTimesVisible( )const{     return nTimesVisible;}
    inline float getTimesUnseen()const{ return (nTimesVisible==0)? 0:float(nTimesVisible-nTimesSeen)/float(nTimesVisible);}
    inline float getVisibility()const{ return  (nTimesVisible==0)?0:float(nTimesSeen)/float(nTimesVisible);}

    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str)  ;
    bool operator==(const MapPoint &p)const;
    uint64_t getSignature()const;



    inline float getDescDistance( const cv::Mat &dsc2)const{ return getDescDistance(_desc,dsc2);}
    inline float getDescDistance( const cv::Mat &dsc2,int row)const{ return getDescDistance(_desc,0,dsc2,row);}
    inline float getDescDistance( MapPoint &mp)const{return getDescDistance(_desc,mp._desc);}
    //gets a copy of the descriptor in the passed mat
    inline void getDescriptor( cv::Mat &copy)const { _desc.copyTo(copy); }
    //used to reescale the whole map
    void scalePoint(float scaleFactor);
    //indicates if a frame is observing this point
    inline bool isObservingFrame(uint32_t fidx)const{     return frames.count(fidx)!=0;}
    //returns the number of frames observing this
    inline  uint32_t getNumOfObservingFrames()const{     return frames.size();}



    inline void setMinMaxConfDistance(float mind,float maxd){ mfMinDistance=mind ; mfMaxDistance=maxd;}

    inline float  getViewCos(const cv::Point3f &camCenter)const { auto v= camCenter -  pos3d; v*=1./cv::norm(v); return v.dot(normal);}

    inline void setDescriptor(cv::Mat desc,int index){
        assert(desc.type()==CV_8UC1 || desc.type()==CV_32F);
        int type=desc.type();
        _desc.create(1,desc.cols,type);
        int elemSize=1;
        if(type==CV_32F) elemSize=4;
        memcpy( _desc.ptr<uchar>(0),  desc.ptr<uchar>(index), desc.cols*elemSize);
    }


    uint32_t id=std::numeric_limits<uint32_t>::max();
    cv::Point3f pos3d=cv::Point3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());//rotation and translation of the point. The translation is the 3d location. The rotation refers to the normal of the patch
    SafeMap<uint32_t,uint32_t> frames;// Frames in which the point is projected. Key: frame_idx, Value: index in ids of the keypoint where it projects
    uint64_t kfSinceAddition=0;
    uint32_t lastFIdxSeen=std::numeric_limits<uint32_t>::max();
    inline bool isBad()const{return flags.is(FLAG_BAD);}
    inline bool isStereo()const{return flags.is(FLAG_STEREO);}
    inline bool isStable()const{return flags.is(FLAG_STABLE);}
    inline void setBad(bool v){flags.set(FLAG_BAD,v);}
    inline void setStereo(bool v){flags.set(FLAG_STEREO,v);}
    inline void setStable(bool v){flags.set(FLAG_STABLE,v);}
private:
    Flag flags;
//    bool isStable=false;//indicates if it is an stable point that need no further recheck
//    bool isBad=false;//excludes this point from optimization
//    bool isStereo=false;

    cv::Mat _desc; //Representative descriptor of the map point
    cv::Point3f normal=cv::Point3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN()); //! Mean viewing direction
    uint16_t nTimesSeen=0,nTimesVisible=0;
    float mfMaxDistance=std::numeric_limits<float>::min(),mfMinDistance=std::numeric_limits<float>::max();



public:


   static inline float getHammDescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2){
        assert(dsc1.type()==dsc2.type());
        assert(dsc1.rows==dsc2.rows  && dsc2.rows   == 1);
        assert(dsc1.cols==dsc2.cols);
        assert(dsc1.type()==CV_8UC1);
         return getHammDescDistance_2(dsc1.ptr<uint64_t>(0),dsc2.ptr<uint64_t>(0),dsc1.total());
    }

   static inline float getHammDescDistance_2(const uint64_t *ptr1,const uint64_t *ptr2,uint32_t descSize){
        int n8= descSize/8;
        int hamm=0;
        for(int i=0;i<n8;i++)
            hamm+=std::bitset<64>(ptr1[i] ^ ptr2[i]).count();

        int extra= descSize - n8*8;
        if (extra==0) return hamm;

        const uint8_t *uptr1=(uint8_t*) ptr1+n8;
        const uint8_t *uptr2=(uint8_t*) ptr2+n8;
        for(int i=0;i<extra;i++)
            hamm+=std::bitset<8>(uptr1[i] ^ uptr2[i]).count();

        //finally, the rest
        return hamm;
   }

    //returns the distance between two descriptors
    static inline float getDescDistance(const cv::Mat &dsc1,const cv::Mat &dsc2){
        if (dsc1.type()==CV_8UC1)
            return getHammDescDistance(dsc1,dsc2);
        else if (dsc1.type()==CV_32F) return  cv::norm(dsc1-dsc2);
        throw std::runtime_error("Invalid descriptor");
    }
    //returns the distance between two descriptors
    static inline float getDescDistance(const cv::Mat &descA,int indexA,const cv::Mat &dscB,int indexB){
        if (descA.type()==CV_8UC1)
            return getHammDescDistance_2(descA.ptr<uint64_t>(indexA),dscB.ptr<uint64_t>(indexB),descA.cols);
        else if (descA.type()==CV_32F) return  cv::norm(descA.row(indexA)-dscB.row(indexB));
        throw std::runtime_error("Invalid descriptor");
    }
    //returns the distance between two descriptors
    static inline float getDescDistance2(const void *dsc1,const void *dsc2,uint32_t descSize,int type){
        if (type==CV_8UC1) return getHammDescDistance_2((uint64_t*)dsc1,(uint64_t*)dsc2,descSize);
        else if (type==CV_32F) {
            throw  std::runtime_error("Not yet implemented");
        }
        throw std::runtime_error("Invalid descriptor");
    }
};

}

#endif
