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
#ifndef ucoslam_Frame_H_
#define ucoslam_Frame_H_
#include <cstdint>
#include <limits>
#include <vector>
#include <memory>
#include <map>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "basictypes/picoflann.h"
#include "basictypes/se3transform.h"
#include "basictypes/reusablecontainer.h"
#include "basictypes/flag.h"
#include "imageparams.h"
#include "ucoslamtypes.h"
#include "ucoslam_exports.h"
#include "marker.h"
using namespace  std;
namespace fbow {
struct fBow;
struct fBow2;
}

namespace ucoslam {




//A image frame
class UCOSLAM_API Frame{
    friend class FrameExtractor;
    struct KdTreeKeyPoints{
        inline float operator()(const cv::KeyPoint&kp,int dim)const{
            return dim==0?kp.pt.x:kp.pt.y;
        }
    };
public:

    enum FlagsTypes: uint8_t{FLAG_NONMAXIMA=0x01,FLAG_OUTLIER=0x02,FLAG_BAD=0x04};

    Frame();
    uint32_t idx=std::numeric_limits<uint32_t>::max();//frame identifier
    std::vector<ucoslam::MarkerObservation> markers;//set of orginal markers detected with in the image
    picoflann::KdTreeIndex<2,KdTreeKeyPoints> keypoint_kdtree;
    cv::Mat desc;//set of descriptors
    std::vector<uint32_t> ids;//for each keypoint, the MapPoint it belongs to (std::numeric_limits<uint32_t>::max() is used if not assigned)
    std::vector<Flag> flags;//flag for each keypoint
    Se3Transform pose_f2g;//frame pose  Convenion: Global -> This Frame
    std::vector<cv::KeyPoint> und_kpts;//set of keypoints with removed distortion
    std::vector<cv::Point2f> kpts;//original position of the keypoints in the image with distortion
    cv::Mat image;//grey image(it may not be stored)
    std::shared_ptr<fbow::fBow> bowvector;//bag of words histogram of the frame
    std::shared_ptr<fbow::fBow2> bowvector_level;//bag of words histogram of the frame
    //identifier of the frame in the sequence in which it was captured.
    uint32_t fseq_idx=std::numeric_limits<uint32_t>::max();
    //scale factor of the keypoint detector employed
    vector<float> scaleFactors;
    ImageParams imageParams;//camera with which it was taken
    bool isBad( )const{return frame_flags.is(FLAG_BAD); }
    void setBad(bool v){ frame_flags.set(FLAG_BAD,v);}
    DescriptorTypes::Type KpDescType=DescriptorTypes::DESC_NONE;//keypoint descriptor type (orb,akaze,...)
    //returns a map point observation from the current information
    cv::Point minXY=cv::Point2f(0,0),maxXY=cv::Point2f(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());//projection limits. Affected by image distortion
    float getDepth(int idx)const;//returns the depth of the idx-th keypoint. If no depth, return 0
private:
    Flag frame_flags;
    std::vector<float> depth;//depth if rgbd or stereo camera
public:

    void clear();

    //returns the position of the marker with id indicated if is in the Frame, and -1 otherwise
    int getMarkerIndex(uint32_t id)const  ;
    //returns the marker indicated
    MarkerObservation getMarker(uint32_t id)const  ;

    //returns the MarkerPosesIPPE info on the marker indicated
     MarkerPosesIPPE getMarkerPoseIPPE(uint32_t id)const  ;

    // I/O
    friend ostream& operator<<(ostream& os, const Frame & f)
    {
       os << "Info about the frame:" << f.idx << endl;
       os << "+ Keypoints: " << f.und_kpts.size() << endl;
        return os;
    }

    //given the pose, returns the camera center location in the world reference system
    inline cv::Point3f getCameraCenter()const;
    //returns a normalized vector with the camera viewing direction
    cv::Point3f getCameraDirection()const;

    std::vector<uint32_t> getKeyPointsInRegion(cv::Point2f p, float radius ,  int minScaleLevel=0,int maxScaleLevel=std::numeric_limits<int>::max()) const;


    //computes a number unique with the current configuration
    uint64_t getSignature()const;

    inline bool isValid()const{return ids.size()!=0 || markers.size()!=0;}
    //---------------------
    //serialization routines
    void toStream(std::ostream &str) const ;
    void fromStream(std::istream &str) ;

    //for internal usage only
    void create_kdtree(){
        keypoint_kdtree.build(und_kpts);
        assert(imageParams.CamSize.area()!=0);
    }

    inline int predictScale( float  currentDist, float MaxDistance){
        int mnScaleLevels=scaleFactors.size();
        auto mfLogScaleFactor=log( scaleFactors[1]);
       int nScale = ceil(log(MaxDistance/currentDist)/ mfLogScaleFactor);
       if(nScale<0) return 0;
       if(nScale>= mnScaleLevels) return mnScaleLevels-1;
       return nScale;
    }
    //given the 3d point in global coordinates, projects it in the frame
    //returns a nan,nan point if the point is not in front of camera
    //or if does not project into camera
    inline cv::Point2f project(cv::Point3f p3d,bool setNanIfDepthNegative=true,bool setNanIfDoNotProjectInImage=false)const{
            cv::Point3f res;
            const float *rt=pose_f2g.ptr<float>(0);
            res.z=p3d.x*rt[8]+p3d.y*rt[9]+p3d.z*rt[10]+rt[11];
            if (res.z<0 && setNanIfDepthNegative )
                return cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
            res.x=p3d.x*rt[0]+p3d.y*rt[1]+p3d.z*rt[2]+rt[3];
            res.y=p3d.x*rt[4]+p3d.y*rt[5]+p3d.z*rt[6]+rt[7];
            //now, project
            const float *cam=imageParams.CameraMatrix.ptr<float>(0);
            cv::Point2f r2d;
            res.z=1./res.z;
            r2d.x= ((cam[0]*res.x)*res.z)+cam[2];
            r2d.y= ((cam[4]*res.y)*res.z)+cam[5];
            if ( setNanIfDoNotProjectInImage){
                if (!( r2d.x>=minXY.x && r2d.y>=minXY.y && r2d.x<maxXY.x &&  r2d.y<maxXY.y) )
                    return cv::Point2f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
            }
            return r2d;
    }
    inline cv::Point3f get3dStereoPoint(uint32_t kptIdx)const{
        assert(depth[kptIdx]>0);
        cv::Point3f p;
        p.z=depth[kptIdx];
        p.x= ((und_kpts[kptIdx].pt.x-imageParams.cx())*p.z)/ imageParams.fx();
        p.y= ((und_kpts[kptIdx].pt.y-imageParams.cy())*p.z)/ imageParams.fy();
        return p;
    }
    //returns the scale factor
    inline float getScaleFactor()const{return scaleFactors.size()==0?1:scaleFactors[1];}

    //returns the list of mappoints ids visible from this frame
    vector<uint32_t> getMapPoints()const;

    //set in the vector invalid the elements that do not have a significant response in the neibhorhood
    void nonMaximaSuppresion();

    vector<uint32_t> getIdOfPointsInRegion(cv::Point2f p, float radius);


    //deep copy of the frame
    void copyTo( Frame &f) const;
    Frame & operator=(const Frame &f);

    //free space by removing the unused keypoints. This can be a time consuming process
    void removeUnUsedKeyPoints();

};

cv::Point3f Frame::getCameraCenter()const{
    const float *ptr=pose_f2g.data();
    return cv::Point3f (

     -  ( ptr[3]*ptr[0] +ptr[7]*ptr[4]+ptr[11]*ptr[8]),
     -  ( ptr[3]*ptr[1]+ptr[7]*ptr[5]+ptr[11]*ptr[9]),
     -  ( ptr[3]*ptr[2]+ptr[7]*ptr[6]+ptr[11]*ptr[10]));

 }





//! \class FrameSet
//! \brief A set of image frames
class FrameSet :public  ReusableContainer<ucoslam::Frame> {// std::map<uint32_t,ucoslam::Frame>{
public:
    FrameSet(){ }



    //returns the id of the next frame to be inserted
    uint32_t getNextFrameIndex()const{return getNextPosition();}

    //! Adds new frame to set and returns its idx (if not set, a new one is assigned)
    inline uint32_t addFrame(const ucoslam::Frame & frame){
         auto  inserted=ReusableContainer<ucoslam::Frame>::insert(frame);
         inserted.first->idx=inserted.second;//set the idx to the inserted frame
         return inserted.first->idx;
    }



    // I/O
    friend ostream& operator<<(ostream& os, const FrameSet & fs)
    {
       os << "Info about the frame-set:" << fs.size() << endl;
       for (const auto &f : fs)  os << f ;

       return os;
    }

    void toStream(ostream &str)const;
    void fromStream(istream &str) ;
    uint64_t getSignature()const;

};

}
#endif
