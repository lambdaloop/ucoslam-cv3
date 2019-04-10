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
*/#include "map.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "basictypes/io_utils.h"
#include "basictypes/misc.h"
#include "basictypes/timers.h"
#include "aruco/aruco.h"
#include "optimization/globaloptimizer.h"
#include "basictypes/debug.h"
#include "basictypes/hash.h"
#include "basictypes/fastmat.h"
#include "basictypes/osadapter.h"
namespace ucoslam{

//bool Map::operator==(const Map &m)const{
//    return map_points==m.map_points;
//}
//  Map *Map::ptr=0;

//Map* Map::singleton(){
//    if (ptr==0)   ptr=new Map();
//    return ptr;
//}

float Map::getTargetFocus()const{
    if (keyframes.size()==0)return -1;
    return keyframes.begin()->imageParams.fx();
}

//! \param camPos 3D position of the camera (i.e. translation)
MapPoint & Map::addNewPoint(uint32_t frameSeqId){
    auto it_idx=map_points.insert(MapPoint());
    it_idx.first->id=it_idx.second;
     return map_points[it_idx.second];
}

 Marker &Map::addMarker(const ucoslam::MarkerObservation &m){
     if( !map_markers.is(m.id)) {
         ucoslam::Marker umarker;
         umarker.id=m.id;
         umarker.size=m.ssize;
         umarker.dict_info=m.dict_info;
         map_markers.insert( {m.id,umarker});
     }
     return map_markers[m.id];
 }


 void Map::addMarkerObservation(uint32_t markerId,uint32_t KeyFrame){
     assert(keyframes.is(KeyFrame));
     assert(map_markers.count(markerId));

     Marker &marker= map_markers.at(markerId);//creates if not yet
     assert(marker.frames.count(KeyFrame)==0);//is not yet added

     //add connection with all frames in this point
     for(auto fi:marker.frames)
         TheKpGraph.createIncreaseEdge( fi,KeyFrame,4);
     //add the observation of this frame to the marker
     marker.frames.insert(KeyFrame);
 }

  Frame &Map::addKeyFrame(const Frame&f ){

      __UCOSLAM_ADDTIMER__
     assert(getTargetFocus()<0 || fabs( getTargetFocus()-f.imageParams.fx())<10);//

     if (getTargetFocus()>0)
         if ( fabs( getTargetFocus()-f.imageParams.fx())>10) throw std::runtime_error("Map::addKeyFrame keyframes added should have a similar focus than these already in the dataset");
             //we must be sure that the id is correct (returns the id, and must be the same already used)
     uint32_t kf_id=keyframes.addFrame(f);
     //work with the new frame, and free the pointer
     Frame &newFrame=keyframes[kf_id];
      newFrame.idx=kf_id;
     //add it to the database
     __UCOSLAM_TIMER_EVENT__("addFrame");
     TheKFDataBase.add(newFrame);
     __UCOSLAM_TIMER_EVENT__("Add to database");
     return newFrame;
 }

 // void Map::addKeyFrameObservation(uint32_t mapId, uint32_t frameId, uint32_t frame_kpidx){
 //     addMapPointObservation(mapId,frameId,frame_kpidx);return;
 //     assert(map_points.is(mapId));
 //     assert(keyframes.is(frameId));
 //     assert(keyframes[frameId].ids[frame_kpidx]==std::numeric_limits<uint32_t>::max());
 //     assert( map_points[mapId].frames.count(frameId)==0);//the points should not be already in this frame. Otherwise it would be projected in two different locations
 //     auto observingFrame=    map_points[mapId].getObservingFrames();
 //     map_points[mapId].addKeyFrameObservation(keyframes[frameId],frame_kpidx);
 //     keyframes[frameId].ids[frame_kpidx]=mapId;
 //     //add an edges to others frames
 //     for(auto of:observingFrame)
 //         if (of.first!=frameId)
 //             TheKpGraph.createIncreaseEdge( frameId, of.first);
 // }


 void Map::addMapPointObservation(uint32_t mapId,uint32_t frameId,uint32_t frame_kpidx){
     assert(keyframes.is(frameId));
     assert(map_points.is(mapId));
     MapPoint &mp=map_points[mapId];
     assert(!isnan(mp.getCoordinates().x));
    //set it in keyframe
     assert (keyframes[frameId].ids[frame_kpidx]==mapId || keyframes[frameId].ids[frame_kpidx]==std::numeric_limits<uint32_t>::max());
     //add connection with all frames in this point
     for(auto fi:mp.getObservingFrames())
         TheKpGraph.createIncreaseEdge( fi.first,frameId);

     //register now the observation
     keyframes[frameId].ids[frame_kpidx]=mapId;
     mp.frames.insert({frameId,frame_kpidx});
     //add the observation of this frame to the point
     updatePointInfo(mapId);
 }

 bool Map::removeMapPointObservation(uint32_t pointId,uint32_t frameId,uint32_t minNumProjPoints){
     assert (map_points.is(pointId));
     auto &TheMapPoint=map_points[pointId];
     if (TheMapPoint.isStereo()) minNumProjPoints--;

     if ( TheMapPoint.getNumOfObservingFrames()<=minNumProjPoints){
         removePoint(pointId);
         return true;
     }
     else{//otherwise, remove only its appearance in the frame
         if (keyframes.is(frameId)){
             keyframes[frameId].ids[ TheMapPoint.frames.at(frameId) ]=std::numeric_limits<uint32_t>::max();
             TheMapPoint.frames.erase(frameId);
             //need to update the covisgraph
             for(auto f_i:TheMapPoint.frames)
                 TheKpGraph.decreaseRemoveEdge(f_i.first,frameId);
             updatePointInfo(pointId);
         }
     }
     return false;
 }




// uint32_t MapPoint::removePointObservation(uint32_t point,uint32_t frame,int32_t minNumProj=0 ){
//     assert(map_points.is[point]);
//     assert(map_points.frames.count(frame));


// }


 void Map::removePoint(uint32_t pid_remove, bool fullRemoval){
     assert(map_points .is(pid_remove));

     auto  &MP=map_points [pid_remove];
     MP.setBad (true);
     vector<uint32_t> frames;frames.reserve(MP.frames.size());
     for(auto f_id:MP.frames){//remove each frame connection
         frames.push_back(f_id.first);
         if ( keyframes[f_id.first].ids[f_id.second]==pid_remove)
             keyframes[f_id.first].ids[f_id.second]=std::numeric_limits<uint32_t>::max();
     }
     if (fullRemoval){
         //now, remove all frame connections
         for(size_t i=0;i<frames.size();i++)
             for(size_t j=i+1;j<frames.size();j++)
                 TheKpGraph.decreaseRemoveEdge(frames[i],frames[j]);
         //finally, remove the point
         map_points.erase(pid_remove);
     }

 }

 void Map::removeKeyFrames(const std::set<uint32_t> &keyFrames,int minNumProjPoints){
     for(auto fidx:keyFrames){
         const auto &frame=keyframes[fidx];
         //first, remove from databse
         TheKFDataBase.del(frame);
         //remove its reference in the map points
         for(auto ids: frame.ids)
             if (ids!=numeric_limits<uint32_t>::max())
                 removeMapPointObservation(ids,fidx,minNumProjPoints);

         //remove its reference in the markers
         for(auto marker:frame.markers)
             map_markers[marker.id].frames.erase(fidx);
         //finally, remove
         keyframes.erase(fidx);
         //also from covis graph
         TheKpGraph.removeNode(fidx);
         _debug_msg_("Removing Frame "<<fidx);
     }
 }

 float Map::getFrameMedianDepth(uint32_t frame_idx){

     assert(keyframes.count(frame_idx)!=0);
     const auto &frame=keyframes[frame_idx];
     //must transform the points to the camera
     //compute the average depth of the frame and check the ratio baseline/depth

     vector<float> depths;
     depths.reserve(frame.ids.size());
      for(auto p:frame.ids){
         if ( p!=std::numeric_limits<uint32_t>::max()){
             auto &mp=map_points[p];
             auto p=  frame.pose_f2g*mp.getCoordinates();
             depths.push_back( p.z);
         }
     }

     //now the marker points
     for(auto m:frame.markers){
         assert(  map_markers.count(m.id) );
         if (  map_markers[m.id].pose_g2m.isValid() ){
             auto p3d=map_markers[m.id].get3DPoints();
             for(auto p:p3d) {
                 p=frame.pose_f2g*p;
                 depths.push_back(p.z);
             }
         }
     }
     std::sort(depths.begin(),depths.end());
    return depths[depths.size()/2];

//     auto outliers=outlierFiltering(depths,3);
//     //compute the mean without the outliers
//     removeFromVector(depths,outliers);
//     float  mean =0;
//     for(auto d:depths) mean+=d;

//     return  mean/float(depths.size());
 }
void Map::clear(){
    map_points.clear();
    map_markers.clear();
    keyframes.clear();
    TheKpGraph.clear();
    TheKFDataBase.clear();
}

uint32_t Map::getNextFrameIndex()const{return keyframes.getNextPosition();}

bool Map::hasPointStereoObservations(uint32_t mpIdx)const{
    for(const auto &frame:map_points.at(mpIdx).frames){
        if ( keyframes[frame.first].getDepth(frame.second)!=0)return true;
    }
    return false;
}

void Map::fuseMapPoints(uint32_t mp1,uint32_t mp2 ,bool fullRemovePoint2){
    //save observations
    auto &Mp2=map_points[mp2];
    auto p2_observations=Mp2 .frames;
    auto &Mp1=map_points[mp1];
    //remove the point
    removePoint(mp2,fullRemovePoint2);
    //add observations to the point that remains
    for(auto p2_obs:p2_observations){
        if(! Mp1.frames.count(p2_obs.first))//the points should not be already in this frame. Otherwise it would be projected in two different locations
            addMapPointObservation(mp1,p2_obs.first,p2_obs.second);
    }
 }

void Map::applyTransform(cv::Mat m4x4){
    if(!(m4x4.cols==m4x4.rows && m4x4.cols==4))
        throw std::runtime_error(string(__PRETTY_FUNCTION__)+" invlida inut  matrix. Must be homogeneous 4x4 matrix");
    cv::Mat m44_32f,m44_32f_rot;
    m4x4.convertTo(m44_32f,CV_32F);
    m4x4.convertTo(m44_32f_rot,CV_32F);
    m44_32f_rot.at<float>(0,2)=0;
    m44_32f_rot.at<float>(1,2)=0;
    m44_32f_rot.at<float>(2,2)=0;


    for(auto &m:map_markers)
        m.second.pose_g2m =   m44_32f*  m.second.pose_g2m;

    for(auto &p:map_points){
         p.pos3d =   m44_32f*  p.pos3d;
        p.normal= m44_32f_rot*p.normal;
    }

    for(auto &kf:keyframes)
         kf.pose_f2g= (m44_32f*kf.pose_f2g.inv()).inv() ;


}
bool Map::centerRefSystemInMarker(uint32_t markerId){


    if (!map_markers.is(markerId)) return false;
    auto &marker=map_markers[markerId];
    if(!marker.pose_g2m.isValid()) return false;

    applyTransform(marker.pose_g2m.inv());
    return false;
}




void Map::toStream(iostream &str)  {
    lock(__FUNCTION__,__FILE__,__LINE__);
    TheKFDataBase.toStream(str);

    map_points.toStream(str);
    toStream__kv_complex(map_markers,str);
    keyframes.toStream(str);
    TheKpGraph.toStream(str);
    unlock(__FUNCTION__,__FILE__,__LINE__);
}

void Map::fromStream(std::istream &str){
    lock(__FUNCTION__,__FILE__,__LINE__);
    clear();

    TheKFDataBase.fromStream(str);
    map_points.fromStream(str);
    fromStream__kv_complexmap(map_markers,str);
    keyframes.fromStream(str);
    TheKpGraph.fromStream(str);
    unlock(__FUNCTION__,__FILE__,__LINE__);
}

void Map::saveToFile(std::string   fp)  {
    std::fstream file(fp, ios::out|ios::binary);
    if (!file) throw std::runtime_error("Could not open file for writing:"+fp);
    uint64_t sig=225237123;//magic number
    file.write((char*)&sig,sizeof(sig));
    toStream(file);
}

void Map::readFromFile(std::string fp){
    std::ifstream file(fp,std::ios::binary);
    if (!file) throw std::runtime_error("Could not open file for reading:"+fp);
    uint64_t sig=225237123;
    file.read((char*)&sig,sizeof(sig));
    if (sig!=225237123)throw std::runtime_error("Map::readFromFile   is not of approrpiate type");
    fromStream(file);
}
uint64_t Map::getSignature(bool print)const{
    Hash sig;
    for(const auto &p:map_points){
        sig+=p.getSignature();
    }
    if(print)cout<<"     Map sig 1. ="<<sig<< " "<<map_points.size()<<endl;
     sig+=keyframes.getSignature();
     if(print)cout<<"     Map sig 2. ="<<sig<<endl;
     for(const auto &m:map_markers){
         sig+=m.first;
         sig+=m.second.getSignature(print);
     }
     if(print) cout<<"     Map sig 3. ="<<sig<<endl;
     sig+=TheKpGraph.getSignature();
     if(print)cout<<"     Map sig 4. ="<<sig<<endl;
     sig+=TheKFDataBase.getSignature();
     if(print)cout<<"     Map sig 5. ="<<sig<<endl;
     return sig;

}

bool Map::checkConsistency(bool checkCovisGraph,bool useMutex){
      std::unique_lock<std::mutex> lockconsistency(consitencyMutex);
    bool consistent=true;
//    if ( _curKFRef!=-1){
//        if (! keyframes.is(_curKFRef)){
//            cerr<<" _curKFRef is invalid"<<endl;
//            consistent=false;
//        }
//    }

    if ( keyframes.size()==1)return true;
 if(useMutex)   lock(__FUNCTION__,__FILE__,__LINE__);

//check the number of elements in the graph is the same as the number of frames
//    if ((TheKpGraph.size()+TheMarkerGraph.size()) >=keyframes.size() ){
//        consistent=false;
//        cerr<<" TheCVGraph.size()+TheMarkerGraph.size()!=keyframes.size()"<<endl;
//    }
    //check all are in
    for(auto f:keyframes)
        if (!TheKpGraph.isNode(f.idx) && !TheKpGraph.isNode(f.idx) ){
            consistent=false;
            cerr<<" Frame "<<f.idx<<" not in any of the Graphs"<<endl;
        }

    //check points are valid
    for(const auto & p:map_points){
        if (! p.isValid()){
            cerr<<"Point << "<<p.id<<" invalid"<<endl;
            consistent=false;
        }
    }

    //check that points-frames are consistent
     for(const auto & p:map_points){
        if (p.isBad())continue;
        //for each frame it projects in
        for(const auto &f:p.frames){
            if (!keyframes.is(f.first)){
                cerr<<"x1 Missing frame :"<<f.first<<endl;
                consistent=false;
            }
          if ( (keyframes)[f.first].ids[f.second]!=p.id){
               consistent=false;
               cerr<<"point "<<p.id<<" is suposed to be in frame "<<f.first<<endl;
            }
        }
    }

    //check consistency of point-frames the other way around
    for(auto &f:keyframes){
        for(auto id:f.ids){
            if (id!= std::numeric_limits<uint32_t>::max()){
                if ( !map_points.is(id)) {
                    consistent=false;
                    cerr<<"Point "<<id<<" is lost but referenced in frmae "<<f.idx<<endl;
                }
                //check that the point has this reference
                if ( !map_points[id].frames.count(f.idx)){
                    cerr<<"Point "<<id<<" should reference to frame "<<f.idx<<endl;
                    consistent=false;
                }
            }
        }
    }

    //everypoint must have at least two projections
        for(const auto &p:map_points){
            bool isStereo=false;
            for(auto f:p.frames)
                if (keyframes[f.first].getDepth (f.second)!=0) isStereo|=true;
            if (p.frames.size()<=1 && !p.isStereo()){
                cerr<<"Point "<<p.id<<" should have at least two projections "<<endl;
                consistent=false;
            }
        }


        for(const auto &p:map_points){
            if (p.frames.size()==0){
                cerr<<"Point "<<p.id<<" should have at least one projection "<<endl;
                consistent=false;
            }
        }

    //////MARKERS
    //check that marker-frames are consistent
    for(const auto & m:map_markers){
        if (m.second.frames.size()==0){
            cerr<<"Marker "<<m.first<<" not assigned to any frame"<<endl;
            consistent=false;
        }
        //for each frame it projects in
        for(auto f:m.second.frames){
            const auto &frame=(keyframes)[f];
            //find the marker in the frame vector(slow)
            int idx=-1;
            for(size_t i=0;i<frame.markers.size();i++)
                if ( frame.markers[i].id==m.first) {idx=i;break;}
            if (idx==-1){consistent=false;cerr<<"Marker "<<m.first<<" is suposed to be in frame "<<f<<endl;
            }
        }
    }

    //check consistency of marker-frames the other way around
    for(auto &f:keyframes){
        for(const auto &m:f.markers){
                if ( !map_markers.count(m.id)) {
                    consistent=false;
                    cerr<<"Marker "<<m.id<<" is lost but referenced in frmae "<<f.idx<<endl;
                }
                //check that the point has this reference
                if ( !map_markers[m.id].frames.count(f.idx)){
                    cerr<<"Marker "<<m.id<<" should reference to frame "<<f.idx;
                    consistent=false;
                }
        }
    }

    //check that all markers detected are in the map, even if without valid position
    {
     for(const auto &frame:keyframes)
        for(auto m:frame.markers)
            if ( map_markers.count(m.id)==0){
                cerr<<"Marker "<<m.id<<" should be registered in the map"<<endl;
                consistent=false;
            }
    }

    //ckeck the covisibility graph of markers is consistent
   if (0) {
        //to do so, create the covis graph here
        std::map<uint64_t,zeroinitvar<uint32_t> > frame_commonmakers;
        for(const auto &marker_:map_markers){
            vector<uint32_t> vframes;
            for(auto ob:marker_.second.frames)vframes.push_back(ob);
                for(size_t i=0;i<vframes.size();i++)
                    for(size_t j=i+1;j<vframes.size();j++)
                        frame_commonmakers[CovisGraph::join(vframes[i],vframes[j])]++;
        }
        //check that the value is the same as in the covisgraph
        for(auto fij:frame_commonmakers){
            auto i_j=CovisGraph::separe(fij.first);
            if (!TheKpGraph.isEdge(i_j.first,i_j.second)){
                cerr<<"Marker graph edege "<<i_j.first<<"-"<<i_j.second<<" is not in the graph "<<endl;
                consistent=false;
            }
            if ( TheKpGraph.getEdge(i_j.first,i_j.second)!= fij.second){
                cerr<<"Marker graph edege "<<i_j.first<<"-"<<i_j.second<<" has weight "<<TheKpGraph.getEdge(i_j.first,i_j.second)<<" but should be "<< fij.second<<endl;
                consistent=false;
            }
        }

        for(auto fij:TheKpGraph.getEdgeWeightMap()){
            if (frame_commonmakers.count(fij.first)==0){
                auto i_j=CovisGraph::separe(fij.first);
                cerr<<"Marker graph edge "<<i_j.first<<"-"<<i_j.second<<" should not be"<<endl;
                consistent=false;

            }
        }

    }


//    //check that if a marker has a valid position, there is at least two views to the marker(ie, they have not been removed)
//    for(const auto &m:map_markers){
//        if (m.second.frames.size()<2 && m.second.pose_g2m.isValid()){
//            cerr<<"The valid marker "<<m.first<<" should be visible in at least two views"<<endl;
//            consistent=false;
//        }
//    }

    //check no nan point
    for(  auto &p:map_points)
    {
        auto p3d=p.getCoordinates();
        if (std::isnan(p3d.x)||std::isnan(p3d.y)||std::isnan(p3d.z)){
            consistent=false;
            cerr<<"point "<<p.id<<" is nan"<<endl;
        }
    }

    //all frames in KFDatabase and no other
    {
        if (!TheKFDataBase.isEmpty()){//it is valid
            int nframes_seen=0;
            for(auto f:keyframes){
                if (!TheKFDataBase.isId(f.idx)){
                    consistent=false;
                    cerr<<"frame :"<<f.idx<<" not found in the database"<<endl;
                }
                else nframes_seen++;
            }
            //now, check that there is the same number of elements in bt
            if (TheKFDataBase.size()!=size_t(nframes_seen)){
                consistent=false;
                cerr<<"The dabase contains frames not in the TheFrameSet"<<endl;
            }
        }
    }
    //check that matches are correct

//    for(auto m:map_matches){
////        if (map_points.is(m.trainIdx)){
////            cout<<"Match to map point "<<m.trainIdx<< " is incorrect"<<endl;
////            consistent=false;
////        }
//        if (m.queryIdx>= _cFrame.ids.size()){
//            cout<<"match query out of index"<<endl;
//            consistent=false;
//        }
//    }

    //covis graph
    if (checkCovisGraph){
         CovisGraph auxCV;
        for(const auto &p:map_points){
            vector<uint32_t> frames;
            for(auto f:p.frames)frames.push_back(f.first);
            for(size_t i=0;i<frames.size();i++)
                for(size_t j=i+1;j<frames.size();j++){
                    auxCV.createIncreaseEdge(frames[i],frames[j]);
                }
        }
        //now, for the markers
        for(auto m:map_markers){
            vector<uint32_t> frames;
            for(auto f:m.second.frames)frames.push_back(f);
            for(size_t i=0;i<frames.size();i++)
                for(size_t j=i+1;j<frames.size();j++){
                    auxCV.createIncreaseEdge(frames[i],frames[j],4);
                }
        }

        //now compare
        auto n1= auxCV.getNodes( );
        auto n2= TheKpGraph.getNodes( );

        //check are equal
        if (n1!=n2){
            cerr<<"Covisgraph missing nodes (y)"<<endl;
            consistent=false;
        }
        //now, check the neighbors of each

        for(auto n:n1){
            auto neigh1=auxCV.getNeighbors(n);
            auto neigh2=TheKpGraph.getNeighbors(n);
            if (neigh1!=neigh2){
                cerr<<"Neigbors of "<<n<<" are incorrect"<< endl;
                consistent=false;
            }
            //now, check weights
            for(auto ne1:neigh1)
                if (auxCV.getEdge(n,ne1)!=TheKpGraph.getEdge(n,ne1) ){
                    cerr<<"Edge of nodes "<<n<<","<<ne1<< " is incorrect. It is:"<<TheKpGraph.getEdge(n,ne1) <<" and shoudl be :"<<auxCV.getEdge(n,ne1) <<endl;
                    consistent=false;

                }
        }
    }



    if(useMutex)  unlock(__FUNCTION__,__FILE__,__LINE__);

    _debug_msg_("Consistency CHECK PASSED="<<consistent);
    return consistent;
}
vector<uint32_t> Map::relocalizationCandidates(Frame &frame,const std::set<uint32_t> &excludedFrames){
    return TheKFDataBase.relocalizationCandidates(frame,keyframes,TheKpGraph,true,0,excludedFrames);
}


std::vector<cv::DMatch> Map::matchFrameToMapPoints( const std::vector<uint32_t > &  used_frames, Frame & curframe, const cv::Mat & pose_f2g_, float minDescDist, float maxRepjDist,bool markMapPointsAsVisible,bool useAllPoints,std::set<uint32_t> excludedPoints)
{

    Se3Transform pose_f2g;pose_f2g=pose_f2g_;
    __UCOSLAM_ADDTIMER__
    //remove this already seen that have been marked with the lastSeqFrameIdx in the map point
    std::vector<uint32_t>  smap_ids=getMapPointsInFrames(used_frames.begin(),used_frames.end(),excludedPoints);
    __UCOSLAM_TIMER_EVENT__("step1");
    if (!useAllPoints){
        for(size_t i=0;i< smap_ids.size();i++){
            if ( map_points.is ( smap_ids[i]))
                if ( map_points[ smap_ids[i]].lastFIdxSeen==curframe.fseq_idx)//already seen and projected
  //              if ( map_points[ smap_ids_all[i]].lastTimeSeenFSEQID==curframe.fseq_idx)//already seen and projected
                    smap_ids[i]= std::numeric_limits<uint32_t>::max();
        }
        smap_ids.erase(std::remove(smap_ids.begin(), smap_ids.end(),  std::numeric_limits<uint32_t>::max()),smap_ids.end());
    }

    __UCOSLAM_TIMER_EVENT__("step2");
    if (smap_ids.size()==0) return {};

    //Condition 2. Compute angle between current viewing ray and the map point mean viewing direction
    //             Discard if v.dot(n) < cos(60)

    cv::Point3f camCenter= pose_f2g.inv()*cv::Point3f(0,0,0);  //obtain camera center in global reference system
    // Project points given pose
    __UCOSLAM_TIMER_EVENT__("step3");


    std::vector<cv::DMatch> map_matchesLocalMap; // [query=kpts; train=map]
    map_matchesLocalMap.reserve(smap_ids.size());

    float fx=curframe.imageParams.CameraMatrix.at<float>(0,0);
    float fy=curframe.imageParams.CameraMatrix.at<float>(1,1);
    float cx=curframe.imageParams.CameraMatrix.at<float>(0,2);
    float cy=curframe.imageParams.CameraMatrix.at<float>(1,2);

    // Find matches map-points keypoints
    for (uint32_t mpix = 0; mpix < smap_ids.size(); mpix++)
    {

        MapPoint &mapPoint=map_points[ smap_ids[mpix] ];
        float viewCos=mapPoint.getViewCos(camCenter);
        if (viewCos<0.5) continue;
        //move point to camera coordinates
        cv::Point3f p3d=pose_f2g* mapPoint.getCoordinates();
        if (p3d.z<0)//only consider if in front of camera
            continue;
        //only consider if the keypoint is into margin limits
        //compute distance to point
        float distToPoint= cv::norm(p3d);
        if (   !(0.8f*mapPoint.getMinDistanceInvariance() < distToPoint && distToPoint< 1.2f*mapPoint.getMaxDistanceInvariance()))
            continue;
        p3d.z=1./p3d.z;
        cv::Point2f p2d(  p3d.x*fx*p3d.z+cx,p3d.y*fy*p3d.z+cy);

        if ( !(p2d.x>curframe.minXY.x  &&p2d.y>curframe.minXY.y  &&p2d.x<curframe.maxXY.x &&p2d.y<curframe.maxXY.y))
            continue;
        //mark as visible
        if (markMapPointsAsVisible)
            mapPoint.setVisible( );

        // ----------------------------------------------------------
        //now, analyze these and find the one with minimal descriptor distance
        int best_candidate_kp=-1,bestLevel=0,bestLevel2=-1;
        //now, predict the octave
        int predictedOctave=curframe.predictScale(distToPoint,mapPoint.getMaxDistanceInvariance() );//scaleFactorLog,curframe.scaleFactors.size());

        float radius_scale=curframe.scaleFactors[predictedOctave];

        if(viewCos<0.98) radius_scale*=1.6;

        float best_candidate_distance=std::numeric_limits<float>::max(),bestdistance2=std::numeric_limits<float>::max();

        // Find keypoints closer (spatial distance) to this map point
        vector<uint32_t> keyp_region=curframe.getKeyPointsInRegion(p2d,radius_scale*maxRepjDist,predictedOctave-1,predictedOctave);

#pragma message "todo: check disparity to discard outliers"
        for (auto kp_idx : keyp_region )
        {
                //compute visual distance to map point
                float desc_dist= mapPoint.getDescDistance(curframe.desc,kp_idx);
                if (desc_dist<minDescDist){
                    if( desc_dist<best_candidate_distance){
                        best_candidate_distance=desc_dist;
                        best_candidate_kp=kp_idx;
                        bestLevel=curframe.und_kpts[kp_idx ].octave;
                    }
                    else if (desc_dist<bestdistance2){
                        bestdistance2=desc_dist;
                        bestLevel2=curframe.und_kpts[kp_idx].octave;
                    }

                }
        }


        if (best_candidate_kp!=-1){
            bool valid=true;
            if ( bestLevel2==bestLevel && best_candidate_distance> 0.8*bestdistance2)  valid=false;
            if (valid){
                cv::DMatch mt;
                mt.queryIdx = best_candidate_kp; // kp
                mt.trainIdx = smap_ids[mpix];                       // map
                mt.distance = best_candidate_distance;
                map_matchesLocalMap.push_back(mt);
            }
        }
    }
    __UCOSLAM_TIMER_EVENT__("step4");



    _debug_msg("final number of matches= "<<map_matchesLocalMap.size(),10);
    filter_ambiguous_query(map_matchesLocalMap);


    _debug_msg("final number of matches2= "<<map_matchesLocalMap.size(),10);
    return map_matchesLocalMap;
}

double Map::globalReprojChi2( const std::vector<uint32_t> &used_frames , std::vector<float > *chi2vector,
                               std::vector<std::pair<uint32_t,uint32_t> > *map_frame_ids,bool useKeyPoints,bool useMarkers) {

    double chi2Sum=0;
    int np=0;
    if (chi2vector!=0) {
        chi2vector->clear();
        chi2vector->reserve(map_points.size());
    }
    if(map_frame_ids!=0){
        map_frame_ids->clear();
        map_frame_ids->reserve(map_points.size());
    }

    //set all used frames info
    vector<bool> usedFramesVector;
    if ( used_frames.size()==0) usedFramesVector=std::vector<bool>(keyframes.capacity(),true);
    else {
        usedFramesVector=std::vector<bool>(keyframes.capacity(),false);
        for(auto f:used_frames) usedFramesVector[f]=true;
    }

    //set in the frames the projection matrix
    if (useKeyPoints){
    for(  auto & p:map_points){
        cv::Point3f p3d=p.getCoordinates();
        //for each frame it projects in
        for(auto f:p.frames){
            if (usedFramesVector[f.first]){
                const Frame &frame=keyframes[f.first];
                cv::Point2f p2= frame.project(p3d,true,false);
                if ( !isnan(p2.x)){
                        cv::Point2f kp= frame.und_kpts[f.second].pt;
                        auto ee=p2-kp;
                        float inv_scale_factor=1./frame.scaleFactors[frame.und_kpts[f.second].octave];
                        float chi2=inv_scale_factor *( ee.x*ee.x+ee.y*ee.y);
                        chi2Sum+=chi2;
                        if (chi2vector) chi2vector-> push_back(chi2  );
                        if(map_frame_ids)map_frame_ids->push_back({p.id,f.first});
                        np++;
                }
            }
        }
    }
    }

    //do the same for markers
    if(useMarkers){
        for(const auto & m:map_markers){
            if ( m.second.pose_g2m.isValid()){
                for(auto f:m.second.frames){
                    if (usedFramesVector[f]){
                    auto points2d=keyframes[f].getMarker(m.second.id).und_corners;
                    auto points3d=m.second.get3DPoints();
                    for(int i=0;i<4;i++){
                        auto p2= keyframes[f].project(points3d[i],true);
                        if ( !isnan(p2.x)){
                            auto ee=p2-points2d[i];
                            chi2Sum+=ee.x*ee.x;
                            chi2Sum+=ee.y*ee.y;
                            np++;
                        }
                    }
                }
                }
            }
        }
    }
    return chi2Sum;
}
void Map::removeBadAssociations(const vector<std::pair<uint32_t,uint32_t>> &BadAssociations,int minNumPtObs){
     for(auto badAssoc:BadAssociations){
        if (map_points.is(badAssoc.first))
        removeMapPointObservation(badAssoc.first,badAssoc.second,minNumPtObs);
    }
}

void Map::updatePointNormalAndDistances(uint32_t pid){
    struct bestObs{
        uint32_t frame=std::numeric_limits<uint32_t>::max();
        int octave=std::numeric_limits<int>::max();
        uint32_t kpIdx=std::numeric_limits<uint32_t>::max();
    } bOb;
    assert(map_points.is(pid));
    MapPoint &mp=map_points[pid];
    assert(mp.frames.size()>0);

    //compute the new normal of the point and the best observation(lowest octave)
    auto p3d=mp.getCoordinates();
    cv::Point3f avrgNormal(0,0,0);
    double normSum=0;
     for(auto f:mp.frames){
        const Frame &frame=keyframes[f.first];
        cv::Point3f normal=frame.getCameraCenter()-p3d;
        avrgNormal+=normal;
        normSum+=cv::norm(normal);

        if(bOb.octave>frame.und_kpts[f.second].octave){
            bOb.octave=frame.und_kpts[f.second].octave;
            bOb.frame=f.first;
            bOb.kpIdx=f.second;
        }
    }

     avrgNormal*=1./normSum;
     mp.setNormal(avrgNormal);


     //compute the min and max invariance distance of the keypoint
     {
         auto &BestObsFrame=keyframes[bOb.frame];
         const float dist = cv::norm(mp.getCoordinates() - BestObsFrame.getCameraCenter());
         const float levelScaleFactor =  BestObsFrame.scaleFactors[BestObsFrame.und_kpts[bOb.kpIdx].octave];
         float mfMaxDistance = dist*levelScaleFactor;
       //int maxOct=  std::min(BestObsFrame.und_kpts[bOb.kpIdx].octave+3,int(BestObsFrame.scaleFactors.size()-1));
         float mfMinDistance =( mfMaxDistance/BestObsFrame.scaleFactors.back());
         mp.setMinMaxConfDistance(mfMinDistance,mfMaxDistance);
     }

}
void Map::updatePointInfo(uint32_t pid){
    assert(map_points.is(pid));
    updatePointNormalAndDistances(pid);
    MapPoint &mp=map_points[pid];
    auto obsFrames=map_points[pid].getObservingFrames();
    assert(obsFrames.size()>0);
    //now, compute the best descriptor

     //save distances between all observed ones
    FastMat<float> fm_desc_dist(obsFrames.size(),obsFrames.size());//oversized matrix
    fm_desc_dist.setTo(std::numeric_limits<float>::max());

    for(size_t i=0;i<obsFrames.size();i++){
        fm_desc_dist(i,i)=0;
         for(size_t j=i+1;j<obsFrames.size();j++){
            fm_desc_dist(j,i)=fm_desc_dist(i,j)=MapPoint::getDescDistance(keyframes[obsFrames[i].first].desc,obsFrames[i].second ,keyframes[obsFrames[j].first].desc,(obsFrames[j].second) );
        }
    }


    //accumulate values and get best
    pair<int,float> best(-1,std::numeric_limits<float>::max());
    for(size_t r=0;r<obsFrames.size();r++){
        float sum=0;
        float *ptr= fm_desc_dist.ptr(r);
        for(size_t c=0;c<obsFrames.size();c++) sum+=ptr[c];

        if (sum<best.second)
            best={r,sum};
    }


    auto bestObsDescFrame=obsFrames[best.first];

    mp.setDescriptor( keyframes[bestObsDescFrame.first].desc,bestObsDescFrame.second);



}



void  Map::exportToFile(std::string filepath,  const cv::Scalar &colorPoints,const cv::Scalar &colorKeyFrames,const cv::Scalar &colorMarkers, const set<uint32_t> &specialKeyFrames,cv::Scalar colorSpecialKeyFrame)const{

    auto float2Uchar=[](float f,int index){
        uchar *uc=(uchar*)&f;
        return (int)uc[index];
    };

    auto getExtension=[](std::string str){
        string ext;
        if(str.size()<=3)return ext;
        for(int i=str.size()-3;i<str.size();i++)
            ext.push_back(str[i]);
        return ext;
    };
    string ext=getExtension(filepath);
    if(ext!="pcd" && ext!="ply")  throw std::runtime_error(" Map::exportToFile Invalid extension: "+ext);
    auto points=getMapPoints4Export (   colorPoints,  colorKeyFrames, colorMarkers, specialKeyFrames,  colorSpecialKeyFrame);
    std::ofstream filePCD ( filepath, std::ios::binary );


    if(ext=="pcd"){
        filePCD<<"# .PCD v.7 - Point Cloud Data file format\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nVIEWPOINT 0 0 0 1 0 0 0\nWIDTH "<<points.size()<<"\nHEIGHT 1\nPOINTS "<<points.size()<<"\nDATA binary\n";
        filePCD.write((char*)&points[0],points.size()*sizeof(points[0]));
    }
    else if(ext=="ply"){
        filePCD<<"ply"<<endl;
        filePCD<<"format binary_little_endian 1.0"<<endl;// format binary_little_endian 1.0
        filePCD<<"element vertex "<<points.size()<<endl;
        filePCD<<"property float x"<<endl;
        filePCD<<"property float y"<<endl;
        filePCD<<"property float z"<<endl;
        filePCD<<"property uchar diffuse_red"<<endl;
        filePCD<<"property uchar diffuse_green"<<endl;
        filePCD<<"property uchar diffuse_blue"<<endl;
        filePCD<<"end_header"<<endl;
        for(auto &p:points){
            filePCD.write((char*)&p[0],15);
//            filePCD<<p[0]<<" "<<p[1]<<" "<<p[2]<<" "<<float2Uchar(p[3],0)<<" "<<float2Uchar(p[3],1)<<" "<<float2Uchar(p[3],2)<<endl;
        }

    }
}



std::vector<cv::Vec4f> Map::getMapPoints4Export(  cv::Scalar colorPoints, cv::Scalar colorKeyFrames,cv::Scalar colorMarkers, const set<uint32_t> & specialKeyFrame, cv::Scalar colorSpecialKeyFrame)const{
    auto color2float=[](cv::Scalar color){
        float fcolor;uchar *c=(uchar*)&fcolor;
        for(int i=0;i<3;i++)c[i]=color[i];
        c[3]=0;
        return fcolor;
    };
    std::vector<cv::Vec4f> points2write;
    //markers
    float markerSize=-1;
    if(map_markers.size()!=0)
        markerSize=map_markers.begin()->second.size;

    if(colorMarkers[0]!=-1){
        for(const auto &m:map_markers)
        {
            markerSize=m.second.size;
            assert(m.second.size>0);
            if (m.second.pose_g2m.isValid()){
                auto points4=getPcdPoints(m.second.get3DPoints(),colorMarkers);
                points2write.insert(points2write.end(),points4.begin(),points4.end());
                auto points_id=getMarkerIdPcd(m.second.pose_g2m,m.second.size,m.second.id,colorMarkers);
                points2write.insert(points2write.end(),points_id.begin(),points_id.end());
                //max_msize=std::max(max_msize,m.second.markerSize);
            }
        }

    }

    if(colorKeyFrames[0]!=-1){
        vector<cv::Point3f> marker_points = { cv::Point3f ( -markerSize/2., markerSize/2.,0 ),cv::Point3f ( markerSize/2., markerSize /2.,0 ),
                                              cv::Point3f ( markerSize/2., -markerSize/2.,0 ),cv::Point3f ( -markerSize/2., -markerSize/2.,0 )  };
        for(auto frame:keyframes){
            cv::Mat f2c=frame.pose_f2g.inv();
            vector<cv::Point3f> mpoints(4);
            for(int i=0;i<4;i++)  mpoints[i]=mult(f2c, marker_points[i]);
            //  for(auto p:mpoints)cout<<p<<endl;

            cv::Scalar kfColor=colorKeyFrames;
            if( specialKeyFrame.count(frame.idx)!=0)  kfColor=colorSpecialKeyFrame;

            auto pcam=getPcdPoints( mpoints,kfColor,25);
            points2write.insert(points2write.end(),pcam.begin(),pcam.end());
            auto points_id=getMarkerIdPcd(frame.pose_f2g.inv(), cv::norm(marker_points[0]-marker_points[1]),frame.idx,kfColor);
            points2write.insert(points2write.end(),points_id.begin(),points_id.end());
        }
    }
    //3d points
    if(colorPoints[0]!=-1)   {
        for(const auto &p:map_points){
            auto color=colorPoints;
            for(auto of:specialKeyFrame){
                if( p.frames.count(of)!=0 ){
                    color=colorSpecialKeyFrame;
                    break;
                }
            }
            cv::Point3f p3d=p.getCoordinates();
            points2write.push_back(cv::Vec4f(p3d.x,p3d.y,p3d.z,color2float(color)));
        }
    }

    //add the viewing direction
    float pointViewDirectionDist=0;
    if (pointViewDirectionDist>0){
        for(  auto  p:map_points){
            auto v=p.getNormal();
            auto pc=p.getCoordinates();
            auto pviewdir=getPcdPointsLine(pc,pc+(v*pointViewDirectionDist),cv::Scalar  (255,125,125),25);
            points2write.insert(points2write.end(),pviewdir.begin(),pviewdir.end());

        }
    }
    return points2write;

}

std::vector<cv::Vec4f> Map::getPcdPointsLine(const cv::Point3f &a,const cv::Point3f &b,cv::Scalar color,int npoints ) const{
    vector<cv::Vec4f> points;
    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    //lines joining points
    cv::Point3f v=b-a;
    float ax=1./float(npoints);//npoints
    for(float x=0;x<=1;x+=ax){
        cv::Point3f p3=a+v*x;
        points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
    }
    return points;
}

std::vector<cv::Vec4f> Map::getPcdPoints(const vector<cv::Point3f> &mpoints,cv::Scalar color,int npoints ) const{
   vector<cv::Vec4f> points;
   double msize=cv::norm(mpoints[0]-mpoints[1]);
   float fcolor;uchar *c=(uchar*)&fcolor;
   for(int i=0;i<3;i++)c[i]=color[i];

   //lines joining points
   for(size_t i=0;i<mpoints.size();i++){
       cv::Point3f v=mpoints[(i+1)%mpoints.size()]-mpoints[i];
       float ax=1./float(npoints);//npoints
       for(float x=0;x<=1;x+=ax){
           cv::Point3f p3=mpoints[i]+v*x;
           points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));
       }
   }

   //line indicating direction
   //take first and second, first and last , and get the cross vector indicating the direction
   cv::Point3f v1=mpoints[1]-mpoints[0];
   cv::Point3f v2=mpoints[3]-mpoints[0];
    v1*=1./cv::norm(v1);
   v2*=1./cv::norm(v2);
   cv::Point3f vz=v2.cross(v1);
   vz*=1./cv::norm(vz);//now, unit

   //set the center
   cv::Point3f center=(mpoints[0]+mpoints[1]+mpoints[2]+mpoints[3])*0.25;
   float ax=(msize/3)/100;
   for(float x=0;x<=msize/3;x+=ax){
       cv::Point3f p3=center+vz*x;
       points.push_back(cv::Vec4f(p3.x,p3.y,p3.z, fcolor));

   }


   return points;

}

std::vector<cv::Vec4f> Map::getMarkerIdPcd( cv::Mat rt_g2m_, float _markerSize,uint32_t id,cv::Scalar color )const{

    auto _to_string=[](const uint32_t&val){
        std::stringstream sstr;
        sstr<<val;
        return sstr.str();
    };

    vector<cv::Point3f> marker_points = { cv::Point3f ( -_markerSize/2., _markerSize/2.,0 ),cv::Point3f ( _markerSize/2., _markerSize /2.,0 ),
                                          cv::Point3f ( _markerSize/2., -_markerSize/2.,0 ),cv::Point3f ( -_markerSize/2., -_markerSize/2.,0 )  };

    cv::Mat rt_g2m;
    rt_g2m_.convertTo(rt_g2m,CV_32F);
     //marker id as a set of points
    string text = _to_string(id);
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline=0;
    float markerSize_2=_markerSize/2;
    cv::Size textSize = cv::getTextSize(text, fontFace,
                                        fontScale, thickness, &baseline);
    cv::Mat img(textSize +cv::Size(0,baseline/2), CV_8UC1,cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0,textSize.height+baseline/4), fontFace, fontScale,cv::Scalar::all(255), thickness, 8);
    //raster 2d points as 3d points
    vector<cv::Point3f> points_id;
    for(int y=0;y<img.rows;y++)
        for(int x=0;x<img.cols;x++)
            if (img.at<uchar>(y,x)!=0) points_id.push_back( cv::Point3f((float(x)/float(img.cols))-0.5,(float(img.rows-y)/float(img.rows))-0.5,0));


    //now,scale
    for(auto &p:points_id)p*=markerSize_2;
    //finally, translate
    for(auto &p:points_id)p= mult<float>( rt_g2m,p);
    //now, add to ouput

    float fcolor;uchar *c=(uchar*)&fcolor;
    for(int i=0;i<3;i++)c[i]=color[i];

    vector<cv::Vec4f> res;
    for(auto &p:points_id)
        res.push_back(cv::Vec4f(p.x,p.y,p.z, fcolor));



    return res;
}



int64_t Map::getReferenceKeyFrame(const Frame &frame, float minDist) {

    //find the ids of the points detected and the frames they belong to
    std::map<uint32_t,zeroinitvar<int> > frame_counter;
    for(auto id:frame.ids){
        if (id!=std::numeric_limits<uint32_t>::max()){
            //take the keypoint and add the frames it appears in
            if ( map_points.is(id)){
                if  ( !map_points[id].isBad())
                    for(auto f_id:map_points[id].frames){
                        frame_counter[f_id.first]++;
                    }
            }
        }
    }
    //no matches found,
    if  (frame_counter.size()==0)   return  -1;

    pair<int64_t,int> BestOne(-1,0);
    for(auto fc:frame_counter)
        if (fc.second >  BestOne.second) BestOne=std::make_pair(fc.first,fc.second);

    return BestOne.first;

}

se3 Map::getBestPoseFromValidMarkers(const Frame &frame,const vector<uint32_t> &markersOfFrameToBeUsed_,float minErrorRatio){

    vector<uint32_t> markersValidPose;
    for(auto m:markersOfFrameToBeUsed_){
        auto it=map_markers.find(m);
        if(it!=map_markers.end()){
            if(it->second.pose_g2m.isValid())
                markersValidPose.push_back(it->first);
        }
    }

    struct minfo{
        int id;
        cv::Mat rt_f2m;
        double err;
    };
    se3 pose_f2g_out;//result
    //estimate the markers locations and see if there is at least one good enough
    vector<minfo> good_marker_locations;
    vector<minfo> all_marker_locations;
    for(const auto m:markersValidPose){//for ech visible marker
        const auto &marker_poseinfo=frame.getMarkerPoseIPPE(m);
        minfo mi;
        mi.id=m;
        mi.err=marker_poseinfo.errs[0];
        mi.rt_f2m=marker_poseinfo.sols[0];
        all_marker_locations.push_back(mi);
        if( marker_poseinfo.err_ratio > minErrorRatio)
            good_marker_locations.push_back(mi);
        mi.rt_f2m=marker_poseinfo.sols[1];
        all_marker_locations.push_back(mi);
    }


    //try using more than one marker approach
    if (markersValidPose.size()>=2) {
        //collect all the markers 3d locations
        vector<cv::Point2f> markerPoints2d;
        vector<cv::Point3f> markerPoints3d;
        for(auto ml:all_marker_locations){
            auto p2d=frame.getMarker(ml.id).und_corners;
            markerPoints2d.insert(markerPoints2d.end(),p2d.begin(),p2d.end());
            auto p3d=map_markers[ml.id].get3DPoints();
            markerPoints3d.insert(markerPoints3d.end(),p3d.begin(),p3d.end());
        }

        //take the all poses and select the one that minimizes the global reproj error
        vector<cv::Mat> allPoses;
        for(auto & ml:all_marker_locations){
            auto pose= ml.rt_f2m *map_markers[ml.id].pose_g2m.inv();
            //now,  compute the repj error of all markers using this info
            ml.err=reprj_error(markerPoints3d,markerPoints2d,frame.imageParams.undistorted(),  pose);
        }
        //sort and get the best
        std::sort(all_marker_locations.begin(),all_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
        _debug_msg("err="<<all_marker_locations.front().err,10);
        auto &best=all_marker_locations.front();
        pose_f2g_out=best.rt_f2m *map_markers[best.id].pose_g2m.inv();
        //now, do a  finer optimization
        cv::Mat rv=pose_f2g_out.getRvec(),tv=pose_f2g_out.getTvec();
        cv::solvePnP(markerPoints3d,markerPoints2d,frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,true);
        pose_f2g_out=se3(rv,tv);
        _debug_msg("err opt="<< reprj_error(markerPoints3d,markerPoints2d,frame.imageParams.undistorted(),  pose_f2g_out),10);
    }

    if ( pose_f2g_out.isValid()==false &&  good_marker_locations.size()>0){
        std::sort(good_marker_locations.begin(),good_marker_locations.end(),[](const minfo &a,const minfo &b){return a.err<b.err;});
        auto best=good_marker_locations[0];
        //estimate current location
        pose_f2g_out= best.rt_f2m *map_markers.at(best.id).pose_g2m.inv();
    }
    return   pose_f2g_out;

}
std::set<uint32_t> Map::getNeighborKeyFrames(uint32_t fidx,bool includeFidx){

    return TheKpGraph.getNeighbors(fidx,includeFidx);
}


void Map::saveToMarkerMap(std::string filepath)const {
    aruco::MarkerMap Mmap;
    std::string dict;
    if (map_markers.size()!=0)
        dict=map_markers.begin()->second.dict_info;
    Mmap.setDictionary(dict);
    Mmap.mInfoType=aruco::MarkerMap::METERS;
    for(const auto &mm:map_markers){
        const ucoslam::Marker &marker=mm.second;
        if (!marker.pose_g2m.isValid()) continue;
        aruco::Marker3DInfo m3di;
        m3di.id=marker.id;
        auto points3d=marker.get3DPoints();
        m3di.points.insert(m3di.points.end(),points3d.begin(),points3d.end());
        Mmap.push_back(m3di);
    }
    Mmap.saveToFile(filepath);
}

void Map::removeUnUsedKeyPoints(){

    for(auto &kf:keyframes){
        kf.removeUnUsedKeyPoints();//returns the indices
        //must change in the mappoint references
        for(int i=0;i <kf.ids.size();i++){
            auto &mp=map_points[ kf.ids[i]];
            mp.frames[kf.idx]=i;
        }

    }
}

void Map::removeWeakConnections(uint32_t kf,float minWeight){
    auto isMakerInFrame=[](int mid,const Frame &f){
        for(auto m:f.markers)
            if(m.id==mid)return true;
        return false;
    };
    if( !keyframes.is(kf)) return;
    auto &KF=keyframes[kf];
    if( !KF.isValid() || KF.isBad())return;
    auto neigh=getNeighborKeyFrames(kf,false);
    for(auto kf2:neigh){
        //may be no longer neighbors because removal of observations inside the loop
        if( !TheKpGraph.isEdge(kf,kf2)) continue;
        auto &KF2=keyframes[kf2];
        if( !KF2.isValid() || KF2.isBad())continue;
        //they must not share markers
        for(auto m:KF.markers)
            if( isMakerInFrame(m.id,KF2)) continue;

        if( TheKpGraph.getWeight(kf,kf2)<minWeight){
            //find shared mappoints and remove observations
            for(auto pidx:KF.ids){
                if( pidx!=std::numeric_limits<uint32_t>::max()){
                    if( map_points[pidx].isObservingFrame(kf2)){
                        bool removed=removeMapPointObservation(pidx,kf2,3);
                        if(!removed) removeMapPointObservation(pidx,kf,3);
                    }
                }
            }
        }
    }
}

}
