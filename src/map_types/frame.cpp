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
#include "frame.h"
#include "basictypes/hash.h"
#include "basictypes/io_utils.h"
#include <fbow/fbow.h>
namespace ucoslam {
Frame::Frame(){
    bowvector=std::make_shared<fbow::fBow>();
    bowvector_level=std::make_shared<fbow::fBow2>();
}

void Frame::copyTo( Frame &f)const{

    f.idx=idx;
    f.markers=markers;
    f.keypoint_kdtree=keypoint_kdtree;
    desc.copyTo(f.desc);
    f.ids=ids;

    f.flags=flags;
//    f.nonMaxima=nonMaxima;
    pose_f2g.copyTo(f.pose_f2g);
    f.und_kpts=und_kpts;
    f.kpts=kpts;
    f.depth=depth;

    image.copyTo(f.image);
    f.bowvector= std::make_shared<fbow::fBow>(*bowvector); ;
    f.bowvector_level= std::make_shared<fbow::fBow2> (*bowvector_level);
    f.fseq_idx=fseq_idx;
    f. scaleFactors= scaleFactors;
    f.imageParams=imageParams;//camera with which it was taken
    f.frame_flags=frame_flags;
    f.KpDescType=KpDescType;
    f.minXY=minXY;
    f.maxXY=maxXY;
 }


Frame & Frame::operator=(const Frame &f){
    f.copyTo(*this);
    return *this;
}


void Frame::nonMaximaSuppresion(){



        //for each assigned element, remove these around
        for(size_t i=0;i<ids.size();i++){
            if (ids[i]!=std::numeric_limits<uint32_t>::max()){
                //find points around
                auto kpts=getKeyPointsInRegion(und_kpts[i].pt, scaleFactors[ und_kpts[i].octave]*2.5,und_kpts[i].octave,und_kpts[i].octave);
                //these unassigned, remove them
                for(auto k:kpts)
                    if ( ids[k]==std::numeric_limits<uint32_t>::max())
                        flags[k].set(Frame::FLAG_NONMAXIMA,true);
            }
        }

        //now, suppress non maxima in octaves

        for(size_t i=0;i<ids.size();i++){
            if (ids[i]==std::numeric_limits<uint32_t>::max() && !flags[i].is(Frame::FLAG_NONMAXIMA)){
                auto kpts=getKeyPointsInRegion(und_kpts[i].pt, scaleFactors[ und_kpts[i].octave]*2.5,und_kpts[i].octave,und_kpts[i].octave);
                //get the one with maximum response

                pair<uint32_t ,float> best(std::numeric_limits<uint32_t>::max(),0);
                for(auto k:kpts){
                    if ( !flags[k].is(Frame::FLAG_NONMAXIMA))
                        if (best.second<und_kpts[k].response)
                            best={k,und_kpts[k].response};
                }
                assert(best.first!=std::numeric_limits<uint32_t>::max());
                //set invalid all but the best
                for(auto k:kpts)
                    if(  k!=best.first)
                        flags[k].set(Frame::FLAG_NONMAXIMA,true);
            }
        }

}

std::vector<uint32_t> Frame::getKeyPointsInRegion(cv::Point2f p, float radius ,  int minScaleLevel,int maxScaleLevel)const{

    cv::KeyPoint kp;
    kp.pt=p;
    auto  idx_dist=keypoint_kdtree.radiusSearch(und_kpts,kp,radius,false);

    std::vector<uint32_t> ret;ret.reserve(idx_dist.size());
    //find and mark these that are not in scale limits
    for( auto &id:idx_dist)
        if ( und_kpts[id.first].octave>=minScaleLevel && und_kpts[id.first].octave<=maxScaleLevel)
            ret.push_back(id.first);
    return ret;

}

int Frame::getMarkerIndex(uint32_t id)const  {
    for(size_t i=0;i<markers.size();i++)
        if (uint32_t(markers[i].id)==id)return i;
    return -1;

}

ucoslam::MarkerObservation Frame::getMarker(uint32_t id) const{
    for(const auto &m:markers)
        if (uint32_t(m.id)==id)return m;
    throw std::runtime_error("Frame::getMarker Could not find the required marker");
}
#pragma warning "try to remove"
 MarkerPosesIPPE Frame::getMarkerPoseIPPE(uint32_t id)const
{
    for(size_t i=0;i<markers.size();i++)
        if (uint32_t(markers[i].id)==id) return markers[i].poses;
    throw std::runtime_error("Frame::getMarkerPoseIPPE Could not find the required marker");

}




  cv::Point3f Frame::getCameraDirection()const{
        Se3Transform mPose=pose_f2g.inv();
        auto v0=mPose*cv::Point3f(0,0,0);  //obtain camera center in global reference system
        auto v1=mPose*cv::Point3f(0,0,1);
        auto vd=v1-v0;
        return vd * (1./cv::norm(vd));
  };

  uint64_t  __getSignature(const vector<cv::KeyPoint> &kp,const cv::Mat &desc){
    uint64_t sig=0;
    for(const cv::KeyPoint &p:kp)
        sig+=p.pt.x*1000+p.pt.y*1000+p.angle*100+p.octave*100+p.response*100;
    //now, descriptor
    int nem= desc.elemSize()*desc.cols/sizeof(char);
    for(int r=0;r<desc.rows;r++)
        for(int i=0;i<nem;i++) sig+=desc.ptr<char>(r)[i];
    return sig;

  }
  vector<uint32_t> Frame::getMapPoints()const{
      vector<uint32_t> res;res.reserve(this->und_kpts.size());
      for(auto id:this->ids)
          if (id!=std::numeric_limits<uint32_t>::max())
              res.push_back(id);
      return res;
  }

  vector<uint32_t> Frame::getIdOfPointsInRegion( cv::Point2f  p,float radius){

      cv::KeyPoint kp;kp.pt=p;
      auto v_id_dist=keypoint_kdtree.radiusSearch( und_kpts,kp,radius,false);
      vector<uint32_t> ret;ret.reserve(v_id_dist.size());
      for(auto id_dist:v_id_dist){
          if ( ids[id_dist.first]!=std::numeric_limits<uint32_t>::max())
              ret.push_back(ids[id_dist.first]);
      }
      return ret;
  }

  uint64_t Frame::getSignature()const{

      Hash hash;

      hash+=idx;

      //cout<<"Frame .1. ="<<hash<<endl;//0
      for(auto m:markers)
            hash+=m.getSignature();
      //cout<<"Frame .2. ="<<hash<<endl;//1

      hash+=desc;
      //cout<<"Frame .4. ="<<hash<<endl;//3
      hash.add(ids.begin(),ids.end());
      //cout<<"Frame .5. ="<<hash<<endl;//4
      hash.add(flags.begin(),flags.end());
      //cout<<"Frame .6. ="<<hash<<endl;//5
      hash+= pose_f2g;
      //cout<<"Frame .7. ="<<hash<<endl;//6
      hash.add(und_kpts.begin(),und_kpts.end());
      //cout<<"Frame .8. ="<<hash<<endl;//7
      hash.add(kpts.begin(),kpts.end());
      //cout<<"Frame .9. ="<<hash<<endl;//8
      hash.add(depth.begin(),depth.end());
      //cout<<"Frame .10. ="<<hash<<endl;//9
      hash+= image;
      //cout<<"Frame .11. ="<<hash<<endl;

      hash.add(bowvector->hash());
      hash.add(bowvector_level->hash());
      //cout<<"Frame .12. ="<<hash<<endl;//11
      hash+=fseq_idx;
      //cout<<"Frame .13. ="<<hash<<endl;//12
      hash.add(scaleFactors.begin(),scaleFactors.end());
      //cout<<"Frame .14. ="<<hash<<endl;//13
      hash.add(imageParams.getSignature());
      //cout<<"Frame .15. ="<<hash<<endl;//14
      hash+=frame_flags;
      //cout<<"Frame .16. ="<<hash<<endl;//15
      hash+=KpDescType;
      //cout<<"Frame .17. ="<<hash<<endl;//15
      hash+=minXY.x;
      hash+=minXY.y;
      hash+=maxXY.x;
      hash+=maxXY.y;

      //cout<<"Frame .18. ="<<hash<<endl;//16
      hash+=keypoint_kdtree.getHash();
      //cout<<"Frame .19. ="<<hash<<endl;//16
      return hash;

  }


void Frame::clear()
{
      idx=std::numeric_limits<uint32_t>::max();//frame identifier
      markers.clear();
      keypoint_kdtree.clear();
      desc=cv::Mat();
      ids.clear();
      flags.clear();
      pose_f2g=se3();
      und_kpts.clear();
      kpts.clear();
      depth.clear();
      image=cv::Mat();
      bowvector=std::make_shared<fbow::fBow>();
      bowvector_level=std::make_shared<fbow::fBow2>();
      fseq_idx=std::numeric_limits<uint32_t>::max();
      scaleFactors.clear();
      imageParams.clear();

      KpDescType=DescriptorTypes::DESC_NONE;//keypoint descriptor type (orb,akaze,...)
      //returns a map point observation from the current information
       minXY=cv::Point2f(0,0);
       maxXY=cv::Point2f(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());

}

void Frame::toStream(std::ostream &str) const  {

    int magic=134243;
    str.write((char*)&magic,sizeof(magic));

    str.write((char*)&idx,sizeof(idx));
    str.write((char*)&fseq_idx,sizeof(fseq_idx));
    str.write((char*)&frame_flags,sizeof(frame_flags));
    str.write((char*)&KpDescType,sizeof(KpDescType));



    toStream__(desc,str);
    toStream__(und_kpts,str);
    toStream__(kpts,str);
    toStream__(depth,str);



    toStream__(ids,str);
    toStream__(flags,str);


     toStream__ts(markers,str);
     pose_f2g.toStream(str);
    bowvector->toStream(str);
    bowvector_level->toStream(str);

    toStream__(scaleFactors,str);
    imageParams.toStream(str);

    toStream__(image,str);


    keypoint_kdtree.toStream(str);
    str.write((char*)&minXY,sizeof(minXY));
    str.write((char*)&maxXY,sizeof(maxXY));



    magic=134244;
    str.write((char*)&magic,sizeof(magic));

}
float  Frame::getDepth(int idx)const{
 if(depth.size()==0)return 0;
 return depth[idx];
}
void Frame::fromStream(std::istream &str) {
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if ( magic!=134243)throw std::runtime_error("Frame::fromStream error in magic");
    str.read((char*)&idx,sizeof(idx));
    str.read((char*)&fseq_idx,sizeof(fseq_idx));
    str.read((char*)&frame_flags,sizeof(frame_flags));
    str.read((char*)&KpDescType,sizeof(KpDescType));

    fromStream__(desc,str);
    fromStream__(und_kpts,str);
    fromStream__(kpts,str);
    fromStream__(depth,str);


    fromStream__(ids,str);
    fromStream__(flags,str);
     fromStream__ts(markers,str);


    pose_f2g.fromStream(str);
    bowvector->fromStream(str);
    bowvector_level->fromStream(str);

    fromStream__(scaleFactors,str);
    imageParams.fromStream(str);

    fromStream__(image,str);

    keypoint_kdtree.fromStream(str);
    str.read((char*)&minXY,sizeof(minXY));
    str.read((char*)&maxXY,sizeof(maxXY));

    str.read((char*)&magic,sizeof(magic));
   if ( magic!=134244)throw std::runtime_error("Frame::fromStream error in magic");
   //  create_kdtree();
}





void FrameSet::toStream(ostream &str) const {
    //set magic
    int magic=88888;
    str.write((char*)&magic,sizeof(magic));
     ReusableContainer<ucoslam::Frame>::toStream(str);
 }

void FrameSet::fromStream(istream &str)
{
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if (magic!=88888) throw std::runtime_error("FrameSet::fromStream error reading magic");

    ReusableContainer<ucoslam::Frame>::fromStream(str);

}
uint64_t FrameSet::getSignature()const{
    uint64_t sig=0;
    for(auto const &f:*this)
        sig+=f.getSignature();
    return sig;
}
void MarkerPosesIPPE::toStream(std::ostream &str) const{
    toStream__(sols[0],str);
    toStream__(sols[1],str);
    str.write((char*)&errs[0],sizeof(errs[0]));
    str.write((char*)&errs[1],sizeof(errs[1]));
    str.write((char*)&err_ratio,sizeof(err_ratio));

}
void MarkerPosesIPPE::fromStream(std::istream &str){
    fromStream__(sols[0],str);
    fromStream__(sols[1],str);
    str.read((char*)&errs[0],sizeof(errs[0]));
    str.read((char*)&errs[1],sizeof(errs[1]));
    str.read((char*)&err_ratio,sizeof(err_ratio));
}
uint64_t MarkerPosesIPPE::getSignature() const{
    Hash sig;
    for(int i=0;i<2;i++){
        sig.add(sols[i]);
        sig.add(errs[i]);
    }
    sig+=err_ratio;
    return sig;
}

void Frame::removeUnUsedKeyPoints(){
     int curIdx=0;
     std::map<uint32_t,uint32_t> old_new;
    for(int i=0;i<ids.size();i++){
        if( ids[i]!=std::numeric_limits<uint32_t>::max()){
            ids[curIdx]=ids[i];
            flags[curIdx]=flags[i];
            und_kpts[curIdx]=und_kpts[i];
            kpts[curIdx]=kpts[i];
            desc.row(i).copyTo(desc.row(curIdx));
            if(bowvector_level)
                old_new.insert({i,curIdx});
            curIdx++;
         }
    }
    ids.resize(curIdx);
    flags.resize(curIdx);
    und_kpts.resize(curIdx);
    kpts.resize(curIdx);
    desc.resize(curIdx);
    create_kdtree();
    //remove rom bowvector_level
    if(bowvector_level){
        vector<uint32_t> toremove;
        for(auto &kv:*bowvector_level.get())
        {
            int nvalid=0;
            for(auto &e: kv.second){
                auto it=old_new.find(e);
                if(it==old_new.end())
                    e=-1;
                else{
                    e=it->second;
                    nvalid++;
                }
            }
            if(nvalid!=0){
                //remove elements with -1
                kv.second.erase(std::remove_if(kv.second.begin(),kv.second.end(), [](const int &e){return e==-1 ;}), kv.second.end());
            }
            else//it will be entirely removed
                toremove.push_back(kv.first);
        }
        for(auto r:toremove) bowvector_level->erase(r);
    }
}

}
