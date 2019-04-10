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
#include "keyframedatabase.h"
#include <map>
#include <vector>
#include "basictypes/io_utils.h"
#include "basictypes/timers.h"
#include "basictypes/hash.h"
#include <fbow/fbow.h>
#include "map_types/frame.h"
#include "map_types/covisgraph.h"

namespace ucoslam{


class System;
class KFDataBaseVirtual{
public:
    virtual  bool isEmpty()const=0;
    virtual bool add(  Frame &f)=0;
    virtual bool del(const Frame &f)=0;
    virtual void clear( )=0;
    virtual std::vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={})=0;
    virtual uint64_t getSignature()const=0;
    virtual bool isId(uint32_t id)const=0;
     virtual void toStream_(std::iostream &str)const =0;
    virtual void fromStream_(std::istream &str)=0;
    virtual void loadFromFile(const std::string &filename)=0 ;
    virtual size_t size()const=0;
    virtual float score(Frame &f,Frame &f2) =0;


};

//A database of frames. It is prepared to be called even if no vocabulary is set.
//So, you can run the system using it or not seamlessly
class KPFrameDataBase:public KFDataBaseVirtual{
public:
    //returns true if the vocabulary is empty or not
    bool isEmpty()const{return frames.size()==0;}
    //adds the frame and returns true. If the  Vocabulary is empty returns false
    bool add(  Frame &f);
    //deletes the frame and returns true. If the  Vocabulary is empty returns false
    bool del(const Frame &f);
    //deletes the frame and returns true. If the  Vocabulary is empty returns false
    bool del(uint32_t fidx);//slower than previous one
    //clear all
    void clear( );
    //return the idx of the best frame candidates for relocalization
    std::vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={});

    //computes the Bow of a frame. returns true if possible and false otherwise.
    //it is not computed if the vocabulary is not created
    //throws an exception if the descriptor of the Frame is different from the descriptors of the vocabulary
    bool computeBow(Frame &f)   ;
    //returns the score between the frames passed
    float score(Frame &f,Frame &f2) ;

    void toStream_(std::iostream &str)const ;
    void fromStream_(std::istream &str);
    void loadFromFile(const std::string &filename) ;

    //is the id indicated
    bool isId(uint32_t id)const;
    //number of frames in the database
    size_t size()const{return frames.size();}

    //return the set of frames in the database
    inline const std::set<uint32_t> getFrames()const{return frames;}


    uint64_t getSignature()const;

    //private:
    //for each word, the set of frames that contain it
    std::map<uint32_t,std::set<uint32_t> > word_frames_;
    fbow::Vocabulary _voc;
    std::set<uint32_t> frames;//ids of the frames in the database(for debbuging) pourposes
};


class DummyDataBase:public KFDataBaseVirtual{
    std::set<uint32_t> frames;
 public:
    bool add(Frame &f){frames.insert(f.idx); return true;}
    bool del(const Frame &f){frames.erase(f.idx);return true;}
    bool del(uint32_t &f){frames.erase(f);return true;}
    bool isEmpty()const{return frames.size()==0;}
    void clear(){frames.clear();}
    size_t size()const{return frames.size();}
    std::vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={}){
        return {};
    }
    void toStream_(std::iostream &str)const;
    void fromStream_(std::istream &str);
    bool isId(uint32_t id)const {return  frames.count(id)!=0;}
    virtual void loadFromFile(const std::string &filename) {assert(false);throw std::runtime_error("DummyDataBase::loadFromFile not implemented");} ;
    uint64_t getSignature()const;
    float score(Frame &f,Frame &f2)  {
        return 0;
    }

};


void DummyDataBase::toStream_(iostream &str)const{
toStream__(frames,str);
}

void DummyDataBase::fromStream_(std::istream &str){
    fromStream__(frames,str);

}
uint64_t DummyDataBase::getSignature()const{
    Hash sig;
    for(auto f:frames) sig+=f;
    return sig;

}


void KPFrameDataBase::loadFromFile(const std::string &filename){
    _voc.readFromFile(filename);
    word_frames_.clear();
}
void KPFrameDataBase::clear(){

    //if(clearVocabulary) _voc.clear();
    frames.clear();
    word_frames_.clear();

}

bool KPFrameDataBase::add(  Frame &f){
    if(_voc.size()==0) return false;
    assert(frames.count(f.idx)==0);
    //create the bag of words vector
    computeBow (f);
 //   cout<<"BOWSIG : "<<f.bowvector.getSignature()<<endl;
    //for each word, register it
    for(auto &w:*f.bowvector)  word_frames_[w.first].insert(f.idx);
    frames.insert(f.idx);
    return true;
}

bool KPFrameDataBase::del(const Frame &f){
    //remove
    if(_voc.size()==0) return false;
    assert(frames.count(f.idx)!=0);
    assert(f.bowvector->size()!=0);
    for(auto &w:*f.bowvector) word_frames_[w.first].erase(f.idx);
    frames.erase(f.idx);
    return true;
}
bool KPFrameDataBase::del(uint32_t fidx){
    assert(frames.count(fidx)!=0);
    //remove
    if(_voc.size()==0) return false;
    for(auto &wf:word_frames_) wf.second.erase(fidx);
    frames.erase(fidx);
    return true;
}
bool KPFrameDataBase::isId(uint32_t id)const{
    return frames.count(id);
}

uint64_t KPFrameDataBase::getSignature()const{
    Hash sig;
    for(auto wf:word_frames_){
        sig+=wf.first;
        sig.add(wf.second.begin(),wf.second.end());
    }
    sig.add(frames.begin(),frames.end());
    sig+=_voc.hash();

    return sig;
}

vector<uint32_t> KPFrameDataBase::relocalizationCandidates(Frame &frame,FrameSet &fset,CovisGraph &covisgraph ,bool sorted,float minScore,const std::set<uint32_t> &excludedFrames){

    __UCOSLAM_ADDTIMER__
    if(_voc.size()==0) throw std::runtime_error("no vocabulary");
    if (frame.bowvector->size()==0) computeBow (frame);
    __UCOSLAM_TIMER_EVENT__("step0");
//    for(auto e:excludedFrames)cout<<e<<endl;
    struct nobs{ uint32_t obs=0;};
    //number of times a word from f is seen in the other frames
    std::map<uint32_t,nobs> frame_nobs;//frame it is seen and how many times

    //determine how many and the max
    uint32_t maxCommonWords=0;
    for(auto &w:*frame.bowvector){
        if ( !word_frames_.count(w.first)) continue;
        const std::set<uint32_t> &frames=word_frames_.at(w.first);
        for (auto &f:frames){
            if (excludedFrames.count(f)) continue;
            nobs & fn = frame_nobs[f];
            fn.obs++;
            if (fn.obs> maxCommonWords)maxCommonWords=fn.obs;
        }
    }


    __UCOSLAM_TIMER_EVENT__("step1");
    if( frame_nobs.size()==0 )return {};
    uint32_t minCommonWords = maxCommonWords*0.8f;
    // Compute similarity score.
    std::map< uint32_t,double> frame_score;
    for(const auto &fn:frame_nobs){
        if (fn.second.obs>minCommonWords){
            assert(fset.count(fn.first));
            double si = fbow::fBow::score(*frame.bowvector,*fset[fn.first].bowvector);
            if (si>minScore)
                frame_score[fn.first]=si;
        }
    }


    __UCOSLAM_TIMER_EVENT__("step2");
    if (frame_score.size()==0)return {};
    if( frame_score.size()==1)return {frame_score.begin()->first};
    //     Lets now accumulate score by covisibility
    std::vector< pair<uint32_t,double> > frame_scoreCovis;
    double bestAccScore=minScore;
    for(auto &fs:frame_score){
        double accScore = fs.second;
        //get the neighbors sorted by decreasing weitgh
        auto n_weight=covisgraph.getNeighborsWeights(fs.first,true);

        if ( n_weight.size()>=2){
            assert(n_weight[0].second>=n_weight[1].second );
        }
        n_weight.resize( std::min(n_weight.size(),size_t(10)));//retain only the 10 best
        for(auto &nw:n_weight){
            auto it=frame_score.find(nw.first);
            //if the neighbor is here, add its score
            if (it!=frame_score.end())  accScore+=it->second;
        }
        frame_scoreCovis.push_back(std::make_pair(fs.first,accScore));
        if (accScore>bestAccScore)bestAccScore=accScore;
    }

    __UCOSLAM_TIMER_EVENT__("step3");

    //    // Return all those keyframes with a score higher than 0.75*bestScore
    double minScoreToRetain = 0.75f*bestAccScore;
    frame_scoreCovis.erase(    std::remove_if(frame_scoreCovis.begin(),frame_scoreCovis.end(),[&](const pair<uint32_t,double> &v){return v.second<minScoreToRetain;}),
                               frame_scoreCovis.end());
    if(sorted){
        std::sort(frame_scoreCovis.begin(),frame_scoreCovis.end(),[&](const pair<uint32_t,double> &a,const pair<uint32_t,double> &b){return a.second>b.second;});
        if (frame_scoreCovis.size()>=2)
            assert(frame_scoreCovis[0].second>=frame_scoreCovis[1].second );
    }
    __UCOSLAM_TIMER_EVENT__("step4");

    //copy remaining to vector and return
    vector<uint32_t> candidates;candidates.reserve(frame_scoreCovis.size());
    for(const auto &fs:frame_scoreCovis)candidates.push_back(fs.first);
    return candidates;
}

void  KPFrameDataBase::toStream_(iostream &str) const {
     _voc.toStream(str);//write voc
    io_write<uint32_t>(word_frames_.size(),str);
    for(const  auto &w:word_frames_){
        io_write<uint32_t>(w.first,str);
        toStream__(w.second,str);
    }
    toStream__(frames,str);

}


void  KPFrameDataBase::fromStream_(std::istream &str){
    _voc.fromStream(str);
    word_frames_.clear();

    int s=io_read<uint32_t>(str);
    for(int i=0;i<s;i++){
        std::pair<uint32_t,std::set<uint32_t> > item;
        item.first=io_read<uint32_t>(str);;
        fromStream__(item.second,str);
        word_frames_.insert(item);
    }
    fromStream__(frames,str);

 }
float KPFrameDataBase::score(Frame &f,Frame &f2) {
    if (f.bowvector->size()==0) computeBow(f);
    if (f2.bowvector->size()==0) computeBow(f2);
    return fbow::fBow::score(*f.bowvector,*f2.bowvector);
}

bool KPFrameDataBase::computeBow(Frame &f) {
    if(_voc.size()==0 || f.desc.rows==0) return false;
    //check that descriptors have the same size than the voc descritors
    if (_voc.getDescSize()!=uint32_t(f.desc.cols) )
        throw std::runtime_error("FrameDataBase::computeBow Vocabulary and descriptor employed have different sizes. May be you are using a wrong descriptor type");
    if (_voc.getDescType()!=uint32_t(f.desc.type()))
        throw std::runtime_error("FrameDataBase::computeBow Vocabulary and descriptor employed have different types. May be you are using a wrong descriptor type");

    __UCOSLAM_ADDTIMER__;
    _voc.transform(f.desc,3,*f.bowvector,*f.bowvector_level);

    return true;
}
KeyFrameDataBase::KeyFrameDataBase(){
     _impl=std::make_shared<DummyDataBase>();
    _type=0;
}


void KeyFrameDataBase::loadFromFile(const std::string &str){
        _impl=std::make_shared<KPFrameDataBase>();
        _type=1;
        _impl->loadFromFile(str);
}

void KeyFrameDataBase::toStream(iostream &str)const
{
    str.write((char*)&_type,sizeof(_type));

    _impl->toStream_(str);
}
void KeyFrameDataBase::fromStream(std::istream &str)
{
    str.read((char*)&_type,sizeof(_type));
    if (_type==0)
        _impl=std::make_shared<DummyDataBase>();
    else if (_type==1)
        _impl=std::make_shared<KPFrameDataBase>();

    _impl->fromStream_(str);
}

bool KeyFrameDataBase::isEmpty()const{return _impl->isEmpty();}
bool KeyFrameDataBase::isId(uint32_t id)const {return _impl->isId(id);}

bool KeyFrameDataBase::add(Frame &f){return _impl->add(f);}
bool KeyFrameDataBase::del(const Frame &f){return _impl->del(f);}
void KeyFrameDataBase::clear(){_impl->clear();}

size_t KeyFrameDataBase::size()const{return _impl->size();}

vector<uint32_t> KeyFrameDataBase::relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted, float minScore, const std::set<uint32_t> &excludedFrames){
    return _impl->relocalizationCandidates(frame,fset,covisgraph,sorted,minScore,excludedFrames);
}
float KeyFrameDataBase::score(Frame &f,Frame &f2)  {return _impl->score(f,f2); }

uint64_t KeyFrameDataBase::getSignature()const{return _impl->getSignature();}

}

