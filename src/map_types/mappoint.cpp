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
#include "mappoint.h"
#include "basictypes/io_utils.h"
#include "basictypes/hash.h"
#include "frame.h"

using namespace std;
namespace ucoslam {

template<typename T>
bool are_ranges_equal(T it1_start,T it1_end,T it2_start,T it2_end){
    auto i1=it1_start;
    auto i2=it2_start;
    while( i1!=it1_end && i2!=it2_end){
        if( !(*i1==*i2) )return false;
        i1++;i2++;
    }
    if (i1!=it1_end || i2!=it2_end)return false;
    return true;
}

MapPoint::MapPoint(){

}

 bool MapPoint::operator==(const MapPoint &p)const{
    if(id!=p.id) return false;
    if (!are_ranges_equal( frames.begin(),frames.end(), p.frames.begin(),p.frames.end())) return false;
    if (pos3d!=p.pos3d) return false;
    if (normal!=p.normal)return false;
    if (_desc.total()!=p._desc.total())return false;
    bool isEqual = (cv::sum(_desc!= p._desc) == cv::Scalar(0,0,0,0));
    if (!isEqual)return false;
    return true;


}
 uint64_t MapPoint::getSignature()const{
     Hash sig;
     sig+=id;

     //cout<<"mp 1. "<<sig<<endl;
     for(const auto &kv:frames) {sig+=kv.first;sig+=kv.second;}
     //cout<<"mp 2. "<<sig<<endl;
     sig.add(pos3d);
     //cout<<"mp 3. "<<sig<<endl;
     sig.add(normal);
     //cout<<"mp 4. "<<sig<<endl;
     sig+=_desc;
     //cout<<"mp 5. "<<sig<<endl;
     sig+=isStable();
     //cout<<"mp 6. "<<sig<<endl;
     sig+=isBad();
     //cout<<"mp 7. "<<sig<<endl;
     sig+=kfSinceAddition;
     //cout<<"mp 8. "<<sig<<endl;
     sig+=isStereo();
     //cout<<"mp 9. "<<sig<<endl;
     sig+=lastFIdxSeen;
     //cout<<"mp 10. "<<sig<<endl;
     sig+=nTimesSeen;
     //cout<<"mp 11. "<<sig<<endl;
     sig+=nTimesVisible;
     //cout<<"mp 12. "<<sig<<endl;
     sig+=mfMaxDistance;
     //cout<<"mp 13. "<<sig<<endl;
     sig+=mfMinDistance;
     //cout<<"mp 14. "<<sig<<endl;

     return sig;
 }


void MapPoint::fromStream(std::istream &str){
    int magic;
    str.read((char*)&magic,sizeof(magic));
    if (magic!=123200) throw std::runtime_error("Error in MapPoint::fromSteream");


    str.read((char*)&id,sizeof(id));

    str.read((char*)&pos3d,sizeof(pos3d));
    fromStream__(_desc,str);
    fromStream__kv(frames,str);

    str.read((char*)&normal,sizeof(normal));



    str.read((char*)&nTimesSeen,sizeof(nTimesSeen));
    str.read((char*)&nTimesVisible,sizeof(nTimesVisible));
    str.read((char*)&flags,sizeof(flags));
    str.read((char*)&mfMaxDistance,sizeof(mfMaxDistance));
    str.read((char*)&mfMinDistance,sizeof(mfMinDistance));
     str.read((char*)&kfSinceAddition,sizeof(kfSinceAddition));
     str.read((char*)&lastFIdxSeen,sizeof(lastFIdxSeen));



}

void MapPoint::toStream(std::ostream &str)const{

    //let us write a magic number to avoid reading errors
    int magic=123200;
    str.write((char*)&magic,sizeof(magic));


    str.write((char*)&id,sizeof(id));
    str.write((char*)&pos3d,sizeof(pos3d));
    toStream__(_desc,str);
    toStream__kv(frames,str);

    str.write((char*)&normal,sizeof(normal));



    //
    str.write((char*)&nTimesSeen,sizeof(nTimesSeen));
    str.write((char*)&nTimesVisible,sizeof(nTimesVisible));
    str.write((char*)&flags,sizeof(flags));
    str.write((char*)&mfMaxDistance,sizeof(mfMaxDistance));
    str.write((char*)&mfMinDistance,sizeof(mfMinDistance));
     str.write((char*)&kfSinceAddition,sizeof(kfSinceAddition));
     str.write((char*)&lastFIdxSeen,sizeof(lastFIdxSeen));

}
void MapPoint::scalePoint(float scaleFactor){
    pos3d*=scaleFactor;
    mfMaxDistance *= scaleFactor;
    mfMinDistance *= scaleFactor;
}


vector<pair<uint32_t,uint32_t> > MapPoint::getObservingFrames()const{
     vector<pair<uint32_t,uint32_t> > res;res.reserve(frames.size());
    for(auto f:frames)
        res.push_back(f);
    return res;
}

}
