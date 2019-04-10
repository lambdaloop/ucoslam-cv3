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
#ifndef _UCOSLAM_GLOBAL_OPTIMIZER_G2O_H_
#define _UCOSLAM_GLOBAL_OPTIMIZER_G2O_H_
#include "globaloptimizer.h"

namespace g2o{
class SparseOptimizer;
};

namespace ucoslam{

/**Performs a global optimization of points,markers and camera locations
 */
class   GlobalOptimizerG2O: public GlobalOptimizer{
public:

    void setParams(std::shared_ptr<Map> map, const ParamSet &p=ParamSet() );
    void optimize(bool *stopASAP=nullptr) ;
    void getResults(std::shared_ptr<Map> map);

    void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() ) ;
    vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return _badAssociations;}

    string getName()const{return "g2o";}


private:

    template<typename T>
    struct zeroinitvar{
        operator T& (){return val;}
        operator T ()const{return val;}
        void operator++(int){val++;}
        T val=0;
    };



    ParamSet _params;


    void saveToStream_impl(std::ostream &str){};
    void readFromStream_impl(std::istream &str){};
    std::shared_ptr<g2o::SparseOptimizer> Optimizer;


    uint64_t join(uint32_t a ,uint32_t b){
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    }
    inline pair<uint32_t,uint32_t> separe(uint64_t a_b){         uint32_t *_a_b_16=(uint32_t*)&a_b;return make_pair(_a_b_16[1],_a_b_16[0]);}



    vector<std::pair<uint32_t,uint32_t> > _badAssociations;
    vector<float> _InvScaleFactors;
    struct edge_frameId_stereo{
        edge_frameId_stereo(void *f,uint32_t s,bool isSt){
            first=f;
            second=s;
            isStereo=isSt;
        }
        void *first;
        uint32_t second;
        bool isStereo=false;
    };


    struct markerInfo{
        void *vertex;
        int IdOptmz;
    };

    std::vector<std::vector< edge_frameId_stereo > > point_edges_frameId;
     std::vector<void* > marker_edges;
    std::map<uint32_t,markerInfo> marker_info;

    std::map<uint32_t, zeroinitvar<double> > frame_kpOptWeight;//for each frame, the total weight of the errors due to keypoints
    std::map<uint32_t, zeroinitvar<double> > frame_MarkerWeight;//for each frame, the total weight of the errors due to markers

    vector<uint32_t> usedFramesIdOpt,usedPointsIdOpt,usedMapPoints;
    vector<char> isFixedFrame;
    const uint32_t INVALID_IDX=std::numeric_limits<uint32_t>::max();
    const uint32_t INVALID_VISITED_IDX=std::numeric_limits<uint32_t>::max()-1;

    const char UNFIXED=0;
    const char FIXED_WITHOUTPOINTS=1;
    const char FIXED_WITHPOINTS=2;



    const float Chi2D = 5.99;
    const float Chi3D = 7.815;
    const float Chi8D = 15.507;

    const float thHuber2D = sqrt(Chi2D);
    const float thHuber3D= sqrt(Chi3D);
    const float thHuber8D = sqrt(Chi8D);

};
}
#endif
