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
#ifndef _UCOSLAM_GlobalOptimizerH_
#define _UCOSLAM_GlobalOptimizerH_
#include "map.h"
#include <unordered_set>
namespace ucoslam{
class System;

/**Base class for global optimization of points,markers and camera locations
 */
class GlobalOptimizer{
 public:

    struct ParamSet {
        ParamSet(bool Verbose=false):verbose(Verbose){}
        std::unordered_set<uint32_t> used_frames;//which are used. If empty, all
        std::set<uint32_t> fixed_frames;//which are set as not movable
        bool fixFirstFrame=true;
        float minStepErr=1e-2;
        float minErrPerPixel=0.25;//below this value, the optimization stops
        int nIters=100;//number of iterations
        bool verbose=false;
        float markersOptWeight=0.5;//importance of markers in the final error. Value in range [0,1]. The rest if assigned to points
        int minMarkersForMaxWeight=5;
        bool InPlaneMarkers=false;
          //---do not use from here
        float markerSize=-1;
        uint64_t getSignature()const;
    };

    //set the required params
    virtual void setParams(std::shared_ptr<Map>   map, const ParamSet &ps )=0;
    virtual void optimize(bool *stopASAP=nullptr) =0;
    virtual void getResults(std::shared_ptr<Map> map)=0;

    //one funtion to do everything
    virtual void optimize(std::shared_ptr<Map> map,const ParamSet &p=ParamSet() )=0;
    virtual string getName()const=0;

    //returns a vector of mapPointId,FrameId indicating the bad associations that should be removed
    virtual vector<std::pair<uint32_t,uint32_t>> getBadAssociations( ){return {};}


    static std::shared_ptr<GlobalOptimizer> create(string type="");

    void saveToStream(std::ostream &str);
    void readFromStream(std::istream &str);
protected:
    virtual void saveToStream_impl(std::ostream &str)=0;
    virtual void readFromStream_impl(std::istream &str)=0;
};
}
#endif
