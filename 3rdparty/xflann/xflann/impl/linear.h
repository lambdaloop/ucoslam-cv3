/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2016  Rafael Muñoz Salinas (rmsalinas@uco.es). All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/
#ifndef _XFLANN_LINEAR_H
#define _XFLANN_LINEAR_H
#include "../xflann_exports.h"
#include "distances.h"
#include "../types.h"
#include "resultset.h"
#include "heap.h"
#include <mutex>
namespace xflann{
namespace impl{
/**This class creates the index
 */
class XFLANN_API Linear:public impl::IndexImpl
{
public:
    std::string getName()const{return "linear";}


    void build ( Matrix features,   const ParamSet &params);
    bool storeFeatures()const {return _params._store_data;}

    void search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet &sparams, std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());

    uint32_t size()const{return _features.size();}
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
    uint64_t hash() const;
private:
    struct Params{
        bool _store_data;
        int _desc_size;
        int _aligment;
    };
    Params _params;
    xflann::Matrix _features;
    Vector<char> _stored_features;
    std::shared_ptr<cpu>  cpu_info;
    std::mutex cpu_info_mutex;
    template<typename Computer>
    void _knnsearch(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  search_params){
         using DType=typename Computer::DType;//distance type
        using TData=typename Computer::TData;//data type
        Computer comp;

        ResultSet<DType> hres(nn,search_params.asDouble("maxDist"));
        int nfeattdata= comp.getNFeatOps(_params._desc_size);
        for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
            TData *feat=features.ptr<TData>(cur_feature);
            hres.reset(distances.ptr<DType>(cur_feature),indices.ptr<int>(cur_feature));
            for(int i=0;i<_features.rows;i++){
                    auto dist=comp.computeDist_((TData*)feat,_features.ptr<TData>(i),nfeattdata);
                    hres.push(ResultInfo<DType>(dist,i));
            }
            //
            for(int i=hres.size();i<nn;i++){
                indices.ptr<int>(cur_feature)[i]=-1;
                distances.ptr<DType>(cur_feature)[i]=std::numeric_limits<DType>::quiet_NaN();
            }
        }
    }
};
}
}
#endif
