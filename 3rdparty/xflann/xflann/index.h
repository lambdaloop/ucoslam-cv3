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
#ifndef _XFLANN_INDEX_H
#define _XFLANN_INDEX_H
#include <memory>
#include <vector>
#include "types.h"

#include "xflann_exports.h"
namespace  xflann{

class cpu;
/** Class representing an index. Internally dedices which index to use according to the parameters used
 */
class XFLANN_API Index{

public:

    /**Empty constructor
    */
    Index();

    /**
     * @brief build the index using the passed features and the indicated params.
     * @param features input data to create index
     * @param params indexing params. Either of type KdTreeParamSet or  KdMeansParamSet
     * @param desc_name optional parameter indicating the descriptor name so it can be saved to disk along the rest of data
     */
    Index(Matrix  features, const ParamSet &params);
    /**
     * @brief build the index using the passed features and the indicated params.
     * @param features input data to create index
     * @param params indexing params. Either of type KdTreeParamSet or  KdMeansParamSet
     * @param desc_name optional parameter indicating the descriptor name so it can be saved to disk along the rest of data
     */
    void  build(Matrix  features, const ParamSet &params);
    /**
     * @brief performs search (knn or  radius)
     * @param features query elements
     * @param nn number of neighbors to be search
     * @param indices output matrix. Must be already reserved with size features.rows x nn . Must be of integer type: XFLANN_X32S.
     * @param distances output matrix. Must be already reserved with size features.rows x nn . Must be of float type: XFLANN_X32F.
     * @param search_params optional search params (not used at this point)
     * @param Tcpu optional cpu info. It can be used to disable certain optimizations(avx,mmx or sse). Used normally in debug mode to test the speed of this method againts other without considering the hardware optimizations.
     * @returns true if the search was done and false otherwise
     */
    bool search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());



#if (defined XFLANN_OPENCV )
    //specialized version for opencv
    bool search(const cv::Mat &features, int nn,  cv::Mat &indices,  cv::Mat &distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());

#endif

    /**
     * @brief saveToFile Saves this index from a file
     * @param filepath output file path
     */
    void saveToFile(const std::string &filepath);
    /**
     * @brief readFromFile reads the index from a file generated with saveToFile
     * @param filepath input file path
     */
    void readFromFile(const std::string &filepath);
    /**
     * @brief saveToStream saves the index to a binary stream
     * @param str
     */
    void toStream(std::ostream &str) const;
    /**
     * @brief readFromStream reads the index from a binary stream
     * @param str
     */
    void fromStream(std::istream &str);

    /**Clears this object from data*/
    void clear();

    /**returns a hash with the info of this*/

    uint64_t hash()const;

private:
    bool _search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());

    std::shared_ptr< impl::IndexImpl  > _impl;
    void parallel_search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());

    //sort results
    template<typename DType>
    void sort(Matrix  indices,Matrix distances){
        for(int r=0;r<indices.rows;r++){
            auto idx_ptr=indices.ptr<int>(r);
            auto dist_ptr=distances.ptr<DType>(r);
            for(int i=0;i<indices.cols-1;i++){
                if ( idx_ptr[i]!=-1)
                    for(int j=i+1;j<indices.cols;j++){
                        if ( dist_ptr[i]>dist_ptr[j]){
                            std::swap(dist_ptr[i],dist_ptr[j]);
                            std::swap(idx_ptr[i],idx_ptr[j]);
                        }
                    }
            }
        }
    }
};


}
#endif
