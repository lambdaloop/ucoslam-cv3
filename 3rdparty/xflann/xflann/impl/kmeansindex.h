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
#ifndef _XFLANN_KMeanIndex_H
#define _XFLANN_KMeanIndex_H
#include "../xflann_exports.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include "../types.h"
#include "../cpu.h"
#include "heap.h"
#include "resultset.h"
#include <mutex>
namespace xflann{
namespace impl{

/**Main class to represent a vocabulary of visual words
 */
class XFLANN_API KMeansIndex:public IndexImpl
{
    friend class KMeansIndexCreator;
 public:

    ~KMeansIndex();

    /**
     * @brief knnsearch performs a knn search to find the nearest neighbor
     * @param features matrix of features to search. One per row
     * @param nn number of nearest neighbors
     * @param indices matrix already allocated where the indices will be written. Type int
     * @param distances matrix already allocates where distances will be written. Type float or uint32_t depending on the input data.
     * @param search_params
     */
    void search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet &search_params, std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());
    bool storeFeatures()const {return  true;}
    std::string getName()const{return "kmeans";}
    uint32_t size()const {return _params._npoints;}

    //loads/saves from a file
    void readFromFile(const std::string &filepath);    
    void saveToFile(const std::string &filepath);
    ///save/load to binary streams
    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str);
    //returns the descriptor type (CV_8UC1, CV_32FC1  )
    int getDescType()const{return _params._desc_type;}
    //returns desc size in bytes or 0 if not set
    uint32_t getDescSize()const{return _params._desc_size;}
      //indicates whether this object is valid
    bool isValid()const{return _data!=0;}
    //returns a hash identifying this
    uint64_t hash()const;
    std::string hash_str()const;
private:
 //    void  setParams(  int aligment,int k,int desc_type,int desc_size, int nblocks,std::string desc_name);
     void  setParams(int aligment,  int desc_type, int desc_size, uint64_t total_size, uint32_t npoints);
    struct params{
         uint32_t _aligment;//memory aligment and total number of blocks
        uint64_t _desc_size_bytes_wp=0;//size of the descriptor(includes padding)
        uint64_t _total_size=0;
        int _desc_type=XFLANN_NONE;
        uint32_t _desc_size=0;//original descriptor types and sizes (without padding)
        uint32_t _npoints=0;//total number of points in the index
     };
    params _params;
    char * _data=0;//pointer to data
    //removes all data
    void clear();
    //structure represeting a information about node in a block

    //if it is non leaf, the data represents the offset to reach the child block
    //if leaf, the data represents the index of the point from the dataset
    //first bit is employed to define if leaf or not leaf
    struct block_node_info{
        uint64_t pidx_blockoffset;
        inline bool isleaf()const{return ( pidx_blockoffset& 0x8000000000000000);}
        // return the value
        inline uint64_t get()const{return ( pidx_blockoffset&0x7FFFFFFFFFFFFFFF);}
        //sets as leaf, and sets the index of the feature it represents and its weight
        inline void setLeaf(uint64_t pidx){
            assert(!(pidx & 0x8000000000000000));//check msb is zero
            pidx_blockoffset=pidx;
            pidx_blockoffset|=0x8000000000000000;//set the msb to one to distinguish from non leaf
        }
        //sets as non leaf and sets the offset of the block where the chilren are
        inline void setNonLeaf(uint64_t blockoffset){
            //ensure the msb is 0
            assert( !(blockoffset & 0x8000000000000000));//64 bits 100000000...0.check msb is not set
            pidx_blockoffset=blockoffset;
        }
    };
    struct block_header_info{
        //N :16 bits : number of nodes in this block. Must be <=leafSize.
        //isLeaf:8 bit inicating if all nodes in this block are leaf or not
        uint16_t nelements;
        uint8_t isLeaf;
        uint32_t header_size;//size of this header info. Ie, how long to the start of the feature part from the start of the block (it is aligned)
    };



    //a block represent all the child nodes of a parent, with its features and also information about where the child of these are in the data structure
    //a block structure is as follow: N|isLeaf|BlockParentId|p|F0...FN|C0W0 ... CNWN..
    //N :16 bits : number of nodes in this block. Must be <=branching factor k. If N<k, then the block has empty spaces since block size is fixed
    //isLeaf:16 bit inicating if all nodes in this block are leaf or not
    //BlockParentId:31: id of the parent
    //p :possible offset so that Fi is aligned
    //Fi feature of the node i. it is aligned and padding added to the end so that F(i+1) is also aligned
    //CiWi are the so called block_node_info (see structure up)
    //Ci : either if the node is leaf (msb is set to 1) or not. If not leaf, the remaining 31 bits is the block where its children are. Else, it is the index of the feature that it represent
    //Wi: float value empkoyed to know the weight of a leaf node (employed in cases of bagofwords)
    struct Block{

        Block(char *datastart, char * bsptr,uint64_t ds,uint64_t ds_wp,uint64_t tsize){
            _data_start=datastart;
            _blockstart=bsptr;
            _desc_size_bytes=ds;
            _desc_size_bytes_wp=ds_wp;
            _total_size=tsize;
        };
        Block(uint64_t ds,uint64_t ds_wp):_desc_size_bytes(ds),_desc_size_bytes_wp(ds_wp){}


        inline void setHeaderInfo(uint16_t nelements, uint8_t isLeaf,uint32_t aligment,int descSize) {
            ((block_header_info*)(_blockstart))->nelements=nelements;
            ((block_header_info*)(_blockstart))->isLeaf=isLeaf;
            ((block_header_info*)(_blockstart))->header_size=getBlockHeaderSize(aligment,descSize,nelements);
        }

        inline  uint16_t getN() { return ((block_header_info*)(_blockstart))->nelements;}
        inline uint8_t isLeaf() { return ((block_header_info*)(_blockstart))->isLeaf;}

        inline  block_node_info * getBlockNodeInfo(int i){  return (block_node_info *)(_blockstart+sizeof(block_header_info)+i*sizeof(block_node_info)); }

        inline  void setFeature(int i,const char *feature_ptr){
            assert((uint64_t)(getFeature<uint8_t>(i)+_desc_size_bytes)<=(uint64_t)(_data_start+ _total_size));
            memcpy(getFeature<uint8_t>(i),feature_ptr, _desc_size_bytes);
        }

        inline  void getFeature(int i,  char *feature_ptr){
            assert((uint64_t)(getFeature<uint8_t>(i)+_desc_size_bytes)<=(uint64_t)(_data_start+ _total_size));
            memcpy( feature_ptr, getFeature<uint8_t>(i),_desc_size_bytes );
        }

        template<typename T> inline  T*getFeature(int i){
            return (T*) (_blockstart+((block_header_info*)(_blockstart))->header_size +i*_desc_size_bytes_wp);}


        char *_blockstart;
        char *_data_start;
        uint64_t _desc_size_bytes=0;//size of the descriptor(without padding)
        uint64_t _desc_size_bytes_wp=0;//size of the descriptor(includding padding)

        uint64_t _total_size;
        //generic      calculate block data size
        static uint64_t getBlockSize(uint32_t aligment, uint32_t descSize, uint32_t npoints);
        static uint32_t getBlockHeaderSize(uint32_t aligment, uint32_t descSize, uint32_t npoints);
        inline void setBlockStart(char * start){ _blockstart=start;}

        inline void setOffset(uint64_t offset){
            assert(offset<_total_size);
            _blockstart=_data_start+offset;
        }
        inline uint32_t getOffSet(){ return uint32_t(_blockstart -_data_start);}
    };


    ///! returns block to first node
    inline Block getBlock(){ return Block(_data, _data,_params._desc_size, _params._desc_size_bytes_wp,_params._total_size);}
    inline Block getBlock(uint64_t offset){ return Block(_data, _data+offset,_params._desc_size, _params._desc_size_bytes_wp,_params._total_size);}



    //information about the cpu so that mmx,sse or avx extensions can be employed
    std::shared_ptr<cpu> cpu_info;
    std::mutex cpuinfomutex;





    template< typename Computer>
    void _knnsearch(Matrix features,int nn,Matrix indices,Matrix distances,const ParamSet&  sparams){
        int maxchecks=sparams("maxChecks");
        if (nn==1 &&maxchecks==1)
            _knnsearch_one< Computer>(features,indices,distances);
        else if (nn==2 &&maxchecks<=2)
          _knnsearch_two< Computer>(features,indices,distances);
        else{
            _knnsearch_nn< Computer>(features,nn,indices,distances,sparams);
        }
    }

    template< typename Computer>
    void _knnsearch_one(Matrix features,Matrix indices,Matrix distances){
        Computer comp;
        using DType=typename Computer::DType;//distance type
        using TData=typename Computer::TData;//data type
        comp.setParams(_params._desc_size,_params._desc_size_bytes_wp);
        std::pair<DType,uint32_t> best_dist_idx(std::numeric_limits<uint32_t>::max(),0);//minimum distance found
        block_node_info *bn_info;
        Block c_block=getBlock();
        for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
            //prepare data
            comp.startwithfeature(features.ptr<TData>(cur_feature));
            c_block.setOffset(0);            //go to first block
            do{
                //given the current block, finds the node with minimum distance
                best_dist_idx.first=std::numeric_limits<uint32_t>::max();
                for(int cur_node=0;cur_node<c_block.getN();cur_node++)
                {
                    DType d= comp.computeDist(c_block.getFeature<TData>(cur_node));
                    if (d<best_dist_idx.first) best_dist_idx=std::make_pair(d,cur_node);
                }
                bn_info=c_block.getBlockNodeInfo(best_dist_idx.second);
                //if the node is leaf get word id and weight,else go to its children
                if ( !bn_info->isleaf()) c_block.setOffset(bn_info->get());
                else {
                    *indices.ptr<int>(cur_feature)=bn_info->get();
                    *distances.ptr<DType>(cur_feature)=best_dist_idx.first;
                  }
            }while( !bn_info->isleaf());
        }
    }



    template< typename Computer>
    void _knnsearch_two(Matrix features,Matrix indices,Matrix distances){
        using DType=typename Computer::DType;//distance type
        using TData=typename Computer::TData;//data type

         Computer comp;
        comp.setParams(_params._desc_size,_params._desc_size_bytes_wp);
        std::pair<DType,uint32_t> best_dist_idx(std::numeric_limits<uint32_t>::max(),0),
                second_dist_idx(std::numeric_limits<uint32_t>::max(),0);//minimum distance found

        block_node_info *bn_info;
        Block c_block=getBlock();
        for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
            //prepare data
              std::pair<DType,uint32_t>  second_best_branch(std::numeric_limits<uint32_t>::max(),0);
            comp.startwithfeature(features.ptr<TData>(cur_feature));
            c_block.setOffset(0);
            while(!c_block.isLeaf()){
                //given the current block, finds the node with minimum distance
                second_dist_idx.first=best_dist_idx.first=std::numeric_limits<uint32_t>::max();
                for(int cur_node=0;cur_node<c_block.getN();cur_node++)
                {
                    DType d= comp.computeDist(c_block.getFeature<TData>(cur_node));
                    if (d<second_dist_idx.first) {
                        second_dist_idx=std::make_pair(d,cur_node);
                        if (second_dist_idx.first<best_dist_idx.first)
                            swap(second_dist_idx,best_dist_idx);
                    }
                }
                if (second_dist_idx.first<second_best_branch.first){
                    second_best_branch={second_dist_idx.first,c_block.getBlockNodeInfo(second_dist_idx.second)->get()};
                }

                c_block.setOffset(c_block.getBlockNodeInfo(best_dist_idx.second)->get());
            };

            if (c_block.getN()>=2){
                //now, in leaf block. Get the two best elements (if there are two)
                int*idist=indices.ptr<int>(cur_feature);
                DType *mdist=distances.ptr<DType>(cur_feature);
                mdist[0]=mdist[1]=std::numeric_limits<DType>::max();;
                for(int cur_node=0;cur_node<c_block.getN();cur_node++){
                    DType dist=comp.computeDist(c_block.getFeature<TData>(cur_node));
                    if ( dist< mdist[1]){
                        mdist[1]=dist;
                        idist[1]=cur_node;
                        if ( mdist[1]< mdist[0]){
                            std::swap(mdist[1],mdist[0] );
                            std::swap(idist[1],idist[0] );
                        }
                    }
                }

                //now, change idist indices for the real ones
                idist[0]=c_block.getBlockNodeInfo(idist[0])->get();
                idist[1]=c_block.getBlockNodeInfo(idist[1])->get();
            }

            else{//must examine this, and go down the other best branch

                distances.ptr<DType>(cur_feature)[0]= comp.computeDist(c_block.getFeature<TData>(0));
                indices.ptr<int>(cur_feature)[0]=c_block.getBlockNodeInfo(0)->get();
                //now, go down
                c_block.setOffset(second_best_branch.second);

                do{
                    //given the current block, finds the node with minimum distance
                    best_dist_idx.first=std::numeric_limits<uint32_t>::max();
                    for(int cur_node=0;cur_node<c_block.getN();cur_node++)
                    {
                        DType d= comp.computeDist(c_block.getFeature<TData>(cur_node));
                        if (d<best_dist_idx.first) best_dist_idx=std::make_pair(d,cur_node);
                    }
                    bn_info=c_block.getBlockNodeInfo(best_dist_idx.second);
                    //if the node is leaf get word id and weight,else go to its children
                    if ( !bn_info->isleaf()) c_block.setOffset(bn_info->get());
                    else {
                        indices.ptr<int>(cur_feature)[1]=bn_info->get();
                        distances.ptr<DType>(cur_feature)[1]=best_dist_idx.first;
                    }
                }while( !bn_info->isleaf());


            }
        }
    }

    template<typename DType>
    struct BranchInfo{

        BranchInfo(){}
        BranchInfo(DType  D,uint32_t Off){dist=D;offset=Off;}
        inline bool operator<(const BranchInfo & dist_index) const { return (dist < dist_index.dist);      }

        uint32_t offset;
        DType dist;
    };

    template<typename Computer>
    void _knnsearch_nn(Matrix features,int nn,Matrix indices,Matrix distances,const ParamSet&  sparams){
        using DType=typename Computer::DType;//distance type
        using TData=typename Computer::TData;//data type

        Computer comp;
        comp.setParams(_params._desc_size,_params._desc_size_bytes_wp);
        Block c_block=getBlock();
        ResultSet<DType> hres(nn,sparams.asDouble("maxDist"));
        Heap<BranchInfo<DType> > bestbranch;
        int maxChecks=sparams("maxChecks");
        for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
          //  std::cout<<"feat : "<<cur_feature<<std::endl;
            //prepare data
            comp.startwithfeature(features.ptr<TData>(cur_feature));
            hres.reset(distances.ptr<DType>(cur_feature),indices.ptr<int>(cur_feature));
            bestbranch.reset();
            bestbranch.push(BranchInfo<DType>(0,0));//insert top node
            int nchecks=0;
           std::pair<DType,int32_t> best_dist_idx;
            while( nchecks<maxChecks  && bestbranch.size()!=0){
                c_block.setOffset(bestbranch.pop().offset);
                while(!c_block.isLeaf()){
                    best_dist_idx={std::numeric_limits<DType>::max(),-1};
                    //get min for print
                    for(int cur_node=0;cur_node<c_block.getN();cur_node++)
                    {
                        DType d= comp.computeDist(c_block.getFeature<TData>(cur_node));
                        if (d<best_dist_idx.first) {
                            if ( best_dist_idx.second!=-1)  bestbranch.push( {best_dist_idx.first, uint32_t(best_dist_idx.second)});
                            best_dist_idx= { d,c_block.getBlockNodeInfo(cur_node)->get()};
                        }
                        else  bestbranch.push( { d,uint32_t( c_block.getBlockNodeInfo(cur_node)->get())});
                    }
                    c_block.setOffset(best_dist_idx.second);
                };
                //check the node elements and add them to the heap
                //std::cout<<"nodes:";
                for(int cur_node=0;cur_node<c_block.getN();cur_node++){
                    hres.push(ResultInfo<DType>(comp.computeDist(c_block.getFeature<TData>(cur_node)), c_block.getBlockNodeInfo(cur_node)->get()));
                //    std::cout<< c_block.getBlockNodeInfo(cur_node)->get()<<" ";
                }//std::cout<<std::endl;
                nchecks+=c_block.getN();

            }
            int*indx=indices.ptr<int>(cur_feature);
            DType *dists=distances.ptr<DType>(cur_feature);
            //just in case that number of elements in hres is less than nn
            for(int i=hres.size();i<nn;i++){
                indx[i]=-1;
                dists[i]=std::numeric_limits<DType>::quiet_NaN();

            }

        }
    }
};

}
}
#endif


