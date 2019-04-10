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
#ifndef _XFLANN_KDTREE_H
#define _XFLANN_KDTREE_H
#include <vector>
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
class XFLANN_API KDTree:public impl::IndexImpl
{
public:
    std::string getName()const{return "kdtree";}
    void build ( Matrix features,   const ParamSet &params);
    bool storeFeatures()const {return false;}
    void search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet &search_params , std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>());
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
    uint32_t size()const {return _features.rows;}
    uint64_t hash()const;
private:
    // Params _params;
    struct Params{
    int   _verbose,_aligment, _minLeafSize;
    int  _desc_size;
    int _store_data;
    int _total_nodes;

    };
    Params _params;
    int _median_split=0;
    Vector<char> _stored_features;
    Matrix _features;
    Vector<float> var,mean ;//create the vector

    int MAX_ELEM_MEAN=100;//maximum number of elements considered for the mean



    struct Node{
        Node(){}
        inline bool isLeaf()const{return _ileft==-1 && _iright==-1;}
        inline void setNodesInfo(uint32_t l,uint32_t r){_ileft=l; _iright=r;}
        float div_val;
        uint16_t col_index;//column index of the feature vector
        vector<int> idx;
        float divhigh,divlow;
        int64_t _ileft=-1,_iright=-1;//children
        void toStream(std::ostream &str) const;
        void fromStream(std::istream &str) ;
        uint64_t hash() const;
     };


    //access to left and rigth nodes
    inline Node*left(Node *parent){return &allNodes[parent->_ileft];}
    inline Node*right(Node *parent){return &allNodes[parent->_iright];}


    using NodePtr= Node *;
    //all nodes used
    vector<Node> allNodes;//we do save here all required nodes, no need to do dynamic allocation

    //temporal used during creation of the tree
    std::vector<uint32_t> all_indices;



    //computes the mean of a set of features
    void mean_var_calculate(xflann::Matrix &features,int startindex,int endIndex, impl::Vector<float> &var_,impl::Vector<float> &_mean);

    struct Interval
      {
          float low, high;

      };

      typedef std::vector<Interval> BoundingBox;
      BoundingBox rootBBox;



      void computeBoundingBox(BoundingBox& bbox, int start,int end) ;
      //recursively creates the nodes
       void  createNode_bb(NodePtr node, int startIndex, int endIndex, BoundingBox &bbox);

      void  planeSplit(uint32_t *ind, int count, int cutfeat, float cutval, int& lim1, int& lim2);
      void middleSplit(uint32_t *ind, int count, int& index, int& cutfeat, float& cutval, const BoundingBox& bbox);
      void computeMinMax(uint32_t *ind, int count, int dim, float& min_elem, float& max_elem);

    //
    std::shared_ptr<cpu>  cpu_info;
    std::mutex cpu_info_mutex;


    template<typename Computer,typename RSET>
    void searchExactLevel(NodePtr node,float *feat,RSET  &res,int nfeatdata,Computer& comp,float mindistsq,std::vector<float>& dists,float epsError )
    {
        using TData=typename Computer::TData;//data type
        if (node->isLeaf()){
            for(size_t i=0;i<node->idx.size();i++){
                res.push( { comp.computeDist_((TData*)feat,_features.ptr<TData>(node->idx[i]),nfeatdata),uint32_t(node->idx[i])} );
            }
        }
        else{

            int idx = node->col_index;
            float val = feat[idx];
            float diff1 = val - node->divlow;
            float diff2 = val - node->divhigh;

            NodePtr bestChild;
            NodePtr otherChild;
            float cut_dist;
            if ((diff1+diff2)<0) {
                bestChild = left(node);;
                otherChild = right(node);
                cut_dist = diff2*diff2 ;
            }
            else {
                bestChild = right(node);
                otherChild = left(node);
                cut_dist =  diff1*diff1;
            }
            /* Call recursively to search next level down. */
            searchExactLevel<Computer> (bestChild,feat,res,nfeatdata,comp, mindistsq, dists ,epsError );

            float dst = dists[idx];
            mindistsq = mindistsq + cut_dist - dst;
            dists[idx] = cut_dist;
            if (mindistsq*epsError <=res.worstDist())
                 searchExactLevel<Computer>  (otherChild,feat,res,nfeatdata,comp, mindistsq, dists,epsError );
            dists[idx] = dst;
        }
    }
    float computeInitialDistances(const float* vec, std::vector<float>& dists) const;


    template<typename Computer>
    void _knnsearch_exact(Matrix features, Matrix indices, Matrix distances,int nn,  const ParamSet&  search_params){
        static_assert(std::is_same<typename Computer::DType,float>::value,"Only valid for float");
        using DType=typename Computer::DType;//distance type
        Computer comp;
        ResultSet<DType> hres(nn,search_params.asDouble("maxDist"));
        int nfeattdata= comp.getNFeatOps(_params._desc_size);
        std::vector<float> dists(features.cols);

         for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
            hres.reset(distances.ptr<DType>(cur_feature) , indices.ptr<int>(cur_feature) );
            memset(&dists[0],0,sizeof(float)*dists.size());
            float distsq = computeInitialDistances(features.ptr<float>(cur_feature), dists);
            searchExactLevel<Computer>(&allNodes[0],features.ptr<float>(cur_feature),hres,nfeattdata,comp,distsq,dists,1);
            //set to -1 the incompleted elements
            //possibly not found as many as desired.
            for(int i=hres.size();i<nn;i++){
                indices.ptr<int>(cur_feature)[i]=-1;
                distances.ptr<DType>(cur_feature)[i]=std::numeric_limits<DType>::quiet_NaN();
            }
         }

    }


    template<typename DType>
    struct BranchInfo{
        BranchInfo(){}
        BranchInfo(DType Dist,NodePtr ptr):dist(Dist),nptr(ptr){}
        inline BranchInfo&operator=(DType d){dist=d;return *this;}
        inline bool operator<(const BranchInfo & dist_index) const { return (dist < dist_index.dist);      }
        inline bool operator>(const BranchInfo & dist_index) const { return (dist > dist_index.dist);      }
        DType dist;
        NodePtr  nptr;
    };

    template<typename DType>
    inline NodePtr evaluate(NodePtr node,float*feat, BranchInfo<DType> &bi){
        float d1=feat[node->col_index]-node->div_val;
        bi.dist=d1*d1;
        if (d1<0 ){bi.nptr=right(node);  return left(node);}
        else { bi.nptr=left(node); return  (right(node));}
    }


    template<typename Computer>
    void _knnsearch_nn(Matrix features, Matrix indices, Matrix distances,int nn,const ParamSet&  search_params){
        using DType=typename Computer::DType;//distance type
        using TData=typename Computer::TData;//data type
        Computer comp;

        ResultSet<DType> hres(nn,search_params.asDouble("maxDist"));
        Heap<BranchInfo<DType>> bestbranch;
        int nfeattdata= comp.getNFeatOps(_params._desc_size);
        int maxchecks=std::max(search_params("maxChecks"),nn);
        for(int cur_feature=0;cur_feature<features.rows;cur_feature++){
            bestbranch.reset();
            hres.reset(distances.ptr<DType>(cur_feature),indices.ptr<int>(cur_feature));
            TData *feat=features.ptr<TData>(cur_feature);
            //go down until first leaf node

            BranchInfo<DType> other_branch;
            NodePtr node=evaluate(&allNodes[0],(float*)feat,other_branch);
            bestbranch.push(other_branch);
            int nchecks=0 ;
            while(  nchecks<maxchecks  && !bestbranch.empty()){
                while(!node->isLeaf()){
                    node=evaluate(node,(float*)feat,other_branch);
                    bestbranch.push(other_branch);
                }
                nchecks+=node->idx.size();
                for(size_t i=0;i<node->idx.size();i++)
                    hres.push(ResultInfo<DType>(comp.computeDist_((TData*)feat,_features.ptr<TData>(node->idx[i]),nfeattdata),node->idx[i]) );
                node= bestbranch.pop().nptr;
            }
            //possibly not found as many as desired.
            for(int i=hres.size();i<nn;i++){
                indices.ptr<int>(cur_feature)[i]=-1;
                distances.ptr<DType>(cur_feature)[i]=std::numeric_limits<DType>::quiet_NaN();
            }
        }
    }


    template<typename Computer>
    void _knnsearch(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  search_params){
        static_assert(std::is_same<typename Computer::DType,float>::value,"Only valid for float");
        int maxChecks=search_params("maxChecks");
        if ( maxChecks<0)_knnsearch_exact<Computer>(features,indices,distances,nn,search_params);
        else   _knnsearch_nn<Computer>( features,indices,distances,nn,search_params);
    }

};

}
}
#endif
