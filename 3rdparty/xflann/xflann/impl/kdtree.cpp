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
#include "kdtree.h"
#include "../cpu.h"
#include "hash.h"
#include <iomanip>
#include <chrono>
#include <algorithm>
namespace xflann{
namespace  impl{

void KDTree::computeBoundingBox(BoundingBox& bbox, int start,int end)
{
    bbox.resize(_features.cols);
    for (int i=0; i<_features.cols; ++i)
        bbox[i].high = bbox[i].low = _features.ptr<float>(all_indices[start])[i];// (float)points_[0][i];

    for (int k=start+1; k<end; ++k) {
        for (int i=0; i<_features.cols; ++i) {
            float v=_features.ptr<float>(all_indices[k])[i];
             if (v<bbox[i].low) bbox[i].low = v;
            if (v>bbox[i].high) bbox[i].high = v;
         }
    }
}
float KDTree::computeInitialDistances(const float* vec, std::vector<float>& dists) const
{
    float distsq = 0.0;

    for (int i = 0; i < _features.cols; ++i) {
        if (vec[i] < rootBBox[i].low) {
            dists[i] = (vec[i]- rootBBox[i].low) *(vec[i]- rootBBox[i].low);// distance_.accum_dist(vec[i], root_bbox_[i].low, i);
            distsq += dists[i];
        }
        if (vec[i] > rootBBox[i].high) {
            dists[i] = (vec[i]- rootBBox[i].high)*(vec[i]- rootBBox[i].high);//distance_.accum_dist(vec[i], root_bbox_[i].high, i);
            distsq += dists[i];
        }
    }
    return distsq;
}

void KDTree::computeMinMax(uint32_t* ind, int count, int dim, float& min_elem, float& max_elem)
{
    min_elem =  _features.ptr<float>(ind[0])[dim];
    max_elem =  _features.ptr<float>(ind[0])[dim];
    for (int i=1; i<count; ++i) {
        float val = _features.ptr<float>(ind[i])[dim];
        if (val<min_elem) min_elem = val;
        if (val>max_elem) max_elem = val;
    }
}

void KDTree::middleSplit(uint32_t* ind, int count, int& index, int& cutfeat, float &cutval, const BoundingBox& bbox)
{
    // find the largest span from the approximate bounding box
    float max_span = bbox[0].high-bbox[0].low;
    cutfeat = 0;
    cutval = (bbox[0].high+bbox[0].low)/2;
    for (int i=1; i<_features.cols; ++i) {
        float span = bbox[i].high-bbox[i].low;
        if (span>max_span) {
            max_span = span;
            cutfeat = i;
            cutval = (bbox[i].high+bbox[i].low)/2;
        }
    }

    // compute exact span on the found dimension
    float min_elem, max_elem;
    computeMinMax(ind, count, cutfeat, min_elem, max_elem);
    cutval = (min_elem+max_elem)/2;
    max_span = max_elem - min_elem;

    // check if a dimension of a largest span exists
    int k = cutfeat;
    for (int i=0; i<_features.cols; ++i) {
        if (i==k) continue;
        float span = bbox[i].high-bbox[i].low;
        if (span>max_span) {
            computeMinMax(ind, count, i, min_elem, max_elem);
            span = max_elem - min_elem;
            if (span>max_span) {
                max_span = span;
                cutfeat = i;
                cutval = (min_elem+max_elem)/2;
            }
        }
    }
    int lim1, lim2;
    planeSplit(ind, count, cutfeat, cutval, lim1, lim2);

    if (lim1>count/2) index = lim1;
    else if (lim2<count/2) index = lim2;
    else index = count/2;

    assert(index > 0 && index < count);
}

void KDTree::createNode_bb(NodePtr node,int startIndex,int endIndex ,BoundingBox &bbox){

    int count=endIndex-startIndex;
    assert(startIndex<endIndex);
    assert(count>=_params._minLeafSize);
    if ( _params._minLeafSize==1 ){
        if(  count==1){//classic kdtree
            node->idx.resize(1);
            node->idx[0]= all_indices[startIndex];
            computeBoundingBox(bbox,startIndex,startIndex+1);
            return;
        }
    }
    else if (count<=2*_params._minLeafSize){//xtree
        node->idx.resize(count);
        for(int i=0;i<count;i++)
            node->idx[i]= all_indices[startIndex+i];
        computeBoundingBox(bbox,startIndex,endIndex);
        return;
    }
    node->setNodesInfo( _params._total_nodes+1,_params._total_nodes+2);
    _params._total_nodes+=2;
    assert(size_t(_params._total_nodes)<allNodes.size());


    ///SELECT THE COL (DIMENSION) ON WHICH PARTITION IS MADE
    //compute the variance of the features to  select the highest one
    mean_var_calculate(_features,startIndex,endIndex, var, mean);
    node->col_index=0;
    //select element with highest variance
    for(int i=1;i<var.size();i++)
        if (var[i]>var[node->col_index]) node->col_index=i;

    int split_index;
    //now sort all indices according to the selected value
    if (false){
        std::sort(all_indices.begin()+ startIndex ,all_indices.begin()+endIndex,[&](const uint32_t &a,const uint32_t&b){
            return _features.ptr<float>(a)[node->col_index]<_features.ptr<float>(b)[node->col_index];
        });
        split_index=(count)/2;
        node->div_val=_features.ptr<float>(all_indices[split_index])[node->col_index];
    }

    else if(true){

    node->div_val=mean[node->col_index];
    int lim1,lim2;
    planeSplit ( &all_indices[startIndex],count,node->col_index,node->div_val,lim1,lim2);


    if (lim1>count/2) split_index = lim1;
    else if (lim2<count/2) split_index = lim2;
    else split_index = count/2;

    /* If either list is empty, it means that all remaining features
         * are identical. Split in the middle to maintain a balanced tree.
         */
    if ((lim1==count)||(lim2==0)) split_index = count/2;

    //create partitions with at least minLeafSize elements
    if (_params._minLeafSize!=1)
        if ( split_index< _params._minLeafSize || count-split_index<_params._minLeafSize) {
            std::sort(all_indices.begin()+ startIndex ,all_indices.begin()+endIndex,[&](const uint32_t &a,const uint32_t&b){
                return _features.ptr<float>(a)[node->col_index]<_features.ptr<float>(b)[node->col_index];
            });
            split_index=count/2;
             node->div_val=_features.ptr<float>(all_indices[startIndex+split_index])[node->col_index];
        }

    }
    else {
         int cutfeat;
        float cutval;
        middleSplit(&all_indices[startIndex], count, split_index, cutfeat, cutval, bbox);
        node->div_val = cutval;
        node->col_index=cutfeat;

        //create partitions with at least minLeafSize elements
        if (_params._minLeafSize!=1)
            if ( split_index< _params._minLeafSize || count-split_index<_params._minLeafSize) {
                std::sort(all_indices.begin()+ startIndex ,all_indices.begin()+endIndex,[&](const uint32_t &a,const uint32_t&b){
                    return _features.ptr<float>(a)[node->col_index]<_features.ptr<float>(b)[node->col_index];
                });
                split_index=count/2;
                 node->div_val=_features.ptr<float>(all_indices[startIndex+split_index])[node->col_index];
            }
    }
  //  node->div_val=_features.ptr<float>(all_indices[split_index])[node->col_index];

    BoundingBox left_bbox(bbox);
    left_bbox[node->col_index].high = node->div_val;
    createNode_bb(left(node),startIndex,startIndex+split_index,left_bbox);
    assert(left_bbox[node->col_index].high <=node->div_val);
    BoundingBox right_bbox(bbox);
    right_bbox[node->col_index].low = node->div_val;
    createNode_bb(right(node),startIndex+split_index,endIndex,right_bbox);

    node->divlow = left_bbox[node->col_index].high;
    node->divhigh = right_bbox[node->col_index].low;
    assert(node->divlow<=node->divhigh);

    for (int i=0; i<_features.cols; ++i) {
        bbox[i].low = std::min(left_bbox[i].low, right_bbox[i].low);
        bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
    }

}
/**
 *  Subdivide the list of points by a plane perpendicular on axe corresponding
 *  to the 'cutfeat' dimension at 'cutval' position.
 *
 *  On return:
 *  dataset[ind[0..lim1-1]][cutfeat]<cutval
 *  dataset[ind[lim1..lim2-1]][cutfeat]==cutval
 *  dataset[ind[lim2..count]][cutfeat]>cutval
 */
void KDTree::planeSplit(uint32_t* ind, int count, int cutfeat, float cutval, int& lim1, int& lim2)
{
    /* Move vector indices for left subtree to front of list. */
    int left = 0;
    int right = count-1;
    for (;; ) {
        while (left<=right && _features.ptr<float>(ind[left])[cutfeat]<cutval) ++left;
        while (left<=right &&  _features.ptr<float>(ind[right])[cutfeat]>=cutval) --right;
        if (left>right) break;
        std::swap(ind[left], ind[right]); ++left; --right;
    }
    lim1 = left;
    right = count-1;
    for (;; ) {
        while (left<=right &&  _features.ptr<float>(ind[left])[cutfeat]<=cutval) ++left;
        while (left<=right &&  _features.ptr<float>(ind[right])[cutfeat]>cutval) --right;
        if (left>right) break;
        std::swap(ind[left], ind[right]); ++left; --right;
    }
    lim2 = left;
}

//computes the mean of a set of features
void KDTree::mean_var_calculate(xflann::Matrix &features, int startindex, int endIndex, impl::Vector<float> &var_, impl::Vector<float> &_mean){
    var_.resize(features.cols);
    _mean.resize(features.cols);

    //recompute centers
    //compute new center
    vector<float>  sum2(features.cols);
    memset(_mean.ptr(),0,sizeof(float)*features.cols);
    memset(&sum2[0],0,sizeof(float)*features.cols);
    //finish when at least MAX_ELEM_MEAN elements computed
    int cnt=min(MAX_ELEM_MEAN,endIndex-startindex );
    for(int i=startindex;i<startindex+cnt;i++) {
        auto fptr=features.ptr<float>(all_indices[i]);
        for(int c=0;c<features.cols;c++)    {
            _mean[c] += fptr[c];
             sum2[c] += fptr[c]*fptr[c];
        }
    }
    float *var=var_.ptr ();
    float invcnt=1./float(cnt);
    for(int c=0;c<features.cols;c++) {
        _mean[c]*=invcnt;
         var[c]= sum2[c]*invcnt - _mean[c]*_mean[c];
    }
}
void KDTree::build(Matrix features, const ParamSet &params){
    //select distance function
    if( features.type()!=XFLANN_32F)
        std::runtime_error("Descriptors must be float  CV_32FC1");

    allNodes.resize(features.rows*2);
     _params._total_nodes=0;
    _params._verbose=params("verbose");
    _params._store_data=params("store_data");
    _params._minLeafSize=max(1,params("minLeafSize"));
    _median_split=params("median_split");
    _features=features;
    _params._desc_size=_features.colSize();
    if (_params._store_data){
        //use padding to speed up search
        _stored_features.resize(_features.size());
        memcpy(_stored_features.ptr(),_features.ptr<char>(0),_features.size());
        //redirect _features to this
        _features.data=_stored_features.ptr();
    }

    //Create root and assign all items
    all_indices.resize(features.rows);
    for(int i=0;i<features.rows;i++)  all_indices[i]=i;

    computeBoundingBox(rootBBox,0,all_indices.size());

    createNode_bb(&allNodes[0],0,all_indices.size(),rootBBox );


    }
void KDTree::search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  sparams,std::shared_ptr<cpu> Tcpu){
    if (features.type()!=XFLANN_32F   )
        throw std::runtime_error("KDTreeIndex::knnsearch  features must be floating point");

    if (features.rows!=indices.rows ||  distances.rows!=features.rows)
        throw std::runtime_error("KDTreeIndex::knnsearch indices and distances must be already allocated with the same size as the number of features");
    if ( distances.cols!=nn || indices.cols!=nn)
        throw std::runtime_error("KDTreeIndex::knnsearch indices and distances number of cols must == nn");

    //get host info to decide the version to execute
    std::shared_ptr<cpu> use_cpu_info;
    {
        std::unique_lock<std::mutex> lock(cpu_info_mutex);
        if (!Tcpu){//if cpu info not passed, use our own
            if (!cpu_info) cpu_info=std::make_shared<cpu>();
            use_cpu_info=cpu_info;
        }
        else use_cpu_info=Tcpu;
    }




    _params._aligment=std::min(_features.aligment(),features.aligment());

    if( use_cpu_info->isSafeAVX() && _params._aligment%32==0  && _params._desc_size>=32){ //AVX version
        if (  _params._desc_size==256)  _knnsearch<distances::L2_avx_8w>(features,nn,indices,distances,sparams);//specific for surf 256 bytes
        else if (  _params._desc_size==512)  _knnsearch<distances::L2_avx_16w>(features,nn,indices,distances,sparams);//specific for surf 512 bytes
        else   _knnsearch<distances::L2_avx_generic>(features,nn,indices,distances,sparams);//any other
    }
    else if( use_cpu_info->isSafeSSE() && _params._aligment%16==0  &&_params._desc_size>=16){//SSE version
        if (  _params._desc_size==256)  _knnsearch<distances::L2_sse3_16w>(features,nn,indices,distances,sparams);//specific for surf 256 bytes
        else   _knnsearch<distances::L2_sse3_generic>(features,nn,indices,distances,sparams);//any other
    }
    else if (_features.cols==2)_knnsearch<distances::L2_generic_2>(features,nn,indices,distances,sparams);
    else if (_features.cols==3)_knnsearch<distances::L2_generic_3>(features,nn,indices,distances,sparams);
    else _knnsearch<distances::L2_generic>(features,nn,indices,distances,sparams);


}

uint64_t KDTree::Node::hash()const{
Hash sig;
sig.add(div_val);
sig.add(col_index);
sig.add(_ileft);
sig.add(_iright);
sig.add(idx.begin(),idx.end());
return sig;
}

void KDTree::Node::toStream(std::ostream &str) const{


    str.write((char*)&div_val,sizeof(div_val));
    str.write((char*)&col_index,sizeof(col_index));
    str.write((char*)&_ileft,sizeof(_ileft));
    str.write((char*)&_iright,sizeof(_iright));
     uint32_t s=idx.size();
    str.write((char*)&s,sizeof(s));
    str.write((char*)&idx[0],sizeof(idx[0])*s);

}

void KDTree::Node::fromStream(std::istream &str) {
    str.read((char*)&div_val,sizeof(div_val));
    str.read((char*)&col_index,sizeof(col_index));
    str.read((char*)&_ileft,sizeof(_ileft));
    str.read((char*)&_iright,sizeof(_iright));
     uint32_t s;
    str.read((char*)&s,sizeof(s));
    idx.resize(s);
    str.read((char*)&idx[0],sizeof(idx[0])*s);

}

uint64_t KDTree::hash()const{
Hash sig;
sig.add(_params);
for(const auto &n:allNodes)
    sig+=n.hash();
for(int i=0;i<_stored_features.size();i++)
        sig+=_stored_features[i];
return sig;
}

void KDTree::toStream(std::ostream &str)const{
    str.write((char*)&_params,sizeof(_params));
    for(int i=0;i<_params._total_nodes;i++)
        allNodes[i].toStream(str);
    _stored_features.toStream(str);
}
void KDTree::fromStream(std::istream &str){
    str.read((char*)&_params,sizeof(_params));
    allNodes.resize(_params._total_nodes);
    for(int i=0;i<_params._total_nodes;i++)
        allNodes[i].fromStream(str);
    _stored_features.fromStream(str);
}

}

}
