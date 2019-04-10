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
#include "kmeansindexcreator.h"
#ifdef USE_OPENMP
#include <omp.h>
#endif
#include <iostream>
#include <fstream>
#include <set>

#include <cmath>
#include "../cpu.h"
using namespace std;
namespace xflann{

namespace impl{

void KMeansIndexCreator::build(const  Matrix  &features,  const ParamSet &params)
{
    _params.k=params("k");
    _params.maxLeafSize=_params.k;//params("maxLeafSize");
    _params.L=params("L");
    _params.nthreads=params("nthreads");
    _params.maxIters=params("maxIters");
    _params.recursive=params("recursive");
    _params.verbose=params("verbose");


    auto nonan=[](const Matrix &f){
        if (f.type()== XFLANN_32F){

            for(int r=0;r<f.rows;r++){
                const float *ptr=f.ptr<float>(r);
                for(int c=0;c<f.cols;c++)
                    if (std::isnan(ptr[c]))return false;
            }

        }
        return true;
    };

    assert(nonan(features));
    assert(features.cols>0);
    _features=features;


    //select the funciton
    _descCols=features.cols;
    _descType=features.type();
    _descNBytes=features.colSize();
    //  _params.nthreads=std::min(maxthreads,_params.nthreads);



    //select distance function
    if(!(_descType==XFLANN_8U|| _descType==XFLANN_32F))
        std::runtime_error("Descriptors must be binary CV_8UC1 or float  CV_32FC1");



    //Create root and assign all items
    _TreeRoot=std::make_shared<Node>();
    _TreeRoot->assigment.resize(_features.rows);

    for(int i=0;i<_features.rows;i++) _TreeRoot->assigment[i]=i;


    cpu cpu_info;
    cpu_info.detect_host();

    int aligment=features.aligment();

    //non recursive mode not working properly
    bool doRecursive=true;//_params.recursive || _params.nthreads==1;

    if ( _descType==XFLANN_32F ){
        if (cpu_info.HW_AVX && aligment%32==0 && _descCols>=8){
            if ( _descCols==64)//surf 64 floats
                createNode<distances::L2_avx_8w,MC_float_generic>( _TreeRoot.get(),doRecursive);
            else if(_descCols==128)
                createNode<distances::L2_avx_16w,MC_float_generic>( _TreeRoot.get(),doRecursive);
            else createNode<distances::L2_avx_generic,MC_float_generic>( _TreeRoot.get(),doRecursive);
        }
        else if (  cpu_info.HW_SSE && aligment%16==0 && _descCols>=4) {
            if (_descCols==64)//surf  64 floats
                createNode<distances::L2_sse3_16w,MC_float_generic>( _TreeRoot.get(),doRecursive);
            else
                createNode<distances::L2_sse3_generic,MC_float_generic>( _TreeRoot.get(),doRecursive);
        }
        else
            createNode<distances::L2_generic,MC_float_generic>( _TreeRoot.get(),doRecursive);

    }
    else if (_descType==XFLANN_8U ){
        if (cpu_info.HW_x64 && aligment%16==0){
            if (_descCols==32)   createNode<distances::Hamming_x64_32bytes,MC_binary_generic>(_TreeRoot.get(),doRecursive);
            else createNode<distances::Hamming_x64,MC_binary_generic>(_TreeRoot.get(),doRecursive);
        }
        else if (aligment%4==0)
            createNode<distances::Hamming_x32,MC_binary_generic>(_TreeRoot.get(),doRecursive);
        else
            createNode<distances::Hamming_unaligned,MC_binary_generic>(_TreeRoot.get(),doRecursive);
    }

    //parallel mode
    if (!doRecursive){
        //now, create threads
        for(uint32_t i=0;i<_params.nthreads;i++)  _Threads.push_back(std::thread(&KMeansIndexCreator::thread_consumer,this));
        //wait for threads to stop
        ParentDepth_ProcesQueue.waitForNonMorePendingTasks();
         //add exit info for the threads
        for(uint32_t i=0;i<_params.nthreads;i++) ParentDepth_ProcesQueue.addNewTask( {NodePtr()});
        //wait for them
        for(std::thread &th:_Threads) th.join();
    }

     copyFeaturesToLeafNodes();


}
string KMeansIndexCreator::printInfo( ){
    //    stringstream str;
    auto &str=cout;
    std::list<NodePtr> queue;
    queue.push_back(_TreeRoot.get());
    while(queue.size()!=0){
        auto node=queue.front();
        queue.pop_front();
        if (node->isLeaf()){
            //str<<node->parent->node_id<<" - "<< node->node_id<<":";
            for(auto a:node->assigment) str<<a<<" ";
            str<<endl;
        }
        else {
            for(auto child:node->children){
            //    str<< node->node_id<<" - "<<child->node_id<<endl;
                queue.push_back(child);
            }
        }
    }
    return "";
    //return str.str();
}



void KMeansIndexCreator::thread_consumer( ){
//    auto parent=ParentDepth_ProcesQueue.getNextTask();//wait
//    while(parent )//non empty task
//    {
//        cout<<"node:"<<parent->node_id<<endl;
//        createNode(parent ,false);
//        ParentDepth_ProcesQueue.markAsCompleted(parent);
//        parent=ParentDepth_ProcesQueue.getNextTask();//wait
//     }
}






std::string KMeansIndexCreator::hash2str(std::size_t _hval )const{
    std::string alpha="AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz0123456789";
    uint8_t *ptr=(uint8_t*)&_hval;
    std::string res;res.resize(sizeof(_hval));
    for(size_t i=0;i<sizeof(_hval);i++)
        res[i]=alpha[ ptr[i]%alpha.size()];
    return res;
}


std::size_t KMeansIndexCreator::vhash(NodePtr parent)  {
    std::size_t seed = 0;
    for(auto child:  parent->children )
        for(auto id:child->assigment)
            seed ^= id + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}

void KMeansIndexCreator::copyFeaturesToLeafNodes(){

    _aligment=32;

    _descNBytes_wp=getSizeWithPadding(_aligment,_descNBytes);
    //go findind dhilren and add the features
    list<NodePtr> queue;
    queue.push_back( _TreeRoot.get());
    while(queue.size()!=0){
        NodePtr node=queue.front();
        queue.pop_front();
        if (node->isLeaf() ){

            //how many bytes do we need (we want it aligned at the end)
            node->_leafData.setParams(_aligment,_features.colSize(),node->assigment.size());
            for(size_t i=0;i< node->assigment.size();i++){
                int pidx=node->assigment[i];
                node->_leafData.setPoint(i,_features.ptr(pidx),pidx);
            }
        }
        else{
            node->_childData.setParams(_aligment,_features.colSize(),node->children.size());
            int idx=0;
            for(auto child:node->children)  {
                node->_childData.setPoint(idx++,child->_feature.ptr(),-1);
                queue.push_back(child);
            }
        }
    }


}


void KMeansIndexCreator::convert(KMeansIndex &Voc ){

    vector<bool> allfeatures(_features.rows,false);
    int aligment=8;
    if (_descType==XFLANN_32F) aligment=32;

    std::map<Node*,uint64_t>  node_offset;
    uint64_t total_size=0;
    //compute the offset of each block
    list<NodePtr> queue;
    queue.push_back(_TreeRoot.get());
    while(queue.size()!=0){
        auto node=queue.front();
        queue.pop_front();
        node_offset[node ]=total_size;
        int nelements=0;
        if (node->isLeaf()) nelements=node->assigment.size();
        else  {
            for(auto child:node->children) queue.push_back(child);
            nelements=node->children.size();
        }
        total_size+=KMeansIndex::Block::getBlockSize(aligment,_descNBytes,nelements);
    }

    //set params and alloc

    Voc.setParams(aligment,_descType,_descNBytes,total_size ,_features.rows);
    //start writing

    queue.push_back(_TreeRoot.get());
    while(queue.size()!=0){
        auto node=queue.front();
        queue.pop_front();
        assert( node_offset.count(node));

        //access to the block in the specified position previously computed
        KMeansIndex::Block block=Voc.getBlock( node_offset[node ]);

        //if leaf, write the assigments(rows of the feature matrix)
        if (node->isLeaf()) {
            //initialize header info
            block.setHeaderInfo(node->assigment.size(), 1,aligment,_descNBytes);
            int idx=0;
            for(auto a:node->assigment) {
                assert(!allfeatures[a]);allfeatures[a]=true;
                block.setFeature(idx,_features.ptr<char>(a));
                block.getBlockNodeInfo(idx)->setLeaf(a);//set the row of the feature in the feature matrix
                idx++;
            }
        }
        else {
            //initialize header info
            block.setHeaderInfo(node->children.size(), 0,aligment,_descNBytes);
            int idx=0;
            //write the children features and set the offset to the children nodes
            for(auto child:node->children) {
                queue.push_back(child);
                block.setFeature(idx,child->_feature.ptr());
                block.getBlockNodeInfo(idx)->setNonLeaf(  node_offset[child ] );//points to the block where the node info is
                idx++;
            }
        }
    }
     assert((uint64_t)Voc.getBlock(0).getFeature<float>(0)%aligment==0);
}
std::size_t KMeansIndexCreator::hash(const vector<uint32_t> & vec)  {
    std::size_t seed = vec.size();
    for( auto v:vec )
        seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}

std::size_t  KMeansIndexCreator::hash(Matrix &mat)  {
    assert(mat.type()==XFLANN_32F);
    std::size_t seed = mat.rows*mat.cols;
    for(int r=0;r<mat.rows;r++){
        auto ptr=mat.ptr<int>(r);
        for(int c=0;c<mat.cols;c++){
            seed ^= ptr[c] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
    }
    return seed;
}

std::size_t  KMeansIndexCreator::hash(){
    std::queue< NodePtr> nodes;
    size_t seed=0;
    nodes.push(_TreeRoot.get());
    while(!nodes.empty()){
        auto node=nodes.back();
        nodes.pop();
        for(auto child:node->children){
            nodes.push(child);
            seed ^= hash(child) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
    }
    return seed;
}
std::size_t  KMeansIndexCreator::hash(NodePtr node){
    std::size_t seed =  0;
    if(node->isLeaf()) {
        for( auto v:node->assigment )
            seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    else  seed=1;
    return seed;

}


}

}
