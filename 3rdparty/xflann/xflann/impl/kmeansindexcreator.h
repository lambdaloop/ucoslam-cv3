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
#ifndef _XFLANN_KMEANSINDEXCREATOR_H
#define _XFLANN_KMEANSINDEXCREATOR_H
#include "../xflann_exports.h"
#include <cassert>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>
#include <fstream>
#include "distances.h"
#include "kmeansindex.h"
#include <random>
#include  <algorithm>
#include <iomanip>
namespace xflann{
namespace impl{

/**This class creates the index
 */
class XFLANN_API KMeansIndexCreator
{
public:


    //create this from a set of features


     void build( const Matrix &features, const ParamSet &params);
     void convert(KMeansIndex &Voc);


private:
    struct Params{
        uint32_t k=32;
        uint32_t maxLeafSize=k;
        int L=-1;
        uint32_t nthreads=1;
        int maxIters=11;
        bool recursive=false;//one thread recursive mode
        bool verbose=false;
};

    Params _params;

    Matrix _features;
    int _descCols,_descNBytes,_descNBytes_wp;
    int _ncols_regtype;//number of words of the register type employed to calculate the distance
    int _descType,_aligment;

    struct FeatureSet{

        void setParams(uint32_t aligment,uint32_t pointsize_bytes,uint32_t npoints){
            _n=npoints;
            _psize_wp=getSizeWithPadding(pointsize_bytes, aligment);
            _psize=pointsize_bytes;
            points.resize(_psize_wp*npoints);
            memset(points.ptr(),0,_psize_wp*npoints);
            pidx.resize(npoints);
        }

        inline void setPoint(int i,void *data,int pidx_){
            memcpy( ptr<char>(i),data,_psize );
            pidx[i]=pidx_;
        }

        template<typename T>
        inline T * ptr(int i){return (T*)(points.ptr()+i*_psize_wp);}



        inline bool isValid()const{return _n!=0;}
        inline uint32_t size()const{return _n;}
        uint32_t _n=0;//number of points
        uint32_t _psize;//point size without padding
        uint32_t _psize_wp;//point size with padding
        Vector<char> points;
        std::vector<int> pidx;//indices of the points in the original dataset
    };
    struct Node{
        ~Node(){for(auto &c:children) delete c;}
        Node(){}
        Node( Node* Parent,  uint32_t Feat_idx=std::numeric_limits<uint32_t>::max()):parent(Parent) ,feat_idx(Feat_idx){
            if(Parent) depth=Parent->depth+1;
        }
        bool isLeaf()const{return children.size()==0;}
        Node* parent;
        std::vector<Node*> children;
        uint32_t feat_idx=std::numeric_limits<uint32_t>::max();
        std::vector<uint32_t> assigment;//list of elements it has
        int depth=0;//  depth of this node

        //debug
         impl::Vector<char> _feature;//feature of this node
        FeatureSet _leafData;
        FeatureSet _childData;

    };
    using NodePtr= Node*;
    //tree root
    std::shared_ptr<Node> _TreeRoot;





    std::size_t vhash(NodePtr parent)  ;
    std::size_t  hash(const vector<uint32_t> & vec)  ;
    std::size_t  hash(Matrix &mat)  ;

    std::string hash2str(std::size_t  _hval)const;

    std::size_t  hash()  ;
    std::size_t  hash(NodePtr node);

    /////////////////////////////////////////////////////
    ////// THREADS
    /////////////////////////////////////////////////////
    void  thread_consumer();

    //thread sage queue to implement producer-consumer
    class TaskAssigments
    {
    public:
        void addNewTask(  std::list<NodePtr >  items)
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            for(auto item:items){
                item2BProcessed.push(item);
                itemsInProcess.insert(item );
            }
            mlock.unlock();
            for(size_t i=0;i<items.size();i++ ) cond_.notify_one();
        }
        NodePtr getNextTask()
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            while (item2BProcessed.empty()) cond_.wait(mlock);
            auto item = item2BProcessed.front();
            item2BProcessed.pop();
            return item;
        }



        void markAsCompleted(  NodePtr  item){
            std::unique_lock<std::mutex> mlock(mutex_);
            itemsInProcess.erase(item );
            if (itemsInProcess.size()==0 && item2BProcessed.size()==0)
                cond2_.notify_all();
        }
        bool areTherePendingTasks(){
            std::unique_lock<std::mutex> mlock(mutex_);
            return !(itemsInProcess.size()==0 && item2BProcessed.size()==0);
        }
        void waitForNonMorePendingTasks(){
            std::unique_lock<std::mutex> mlock(mutex2_);
            while(areTherePendingTasks())
                cond2_.wait(mlock);
        }
        size_t size()
        {
            std::unique_lock<std::mutex> mlock(mutex_);
            size_t s=item2BProcessed.size();
            return s;
        }
    private:
        std::queue<NodePtr > item2BProcessed;
        std::mutex mutex_;
        std::condition_variable cond_;
        std::condition_variable cond2_;
        std::mutex mutex2_;
        std::set<Node*> itemsInProcess;

    };
    TaskAssigments  ParentDepth_ProcesQueue;//queue of parents to be processed
    std::vector<std::thread> _Threads;



    //------------

public:
     std::string  printInfo( );






     //////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////////////
     //////////////////////////////////////////////////////////////

     void print(float *ptr,int c){
         for(int i=0;i<c;i++)cout<< std::setprecision(10)<< ptr[i]<<" ";cout<<endl;
     }

     //ready to be threaded using producer consumer
    template<typename DistComputer,typename MComputer>
     void   createNode( NodePtr parent, bool recursive=true){
          //find initial clusters
         DistComputer dcomp;
         using TData=typename DistComputer::TData;//data type

         //determine the number of elements of the register employed (will be later employed  in the distance function)
         _ncols_regtype=_descNBytes/sizeof(TData);
         auto centers=getInitialClusterCenters<DistComputer>(parent );
         //create the chidren nodes
         for(size_t i=0;i< centers.size();i++ ) {
             auto  child_node=new Node(parent);
             _features.vptr(child_node->_feature,centers[i]);//copy the data in the feature part
              parent->children.push_back(child_node);
         }
         //initial assigment
         assignPointsToChildren<DistComputer>(parent);

         //do k means evolution to move means
         MComputer cComp;
         size_t prev_hash=0,cur_hash=1,niters=0;
         while(cur_hash!=prev_hash &&  ( (_params.maxIters==-1) || (_params.maxIters!=-1 && niters++<size_t(_params.maxIters))) )
         {

             std::swap(prev_hash,cur_hash);
             //recompute centers
             int ci=0;
             for(auto child:parent->children){
                 // if one cluster converges to an empty cluster, move the original element into that cluster
                 if (child->assigment.size()==0) child->assigment.push_back( centers[ci]);
                 //compute the mean
                 cComp(_features,child->assigment,child->_feature);

                 //child->_feature.print<float>();
                 ci++;
             }

             //recalculate  assigments
             assignPointsToChildren<DistComputer>(parent);

             //recompute hash
             cur_hash=vhash(parent);
          }
         //we can now remove the assigments of the parent
         parent->assigment.clear();
         //remove empty clusters
         for( auto it=parent->children.begin();it!=parent->children.end();){
             if ( (*it)->assigment.size()==0) it=parent->children.erase(it);
             else it++;
         }


         //should we go deeper?
         if ( ( (_params.L!=-1 && parent->depth<(_params.L-1)) || _params.L==-1))
         {
             if (recursive) {
                 for(auto child:parent->children)
                     if (child->assigment.size()> _params.maxLeafSize) //only create the children if there are enough points in the cluster
                         createNode<DistComputer,MComputer>(child);
                // else cerr<<child->assigment.size()<<" " ;
             }
             else{
                 std::list<NodePtr> children_toprocess;
                 for(auto child:parent->children){
                     if (child->assigment.size()> _params.maxLeafSize) //only create the children if there are enough points in the cluster
                         children_toprocess.push_back(child);
                 }
                 ParentDepth_ProcesQueue.addNewTask( children_toprocess);
             }
         }
     }




     template<typename DistComputer>
     void  assignPointsToChildren(NodePtr parent){
         using DType=typename DistComputer::DType;//distance type
         using TData=typename DistComputer::TData;//data type
         DistComputer dcomp;
         assert( parent->children.size()>0);
         assert( parent->assigment.size()>0);
         //remove previous assigments
         for(auto child:parent->children) child->assigment.clear();

         //for each parent assignment, decide to which child it is assigned
         for(auto fi: parent->assigment){
             auto feature=_features.ptr(fi);
             std::pair<NodePtr,float> best_assig_dist(NodePtr(),std::numeric_limits<float>::max());
             for(auto child:   parent->children ){
                 DType dist= dcomp.computeDist_((TData*)child->_feature.ptr(),(TData*)feature,_ncols_regtype);
                 if (dist< best_assig_dist.second)
                     best_assig_dist=std::make_pair(child,dist);
                 if ( best_assig_dist.second<1e-16) break;
             }
             best_assig_dist.first->assigment.push_back(fi);
         }
     }

     template<typename DistComputer>
     std::vector<uint32_t>  getInitialClusterCenters(NodePtr parent ){
         using DType=typename DistComputer::DType;//distance type
         using TData=typename DistComputer::TData;//data type
         DistComputer dcomp;
         //         std::random_device rd;
         //         std::mt19937 g(rd());
         //         g.seed(0);

         std::mt19937 g;

         std::shuffle(parent->assigment.begin(),parent->assigment.end(), g);


         //randomly reorder the points assigned to this
         //         srand(0);
         //         std::random_shuffle(parent->assigment.begin(),parent->assigment.end());
         std::vector<uint32_t>   centers;


         centers.reserve(_params.k);
         //select randomly until filling the vector
         int next_selected=0;
         while(centers.size()<_params.k){
             DType sq;
             int selected=-1;
             //we ensure no too close elements are added as centers
             do{
                 //finished, no more possible assigments
                 if (size_t(next_selected) == parent->assigment.size()) {assert(centers.size()!=0); return centers;}
                 else   selected = parent->assigment[next_selected++];
                 //find distance to nearest
                 sq=std::numeric_limits<DType>::max();
                 for (auto c: centers)
                     sq =std::min(dcomp.computeDist_((TData*)_features.ptr( selected), (TData*)_features.ptr(c),_ncols_regtype) ,sq);
             }while (sq<1e-16) ;
             centers.push_back(selected);
         }
         return centers;
     }


     //computes the mean of a set of features
     struct MC_float_generic{
         inline void operator()(xflann::Matrix &features,const std::vector<uint32_t> &indices,impl::Vector<char> &result){
             //recompute centers
             //compute new center
                              result.reset();
                              float *mean=(float*)result.ptr();
                              for(auto i:indices) {
                                  auto fptr=features.ptr<float>(i);
                                  for(int c=0;c<features.cols;c++)    mean[c] += fptr[c];
                              }
                              float inv=1./double(indices.size());
                              for(int c=0;c<features.cols;c++) mean[c]*=inv;


         }
     };




     struct MC_binary_generic{
         inline void operator()(xflann::Matrix &features,const std::vector<uint32_t> &indices,impl::Vector<char> &result){

             //determine number of bytes of the binary descriptor
             std::vector<int> sum( features.cols * 8, 0);
             for(auto i:indices)
             {
                 const unsigned char *p = features.ptr<unsigned char>(i);
                 for(int j = 0; j < features.cols; ++j, ++p)
                 {
                     if(*p & 128) ++sum[ j*8     ];
                     if(*p &  64) ++sum[ j*8 + 1 ];
                     if(*p &  32) ++sum[ j*8 + 2 ];
                     if(*p &  16) ++sum[ j*8 + 3 ];
                     if(*p &   8) ++sum[ j*8 + 4 ];
                     if(*p &   4) ++sum[ j*8 + 5 ];
                     if(*p &   2) ++sum[ j*8 + 6 ];
                     if(*p &   1) ++sum[ j*8 + 7 ];
                 }
             }

             result.reset();
             unsigned char *p =(unsigned char *) result.ptr();
             const int N2 = (int)indices.size() / 2 + indices.size() % 2;
             for(size_t i = 0; i < sum.size(); ++i)
             {
                 // set bit
                 if(sum[i] >= N2) *p |= 1 << (7 - (i % 8));
                 if(i % 8 == 7) ++p;
             }

         }
     };

     static inline uint32_t getSizeWithPadding(uint32_t size,uint32_t aligment){
          uint32_t anb=size/aligment;
          if ( size%aligment!=0) anb++;//adds padding to met aligment at the end
          return anb*aligment;
      }


     std::shared_ptr<cpu> cpu_info;

     void copyFeaturesToLeafNodes();

};
}
}
#endif
