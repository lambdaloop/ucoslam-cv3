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
#include "index.h"
#include "impl/kdtree.h"
#include "impl/kmeansindex.h"
#include "impl/linear.h"
#include "impl/kmeansindexcreator.h"
#include <thread>
namespace xflann{


using namespace impl;
Index::Index(){}

Index::Index(Matrix  features, const ParamSet &params){
    build(features,params);
}

void Index::build(Matrix features, const ParamSet &params )
{

    _impl.reset();
    if(features.rows<=0) return;

  if (params.type=="kdtree"){
        if (features.type()!=XFLANN_32F  )  throw std::runtime_error("Index::build can create a xtree. Features must be float or binary");
        _impl=std::make_shared<KDTree>();
        ((KDTree*)(_impl.get()))->build( features,params);
    }
    else if ( params.type=="kmeans"){
        KMeansIndexCreator creator;
            creator.build(features,   params);
            _impl=std::make_shared<KMeansIndex>();
            creator.convert( *(reinterpret_cast<KMeansIndex*>(_impl.get())));
    }
    else if ( params.type=="linear"){
        _impl=std::make_shared<Linear>();
        ((Linear*)(_impl.get()))->build(features,params);
    }
    else{
        throw std::runtime_error ("Index::build invalid type of index");
    }
}

void  Index::clear()
{
    _impl.reset();
}


bool Index::search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  search_params,std::shared_ptr<cpu> Tcpu){
  return  _search(features,nn,indices,distances,search_params,Tcpu);
}
bool Index::_search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu){

    if (_impl==0) {
        cerr<< "Index::_search Could not run search because index not created"<<endl;
        return false;
    }


    if (search_params("threads")>1)
        parallel_search(features,nn,indices,distances,search_params);
    else _impl->search(features,nn,indices,distances,search_params,Tcpu);
    //  sorting if required
    if (search_params("sorted")==1) {
        switch(distances.type()){
        case XFLANN_32F:
            sort<float>(indices,distances);
            break;
        case XFLANN_32S:
            sort<int32_t>(indices,distances);
            break;
        default:
            throw std::runtime_error("Index::search undefined search distance type");
        };
    }
    return true;
}



void Index::parallel_search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  search_params,std::shared_ptr<cpu> Tcpu){
    //divide in as many threads as indicated
    int nthreads=search_params("threads");
    int nelements_per_thread=features.rows/nthreads;//number of items per thread
    int left_over=features.rows%nthreads; //how many extra elements are there?

    vector<std::thread> threads;
    int curStart=0;
    for(int i=0;i<nthreads;i++){
        int nitems= nelements_per_thread+ (left_over?1:0);
        if (left_over>0)left_over--;
        threads.push_back(std::thread([&](){_impl->search(features.subMatrix(curStart,curStart+nitems),nn,indices.subMatrix(curStart,curStart+nitems),distances.subMatrix(curStart,curStart+nitems),search_params,Tcpu);}));
        curStart+=nitems;
    }
    //wait for threas to finish
    for(auto &t:threads) t.join();
}


/**
 * @brief saveToFile Saves this index from a file
 * @param filepath output file path
 */
void Index::saveToFile(const std::string &filepath){
    if (!_impl->storeFeatures())
        throw std::runtime_error("The index does not store the features. Saving the index will be useless since features might move from its current memory location");
    ofstream file(filepath);
    toStream(file);
}

/**
 * @brief readFromFile reads the index from a file generated with saveToFile
 * @param filepath input file path
 */
void Index::readFromFile(const std::string &filepath){
    ifstream file(filepath);
    if (!file) throw std::runtime_error("Could not open file for reading:"+filepath);
    fromStream(file);
}

/**
 * @brief saveToStream saves the index to a binary stream
 * @param str
 */
void Index::toStream(std::ostream &str)const{
    if (!_impl->storeFeatures()) throw std::runtime_error("The index does not store the features. Saving the index will be useless since features might move from its current memory location");
    //save a signature number
    uint64_t sig=12837333433;
    str.write((char*)&sig,sizeof(sig));
    //now, save the signature of the particular implementation
    std::hash<std::string> hash_fn;
    uint64_t str_hash = hash_fn( _impl->getName());
    str.write((char*)&str_hash ,sizeof(str_hash ));
    _impl->toStream(str);
}

/**
 * @brief readFromStream reads the index from a binary stream
 * @param str
 */
void Index::fromStream(std::istream &str){
    //read  signature number
    uint64_t sig;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=12837333433) throw std::runtime_error("Invalid signature in stream");
    //now, read the hash
    uint64_t impl_hash;
    str.read((char*)&impl_hash,sizeof(sig));
    std::hash<std::string> hash_fn;
    if ( hash_fn("kdtree")==impl_hash)
        _impl=std::make_shared<KDTree>();
    else if( hash_fn("kmeans")==impl_hash)
        _impl=std::make_shared<KMeansIndex>();
    else if( hash_fn("linear")==impl_hash)
        _impl=std::make_shared<Linear>();
    else throw std::runtime_error("Could not determine the type of implementation from the hash readed");

    //finally, read
    _impl->fromStream(str);
}

uint64_t Index::hash()const{
    return _impl->hash();
}

#if (defined XFLANN_OPENCV )
    //specialized version for opencv
bool Index::search(const cv::Mat &features, int nn,  cv::Mat &indices,  cv::Mat &distances, const ParamSet& search_params,std::shared_ptr<cpu> Tcpu){
    indices.create(features.rows,nn,CV_32S);
    if ( features.type()==CV_8U)
        distances.create(features.rows,nn,CV_32S);
    else
        distances.create(features.rows,nn,CV_32F);
    return _search(features,nn,indices,distances,search_params,Tcpu);

}

#endif
}
