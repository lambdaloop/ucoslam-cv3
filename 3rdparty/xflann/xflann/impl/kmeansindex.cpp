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
#include "kmeansindex.h"
#include <fstream>
#include <cstring>
#include "distances.h"
#include "../cpu.h"
namespace xflann{
namespace impl{

KMeansIndex::~KMeansIndex(){
    if (_data!=0)
        AlignedFree( _data);
    _data=0;
}
uint32_t KMeansIndex::Block::getBlockHeaderSize(uint32_t aligment, uint32_t descSize, uint32_t npoints){


    //return the number of bytes required to make size aligned
    auto getSizeWithPadding=[](uint32_t size,uint32_t aligment){
        uint32_t anb=size/aligment;
        if ( size%aligment!=0) anb++;//adds padding to met aligment at the end
        return anb*aligment;
    };

    //calculate block data size
    //size of block header :It should end in a aligned position
    uint32_t header_size=getSizeWithPadding ( sizeof(block_header_info)+sizeof(block_node_info)*npoints,aligment);
    assert(header_size%aligment==0);
    return header_size;
}
//returns block size
 uint64_t KMeansIndex::Block::getBlockSize(uint32_t aligment, uint32_t descSize, uint32_t npoints){


    //return the number of bytes required to make size aligned
    auto getSizeWithPadding=[](uint32_t size,uint32_t aligment){
        uint32_t anb=size/aligment;
        if ( size%aligment!=0) anb++;//adds padding to met aligment at the end
        return anb*aligment;
    };

    //calculate block data size
    //size of block header :It should end in a aligned position
    uint32_t header_size=getSizeWithPadding ( sizeof(block_header_info)+sizeof(block_node_info)*npoints,aligment);
    assert(header_size%aligment==0);
    uint32_t one_desc_size=getSizeWithPadding(descSize,aligment);
    assert(one_desc_size%aligment==0);
    uint32_t block_size= header_size+ npoints*one_desc_size;
    assert( block_size%aligment==0);
    return block_size;
}

void KMeansIndex::setParams(int aligment,  int desc_type, int desc_size, uint64_t total_size,uint32_t npoints )
{
    //return the number of bytes required to make size aligned
    auto getSizeWithPadding=[](uint32_t size,uint32_t aligment){
        uint32_t anb=size/aligment;
        if ( size%aligment!=0) anb++;
        return anb*aligment;
    };

    _params._aligment=aligment;
    _params._desc_type=desc_type;
    _params._desc_size=desc_size ;//size of the descriptor(without padding)
    _params._desc_size_bytes_wp=getSizeWithPadding(_params._desc_size,_params._aligment) ;//size of the descriptor(includding padding) for proper aligment
    _params._total_size=total_size;
    _params._npoints=npoints;
    _data=(char*)AlignedAlloc(_params._aligment,_params._total_size);
    memset( _data,0,_params._total_size);

}


// cv::Mat KMeanIndex::knnsearch(const cv::Mat &features,int nn,SearchParams sparams){
void KMeansIndex::search(Matrix  features, int nn, Matrix  indices, Matrix  distances, const ParamSet&  sparams, std::shared_ptr<cpu> Tcpu){

    if (features.rows!=indices.rows ||  distances.rows!=features.rows)
        throw std::runtime_error("KMeanIndex::knnsearch indices and distances must be already allocated with the same size as the number of features");
    if ( distances.cols!=nn || indices.cols!=nn)
        throw std::runtime_error("KMeanIndex::knnsearch indices and distances number of cols must == nn");

    //check input features and index are compatible
    if ( features.type()!=_params._desc_type)
        throw std::runtime_error("KMeanIndex::knnsearch Index is not of the same type than features");
    if ( features.colSize() !=_params._desc_size)
        throw std::runtime_error("KMeanIndex::knnsearch Index is not of the same size than features");
    //check distance matrix is ok
    if ( _params._desc_type==XFLANN_8U && distances.type()!=XFLANN_32S)
        throw std::runtime_error("KMeanIndex::knnsearch binary features employ integer distances (uint32_t)");
    if ( _params._desc_type==XFLANN_32F && distances.type()!=XFLANN_32F)
        throw std::runtime_error("KMeanIndex::knnsearch float features require float distances");

     //get host info to decide the version to execute

    //get host info to decide the version to execute
    std::shared_ptr<cpu> use_cpu_info;
    {
    std::unique_lock<std::mutex> lock(cpuinfomutex);
    if (!Tcpu){//if cpu info not passed, use our own
        if (!cpu_info) cpu_info=std::make_shared<cpu>();
        use_cpu_info=cpu_info;
        use_cpu_info->detect_host();
    }
    else use_cpu_info=Tcpu;
    }


      //decide the version to employ according to the type of features, aligment and cpu capabilities
     if (_params._desc_type==XFLANN_8U){
         if (use_cpu_info->isSafeAVX()){
             //orb
             if (_params._desc_size==32)   _knnsearch<distances::Hamming_x64_32bytes>(features,nn,indices,distances,sparams);
             //full akaze
             else if( _params._desc_size==61 && _params._aligment%8==0)   _knnsearch<distances::Hamming_x64_61bytes>(features,nn,indices,distances,sparams);
             //generic
             else  _knnsearch<distances::Hamming_x64>(features,nn,indices,distances,sparams);
         }
         else   _knnsearch<distances::Hamming_x32>(features,nn,indices,distances,sparams);

     }
    else if(_params._desc_type==XFLANN_32F){
         if( use_cpu_info->isSafeAVX() && _params._aligment%32==0 &&_params._desc_size>=32){ //AVX version
             if ( _params._desc_size==256)  _knnsearch<distances::L2_avx_8w>(features,nn,indices,distances,sparams);//specific for surf 256 bytes
            else if ( _params._desc_size==512)  _knnsearch<distances::L2_avx_16w>(features,nn,indices,distances,sparams);//specific for surf 512 bytes
            else   _knnsearch<distances::L2_avx_generic>(features,nn,indices,distances,sparams);//any other
        }
        else if( use_cpu_info->isSafeSSE() && _params._aligment%16==0 &&_params._desc_size>=16){//SSE version
             if ( _params._desc_size==256)  _knnsearch<distances::L2_sse3_16w>(features,nn,indices,distances,sparams);//specific for surf 256 bytes
            else   _knnsearch<distances::L2_sse3_generic>(features,nn,indices,distances,sparams);//any other
        }
        else if (features.cols==2)_knnsearch<distances::L2_generic_2>(features,nn,indices,distances,sparams);
             else if (features.cols==3)_knnsearch<distances::L2_generic_3>(features,nn,indices,distances,sparams);
             else _knnsearch<distances::L2_generic>(features,nn,indices,distances,sparams);
    }
    else std::runtime_error("XFlann::knnsearch invalid feature type. Should be CV_8UC1 or CV_32FC1");


}


void KMeansIndex::clear()
{
    if (_data!=0) AlignedFree(_data);
    _data=0;
    memset(&_params,0,sizeof(_params));
}


//loads/saves from a file
void KMeansIndex::readFromFile(const std::string &filepath){
    std::ifstream file(filepath);
    if (!file) std::runtime_error("XFlann::readFromFile could not open:"+filepath);
    fromStream(file);
}

void KMeansIndex::saveToFile(const std::string &filepath){
    std::ofstream file(filepath);
    if (!file)std::runtime_error("XFlann::saveToFile could not open:"+filepath);
    toStream(file);

}
///
uint64_t KMeansIndex::hash()const{
    if (_data==0)return 0;
    uint64_t nwords= _params._total_size / 4,seed = 0;
    uint32_t *ptr=(uint32_t *)_data;
    for(uint64_t i=0;i<nwords;i++)
        seed ^= ptr[i] + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}
std::string KMeansIndex::hash_str()const{
    std::string alpha="AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz0123456789";
    auto _hval=hash();
    uint8_t *ptr=(uint8_t*)&_hval;
    std::string res;res.resize(sizeof(_hval));
    for(size_t i=0;i<sizeof(_hval);i++)
        res[i]=alpha[ ptr[i]%alpha.size()];
    return res;

}
///save/load to binary streams
void KMeansIndex::toStream(std::ostream &str)const{
    //magic number
    uint64_t sig=55824124;
    str.write((char*)&sig,sizeof(sig));
    //save string
    str.write((char*)&_params,sizeof(params));
    str.write(_data,_params._total_size);
}

void KMeansIndex::fromStream(std::istream &str)
{
    if (_data!=0) free (_data);
    uint64_t sig;
    str.read((char*)&sig,sizeof(sig));
    if (sig!=55824124) std::runtime_error("XFlann::fromStream invalid signature");
    //read string
    str.read((char*)&_params,sizeof(params));
    _data=(char*)AlignedAlloc(_params._aligment,_params._total_size);
    if (_data==0) std::runtime_error("XFlann::fromStream Could not allocate data");
    str.read(_data,_params._total_size);
}

}
}
