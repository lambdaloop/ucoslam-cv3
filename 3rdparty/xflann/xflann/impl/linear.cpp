#include "linear.h"
#include "../cpu.h"
namespace xflann {
namespace impl {

void Linear::build ( Matrix features,   const ParamSet &params){

    _features=features;
    _params._store_data=false;
    _params._desc_size=features.colSize();
    if (params("store_data")){
        _params._store_data=true;
        //use padding to speed up search
        _stored_features.resize(_features.size());
        memcpy(_stored_features.ptr(),_features.ptr<char>(0),_features.size());
        //redirect _features to this
        _features.data=_stored_features.ptr();
    }
}

void Linear::search(Matrix queries, int nn, Matrix indices, Matrix distances, const ParamSet &sparams, std::shared_ptr<cpu> Tcpu){
    if (queries.type()!=_features.type())
        throw std::runtime_error("KDTreeIndex::knnsearch  features must be of the same type than elements in index");
    if (queries.rows!=indices.rows ||  distances.rows!=queries.rows)
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




    _params._aligment=std::min(_features.aligment(),queries.aligment());
    if (_features.type()==XFLANN_32F){
        if( use_cpu_info->isSafeAVX() && _params._aligment%32==0  && _params._desc_size>=32){ //AVX version
            if (  _params._desc_size==256)  _knnsearch<distances::L2_avx_8w>(queries,nn,indices,distances,sparams);//specific for surf 256 bytes
            else if (  _params._desc_size==512)  _knnsearch<distances::L2_avx_16w>(queries,nn,indices,distances,sparams);//specific for surf 512 bytes
            else  _knnsearch<distances::L2_avx_generic>(queries,nn,indices,distances,sparams);//any other
        }
        else if( use_cpu_info->isSafeSSE() && _params._aligment%16==0  &&_params._desc_size>=16){//SSE version
            if (  _params._desc_size==256)  _knnsearch<distances::L2_sse3_16w>(queries,nn,indices,distances,sparams);//specific for surf 256 bytes
            else   _knnsearch<distances::L2_sse3_generic>(queries,nn,indices,distances,sparams);//any other
        }
        else _knnsearch<distances::L2_generic>(queries,nn,indices,distances,sparams);
    }
    else if(_features.type()==XFLANN_8U){

        if (use_cpu_info->isSafeAVX()){
            //orb
            if (_params._desc_size==32)   _knnsearch<distances::Hamming_x64_32bytes>(queries,nn,indices,distances,sparams);
            //full akaze
            else if( _params._desc_size==61 && _params._aligment%8==0)   _knnsearch<distances::Hamming_x64_61bytes>(queries,nn,indices,distances,sparams);
            //generic
            else  _knnsearch<distances::Hamming_x64>(queries,nn,indices,distances,sparams);
        }
        else   _knnsearch<distances::Hamming_x32>(queries,nn,indices,distances,sparams);


    }

}
uint64_t Linear::hash() const{
    assert("Not yet");
    return 0;
}

void Linear::toStream(std::ostream &str)const{
    assert("Not yet");
}
void Linear::fromStream(std::istream &str){
    assert("Not yet");

}
}

}
