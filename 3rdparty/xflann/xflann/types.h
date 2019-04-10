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
#ifndef XFLANN_TYPES_H
#define XFLANN_TYPES_H
#include "xflann_exports.h"
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <sstream>
#include <cassert>
#include <map>
#include <memory>
#if (defined XFLANN_OPENCV)
#include <opencv2/core/core.hpp>
#endif
namespace xflann{
static inline void * AlignedAlloc(int __alignment,int size);
static inline void AlignedFree(void *ptr);

template<typename T>
std::string _to_string(const T&val){
    std::stringstream sstr;
    sstr<<val;
    return sstr.str();
}

////////////////////////////////////////////
////////////////////////////////////////////
//Base class for indexing params
////////////////////////////////////////////
////////////////////////////////////////////
struct ParamSet: std::map<std::string,std::string >{
    std::string type;
    //most common operation, return the param as a int
    int operator()(const std::string &str)const  {
        assert(count(str));
        return stov<int>(std::map<std::string,std::string >::find(str)->second);
    }

    const std::string asString(const std::string &str)const  {
        assert(count(str));
        return std::map<std::string,std::string >::find(str)->second;
    }

    double asDouble(const std::string &str)const  {
        assert(count(str));
        return stov<double>(std::map<std::string,std::string >::find(str)->second);
    }
private:
    template<typename T>T stov(const std::string &str) const {std::stringstream sstr(str); T val; sstr>>val;return val;}
};

//////////////////////////////////////////////
////params to create a linear search index
//////////////////////////////////////////////
struct LinearParams:public ParamSet{
       LinearParams(int store_data=1 ) {
        type="linear";
        insert({"verbose","0"});
        insert({"store_data",_to_string(store_data)});
    }
};

////////////////////////////////////////////
//params to create a kdtree
//The param store_data indicates if the points must be copied in the internal structure
//Otherwise, the kdtree will need the points to remain in memory while doing the search
////////////////////////////////////////////
struct KDTreeParams:public ParamSet{
    KDTreeParams(int minLeafSize=5,int store_data=1 ) {
        type="kdtree";
        insert({"verbose","0"});
        insert({"minLeafSize",_to_string(minLeafSize)});
        insert({"store_data",_to_string(store_data)});
        insert({"median_split", "0"});//default mean_split. If desired, set median_split
    }
};
////////////////////////////////////////////
//Params for hierarchical kmeans clustering
////////////////////////////////////////////
struct HKMeansParams:public ParamSet{
    //maxiter==-1 means unlimited (until convergence)
    HKMeansParams(uint32_t K=8,int MaxIters=11 ) {
        type="kmeans";
        insert({"maxIters",_to_string(MaxIters)});
        insert({"k",_to_string(K)});
        insert({"nthreads","1"});
        insert({"recursive","1"});
        insert({"verbose","0"});
        insert({"L","-1"});//maximum depth (instead of using maxLeafSize)
    }
};



////////////////////////////////////////////
////////////////////////////////////////////
//Search params
////////////////////////////////////////////
////////////////////////////////////////////

/**
 * @brief The SearchParams struct is a generic class to represent the parameters of the search methods
 * You can used the specialized versions KnnSearchParams and RadiusSearchParams too
 */
struct SearchParams:public ParamSet{
    /**
     * @brief SearchParams
     * @param MaxChecks maximum number of checks
     * @param Sorted must result be sorted?
     * @param threads number of parallel threads used in the search
     * @param maxDist used to do radius search.
     */
    SearchParams(int MaxChecks=1, int Sorted=false,int threads=1,double maxDist=-1){
        insert({"maxChecks",_to_string(MaxChecks)});
        insert({"maxDist",_to_string(maxDist)});//used in radius search only
        insert({"sorted",_to_string(Sorted)});
        insert({"threads",_to_string(threads)});
    }
};

/**
 * @brief The KnnSearchParams struct is used search with the nearest neighbor algorithm
 */
struct KnnSearchParams:public ParamSet{
    /**
     * @brief KnnSearchParams Parameters to do knn search
     * @param MaxChecks maximum number of checks. Set -1 for unlimited(it is exact search).
     * @param Sorted set to true if you want results sorted
     * @param threads number of threads in which work is divided
     */
    KnnSearchParams(int MaxChecks=1, int Sorted=false,int threads=1){
        insert({"maxChecks",_to_string(MaxChecks)});
        insert({"maxDist", "-1"});//used in radius search only
        insert({"sorted",_to_string(Sorted)});
        insert({"threads",_to_string(threads)});

    }
};

/**
 * @brief The RadiusSearchParams struct is used for radius search algorithm
 */
struct RadiusSearchParams:public ParamSet{
    /**
     * @brief RadiusSearchParams Parameters to do radius search
     * @param MaxChecks maximum number of checks. -1 means unlimited
     * @param maxDist radius distance. Use squared value. ELememnts with larger distance are not considered
     * @param Sorted set to true if you want results sorted
     * @param threads number of threads in which work is divided
     */
    RadiusSearchParams(int MaxChecks, double maxDist, int Sorted=false,int threads=1){
        insert({"maxChecks",_to_string(MaxChecks)});
        insert({"maxDist", _to_string(maxDist)});//used in radius search only
        insert({"sorted",_to_string(Sorted)});
        insert({"threads",_to_string(threads)});

    }
};

////////////////////////////////////////////
////////////////////////////////////////////
//defines the type of data allocated for data points (same as OpenCv for easy transition)
////////////////////////////////////////////
////////////////////////////////////////////
#define XFLANN_NONE -1
#define XFLANN_8U   0
#define XFLANN_32F  5
#define XFLANN_32S  4
inline size_t _sizeof(int type){
    if (type==XFLANN_8U) return 1;
    if (type==XFLANN_32F) return 4;
    if (type==XFLANN_32S) return 4;
    return 0;
}


////////////////////////////////////////////
namespace impl{//not public
//32bytes aligned vector
template<typename T>
class XFLANN_API  Vector{
    int _size=0,_sizebytes=0;
    T *_data=0;
public:
    Vector(){}
    Vector(int size,bool reset_=false){resize(size);if (reset_)reset();}
    ~Vector(){if (_data!=0)AlignedFree(_data);}

    inline void resize(int n){
        if (n!=_size){
            resizeBytes(sizeof(T)*n);
            _size=n;
        }
    }
    inline T*ptr(){return _data;}

    inline int size()const{return _size;}
    inline bool empty()const {return _size!=0;}
    inline int sizeBytes()const{return _size*sizeof(T);}
    inline void reset(){memset(_data,0,_sizebytes);}
    template<typename T2>
    void print(){
        int s2=_size/sizeof(T2);
        T2*ptr=(T2*)_data;
        for(int i=0;i<s2;i++)std::cout<<ptr[i]<<" ";std::cout<<std::endl;
    }


    T &operator[](int index){
        assert( uint64_t(_data+index )< (uint64_t)(_data+_size));
        return _data[index];
    }
    const T &operator[](int index)const{
        assert( uint64_t(_data+index )< (uint64_t)(_data+_size));
        return _data[index];
    }
    void toStream(std::ostream &str)const{
        str.write((char*)&_size,sizeof(_size));
        str.write((char*)&_sizebytes,sizeof(_size));
        str.write((char*)_data,sizeBytes());
    }
    void fromStream(std::istream &str){
        int new_size,new_sizebytes;
        str.read((char*)&new_size,sizeof(_size));
        str.read((char*)&new_sizebytes,sizeof(_size));
        resizeBytes(new_sizebytes);
        str.read((char*)_data,new_sizebytes);
    }
private:
    inline void resizeBytes(int bytes){
        if (_data!=0) AlignedFree(_data);
        int finalsize_aligned=bytes/32;
        if ( bytes%32!=0) finalsize_aligned++;
        _sizebytes=finalsize_aligned*32;
        _data=(T*)AlignedAlloc(32,_sizebytes);
    }
};
}


////////////////////////////////////////////
/**
 * @brief The Matrix class  references to other data. It is used as an interface to access the underlying user data. This matrix does not allocate nor deallocate data
 */
////////////////////////////////////////////

class XFLANN_API  Matrix
{
    bool allocated=false;//has data been allocated here?
public:
#if (defined XFLANN_OPENCV  )
//converts a opencv matrix into a xflann matrix

 inline Matrix(const cv::Mat &mat)
{
    if ( !((mat.type()==CV_8UC1) || mat.type()==CV_32F) || mat.type()!=CV_32S ) std::runtime_error("Unsupported matrix type");
    _type=mat.type();
    rows=mat.rows;
    cols=mat.cols;
    data=(  char*)mat.ptr<char>(0);
    stride=mat.step[0];
}
#endif
    Matrix(){_type=XFLANN_NONE; data=0;rows=cols=stride=0;}

    /**
     * @brief Matrix Creates from an array already created

     * @param dataType_ XFLANN_8U (for uchar) or  XFLANN_32F (for float) or XFLANN_32S (for int)
     * @param rows_ number of rows
     * @param cols_ number of cols
     * @param data_ pointer to the data
     * @param stride_ if the data has padding at the end of the row, indicate the stride in bytes in this param. Otherwise it is assumed data is Continous (no padding at the end of each row)
     */
    Matrix( int dataType_, size_t rows_, size_t cols_,const void* data_=0,size_t stride_ =0)
    {
        data=((char*)data_);
        rows=rows_;
        cols=cols_;
        stride=stride_;
        _type=dataType_;

        if (dataType_==XFLANN_NONE)                     throw std::runtime_error("xflann::Matrix Invalid DataType");
        if (stride_==0 )  stride = _sizeof(_type)*cols;
        if(data_==0) {
            data=( char*)AlignedAlloc(32,_sizeof(dataType_)*rows_*cols_);
            allocated=true;
        }
    }
    void setParams( int dataType_, size_t rows_, size_t cols_,const void* data_=0,size_t stride_ =0){
        data=(char*)data_;rows=rows_;cols=cols_;stride=stride_;_type=dataType_;
        if (dataType_==XFLANN_NONE)                     throw std::runtime_error("xflann::Matrix Invalid DataType");
        if (stride==0 )  stride = _sizeof(_type)*cols;
        if(data_==0) {
            data=( char*)AlignedAlloc(32,_sizeof(dataType_)*rows_*cols_);
            allocated=true;
        }

    }


    //returns the submatrix indicated
    Matrix subMatrix(int startrow,int endrow,int startcol=-1,int endcol=-1){
        if(startcol==-1) startcol=0;
        if (endcol==-1) endcol=cols;
        return Matrix( _type, endrow-startrow, endcol-startcol, ptr(startrow)+startcol*_sizeof(_type),stride);
    }
    ~Matrix(){if (allocated) AlignedFree(data);}
    //access to rows pointers
    template<typename T=uint8_t>   T*ptr(int row=0){ assert(row<rows); return (T*)(data+row*stride); }
    template<typename T=uint8_t>   const T*ptr(int row=0)const{ assert(row<rows);    return (T*)(data+row*stride); }

    //copies the row-th into the vector v

    inline  void vptr(impl::Vector<char> &v,int row=0) {
        v.resize(stride);
        std::memcpy(v.ptr(),data+row*stride,stride);
    }

    //sets all elements to val
    template<typename T> inline void setTo(T val){ for(int r=0;r<rows;r++)for(int c=0;c<cols;c++) ptr<T>(r)[c]=val;}
    //return the element passed
    template<typename T> inline T& at(int row,int col){ return ptr<T>(row)[col];}
    template<typename T> inline const T& at(int row,int col)const{ return ptr<T>(row)[col];}
    template<typename T>  void print(std::ostream &str){
        for(int r=0;r<rows;r++){for(int c=0;c<cols;c++) str<<ptr<int>(r)[c] <<" ";str<<std::endl;}
    }

    //returns the size in bytes of a column (excluding padding)
    inline size_t colSize()const {return _sizeof(_type)*cols;}
    //total size in bytes
    inline size_t size()const{return stride*rows;}
    //return the type of data
    inline int  type()const {return _type;}
    //returns the aligment of the underlying data
    inline int aligment()const{
        if (data==0)return 0;
        auto ptr0= ptr(0);
        //only one row
        if (rows==1){
            if (uint64_t(ptr0)%32==0 ) return 32;
            if (uint64_t(ptr0)%16==0 ) return 16;
            if (uint64_t(ptr0)%8==0 ) return 8;
            if (uint64_t(ptr0)%4==0 ) return 4;
            if (uint64_t(ptr0)%2==0 ) return 2;
            return 1;
        }
        else{
            //more than one, must check in case of non continous data
            auto ptr1= ptr(1);
            if (uint64_t(ptr0)%32==0 && uint64_t(ptr1)%32==0) return 32;
            if (uint64_t(ptr0)%16==0 && uint64_t(ptr1)%16==0) return 16;
            if (uint64_t(ptr0)%8==0 && uint64_t(ptr1)%8==0) return 8;
            if (uint64_t(ptr0)%4==0 && uint64_t(ptr1)%4==0) return 4;
            if (uint64_t(ptr0)%2==0 && uint64_t(ptr1)%2==0) return 2;
            return 1;
        }
    }


    //changes the variable rows. Does nothing else
    void resize(int nrows){
        rows=nrows;
    }

    int _type;//type of the data
    int rows,cols,stride;//
    char *data;//pointer to  data externally allocated
};
//converts a xflann::matrix to opencv

#if (defined XFLANN_OPENCV  )
inline cv::Mat convert(Matrix m){
    switch (m.type()){
    case  XFLANN_8U:
    case  XFLANN_32F:
    case  XFLANN_32S:
        return cv::Mat(m.rows,m.cols,m.type(),m.data,m.colSize());
        break;
    default:
        throw std::runtime_error("xflann::convert to cvMat. Do not know how to convert data type");
    };

}
#endif
//base class than implements the methods
class cpu;
namespace impl{
struct IndexImpl{
    virtual ~IndexImpl(){}
    virtual std::string getName()const=0;
    virtual void search(Matrix features, int nn, Matrix indices, Matrix distances, const ParamSet&  search_params ,std::shared_ptr<cpu> Tcpu=std::shared_ptr<cpu>())=0;

    //indicates if the features are stored in the structure or not
    virtual bool storeFeatures()const=0;

    //returns the total number of points in the index
    virtual uint32_t size()const=0;
    virtual void toStream(std::ostream &str)const=0;
    virtual void fromStream(std::istream &str)=0;
    virtual uint64_t hash()const=0;//returns a hash with the info of this

};
}

static inline void * AlignedAlloc(int __alignment,int size){
    assert(__alignment<256);

    unsigned char *ptr= (unsigned  char*)malloc(size + __alignment);

    if( !ptr )  return 0;

    // align the pointer

    size_t lptr=(size_t)ptr;
    int off =lptr%__alignment;
    if (off==0) off=__alignment;

    ptr = ptr+off ; //move to next aligned address
    *(ptr-1)=(unsigned char)off; //save in prev, the offset  to properly remove it
    return ptr;
}
static inline void AlignedFree(void *ptr){
    unsigned char *uptr=(unsigned char *)ptr;
    unsigned char off= *(uptr-1);
    uptr-=off;
    std::free(uptr);
}

}

#endif
