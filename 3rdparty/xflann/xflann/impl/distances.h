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
#ifndef _XFLANN_DISTANCES_
#define _XFLANN_DISTANCES_
#include <cassert>
#include <cstdint>
#include <utility>
#include <iostream>
#include <stdexcept>
#ifndef __ANDROID__
#include <immintrin.h>
#endif
#include <bitset>
using namespace std;
namespace  xflann {
static inline void * AlignedAlloc(int __alignment,int size);
static inline void AlignedFree(void *ptr);

namespace  distances{

inline float d_L2_2(float *a,float *b,int size){
    assert(size==2);
    float d0=a[0]-b[0];
    float d1=a[1]-b[1];
    return d0*d0+d1*d1;
}
inline float d_L2_3(float *a,float *b,int size){
    assert(size==2);
    float d0=a[0]-b[0];
    float d1=a[1]-b[1];
    float d2=a[2]-b[2];
    return d0*d0+d1*d1+d2*d2;
}

inline float d_L2_generic(float *a,float *b,int size){
#if 0
    float d=0;
    for(int f=0;f<size;f++)  d+=  (a[f]-b[f])*(a[f]-b[f]);
    return d;
#else
    float result = 0;
    float diff0, diff1, diff2, diff3;
    float* last = a + size;
    float* lastgroup = last - 3;

    /* Process 4 items with each loop for efficiency. */
    while (a < lastgroup) {
        diff0 = (float)(a[0] - b[0]);
        diff1 = (float)(a[1] - b[1]);
        diff2 = (float)(a[2] - b[2]);
        diff3 = (float)(a[3] - b[3]);
        result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
        a += 4;
        b += 4;

    }
    /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
    while (a < last) {
        diff0 = (float)(*a++ - *b++);
        result += diff0 * diff0;
    }
    return result;
#endif
}
#ifndef __ANDROID__

inline float d_L2_se3_generic(__m128 *ptr,__m128 *ptr2,int n){
    __m128 sum=_mm_setzero_ps(), sub_mult;
    //substract, multiply and accumulate
    for(int i=0;i<n;i++){
        sub_mult=_mm_sub_ps(ptr2[i],ptr[i]);
        sub_mult=_mm_mul_ps(sub_mult,sub_mult);
        sum=_mm_add_ps(sum,sub_mult);
    }
    sum=_mm_hadd_ps(sum,sum);
    sum=_mm_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0] ;
}
//not tested
inline float d_L2_se3_1w(__m128 *ptr,__m128 *ptr2,int n){
    __m128 sum;
    //substract, multiply and accumulate
    sum=_mm_sub_ps(ptr2[0],ptr[0]);
    sum=_mm_mul_ps(sum,sum);
    sum=_mm_hadd_ps(sum,sum);
    sum=_mm_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0] ;
}

inline float d_L2_avx_1w(__m256 *ptr,__m256 *ptr2,int n){
    assert(n==1);
    __m256 sum;
    //substract, multiply and accumulate
    assert(((uint64_t)ptr)%32==0);
    assert(((uint64_t)ptr2)%32==0);
    sum=_mm256_sub_ps(ptr2[0],ptr[0]);
    sum=_mm256_mul_ps(sum,sum);
    sum=_mm256_hadd_ps(sum,sum);
    sum=_mm256_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0]+sum_ptr[4];
}

inline float d_L2_avx_generic(__m256 *ptr,__m256 *ptr2,int n){

    __m256 sum=_mm256_setzero_ps();
#if 0
    __m256 sub_mult;
    //substract, multiply and accumulate
    for(int i=0;i<n;i++){
        sub_mult=_mm256_sub_ps(ptr2[i],ptr[i]);
        sub_mult=_mm256_mul_ps(sub_mult,sub_mult);
        sum=_mm256_add_ps(sum,sub_mult);
    }
#else

    __m256 diff0, diff1, diff2, diff3;
    __m256* last = ptr + n;
    __m256* lastgroup = last - 3;

    /* Process 4 items with each loop for efficiency. */
    while (ptr < lastgroup) {
        diff0=_mm256_sub_ps(ptr2[0],ptr[0]);
        diff0=_mm256_mul_ps(diff0,diff0);
        diff1=_mm256_sub_ps(ptr2[1],ptr[1]);
        diff1=_mm256_mul_ps(diff1,diff1);
        diff2=_mm256_sub_ps(ptr2[2],ptr[2]);
        diff2=_mm256_mul_ps(diff2,diff2);
        diff3=_mm256_sub_ps(ptr2[3],ptr[3]);
        diff3=_mm256_mul_ps(diff3,diff3);
        diff0=_mm256_add_ps(diff0,diff1);
        diff2=_mm256_add_ps(diff2,diff3);
        diff0=_mm256_add_ps(diff0,diff2);
        sum=_mm256_add_ps(sum,diff0);

        ptr += 4;
        ptr2 += 4;

    }
    /* Process last 0-3 pixels.  Not needed for standard vector lengths. */
    while (ptr < last) {
        diff0=_mm256_sub_ps(*ptr2++,*ptr++);
        diff0=_mm256_mul_ps(diff0,diff0);
        sum=_mm256_add_ps(sum,diff0);
    }
#endif
    sum=_mm256_hadd_ps(sum,sum);
    sum=_mm256_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0]+sum_ptr[4];
}



//surf 64 words specific
inline float d_L2_sse3_16w(__m128 *ptr,__m128 * ptr2,int n){
    assert(n==16);
    __m128 sum=_mm_setzero_ps(), sub_mult;
    //substract, multiply and accumulate
    for(int i=0;i<16;i++){
        sub_mult=_mm_sub_ps(ptr2[i],ptr[i]);
        sub_mult=_mm_mul_ps(sub_mult,sub_mult);
        sum=_mm_add_ps(sum,sub_mult);
    }
    sum=_mm_hadd_ps(sum,sum);
    sum=_mm_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0] ;
}

//surf 64 words specific
inline float d_L2_avx_8w(__m256 *a,__m256 *b,int n){
    assert(n==8);
    __m256 sum=_mm256_setzero_ps(), sub_mult;
    //substract, multiply and accumulate

    for(int i=0;i<8;i++){
        sub_mult=_mm256_sub_ps(a[i],b[i]);
        sub_mult=_mm256_mul_ps(sub_mult,sub_mult);
        sum=_mm256_add_ps(sum,sub_mult);
    }

    sum=_mm256_hadd_ps(sum,sum);
    sum=_mm256_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0]+sum_ptr[4];
}


//surf   128 words specific
inline float d_L2_avx_16w(__m256 *a,__m256 *b,int n){
    assert(n==16);
    __m256 sum=_mm256_setzero_ps(), sub_mult;
    //substract, multiply and accumulate

    for(int i=0;i<16;i++){
        sub_mult=_mm256_sub_ps(a[i],b[i]);
        sub_mult=_mm256_mul_ps(sub_mult,sub_mult);
        sum=_mm256_add_ps(sum,sub_mult);
    }

    sum=_mm256_hadd_ps(sum,sum);
    sum=_mm256_hadd_ps(sum,sum);
    float *sum_ptr=(float*)&sum;
    return  sum_ptr[0]+sum_ptr[4];
}
#else
inline float d_L2_se3_generic(void *ptr,void *ptr2,int n){throw std::runtime_error("d_L2_se3_generic Not implemented in Android");}
inline float d_L2_se3_1w(void *ptr,void *ptr2,int n){throw std::runtime_error("d_L2_se3_1w Not implemented in Android");}
inline float d_L2_avx_1w(void *ptr,void *ptr2,int n){throw std::runtime_error("d_L2_avx_1w Not implemented in Android");}
inline float d_L2_avx_generic(void *ptr,void *ptr2,int n){throw std::runtime_error("d_L2_avx_generic Not implemented in Android");}
inline float d_L2_sse3_16w(void *ptr,void * ptr2,int n){throw std::runtime_error("d_L2_sse3_16w Not implemented in Android");}
inline float d_L2_avx_8w(void *a,void *b,int n){throw std::runtime_error("d_L2_avx_8w Not implemented in Android");}
inline float d_L2_avx_16w(void *a,void *b,int n){throw std::runtime_error("d_L2_avx_16w Not implemented in Android");}
#endif


inline int32_t popcnt64(uint64_t n)
{
    return std::bitset<64> (n).count();
}

inline int32_t popcnt32(uint32_t n)
{
    return std::bitset<32> (n).count();

}
inline int32_t popcnt8(uint32_t n)
{
    return std::bitset<8> (n).count();
}

inline int32_t d_Hamming_x32(uint32_t *feat_ptr,uint32_t *feat_ptr2,int n){
    int32_t result = 0;
    for(int i = 0; i < n; ++i ) result += popcnt32(feat_ptr[i] ^ feat_ptr2[i]);
    return result;
}
inline int32_t d_Hamming_unaligned(uint8_t *feat_ptr,uint8_t *feat_ptr2,int n){
    int32_t result = 0;
    for(int i = 0; i < n; ++i )
        result+= popcnt8(feat_ptr[i] ^ feat_ptr2[i]);
    return result;
}
inline int32_t d_Hamming_x64(uint64_t *feat_ptr,uint64_t *feat_ptr2,int n){
    int32_t result = 0;
    for(int i = 0; i < n; ++i ) result += popcnt64(feat_ptr[i] ^ feat_ptr2[i]);
    return result;
}


inline int32_t d_Hamming_x64_32bytes(uint64_t *feat_ptr,uint64_t *feat_ptr2,int n){
    assert(n==4);
    return popcnt64(feat_ptr[0]^feat_ptr2[0])+ popcnt64(feat_ptr[1]^feat_ptr2[1])+
            popcnt64(feat_ptr[2]^feat_ptr2[2])+popcnt64(feat_ptr[3]^feat_ptr2[3]);
}
inline int32_t d_Hamming_x64_61bytes(uint64_t *feat_ptr,uint64_t *feat_ptr2,int n){
    assert(n==8);
    return popcnt64(feat_ptr[0]^feat_ptr2[0])+ popcnt64(feat_ptr[1]^feat_ptr2[1])+
            popcnt64(feat_ptr[2]^feat_ptr2[2])+popcnt64(feat_ptr[3]^feat_ptr2[3])+
            popcnt64(feat_ptr[4]^feat_ptr2[4])+popcnt64(feat_ptr[5]^feat_ptr2[5])+
            popcnt64(feat_ptr[6]^feat_ptr2[6])+popcnt64(feat_ptr[7]^feat_ptr2[7]);
}

////////////////////////////////////////////////////////////
//base class for computing distances between feature vectors
template<typename register_type,typename distType, int aligment>
class Lx{
public:
    typedef distType DType;
    typedef register_type TData;



    //returns the number of operations required for a TData with the given info
     inline int getNFeatOps(int desc_size ){
        int nfeattdata= desc_size/sizeof(register_type);
        if (desc_size%aligment!=0) nfeattdata++;
        return nfeattdata;
    }
protected:

    int _nwords,_aligment,_desc_size;
    int _block_desc_size_bytes_wp;
    register_type *_feature=0;
public:
    ~Lx(){if (_feature!=0) AlignedFree(_feature);}
    void setParams(int desc_size, int block_desc_size_bytes_wp){
        assert(block_desc_size_bytes_wp%aligment==0);
        _desc_size=desc_size;
        _block_desc_size_bytes_wp=block_desc_size_bytes_wp;
        assert(_block_desc_size_bytes_wp%sizeof(register_type )==0);
        _nwords=_block_desc_size_bytes_wp/sizeof(register_type );//number of aligned words
        _feature=(register_type*)AlignedAlloc(aligment,_nwords*sizeof(register_type ));
        memset(_feature,0,_nwords*sizeof(register_type ));
    }
    inline void startwithfeature(const register_type *feat_ptr){memcpy(_feature,feat_ptr,_desc_size);}
    inline distType computeDist(register_type *ptr){return computeDist_(ptr,_feature,_nwords);}

    virtual  inline distType computeDist_(register_type *a,register_type *b,int n)=0;
};


struct L2_generic:public Lx<float,float,4>{
    inline float computeDist_(float *fptr,float *fptr2,int n){return d_L2_generic(fptr,fptr2,n);}
};
struct L2_generic_2:public Lx<float,float,4>{
    inline float computeDist_(float *fptr,float *fptr2,int n){return d_L2_2(fptr,fptr2,n);}
};
struct L2_generic_3:public Lx<float,float,4>{
    inline float computeDist_(float *fptr,float *fptr2,int n){return d_L2_3(fptr,fptr2,n);}
};

#ifndef __ANDROID__
struct L2_sse3_generic:public Lx<__m128,float,16>{inline float computeDist_(__m128 *ptr,__m128 *ptr2,int n){return d_L2_se3_generic(ptr,ptr2,n);}};
//surf - 256 bytes
struct L2_sse3_16w:public Lx<__m128,float,16> {inline float computeDist_(__m128 *ptr,__m128 * ptr2,int n){ return d_L2_sse3_16w(ptr,ptr2,n);}};

struct L2_avx_generic:public Lx<__m256,float,32>{inline float computeDist_(__m256 *ptr,__m256 *ptr2,int n){ return d_L2_avx_generic(ptr,ptr2,n);}};
//specific for surf in avx
struct L2_avx_8w:public Lx<__m256,float,32> {inline float computeDist_(__m256 *a,__m256 *b,int n){ return d_L2_avx_8w(a,b,n);}};
//specific for surf in avx
struct L2_avx_1w:public Lx<__m256,float,32> {inline float computeDist_(__m256 *a,__m256 *b,int n){ return d_L2_avx_1w(a,b,n);}};
//specific for surf-ext in avx
struct L2_avx_16w:public Lx<__m256,float,32> {inline float computeDist_(__m256 *a,__m256 *b,int n){ return d_L2_avx_16w(a,b,n);}};


#else
struct L2_sse3_generic:public Lx<float,float,16>{inline float computeDist_(float *ptr,float *ptr2,int n){return d_L2_se3_generic(ptr,ptr2,n);}};
//surf - 256 bytes
struct L2_sse3_16w:public Lx<float,float,16> {inline float computeDist_(float *ptr,float * ptr2,int n){ return d_L2_sse3_16w(ptr,ptr2,n);}};

struct L2_avx_generic:public Lx<float,float,32>{inline float computeDist_(float *ptr,float *ptr2,int n){ return d_L2_avx_generic(ptr,ptr2,n);}};
//specific for surf in avx
struct L2_avx_8w:public Lx<float,float,32> {inline float computeDist_(float *a,float *b,int n){ return d_L2_avx_8w(a,b,n);}};
//specific for surf in avx
struct L2_avx_1w:public Lx<float,float,32> {inline float computeDist_(float *a,float *b,int n){ return d_L2_avx_1w(a,b,n);}};
//specific for surf-ext in avx
struct L2_avx_16w:public Lx<float,float,32> {inline float computeDist_(float *a,float *b,int n){ return d_L2_avx_16w(a,b,n);}};
#endif


//generic hamming distance calculator
struct  Hamming_x64:public Lx<uint64_t,int32_t,8>{inline int32_t computeDist_(uint64_t *ptr,uint64_t *ptr2,int n){return d_Hamming_x64(ptr,ptr2,n);}};

struct  Hamming_x32:public Lx<uint32_t,int32_t,4>{inline int32_t computeDist_(uint32_t *ptr,uint32_t *ptr2,int n){ return d_Hamming_x32(ptr,ptr2,n);}};
struct  Hamming_unaligned:public Lx<uint8_t,int32_t,1>{inline int32_t computeDist_(uint8_t *ptr,uint8_t *ptr2,int n){ return d_Hamming_unaligned(ptr,ptr2,n);}};
//for orb
struct Hamming_x64_32bytes:public Lx<uint64_t,int32_t,8>{inline int32_t computeDist_(uint64_t *ptr,uint64_t *ptr2,int n){return d_Hamming_x64_32bytes(ptr,ptr2,n);}};

//for akaze
struct Hamming_x64_61bytes:public Lx<uint64_t,int32_t,8>{inline int32_t computeDist_(uint64_t *ptr,uint64_t *ptr2,int n){ return d_Hamming_x64_61bytes(ptr,ptr2,n);}};

}
}

#endif
