#ifndef xflann_Hash_H_
#define xflann_Hash_H_
#include <cstdint>
#include <iostream>
namespace xflann {
/**
 * @brief The Hash struct creates a hash by adding elements. It is used to check the integrity of data stored in files in debug mode
 */
struct Hash{
    uint64_t seed=0;

    template<typename T> void add(const T &val){
        char *p=(char *)&val;
        for(uint32_t b=0;b<sizeof(T);b++) seed  ^=  p[b]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    template<typename T> void add(const  T& begin,const T& end){
        for(auto it=begin;it!=end;it++)add(*it);
    }

    void add(bool val){
        seed  ^=   int(val)+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(int val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(uint64_t val){
        seed  ^=   val+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    void add(float val){
        int *p=(int *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    void add(double val){
        uint64_t *p=(uint64_t *)&val;
        seed  ^=   *p+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    void operator+=(bool v){add(v);}
    void operator+=(int v){add(v);}
    void operator+=(char v){add(v);}
    void operator+=(float v){add(v);}
    void operator+=(double v){add(v);}
    void operator+=(uint32_t v){add(v);}
    void operator+=(uint64_t v){add(v);}
    void operator+=(const std::string & str){
        for(const auto &c:str)add(c);
    }


     operator uint64_t()const{return seed;}


    std::string tostring( ){
        return tostring(seed);
    }


    static std::string tostring(uint64_t v){
        std::string sret;
        std::string alpha="qwertyuiopasdfghjklzxcvbnm1234567890QWERTYUIOPASDFGHJKLZXCVBNM";
        unsigned char * s=(uint8_t *)&v;
        int n=sizeof(seed)/sizeof(uint8_t );
        for(int i=0;i<n;i++){
            sret.push_back(alpha[s[i]%alpha.size()]);
        }
        return sret;
    }
    void reset(){seed=0;}
};

}

#endif

