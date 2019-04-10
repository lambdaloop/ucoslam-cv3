/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#ifndef Hash_H_
#define Hash_H_
#include <cstdint>
#include <iostream>
#include <opencv2/core/core.hpp>
namespace ucoslam {
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
    void add(int64_t val){
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

    void add(const cv::Mat & m){
        for(int r=0;r<m.rows;r++){
            const char *ip=m.ptr<char>(r);
            int nem= m.elemSize()*m.cols;
            for(int i=0;i<nem;i++)
                seed  ^=   ip[i]+ 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
    }
    void add(const cv::Point2f & p){
        add(p.x);
        add(p.y);
    }
    void add(const cv::Point3f & p){
        add(p.x);
        add(p.y);
        add(p.z);
    }

    void operator+=(bool v){add(v);}
    void operator+=(int v){add(v);}
    void operator+=(char v){add(v);}
    void operator+=(float v){add(v);}
    void operator+=(double v){add(v);}
    void operator+=(uint32_t v){add(v);}
    void operator+=(uint64_t v){add(v);}
    void operator+=(int64_t v){add(v);}
    void operator+=(const cv::Mat & v){add(v);}
    void operator+=(const cv::Point2f & v){add(v);}
    void operator+=(const cv::Point3f & v){add(v);}
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
        unsigned char * s=(uchar *)&v;
        int n=sizeof(seed)/sizeof(uchar );
        for(int i=0;i<n;i++){
            sret.push_back(alpha[s[i]%alpha.size()]);
        }
        return sret;
    }
    void reset(){seed=0;}
};

}

#endif

