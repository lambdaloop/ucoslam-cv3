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
//Set of functions for I/O
#ifndef IO_UTILS_UCOSLAM_H
#define IO_UTILS_UCOSLAM_H
#include <map>
#include <set>
#include <iostream>
#include <utility>
#include <opencv2/core/core.hpp>
namespace ucoslam{

template<typename T>
inline T io_read(std::istream &str){
    T aux;str.read((char*)&aux,sizeof(T));return aux;
};

template<typename T>
inline void io_write(const T &v,std::ostream &str){
    str.write((char*)&v,sizeof(T));
};

template<typename T>
inline void io_read(T&v,std::istream &str){
    str.read((char*)&v,sizeof(T));
};
//use for simple elements
template< typename T>
void   toStream__ ( const  std::vector<T> &v,std::ostream &str ) {
    uint32_t s=v.size();
    str.write ( ( char* ) &s,sizeof ( s) );    
    str.write (  (char*)&v[0],sizeof(v[0])*v.size());
}


template<typename T>
void   toStream__ ( const  std::set<T> &v,std::ostream &str ) {

    uint32_t s=v.size();
    str.write ( ( char* ) &s,sizeof ( s) );
    for(const auto &e:v)   str.write (  (char*)&e,sizeof(e));
}

template<typename T>
void   fromStream__ (  std::vector<T> &v,std::istream &str ) {

    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    v.resize(s);
     str.read(  (char*)&v[0],sizeof(v[0])*v.size());
}



template<typename T>
void   fromStream__ (  std::set<T> &v,std::istream &str ) {

    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    for(uint32_t i=0;i<s;i++){
        T e;
        str.read(  (char*)&e,sizeof(e));
        v.insert(e);
    }
}

//use for elements of type key-value (maps,unordered-maps) with second element begin writable with write

template<typename key,typename  value>
void toStream__kv(const std::map<key,value> &t,std::ostream &str ){
    uint32_t s=t.size();
    str.write ( ( char* ) &s ,sizeof ( uint32_t ) );
    for ( auto &x: t) {
        str.write ( ( char* ) &x.first,sizeof (key ) );
        str.write ( ( char* ) &x.second,sizeof (value ) );
    }
}



template<typename Key,typename  Value>
void fromStream__kv(std::map<Key,Value> &t,std::istream &str){
    //now, the map
    uint32_t s;
    str.read ( ( char* ) &s,sizeof ( s ) );
    t.clear();
    for ( uint32_t i=0; i<s; i++ ) {
        Key key;Value d;
        str.read ( ( char* ) &key,sizeof ( key ) );
        str.read ( ( char* ) &d,sizeof ( d ) );
        t.insert ( std::make_pair ( key,d ) );
    }
}


//use for elements of type key-value (maps,unordered-maps) with second element having toStream defined
template<typename Tfirst_second>
void toStream__kv_complex(const Tfirst_second &t,std::ostream &str ){
    uint32_t s=t.size();
    str.write ( ( char* ) &s ,sizeof ( uint32_t ) );
    for ( auto &x: t) {
        str.write ( ( char* ) &x.first,sizeof ( x.first ) );
        x.second.toStream ( str );
    }
}

template<typename Tfirst_second,typename key_type,typename data_type>
void fromStream__kv_complex(Tfirst_second &t,std::istream &str){
    //now, the map
    uint32_t s;
    str.read ( ( char* ) &s,sizeof ( s ) );
    t.clear();
    for ( uint32_t i=0; i<s; i++ ) {
        key_type key;
        str.read ( ( char* ) &key,sizeof ( key ) );
        data_type d;
        d.fromStream ( str );
        t.insert ( make_pair ( key,d ) );
    }
}

template<typename key_type,typename data_type>
void fromStream__kv_complexmap(std::map<key_type,data_type> &t,std::istream &str){
    //now, the map
    uint32_t s;
    str.read ( ( char* ) &s,sizeof ( s ) );
    t.clear();
    for ( uint32_t i=0; i<s; i++ ) {
        key_type key;
        str.read ( ( char* ) &key,sizeof ( key ) );
        data_type d;
        d.fromStream ( str );
        t.insert ( std::make_pair ( key,d ) );
    }
}


void   toStream__ ( const  cv::Mat &m,std::ostream &str ) ;
/**
 */

void  fromStream__ ( cv::Mat &m,std::istream &str ) ;

/**
 */
void   toStream__ ( const  std::string &m,std::ostream &str ) ;
/**
 */
void   fromStream__ (  std::string &m,std::istream &str ) ;


//use for   elements with to stream

template< typename T>
void   toStream__ts ( const  std::vector<T> &v,std::ostream &str ) {
    uint32_t s=v.size();
    str.write ( ( char* ) &s,sizeof ( s) );
    for(size_t i=0;i<v.size();i++) v[i].toStream( str);
}

template< typename T>
void   fromStream__ts (    std::vector<T> &v,std::istream &str ) {
    uint32_t s;
    str.read( ( char* ) &s,sizeof ( s) );
    v.resize(s);
    for(size_t i=0;i<v.size();i++) v[i].fromStream( str);
}

//specialization for matrices
template<typename key>
void toStream__kv(const std::map<key,cv::Mat> &t,std::ostream &str ){
    uint32_t s=t.size();
    str.write ( ( char* ) &s ,sizeof ( uint32_t ) );
    for ( auto &x: t) {
        str.write ( ( char* ) &x.first,sizeof (key ) );
        toStream__(x.second,str);
    }
}

template<typename Key>
void fromStream__kv(std::map<Key,cv::Mat> &t,std::istream &str){
    //now, the map
    uint32_t s;
    str.read ( ( char* ) &s,sizeof ( s ) );
    t.clear();
    for ( uint32_t i=0; i<s; i++ ) {
        Key key;cv::Mat m;
        str.read ( ( char* ) &key,sizeof ( key ) );
        fromStream__(m,str);
        t.insert ( std::make_pair ( key,m ) );
    }
}

}

#endif
