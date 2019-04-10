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
#ifndef _ReusableContainer_H
#define _ReusableContainer_H
#include <vector>
#include <list>
#include <cstdint>
#include <iostream>
#include "expansiblecontainer.h"
namespace ucoslam{
/**A container in which removed elements are reused later
 */
template<typename T >
class ReusableContainer{
private:
public:
    struct const_iterator{
        const_iterator(const ReusableContainer &cnt,uint32_t idx):_container(cnt),current(idx){
            if (_container._data.size()==0)return;
            while(  current<_container._data.size()) {
                if ( _container._data[current].first==false )             current++;
                else break;
            }
        }
        inline bool operator!=(const const_iterator &it)const{return it.current!=current;}
        inline const_iterator& operator ++(    ){
            assert(current<_container._data.size());
            do{
                current++;
                if ( current< _container._data.size()){
                    if( _container._data[current].first) break;
                }
                else break;
            }while(true);
            return *this;
        }
        inline const T&  operator*() const
        {
            assert(current<_container._data.size());
            assert(_container._data[current].first);
            return _container._data[current].second;
        }
        inline const T*  operator->() const
        {
            assert(current<_container._data.size());
            assert(_container._data[current].first);
            return &_container._data[current].second;
        }
        //returns the current position
        inline uint32_t pos()const{return current;}


        const ReusableContainer &_container;
        uint32_t current;
    };
    struct  iterator{
        iterator(  ReusableContainer &cnt,uint32_t idx):_container(cnt),current(idx){
            if (_container._data.size()==0)return;
            while(  current<_container._data.size()) {
                if ( _container._data[current].first==false )             current++;
                else break;
            }
        }
        inline bool operator!=(const iterator &it)const{return it.current!=current;}
        inline iterator& operator ++(){
            assert(current<_container._data.size());
            do{
                current++;
                if ( current< _container._data.size()){
                    if( _container._data[current].first) break;
                }
                else break;
            }while(true);
            return *this;
        }
        inline T&  operator*() const
        {
            assert(current<_container._data.size());
            assert(_container._data[current].first);
            return _container._data[current].second;
        }
        inline T*  operator->() const
        {
            assert(current<_container._data.size());
            assert(_container._data[current].first);
            return &_container._data[current].second;
        }

        //returns the current position
        inline uint32_t pos()const{return current;}

        ReusableContainer &_container;
        uint32_t current;
    };

    //inserts an element and returns an interator to the element and its index in the vector
    inline std::pair<iterator,uint32_t>insert(const T&val){
        //is there available spaces removed?
        uint32_t eidx;
        if (_emptySpaces.size()!=0){
            eidx=_emptySpaces.back();
            _emptySpaces.pop_back();
            _data[ eidx]= {true,val};
        }
        else{
//            if (_data.capacity()==_data.size()){
//                _data.reserve(_data.capacity()+incSize);
//            }
            eidx=_data.size();
            _data.push_back( {true,val} );
        }
        return {iterator(*this, eidx),eidx};
    }

    //erases the element in the position indicated
    inline void erase(uint32_t index);
    //returns the element in specified index
    inline T& operator[](uint32_t index) {return at(index);}
    inline const T& operator[](uint32_t index)const {return at(index);}
    inline T& at(uint32_t index);
    inline const T& at(uint32_t index)const;

    //indicates if the index-th element is (if false, it has been erased and no replaced)
    inline bool  is(uint32_t index)const;
    inline bool count(uint32_t index)const {return is(index);}
    //returns number of valid elements
    inline uint32_t size()const{ return _data.size()-_emptySpaces.size();}
    inline void toStream(std::ostream &str)const;
    inline void fromStream(std::istream &str) ;

    inline ReusableContainer<T> & operator =(const ReusableContainer<T> & other);
    inline iterator begin()  {return iterator(*this,0);}
    inline  iterator end()  {return iterator(*this,_data.size());}
    inline const_iterator begin()const {return const_iterator(*this,0);}
    inline const_iterator end() const{return const_iterator(*this,_data.size());}

    bool operator ==(const ReusableContainer &c)const;
    //returns the next position that will be assigned
    inline uint32_t getNextPosition()const;
    //returns the last valid element
    inline T&back() ;
    //returns an iterator to the element specified if valid. Else, an end() iterator is returned
//    iterator find(uint32_t index);
//    const_iterator find(uint32_t index)const;
    //returns the first valid element

    inline T&front() ;
    inline const T&front()const ;
    inline void clear();
    //real size of the internal data
    inline uint32_t capacity()const{return _data.capacity();}

    //sets the increment size. It established how many new elements are created when more space is needed
    void setIncSize(int s){incSize=s;}
private:

    struct Pair{
        bool first;
        T second;
        void toStream(std::ostream &str) { str.write((char*)&first,sizeof(first));second.toStream(str);}
        void fromStream(std::istream &str){ str.read((char*)&first,sizeof(first));second.fromStream(str);}
    };

//    std::pair<bool, T >

    ExpansibleContainer<Pair>  _data;
    std::vector<uint32_t> _emptySpaces;

    int incSize=200;
};

template<typename T>
inline uint32_t ReusableContainer<T>::getNextPosition()const{
    if (_emptySpaces.size()!=0)return _emptySpaces.back();
    return _data.size();
}

template<typename T>
inline T& ReusableContainer<T>::back(void) {
    assert( size()>0);
    int64_t elm=size()-1;
    while( _data[elm].first==false && elm>=0) elm--;
    assert(elm>=0);
    return _data[elm].second;
}

template<typename T>
inline T& ReusableContainer<T>::front(void) {
    assert( size()>0);
    size_t elm=0;
    while( _data[elm].first==false && elm<_data.size()) elm++;
    assert(elm<_data.size());
    return _data[elm].second;
}

template<typename T>
inline const T& ReusableContainer<T>::front(void) const{
    assert( size()>0);
    size_t elm=0;
    while( _data[elm].first==false && elm<_data.size()) elm++;
    assert(elm<_data.size());
    return _data[elm].second;
}

template<typename T>
inline void ReusableContainer<T>::erase(uint32_t index){
    assert(index<_data.size());
    assert(std::find(_emptySpaces.begin(),_emptySpaces.end(),index)==_emptySpaces.end());
    assert(_data[index].first!=false);
    _emptySpaces.push_back(index);
    _data[index].first=false;
}
template<typename T>
void ReusableContainer<T>::clear(){
    _emptySpaces.clear();
    _data.clear();
}
template<typename T>
ReusableContainer<T> & ReusableContainer<T>::operator =(const ReusableContainer<T> & other){
    _emptySpaces=other._emptySpaces;
    _data=other._data;
    return *this;
}


template<typename T>
inline T& ReusableContainer<T>::at(uint32_t index){
    assert(index<_data.size());
    assert(_data[ index ].first==true);
    return    _data[ index ].second ;
}
template<typename T>
inline const T& ReusableContainer<T>::at(uint32_t index)const{
    assert(index<_data.size());
    assert(_data[ index ].first==true);
    return    _data[ index ].second ;
}

//template<typename T>
//typename ReusableContainer<T>::iterator ReusableContainer<T>::find(uint32_t index){
//    if ( index>=_data.size())return iterator(*this,_data.size());
//    if ( _data[index].first==false)return iterator(*this,_data.size());
//    return iterator(*this,index);
//}

//template<typename T>
//typename ReusableContainer<T>::const_iterator ReusableContainer<T>::find(uint32_t index)const{
//    if ( index>=_data.size())return iterator(*this,_data.size());
//    if ( _data[index].first==false)return iterator(*this,_data.size());
//    return const_iterator(*this,index);
//}

template<typename T>
inline bool ReusableContainer<T>::is(uint32_t index)const{
    if ( index>=_data.size()) return false;
    return _data[ index ].first;
}

template<typename T>
void ReusableContainer<T>::toStream(std::ostream &str) const{
    int64_t magic=123299999;
    str.write((char*)&magic,sizeof(magic));
    //
    uint32_t s=_emptySpaces.size();
    str.write((char*)&s,sizeof(s));
    str.write((char*)&_emptySpaces[0],sizeof(_emptySpaces[0])*_emptySpaces.size());

    _data.toStream(str);
//    s=_data.size();
//    str.write((char*)&s,sizeof(s));
//    for(auto &datum:_data) {
//        str.write((char*)&datum.first,sizeof(datum.first));
//        datum.second.toStream(str);
//    }

}

template<typename T>
void ReusableContainer<T>::fromStream(std::istream &str) {
    int64_t magic=123299999;
    str.read((char*)&magic,sizeof(magic));
    if (magic!=123299999) throw std::runtime_error("ReusableContainer<T>::fromStream invalid signature");
    //
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    _emptySpaces.resize(s);
    str.read((char*)&_emptySpaces[0],sizeof(_emptySpaces[0])*_emptySpaces.size());

    _data.fromStream(str);
//    str.read((char*)&s,sizeof(s));
//    _data.resize(s);
//    for(auto &datum:_data) {
//        str.read((char*)&datum.first,sizeof(datum.first));
//        datum.second.fromStream(str);
//    }
}
template<typename T>
bool ReusableContainer<T>::operator ==(const ReusableContainer &c) const{
    if (c._data.size()!=c._data.size())return false;
    if (c._emptySpaces.size()!=_emptySpaces.size())return false;
    for(size_t i=0;i<c._data.size();i++){
        if (c._data[i].first!=_data[i].first) return false;
        if (c._data[i].first)
            if (!(c._data[i].second==_data[i].second) ) return false;
    }
    for(size_t i=0;i<c._emptySpaces.size();i++)
        if (c._emptySpaces[i]!=_emptySpaces[i])return false;
    return true;
}

}
#endif
