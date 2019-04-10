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
#ifndef _SafeMap_H
#define _SafeMap_H
#include <map>
namespace ucoslam{
template<typename Key,typename Val>
class SafeMap:public std::map<Key,Val>{
      public:

    inline bool is(const Key &v)const{return std::map<Key,Val>::count(v)!=0;}

    inline  Val & operator [](const Key &v){
        if( !is(v))throw std::runtime_error("Invalid access to element not inserted");
        return (*((std::map<Key,Val>*)this))[v];
    }

    inline  const Val & operator [] (const Key &v)const{
        if( !is(v))throw std::runtime_error("Invalid access to element not inserted");
        return (*((std::map<Key,Val>*)this))[v];
    }
};
}
#endif
