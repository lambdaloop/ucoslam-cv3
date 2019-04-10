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
#ifndef _UCOSLAM_Debug_H
#define _UCOSLAM_Debug_H
#include <iostream>
#include <fstream>
#include <ctime>
#include "ucoslam_exports.h"
#include <map>
#include <string>
namespace ucoslam{

namespace debug{
class  UCOSLAM_API Debug{
private:
    static  int level;//0(no debug), 1 medium, 2 high
    static bool isInited;

    static std::map<std::string,std::string> strings;

    static bool _showTimer;
public:

    static void showTimer(bool v);
    static bool showTimer(){return _showTimer;}
    static void init();
    static void setLevel(int l);
    static int getLevel();



    static void addString(const std::string &label,const std::string &data="");
    static std::string getString(const std::string &str);
    static bool  isString(const std::string &str);


static std::string getFileName(std::string filepath){
    //go backwards until finding a separator or start
    int i;
    for( i=filepath.size()-1;i>=0;i--){
        if ( filepath[i]=='\\' || filepath[i]=='/') break;
    }
    std::string fn;fn.reserve( filepath.size()-i);
    for(size_t s=i;s<filepath.size();s++)fn.push_back(filepath[s]);
    return fn;
}



};


#ifdef PRINT_DEBUG_MESSAGES
#define _debug_exec(level,x) {if (debug::Debug::getLevel()>=level){x}}
#define _debug_exec_( x) x

#ifndef WIN32
    #define _debug_msg(x,level) {debug::Debug::init();\
    if (debug::Debug::getLevel()>=level)\
              std::cout<<"#" <<debug::Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

    #define _debug_msg_(x) {debug::Debug::init();\
    if (debug::Debug::getLevel()>=5)\
          std::cout<<"#" <<debug::Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

#else
     #define _debug_msg(x,level) {\
     debug::Debug::init();\
     if (debug::Debug::getLevel()>=level)\
       std::cout<<__func__<<":"<< debug::Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#define _debug_msg_(x) {\
debug::Debug::init();\
if (debug::Debug::getLevel()>=5)\
  std::cout<<__func__<<":"<< debug::Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#endif

#else
#define _debug_msg(x,level)
#define _debug_msg_(x)
#define _debug_exec(level,x)
#define _debug_exec_(x)

#endif



}

}

#endif

