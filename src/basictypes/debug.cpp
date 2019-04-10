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
#include "debug.h"
#include <fstream>
namespace ucoslam{
namespace debug{
int Debug::level=0;
std::map<std::string,std::string> Debug::strings;
bool Debug::_showTimer=false;
void Debug::showTimer(bool v){  _showTimer=v;}

void Debug::addString(const std::string &label, const  std::string &data){
    strings.insert(make_pair(label,data));
}

std::string Debug::getString(const std::string &str){
    auto it=strings.find(str);
    if (it==strings.end())return "";
    else return it->second;
}

bool Debug::isString(const std::string &str){
 return strings.count(str)!=0;
}

bool Debug::isInited=false;

void Debug::setLevel ( int l ) {
    level=l;
    isInited=false;
    init();
}
int Debug::getLevel() {
    init();
    return level;
}
void Debug::init() {
    if ( !isInited ) {
        isInited=true;
        if ( level>=1 ) {
        }
    }

}


}
}

