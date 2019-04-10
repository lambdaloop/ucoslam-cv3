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
#ifndef UCOSLAM_FrameDataBase_H
#define UCOSLAM_FrameDataBase_H
#include <iostream>
#include <memory>
#include <vector>
#include <set>
#include "ucoslam_exports.h"
namespace ucoslam{
class KFDataBaseVirtual;
class Frame;
class FrameSet;
class CovisGraph;

class UCOSLAM_API KeyFrameDataBase{
public:
    KeyFrameDataBase();
    void loadFromFile(const std::string &filePathOrNothing);
    bool isEmpty()const;
    bool isId(uint32_t id)const ;

    bool add(Frame &f);
    bool del(const Frame &f);
    void clear();

    size_t size()const;
    void toStream(std::iostream &str)const ;
    void fromStream(std::istream &str);
    std::vector<uint32_t> relocalizationCandidates(Frame &frame, FrameSet &fset, CovisGraph &covisgraph, bool sorted=true, float minScore=0, const std::set<uint32_t> &excludedFrames={});
    float score(Frame &f,Frame &f2);

    uint64_t getSignature()const;
private:
    std::shared_ptr<KFDataBaseVirtual> _impl;
    int _type=-1;
};
}
#endif
