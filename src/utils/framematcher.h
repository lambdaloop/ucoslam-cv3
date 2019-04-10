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
#ifndef ucoslam_FrameMatcher_H
#define ucoslam_FrameMatcher_H

#include "map_types/frame.h"
namespace  ucoslam {

/**
 * Class to analyze matches between two frames.
 * Internally it decides which implemetation to use. If Kp Dictionary is employed, then
 * advantage is taken to speedup search. Otherwise, flann search is employed
 */
namespace _impl {
class FrameMatcher_impl;
}
class FrameMatcher{
    std::shared_ptr<_impl::FrameMatcher_impl> _impl;
public:
    enum Type: int{TYPE_AUTO=0,TYPE_FLANN=1,TYPE_BOW=2};

    enum Mode: int{MODE_ALL=0,MODE_ASSIGNED=1,MODE_UNASSIGNED=2};


    FrameMatcher(Type t=TYPE_AUTO);
    void setParams(const Frame &trainFrame,Mode mode=MODE_ALL,float minDescDist=std::numeric_limits<float>::max(), float nn_match_ratio=0.8, bool checkOrientation=true, int maxOctaveDiff=1);



    //normal match.
    //maxSearch number of comparisons in the  approximated search
    //only searchs for best and  second best
    std::vector<cv::DMatch> match(const Frame &queryFrame, Mode mode );

    //matches considering epipolar constrains
    //F12 is the SE3 matrix moving points from train 2 query
    //nn : number nearest neighbor searched
    //maxSearch number of comparisons in the  approximated search
    std::vector<cv::DMatch> matchEpipolar(const Frame &queryFrame, Mode mode, const cv::Mat &FQ2T );

private:
    Type _type;

};

}
#endif

