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
#ifndef __UCOSLAM_H__
#define __UCOSLAM_H__
#include <memory>
#include <opencv2/core/core.hpp>
#include "ucoslam_exports.h"
#include "ucoslamtypes.h"
#include "imageparams.h"
#include "map.h"
namespace ucoslam{
class UCOSLAM_API UcoSlam{
public:

    //
    UcoSlam();
    //
    ~UcoSlam();


    /**
     * @brief setParams set the required params before processing images
     * @param map a pointer to the map being used. It can be an empty map or an already created one
     * @param params controlling the system behaviuor
     * @param vocabulary optional path to the dictionary BOW. Without it, relocalization is not yet possible.
     */

    void setParams(std::shared_ptr<Map> map, const  ucoslam::Params &params, const std::string &vocabulary="");

    //Clear this object setting it to intial state. It will remove all map data
    void clear();

    //returns system params
    static Params  & getParams() ;


    //Feeds the system with a new image and the parameters of the camera it has been processed with. The final
    //parameter is the index of the sequence
    //Returns the camera pose estimated. The pose is the transform moving points from the global reference sytem to the camera reference ssytem
    cv::Mat process( cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx);
    cv::Mat processStereo( cv::Mat &left_image,const cv::Mat &right_image,const ImageParams &ip,uint32_t frameseq_idx);
    cv::Mat processRGBD( cv::Mat &in_image,const cv::Mat & depth,const ImageParams &ip,uint32_t frameseq_idx);

    //Reset the current frame pose. Use it to start tracking in a known map
     void resetTracker();

    //sets the system mode
    void setMode(MODES mode);

    //sets the system in lost mode
   // void resetCurrentPose();

    //returns the number of the last processed framed
    uint32_t getLastProcessedFrame()const;


    // Saves the current state of the system to a file so that it can be recovered later. It is only safe if the system is in sequential mode
    void saveToFile(std::string filepath);
    //Loads the state of the system from a file
    void readFromFile(std::string filepath);


    //only for debugging pourposes perform the global optimization
    void globalOptimization();

    //waits for all threads to finish
    void waitForFinished();


    //returns the index of the current keyframe
    uint32_t getCurrentKeyFrameIndex();


    //returns a pointer to the map being used
    std::shared_ptr<Map> getMap();


    //returns an string that identifies the system state. It is like a md5 sum the system state
    std::string getSignatureStr()const;


    void setDebugLevel(int level);
    void showTimers(bool v);


    //will update the internal parameters.Not all parameters can be changed
    void updateParams(const  Params &p);

private:
    void *impl;

};
}
#endif
