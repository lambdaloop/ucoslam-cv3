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
#ifndef UCOSLAM_GRIDEXTRACTOR_H
#define UCOSLAM_GRIDEXTRACTOR_H
#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "feature2dserializable.h"
namespace ucoslam
{


//divide the image in a grid of patches and computes the features in them independently
class GridExtractor:public Feature2DSerializable{
public:

    //nParts for each axis
    void setParams(F2S_Type type );
    void detectAndCompute_impl( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors,FeatParams params );
    void toStream_impl(std::ostream &str);
    void fromStream_impl(std::istream &str);
    F2S_Type getType()const{return _ftype;}


    Feature2DSerializable::FeatParams getParams()const{return _featParams;}

    virtual float getMinDescDistance()const;
    DescriptorTypes::Type getDescriptorType()const;
private:
    void  detectAndCompute_thread(cv::InputArray _image, cv::InputArray _mask, cv::Ptr<cv::Feature2D> extractor, cv::Ptr<cv::Feature2D> detector, std::vector<cv::KeyPoint> &vkeypoints,  cv::Mat &vdesc, int y);

    void  createDetectors(Feature2DSerializable::FeatParams param,cv::Size imageSize);

     Feature2DSerializable::FeatParams _featParams;
    F2S_Type _ftype;
     std::vector<cv::Ptr<cv::Feature2D> >_fextractors;
    std::vector<cv::Ptr<cv::Feature2D> >_fdetectors;
     bool _areParamsSet=false;
    int _overlapborder=0;
};

}

#endif
