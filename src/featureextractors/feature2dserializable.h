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
#ifndef UCOSLAM_FEATURE2DSerializable_H
#define UCOSLAM_FEATURE2DSerializable_H
#include <memory>
#include <opencv2/features2d/features2d.hpp>
#include "ucoslamtypes.h"


namespace ucoslam{

/**Class that allows serialization of the feature extractor. Is reimplemented by all feature extractors designed
 */
class   Feature2DSerializable{
public:


    struct FeatParams{
        int nthreads=-1;//auto
        int maxFeatures=4000;
        int nOctaveLevels=8;
        float scaleFactor=1.2;
        float sensitivity=0;

        FeatParams(){}
        FeatParams(int MaxFeatures,int NOctaveLevels,float ScaleFactor,int NThreads){
            maxFeatures=MaxFeatures;
            nOctaveLevels=NOctaveLevels;
            scaleFactor=ScaleFactor;
            nthreads=NThreads;
        }
        bool operator==(const FeatParams &fp){return nthreads==fp.nthreads && maxFeatures==fp.maxFeatures && nOctaveLevels==fp.nOctaveLevels && scaleFactor==fp.scaleFactor;}


        std::vector<float> getScaleFactors(){
            std::vector<float> res(nOctaveLevels);
            double sc=1;
            for(int i=0;i<nOctaveLevels;i++) {res[i]=sc;sc*=scaleFactor;}
            return res;
        }

        int getMaxFeatures( ){ return maxFeatures;}

    };

    enum F2S_Type{F2D_ORB=0,F2D_GRID_AKAZE=1,F2D_GRID_BRISK=2,F2D_GRID_ORB=3,F2D_GRID_FREAK=4,F2D_GRID_SURF=5};

    virtual  ~Feature2DSerializable(){}

    static std::shared_ptr<Feature2DSerializable> create(DescriptorTypes::Type type);

    void detectAndCompute( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors, FeatParams params );

    void toStream( std::ostream &str);
    static std::shared_ptr<Feature2DSerializable> fromStream(std::istream &str);

    void setParams(const std::string &params){str_params=params;}

    virtual Feature2DSerializable::FeatParams getParams()const=0;

    virtual float getMinDescDistance()const=0;

    //sets a value to define how sensitive the descriptor is [0,1]. Values  near 0 should be used for well illuminates scenes.
    //Increase the value for dark scenes. Default value is 0
    virtual void setSensitivity(float v){}
    virtual float getSensitivity( ){return 0;}

    //a factor indicating the percentage of keypoints that must be repeated in another frame to consider it redundant
   virtual DescriptorTypes::Type getDescriptorType()const=0;

protected:
    virtual F2S_Type getType()const=0;
    virtual void detectAndCompute_impl( cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors, FeatParams params  )=0;
    virtual void toStream_impl(std::ostream &str)=0;
    virtual void fromStream_impl(std::istream &str)=0;
    std::string str_params;
};
}
#endif
