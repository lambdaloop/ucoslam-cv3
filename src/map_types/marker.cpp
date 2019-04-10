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
#include "map_types/marker.h"
#include "basictypes/misc.h"
#include "basictypes/hash.h"
#include "basictypes/io_utils.h"
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
namespace ucoslam{



std::vector<cv::Point3f> Marker::get3DPoints(bool global_ref)const{
    assert(size>0);
    vector<cv::Point3f> marker_points = { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };
    cv::Mat m44=pose_g2m;
    if (global_ref){
        for(auto &p:marker_points)
            p=mult<float>(m44,p);
    }
    return marker_points;
}

void Marker::toStream(std::ostream &str)const {
    str.write((char*)&id,sizeof(id));
    pose_g2m.toStream(str);
    str.write((char*)&size,sizeof(size));
    toStream__(frames,str);
    toStream__(dict_info,str);

}
void Marker::fromStream(std::istream &str) {
    str.read((char*)&id,sizeof(id));
    pose_g2m.fromStream(str);
    str.read((char*)&size,sizeof(size));
    fromStream__(frames,str);
    fromStream__(dict_info,str);

}

std::vector<cv::Point3f> Marker::get3DPointsLocalRefSystem( float size ){
 return { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };

}


vector<cv::Point3f> Marker::get3DPoints(Se3Transform m44,float size,bool global_ref){
    vector<cv::Point3f> marker_points = { cv::Point3f ( -size/2., size/2.,0 ),cv::Point3f ( size/2., size /2.,0 ),
                                          cv::Point3f ( size/2., -size/2.,0 ),cv::Point3f ( -size/2., -size/2.,0 )  };
    if (global_ref){
        for(auto &p:marker_points)
            p=m44*p;
    }
    return marker_points;
}
uint64_t Marker::getSignature(bool print) const{
    print=false;
    Hash sig;
    sig+=id;
    if(print)cout<<"\t\t\t1. marker"<<id<<" :"<<sig<<endl;
    if(print)cout<<"\t\t\t1. marker"<<id<<" pose=\n\t\t\t\t"<<pose_g2m<<endl;
    if(pose_g2m.isValid()){
        for(int i=0;i<16;i++)
            sig+=pose_g2m.ptr<float>(0)[i];
    }
    if(print)cout<<"\t\t\t2. marker"<<id<<" :"<<sig<<endl;
    sig+=size;
    if(print)cout<<"\t\t\t3. marker"<<id<<" :"<<sig<<endl;
    sig.add(frames.begin(),frames.end());
    if(print)cout<<"\t\t\t4. marker"<<id<<" :"<<sig<<endl;
    sig+=dict_info;
    if(print)cout<<"\t\t\t5. marker"<<id<<" :"<<sig<<endl;
    return sig;
}

void MarkerObservation::toStream(std::ostream &str)const{
    toStream__(dict_info,str);
    toStream__(corners,str);
    toStream__(und_corners,str);
    str.write((char*)&ssize,sizeof(ssize));
    str.write((char*)&id,sizeof(id));
    poses.toStream(str);

}



void MarkerObservation::fromStream(std::istream &str){
    fromStream__(dict_info,str);
    fromStream__(corners,str);
    fromStream__(und_corners,str);
    str.read((char*)&ssize,sizeof(ssize));
    str.read((char*)&id,sizeof(id));
    poses.fromStream(str);
}

uint64_t MarkerObservation::getSignature()const{
    Hash sig;
    for(const auto &c:corners){
        sig+=c.x;
        sig+=c.y;
    }
    for(const auto &c:und_corners){
        sig+=c.x;
        sig+=c.y;
    }
    sig+=ssize;
    sig+=id;
    sig+=poses.getSignature();
    sig+=dict_info;
    return sig;
}


void MarkerObservation::draw(cv::Mat& in,  cv::Scalar color, int lineWidth, bool writeId, bool writeInfo) const
{

    auto _to_string=[](int i){
        std::stringstream str;str<<i;return str.str();
        };

    if (corners.size() != 4)
        return;
    if (lineWidth == -1)  // auto
        lineWidth = static_cast<int>(std::max(1.f, float(in.cols) / 1000.f));
    cv::line(in, corners[0], corners[1], color, lineWidth);
    cv::line(in, corners[1], corners[2], color, lineWidth);
    cv::line(in, corners[2], corners[3], color, lineWidth);
    cv::line(in, corners[3], corners[0], color, lineWidth);

    auto p2 =  cv::Point2f(2.f * static_cast<float>(lineWidth), 2.f * static_cast<float>(lineWidth));
    cv::rectangle(in, corners[0] - p2, corners[0] + p2, cv::Scalar(0, 0, 255, 255), -1, cv::LINE_AA);
    cv::rectangle(in, corners[1] - p2, corners[1] + p2, cv::Scalar(0, 255, 0, 255), lineWidth, cv::LINE_AA);
    cv::rectangle(in, corners[2] - p2, corners[2] + p2, cv::Scalar(255, 0, 0, 255), lineWidth, cv::LINE_AA);



    if (writeId)
    {
        // determine the centroid
        cv::Point cent(0, 0);
        for (int i = 0; i < 4; i++)
        {
            cent.x += static_cast<int>(corners[i].x);
            cent.y +=  static_cast<int>(corners[i].y);
        }
        cent.x /= 4;
        cent.y /= 4;
        std::string str;
        if(writeInfo) str+= dict_info +":";
        if(writeId)str+=_to_string(id);
        cv::putText(in,str, cent,  cv::FONT_HERSHEY_SIMPLEX, std::max(0.5f, float(lineWidth) * 0.3f),
                    cv::Scalar(255 - color[0], 255 - color[1], 255 - color[2], 255), std::max(lineWidth, 2));
    }
}
}
