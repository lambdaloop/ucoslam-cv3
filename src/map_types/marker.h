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
#ifndef ucoslam_Marker_H
#define ucoslam_Marker_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <set>
#include "ucoslam_exports.h"
#include "basictypes/se3.h"
#include "basictypes/se3transform.h"
namespace ucoslam {

/**A marker in the 3D space.
 */

class UCOSLAM_API Marker{
public:
    Marker(){}
    Marker(uint32_t Id,Se3Transform g2m,float Size):id(Id),pose_g2m(g2m),size(Size){}
    uint32_t id;//id
    Se3Transform pose_g2m=Se3Transform(true);//pose  Marker -> Global
    float size=0;//marker size
    std::set<uint32_t> frames;//key frames in which the marker is visible
    std::string dict_info; //information about the dictionary it belongs to

    //returns the 3d points of the marker in the global_ref
    std::vector<cv::Point3f> get3DPoints(bool global_ref=true)const;
    static std::vector<cv::Point3f> get3DPoints(Se3Transform pose_g2m, float size, bool global_ref=true);
    //returns the 3d points in the local reference system
    static std::vector<cv::Point3f> get3DPointsLocalRefSystem( float size );
    //---------------------
    //serialization routines
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;
    uint64_t getSignature(bool print=false)const;

};

//define the set of poses returned by the IPPE algorithm for a given marker
struct MarkerPosesIPPE
{
    cv::Mat sols[2];
    double errs[2];
    double err_ratio;
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);

    MarkerPosesIPPE(){}
    MarkerPosesIPPE(const MarkerPosesIPPE&M)
    {
        M.copyTo(*this);
    }
    MarkerPosesIPPE & operator=(const MarkerPosesIPPE&M){
        M.copyTo(*this);
        return *this;
    }

    void copyTo(MarkerPosesIPPE &mposes)const{
        for(int i=0;i<2;i++){
            sols[i].copyTo(mposes.sols[i]);
            mposes.errs[i]=errs[i];
        }
        mposes.err_ratio=err_ratio;
    }
    uint64_t getSignature()const;

};
/**The projection of a marker in an image
 */
class UCOSLAM_API MarkerObservation{
public:
    std::vector<cv::Point2f> corners;//original corners in the image
    std::vector<cv::Point2f> und_corners;//undistored corners
    float ssize;
    int id;

    std::string dict_info; //information about the dictionary it belongs to

    MarkerPosesIPPE poses;

    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str) ;
    uint64_t getSignature()const;

    void draw(cv::Mat& in, cv::Scalar color=cv::Scalar(0,0,255), int lineWidth = -1, bool writeId = true,bool writeInfo=false) const;

};

};
#endif
