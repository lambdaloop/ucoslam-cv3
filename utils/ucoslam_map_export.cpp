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

//program that reads a map and exports it to ply or pcd
#include <iostream>
#include "map.h"


int main( int  argc , char**  argv )
{
    try {
        if (argc<2) throw  std::runtime_error ("Usage: map out.(pcd|ply)");
        ucoslam::Map map;
        cerr<<"Reading map"<<endl;
        map.readFromFile(argv[1]);
        cerr<<"Exporting map"<<endl;
        map.exportToFile(argv[2],cv::Scalar(0,0,0),cv::Scalar(255,0,0),cv::Scalar(-1,-1,-1),{1111,1195,1129,1196,1141},cv::Scalar(0,0,255));
        cerr<<"Saved to "<<argv[2]<<endl;

    } catch (std::exception &ex) {
        cerr<<ex.what()<<endl;
    }{}
}
