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

#include "mapviewer.h"
#include <opencv2/highgui/highgui.hpp>


int main( int  argc , char**  argv )
{
    if (argc<2){cerr<<"Usage: world [-system]"<<endl;return -1;}
    std::shared_ptr<ucoslam::Map> map=std::make_shared<ucoslam::Map>();
    bool isSlam=false;
    if(argc>=3){
        if(string(argv[2])=="-system")
            isSlam=true;
    }
    if(isSlam){
        ucoslam::UcoSlam SlamSystem;
        SlamSystem.readFromFile(argv[1]);
        map=SlamSystem.getMap();
    }
    else
        map->readFromFile(argv [1]);


    cout<<"Npoints="<<map->map_points.size()<<endl;
    cout<<"NFrames="<<map->keyframes.size()<<endl;
    ucoslam::MapViewer Theviewer;
    Theviewer.set("mode","0");
    bool finish=false;

    while(!finish){
         int k=Theviewer.show( map ) ;
        if (k==27)finish=true;
    }
    return 0;

}
