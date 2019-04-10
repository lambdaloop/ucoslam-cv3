#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <Eigen/Geometry>
#include <logtools.h>
using namespace std;


std::map<int,string>   loadFile(std::string fp ){
  map<int,string>   fmap;
   ifstream file(fp);
   if(!file)throw std::runtime_error("Could not open file");
   string stamp;
   float tx,ty,tz,qx,qy,qz,qw;
   cv::Mat firstFrameT;
   while(!file.eof()){
       string line;
       std::getline(file,line);
       stringstream sline;sline<<line;
        if (sline>>stamp>>tx>>ty>>tz>>qx>>qy>>qz>>qw){
           fmap.insert( {std::stoi(stamp), line});//refers everything to the first frame
       }
    }

   //now, find the transform from every frame to the first
   return fmap;
}


int main(int argc,char **argv){


    try{
        if (argc!=3)throw std::runtime_error(" Usage: in out");

        auto gt= loadFile(argv[1]);
        //replace by a in,mat
        ofstream file(argv[2]);
        for(auto g:gt)
            file<<g.second<<endl;



    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
