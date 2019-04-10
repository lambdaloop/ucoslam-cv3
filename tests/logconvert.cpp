//valid to create the corresponding logs for TUM, EUROC-MAV
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <cmath>
using namespace std;

vector<string> parseLine(string line){

    std::replace( line.begin(), line.end(), ',', ' '); // replace all 'x' to 'y'
    stringstream sstr;sstr<<line;
    vector<string> data;
    string elem;
    while( sstr>>elem) data.push_back(elem);
    return data;
}

std::pair<double,string> lineSepare(string line){
    stringstream sstr;
    sstr<<line;
    double v;
    sstr>>v;

    string restOfLine;
    while(!sstr.eof()){
        string val;
        if(sstr>>val) restOfLine+=val+" ";
    }
    return {v,restOfLine};
}

double getStamp(string line){
}


std::vector<std::pair<double,string> > readFile(string path){

    ifstream   fileIn(path);
    if(!fileIn)throw std::runtime_error("Could not open file");
    std::vector<std::pair<double,string>> result;
    string line;
    while(!fileIn.eof()){
        std::getline(fileIn,line);
        if(line.find("#")!=std::string::npos)continue;
        std::replace( line.begin(), line.end(), ',', ' '); // replace all 'x' to 'y'
        auto ss=lineSepare(line);
        //cout<<ss.first<<"||"<<ss.second<<endl;
        result.push_back(lineSepare(line));
    }
    return result;
}
int findNearest(std::vector<pair<uint64_t,std::string> > &gt, uint64_t time,int start=0){
    if ( time < gt[0].first)
        return -1;
    cout<<time<<" ";
    double dif=std::numeric_limits<double>::max();
    for(int i=start;i<gt.size();i++){
        cout<<gt[i].first<<endl;;
        double a=gt[i].first;
        double b=time;
        auto cdif=fabs(a-b);
        if (cdif>dif)
            return i;
        else dif=cdif;
    }
    return gt.size()-1;
}


vector<pair<int,std::string> > getMatchedLocations(std::vector<pair<double,std::string> > &gt,std::vector<pair<double,std::string> > &other){
   vector< pair<int,std::string> >  res;
   int o_index=0;
   while( other[o_index].first< gt[0].first)   o_index++;

   int gt_index=0;
    for(size_t i=o_index;i< other.size();i++){
       auto &frame_est=other[i];
       while( gt_index<gt.size()-1){
           if ( fabs( gt[gt_index].first- frame_est.first) < fabs( gt[gt_index+1].first- frame_est.first)){
               res.push_back({i, gt[gt_index].second });
               //cout<<std::setprecision(15)<<gt[gt_index].first<<" "<<frame_est.first<<endl;
               gt_index=std::max(0,gt_index-10);
               break;
           }
           gt_index++;
       }
       if ( gt_index==gt.size()-1)
           res.push_back({ i, gt[gt_index].second });
   }
   return res;
}

int main(int argc,char **argv){


    try{
        if (argc<4)throw std::runtime_error(" Usage: inGt inCam out [startFrame]");

        auto gt=readFile(argv[1]);
        auto cam=readFile(argv[2]);

        auto res=getMatchedLocations(gt,cam);
        cout<<cam.size()<<" "<<res.size()<<endl;

        ofstream outFile(argv[3]);
        outFile<<"#frame tx ty tz qw qx qy qz ..."<<endl;
        int startFrame=0;
        if(argc>=5) startFrame=stoi(argv[4]);

        for(int i=0;i<res.size();i++)
            if (res[i].first>=startFrame)
                outFile<< int(res[i].first)-startFrame<<" "<<res[i].second<<endl;

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
