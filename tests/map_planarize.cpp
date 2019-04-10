#include "ucoslam.h"
#include "basictypes/misc.h"
#include "optimization/globaloptimizer.h"
#include <iostream>
using namespace std;
void printError(std::shared_ptr<ucoslam::Map> map,uint32_t ref_marker){

    if(! map->map_markers[ref_marker].pose_g2m.isValid()){cerr<<"EEEE"<<endl;return;}
    double sumErr=0;

    for(auto m:map->map_markers){
        if( m.first==ref_marker) continue;
        if( !m.second.pose_g2m.isValid()) continue;
        cv::Mat m12= map->map_markers[ref_marker].pose_g2m.inv()*  m.second.pose_g2m;
        cout<<m.first<<":"<<m12.at<float>(2,3)<< ", ";
        sumErr+=fabs(m12.at<float>(2,3));

    }
    cout<<" sum="<<sumErr<<endl;
}

void printError(std::shared_ptr<ucoslam::Map> map){
    double sumErr=0;
    int n=0;
    for(auto m:map->map_markers){
        if( !m.second.pose_g2m.isValid()) continue;
        cv::Mat error(1,4,CV_32F);
        float *_error=error.ptr<float>(0);
        _error[0]=m.second.pose_g2m.at<float>(0,2);
        _error[1]=m.second.pose_g2m.at<float>(1,2);
        _error[2]=1-m.second.pose_g2m.at<float>(2,2);
        _error[3]=m.second.pose_g2m.at<float>(2,3);
        sumErr+=m.second.pose_g2m.at<float>(2,3);
        cout<<m.second.pose_g2m.at<float>(2,3)<<" ";
        n++;
    }
    cout<<endl;
    cout<<"sumErr="<<sumErr <<endl;

}

class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };
int main(int argc,char **argv){

    try {
        CmdLineParser cml(argc,argv);
        if(argc<3)throw std::runtime_error("USage: in.map out.map [-n niters]");

        std::shared_ptr<ucoslam::Map>   map=std::make_shared<ucoslam::Map>();
        map->readFromFile(argv[1]);




        map->centerRefSystemInMarker(0);
        printError(map,0);
        auto opt=ucoslam::GlobalOptimizer::create( "g2o");
        ucoslam::GlobalOptimizer::ParamSet params( true);
        params.fixFirstFrame=true;
        params.nIters=stoi(cml("-n","20"));
        params.minStepErr=1e-5;
        params.markersOptWeight= 0.5;
        params.minMarkersForMaxWeight=5;
        params.InPlaneMarkers=true;
        opt->optimize(map,params );
        printError(map,0);
        map->saveToFile(argv[2]);


    } catch (std::exception &ex) {
        cerr<<ex.what()<<endl;
    }
}
