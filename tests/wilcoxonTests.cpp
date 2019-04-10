#include <map>
#include <set>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iomanip>
#include "dirreader.h"
#include "logtools.h"
using namespace  std;
using namespace  ucoslam::logtools;


cv::Point3f  getT(const cv::Mat &m){
    if(m.type()==CV_64F)
        return cv::Point3f(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3));
    else
        return cv::Point3f(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3));
};
TestResult analyzeTest(TestInfo &ti){


    TestResult tr;
    tr.ti=ti;
    tr.gt=ucoslam::logtools::loadFile(ti.gt_file);
    tr.poses=ucoslam::logtools::loadFile(ti.fullPath);



    tr.matchedFrames =ucoslam::logtools::getMatchedLocations(tr.gt,tr.poses);
    if (tr.matchedFrames .size()==0) return tr;
    ucoslam::logtools::alignAndScaleToGroundTruth(tr.matchedFrames );
    //get the ATE error
    double e=0;
    for(auto &p:tr.matchedFrames )
        e+=p.error=cv::norm(getT(p.first)-getT(p.second));

    tr.ATE=e/double(tr.matchedFrames .size());
    tr.perctFramesTracked=100* double(tr.matchedFrames .size())/double( tr.gt.size());
   // ucoslam::logtools::getMatchedLocations_io(tr.poses,tr.poses);

    return tr;
}



map<string,TestResult> analyzeDataSet(string ResultsPath, string DataSetPath ){
    auto Tests=getDataSetTestInfo(ResultsPath,DataSetPath);
    map<string,TestResult> Results;
    for(auto test:Tests){
        TestResult tr=analyzeTest(test);
        Results[tr.ti.getKey()]=tr;
    }
    return Results;
}

struct PairWiseComparison{
    float ATE_A=std::numeric_limits<float>::max(),ATE_B=std::numeric_limits<float>::max(),TRACK_A=0,TRACK_B;
};

//returns the following values
//0: ATE of first method
//1: ATE of second method
//2: Perct of Frames tracket of first method
//3: Perct of Frames tracket of second method
struct comparison{
    std::string seq_name,cam,method1,method2;
    float ate1=std::numeric_limits<float>::max(),ate2=std::numeric_limits<float>::max(),track1=0,track2=0;
};

comparison  compareMethods2(TestInfo &t1,TestInfo &t2)
{
    comparison comp;
    comp.seq_name=t1.seq_name;
    comp.cam=t1.cam;
    comp.method1=t1.method;
    comp.method2=t2.method;
    //analyze the whole sequence individually, and then, take the errors only in the common frames
    auto t1res=analyzeTest(t1);
    auto t2res=analyzeTest(t2);


//    cout<<t1res.ATE<<" "<<t1res.perctFramesTracked<<" | ";
//    cout<<t2res.ATE<<" "<<t2res.perctFramesTracked<<" | ";

    //intersect the frames tracked to recompute the ate with them
    for(auto it=t1res.matchedFrames.begin();it!=t1res.matchedFrames.end();){
        if (t2res.isFrame(it->frame)) it++;
        else it=t1res.matchedFrames.erase(it);
    }

    for(auto it=t2res.matchedFrames.begin();it!=t2res.matchedFrames.end();){
        if (t1res.isFrame(it->frame)) it++;
        else it=t2res.matchedFrames.erase(it);
    }

    if( t1res.matchedFrames.size()>0){

    ucoslam::logtools::alignAndScaleToGroundTruth(t1res.matchedFrames );
    for(auto &p:t1res.matchedFrames )
        p.error=cv::norm(getT(p.first)-getT(p.second));
    ucoslam::logtools::alignAndScaleToGroundTruth(t2res.matchedFrames );
    for(auto &p:t2res.matchedFrames )
        p.error=cv::norm(getT(p.first)-getT(p.second));



    //recompute ates
    double err=0;

    for(const auto &m:t1res.matchedFrames)
        err+=m.error;
    t1res.ATE=err/double(t1res.matchedFrames.size());
    err=0;
    for(const auto &m:t2res.matchedFrames)
        err+=m.error;
    t2res.ATE=err/double(t2res.matchedFrames.size());

    comp.ate1=t1res.ATE;
    comp.ate2=t2res.ATE;
    comp.track1=t1res.perctFramesTracked;
    comp.track2=t2res.perctFramesTracked;
     }
    else{
        t1res.ATE=0;
        t2res.ATE=0;
    }

    return comp;

}

vector<float>  compareMethods(TestInfo &t1,TestInfo &t2)
{
    comparison comp;
    comp.seq_name=t1.seq_name;
    comp.cam=t1.cam;
    comp.method1=t1.method;
    comp.method2=t2.method;
    //analyze the whole sequence individually, and then, take the errors only in the common frames
    auto t1res=analyzeTest(t1);
    auto t2res=analyzeTest(t2);


//    cout<<t1res.ATE<<" "<<t1res.perctFramesTracked<<" | ";
//    cout<<t2res.ATE<<" "<<t2res.perctFramesTracked<<" | ";

    //intersect the frames tracked to recompute the ate with them
    for(auto it=t1res.matchedFrames.begin();it!=t1res.matchedFrames.end();){
        if (t2res.isFrame(it->frame)) it++;
        else it=t1res.matchedFrames.erase(it);
    }

    for(auto it=t2res.matchedFrames.begin();it!=t2res.matchedFrames.end();){
        if (t1res.isFrame(it->frame)) it++;
        else it=t2res.matchedFrames.erase(it);
    }

    if( t1res.matchedFrames.size()>0){

    ucoslam::logtools::alignAndScaleToGroundTruth(t1res.matchedFrames );
    for(auto &p:t1res.matchedFrames )
        p.error=cv::norm(getT(p.first)-getT(p.second));
    ucoslam::logtools::alignAndScaleToGroundTruth(t2res.matchedFrames );
    for(auto &p:t2res.matchedFrames )
        p.error=cv::norm(getT(p.first)-getT(p.second));



    //recompute ates
    double err=0;

    for(const auto &m:t1res.matchedFrames)
        err+=m.error;
    t1res.ATE=err/double(t1res.matchedFrames.size());
    err=0;
    for(const auto &m:t2res.matchedFrames)
        err+=m.error;
    t2res.ATE=err/double(t2res.matchedFrames.size());

    comp.ate1=t1res.ATE;
    comp.ate2=t2res.ATE;
    comp.track1=t1res.perctFramesTracked;
    comp.track2=t2res.perctFramesTracked;
     }
    else{
        t1res.ATE=0;
        t2res.ATE=0;
//        t1res.perctFramesTracked=0;
//        t2res.perctFramesTracked=0;
    }

    //    cout<<t1res.ATE<<" "<<t2res.ATE<<endl;
    vector<float> res(4);
    res[0]=t1res.ATE;
    res[1]=t2res.ATE;
    res[2]=t1res.perctFramesTracked;
    res[3]=t2res.perctFramesTracked;
    return res;

}

//bool wilcoxonTest(const vector<vector<float>> &data){
//    vector<std::pair<float,int> > wdata;

//    for(const auto &v:data){
//        wdata.push_back({v[0],0 });
//        wdata.push_back({v[1],1 });
//    }

//    //rank
//    std::sort(wdata.begin(),wdata.end(),[](const std::pair<float,int> &a,const std::pair<float,int>&b){return a.first<b.first;});
//    //compute ranks
//    double ranks[2]={0,0};
//    for(auto w:wdata)
//        ranks[w.second]+=w.first;


//}
/////////////////////////////////////
string getSizedString(string in,int minMaxSize){
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back(' ');
    in.resize(minMaxSize);
    return in;
};
string getSizedString(double v,int minMaxSize){
    string in=std::to_string(v);
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back(' ');
    in.resize(minMaxSize);
    return in;
};
struct winnerData{
    float winA_ATE=0,winB_ATE=0;
    float winA_Tracked=0,winB_Tracked=0;
    float scoreA=0,scoreB=0;
};


winnerData winner(const vector<vector<float>> &data,float p){
    winnerData wdata;

    for(const auto &v:data){
        int wA_ATE=0,wB_ATE=0,wA_T=0,wB_T=0;
        float diff=fabs(v[0]-v[1]);
        if ((diff> p* std::min(v[0],v[1]))){
          if( (v[0]-v[1])<0)
              wA_ATE=1;
            else
              wB_ATE=1;
         }

        diff=fabs(v[2]-v[3]);
        if (diff> p* std::max(v[2],v[3])){

            if( (v[2]-v[3])>0)
                wA_T=1;
            else
                wB_T=1;
        }

        wdata.winA_ATE+=wA_ATE;
        wdata.winB_ATE+=wB_ATE;
        wdata.winB_Tracked+=wB_T;
        wdata.winA_Tracked+=wA_T;

        if( wA_ATE && wA_T) wdata.scoreA+=1;
        if( wA_ATE && wA_T==wB_T ) wdata.scoreA+=0.5;
        if( wA_ATE==wB_ATE && wA_T) wdata.scoreA+=0.5;

        if( wB_ATE && wB_T) wdata.scoreB+=1;
        if( wB_ATE && wA_T==wB_T) wdata.scoreB+=0.5;
        if( wA_ATE==wB_ATE && wB_T) wdata.scoreB+=0.5;

    }
    wdata.winA_Tracked/=double(data.size());;
    wdata.winB_Tracked/=double(data.size());;
    wdata.winA_ATE/=double(data.size());;
    wdata.winB_ATE/=double(data.size());;

    return wdata;
}

winnerData winner2(const vector<comparison> &data,float p){
    winnerData wdata;

    for(const auto &v:data){
        int wA_ATE=0,wB_ATE=0,wA_T=0,wB_T=0;
        float diff=fabs(v.ate1-v.ate2);
        if ((diff> p* std::min(v.ate1,v.ate2))){
          if( v.ate1< v.ate2)
              wA_ATE=1;
            else
              wB_ATE=1;
         }

        diff=fabs(v.track1-v.track2);
        if (diff> p* std::max(v.track1,v.track2)){
            if( v.track1 > v.track2)
                wA_T=1;
            else
                wB_T=1;
        }

        wdata.winA_ATE+=wA_ATE;
        wdata.winB_ATE+=wB_ATE;
        wdata.winB_Tracked+=wB_T;
        wdata.winA_Tracked+=wA_T;

        float scoreA=0,scoreB=0;
        if( wA_ATE && wA_T)  scoreA+=1;
        if( wA_ATE && wA_T==wB_T )  scoreA+=0.5;
        if( wA_ATE==wB_ATE && wA_T)  scoreA+=0.5;

        if( wB_ATE && wB_T)  scoreB+=1;
        if( wB_ATE && wA_T==wB_T)  scoreB+=0.5;
        if( wA_ATE==wB_ATE && wB_T) scoreB+=0.5;

        wdata.scoreA+=scoreA;
        wdata.scoreB+=scoreB;


//        cout<<v.seq_name<<" "<<v.cam<<" "<<v.method1<<" "<<v.method2<< " |"<<wA_ATE<<":"<<wB_ATE<< "-"<<wA_T<<":"<<wB_T << "|"<<scoreA<<":"<<scoreB  <<"| ATE= "<<v.ate1<<" TRACK="<<v.track1<<" ATE="<<v.ate2<<
//              " TRACK="<<v.track2<<"|"<<fabs(v.ate1-v.ate2)<<">"<<p* std::min(v.ate1,v.ate2)<<"|"<<diff<<">"<<p* std::max(v.track1,v.track2)<<endl;


    }
    wdata.winA_Tracked/=double(data.size());;
    wdata.winB_Tracked/=double(data.size());;
    wdata.winA_ATE/=double(data.size());;
    wdata.winB_ATE/=double(data.size());;
    return wdata;
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
    try{

        CmdLineParser cml(argc,argv);
        if (argc<3 || cml["-h"])throw std::runtime_error(" Usage: ResultsDir ucoslam-dataset  [-p <float>. Default 0.01] ");

        std::string ResultsBaseDir=argv[1];
        std::string TheDataSetBaseDir=argv[2];

        auto methods=DirReader::read(argv[1]);
        for(auto &m:methods) m=DirReader::basename(m);
        auto datasets=DirReader::read(argv[2]);
        for(auto &d:datasets) d=DirReader::basename(d);
        float pvalue=stof(cml("-p","0.01"));

        //        vector<string> datasets;
        /*

        datasets.clear();
        datasets.push_back("SPM");*/
        //do a pairwise comparison
        if (methods.size()<=1)throw std::runtime_error("Only one method. No comparison possible.");
        for(int m=0;m<methods.size()-1;m++)
            for(int m2=m+1;m2<methods.size();m2++){


                std::map<string, std::map<string,TestInfo> > dset_test_infoA,dset_test_infoB;
                std::set<string> allTests;
                for(auto dset:datasets){
                    auto allTestsA=getDataSetTestInfo(ResultsBaseDir+"/"+methods[m]+"/"+dset,TheDataSetBaseDir+"/"+dset);
                    for(auto &ta:allTestsA){
                        dset_test_infoA[dset][ta.getDesc()]=ta;
                        allTests.insert(ta.getDesc());
                    }

                    auto allTestsB=getDataSetTestInfo(ResultsBaseDir+"/"+methods[m2]+"/"+dset,TheDataSetBaseDir+"/"+dset);
                    for(auto &ta:allTestsB){
                        dset_test_infoB[dset][ta.getDesc()]=ta;
                        allTests.insert(ta.getDesc());
                    }

                }

                vector< vector< float> >  Results;
                vector< comparison >  Results2;
                for(auto dset:datasets){
                    for(auto test:allTests){
                        if ( dset_test_infoA[dset].count(test) &&  dset_test_infoB[dset].count(test) ){
                            // cout<<dset<<endl;
                            Results.push_back(compareMethods( dset_test_infoA[dset].at(test), dset_test_infoB[dset].at(test)));
                            Results2.push_back(compareMethods2( dset_test_infoA[dset].at(test), dset_test_infoB[dset].at(test)));
                        }
                    }
                }


                // cout<<"W="<<wilcoxonSignedRankTest(Results)<<endl;
                //                auto res=winner(Results,pvalue);
                auto res=winner2(Results2,pvalue);
                cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
                cout<<"@@@@@ " <<methods[m]<<" "<<methods[m2]<<"  :Score A-B ="<<double(res.scoreA-res.scoreB)/double(Results.size())<<endl;
                cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
            }







    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
