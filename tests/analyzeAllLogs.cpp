#include <map>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iomanip>
#include "dirreader.h"
#include "logtools.h"
using namespace  std;
using namespace  ucoslam::logtools;




/////////////////////////////////////
string getSizedString(string in,int minMaxSize,bool latex=false){
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back(' ');
    in.resize(minMaxSize);

    string out;
    if(latex){

        for(auto c:in){
            if( c=='_')
                out.push_back('\\');
            out.push_back(c);
        }
        in=out;
    }

    return in;
};

string getSizedString(double v,int minMaxSize){
    string in=std::to_string(v);
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back(' ');
    in.resize(minMaxSize);
    return in;
};
string getSizedString2(double v,int minMaxSize){
    stringstream sstr;
    if(v<1e-3) sstr<<"0.0";
    else sstr<< std::setprecision (3)<<v;
    string in= sstr.str();
    bool addPoint=true;
    for(auto c:in)
        if( c=='.') addPoint=false;
    if( addPoint)
        in.push_back('.');
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back('0');
    in.resize(minMaxSize);
    return in;
};

string getSizedString(int64_t v,int minMaxSize){
    stringstream sstr;
    sstr<< v;
    string in= sstr.str();
    bool addPoint=true;
    for(auto c:in)
        if( c=='.') addPoint=false;
    if( addPoint)
        in.push_back('.');
    for(int i=in.size();i<minMaxSize;i++)
        in.push_back('0');
    in.resize(minMaxSize);
    return in;
};

TestResult analyzeTest(TestInfo &ti){
    auto dateToSeconds=[](std::string date){
        stringstream sstr;sstr<<date;
        vector<string > parts(4);
        if( sstr>>parts[0]>>parts[1]>>parts[2]>>parts[3]){
            //remove :
            for(auto &c:parts[3])if(c==':')c=' ';
            //divide
            int day=stoi(parts[2]);
            int hours,minutes,secs;
            stringstream sstr2;sstr2<<parts[3];
            sstr2>>hours>>minutes>>secs;
            return secs+minutes*60+hours*60*60+ day*60*60*24;
        }
        else return -1;

        };
    auto  getT=[](cv::Mat &m){
        if(m.type()==CV_64F)
            return cv::Point3f(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3));
        else
            return cv::Point3f(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3));
    };

    TestResult tr;
    tr.ti=ti;
    tr.gt=ucoslam::logtools::loadFile(ti.gt_file);
    tr.poses=ucoslam::logtools::loadFile(ti.fullPath);
    auto gt_other_2=ucoslam::logtools::getMatchedLocations( tr.gt,tr.poses);
    if (gt_other_2.size()==0) return tr;
    ucoslam::logtools::alignAndScaleToGroundTruth(gt_other_2);


    //get the ATE error
    double e=0;
    for(auto p:gt_other_2)
        e+= cv::norm(getT(p.first)-getT(p.second));
    tr.ATE=e/double(gt_other_2.size());

    tr.perctFramesTracked=100* float(gt_other_2.size())/float( tr.gt.size());
    ucoslam::logtools::getMatchedLocations_io( tr.gt,tr.poses);

    //read time file if it is
    if(!ti.time_file.empty()){
        ifstream timeF(ti.time_file);
        if(timeF.is_open()){
            vector<string> lines;
            while(!timeF.eof()){
                string line;
                std::getline(timeF,line);
                if(!line.empty()) lines.push_back(line);
            }
             if(lines.size()>=2)
                tr.timeMapping=  dateToSeconds(lines[1])-dateToSeconds(lines[0]);
            if(lines.size()>=3)
                tr.timeTracking=  dateToSeconds(lines[2])-dateToSeconds(lines[1]);
        }
    }
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



float wilcoxonSignedRankTestForTime(const  vector<TestResult>  &A,const vector<TestResult> &B){

    vector<float > wdata;

    for(size_t i=0;i<A.size();i++)
        for(size_t j=0;j<A.size();j++){
            wdata.push_back(A[i].timeMapping-B[i].timeMapping);
        }

    //rank
    std::sort(wdata.begin(),wdata.end());
    vector<float > ranks(wdata.size());
    for(size_t i=0;i<ranks.size();i++) ranks[i]=i;
    //if equal elements in the sequence, share the rank

    //set sig to ranks
    for(size_t i=0;i<ranks.size();i++)
        if (wdata[i]<0) ranks[i]*=-1;

 double w=0;
 for(size_t i=0;i<ranks.size();i++)
     w+=ranks[i];

return w;

}


int main(int argc,char **argv){
    try{

        if (argc<3)throw std::runtime_error(" Usage: ResultsDir ucoslam-dataset     [outFile]");

        std::string ResultsBaseDir=argv[1];
        std::string TheDataSetBaseDir=argv[2];

        auto methods=DirReader::read(argv[1]);
        for(auto &m:methods) m=DirReader::basename(m);
        auto datasets=DirReader::read(argv[2]);
        for(auto &d:datasets) d=DirReader::basename(d);

        map<string,map<string,map<string,TestResult>> > dset_method_seq_result;
        for(auto dset:datasets)
            for(auto method:methods)
                dset_method_seq_result[dset] [ method]=analyzeDataSet(ResultsBaseDir+"/"+method+"/"+dset,TheDataSetBaseDir+"/"+dset);




        //present the data in terms of dataset-seq-method instead of in terms of dataset-method-seq

        map<string,map<string,map<string,TestResult>> > dset_seq_method_result;

        for(auto dmsr:dset_method_seq_result){
            for(auto msr:dmsr.second){
                for(auto r:msr.second){
                    dset_seq_method_result[dmsr.first][r.first][msr.first]=r.second;
                }
            }
        }

        map<string, vector< TestResult>  > DatasetResults;
        for(auto dmsr:dset_method_seq_result){
            for(auto msr:dmsr.second){
                for(auto r:msr.second){
                    DatasetResults[msr.first].push_back( r.second);
                }
            }
        }

        for(auto dsmr:dset_seq_method_result){
            cout<<"-------------------------------------"<<endl;
            cout<<dsmr.first<<endl;
            cout<<"-------------------------------------"<<endl;
            for(auto smr:dsmr.second)
                for(auto mr:smr.second)
                    cout<<getSizedString(mr.first,15)<< " "<<getSizedString(mr.second.ti.seq_name,20)<< " "<< getSizedString(mr.second.ti.cam,4)<<" "<<getSizedString(mr.second.ATE,10)<<" "<<getSizedString(mr.second.perctFramesTracked,4)<<endl;
        }
        string TableSep=" , ";
        string NumSep=" ";
        if( 0 ){
            cout<<"METHOD ORDER:";
            for(auto mt:methods)
                cout<<mt<<" ";
            cout<<endl;
            for(auto dsmr:dset_seq_method_result){
                for(auto smr:dsmr.second){
                    map<string,TestResult> method_ti;
                    bool tt=false;
                    for(auto mr:smr.second){
                        method_ti[mr.first]=mr.second;
                        if(tt==false){
                            tt=true;
                            cout <<getSizedString(dsmr.first,10,true)<<TableSep<<getSizedString(mr.second.ti.seq_name,20,true)<< TableSep<< getSizedString(mr.second.ti.cam,4)<< TableSep;
                        }
                    }
                    int counter=0;
                    for(auto mt:methods){
                        auto&t=method_ti[mt];
                        if( t.ATE==-1)
                            cout<<   getSizedString( "N/A",10)<<TableSep<<getSizedString( "0",4);
                        else
                            cout<<   NumSep<<getSizedString2(t.ATE,5)<<NumSep<<TableSep<<NumSep<<getSizedString(t.perctFramesTracked,4)<< NumSep;
                        if( counter++<methods.size()-1)
                            cout<<TableSep;
                    }
                    cout<<"\\\\"<<endl;
                }
            }
        }


        else{//TIMES


            std::map<std::string,std::pair<double,int> > avrgMapping,avrgTracking;
            cout<<"METHOD ORDER:";
            for(auto mt:methods)
                cout<<mt<<" ";
            cout<<endl;
            for(auto dsmr:dset_seq_method_result){
                for(auto smr:dsmr.second){
                    map<string,TestResult> method_ti;
                    bool tt=false;
                    for(auto mr:smr.second){
                        method_ti[mr.first]=mr.second;
                        if(tt==false){
                            tt=true;

                            cout <<getSizedString(dsmr.first,10,true)<<TableSep<<getSizedString(mr.second.ti.seq_name,20,true)<< TableSep<< getSizedString(mr.second.ti.cam,4)<< TableSep;
                        }
                    }
                    int counter=0;
                    for(auto mt:methods){
                        auto&t=method_ti[mt];
                        if(t.timeMapping!=-1){
                       //     cout<<   NumSep<< getSizedString(t.timeMapping* (t.perctFramesTracked/100.),4)<<NumSep<<TableSep;
                            if( avrgMapping.count(mt)==0) avrgMapping[mt]={t.getMappingFPS(),1 };
                            else{
                                avrgMapping[mt].first+=t.getMappingFPS();
                                avrgMapping[mt].second++;
                            }
                            cout<<   NumSep<< getSizedString(t.getMappingFPS(),4)<<NumSep<<TableSep;
                        }
                        else cout<< NumSep<<"  -1"<<NumSep<<TableSep;
                        if( t.timeTracking!=-1){
                            cout<<NumSep<<getSizedString(t.getTrackingFPS(),4)<< NumSep;
                            if( avrgTracking.count(mt)==0) avrgTracking[mt]={t.getTrackingFPS(),1 };
                            else{
                                avrgTracking[mt].first+=t.getTrackingFPS();
                                avrgTracking[mt].second++;
                            }
                        }
                            // cout<<NumSep<<getSizedString(t.timeTracking* (t.perctFramesTracked/100.),4)<< NumSep;
                        else
                            cout<<   NumSep<<"  -1"<<NumSep;

                        // if( counter++<methods.size()-1)
                            cout<<TableSep ;
                    }
                    cout<<"\\\\"<<endl;
                }
            }
            cout<<"Mapping"<<endl;
            for(auto tt:avrgMapping)
                cout<<tt.first<<" ="<<tt.second.first/tt.second.second<<endl;
            cout<<"Tracking"<<endl;
            for(auto tt:avrgTracking)
                cout<<tt.first<<" ="<<tt.second.first/tt.second.second<<endl;
        }




    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}
