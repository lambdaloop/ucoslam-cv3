#include <fstream>
#include "threads.h"
using namespace std;

vector<string> readFileOfProcesses(string path){
    vector<string>  res;
    ifstream file(path);
    if(!file)throw std::runtime_error("Could not open file"+path);
    while(!file.eof()){
        string line;
        std::getline(file,line);
       if(!line.empty()) res.push_back(line);
    }
    return res;
}
int main(int argc,char **argv){

    try{
        if (argc<2)throw std::runtime_error(" Usage: inFile [nThreads] [maxSecsPerThread] [maxSecsTotalTime]");

        int maxThreads=1;
        double maxMsPerThread=-1;
        double maxMsTotalTime=-1;
        if(argc>=3) maxThreads=stoi(argv[2]);
        if(argc>=4) maxMsPerThread=stod(argv[3])*1000;
        if(argc>=5) maxMsTotalTime=stod(argv[4])*1000;

        auto commands=readFileOfProcesses(argv[1]);
        RunCommandSet RCS(maxThreads);
        for(auto cmd:commands)
        RCS.addCommand(cmd);
        vector<int> results=RCS.synch_exec(maxMsPerThread,maxMsTotalTime);


    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
        return -1;
    }
}

