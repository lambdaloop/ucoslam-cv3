#ifndef _CODESTARTER_THREADS_
#define _CODESTARTER_THREADS_

#include <memory>
#include <thread>
#include <list>
#include <iostream>
#include <vector>
#include <chrono>
#include <sstream>
class Thread{


public:
    int id=-1;//internal id

    Thread(){running=false; killed=false;exit_code=std::numeric_limits<int>::lowest();}
    virtual int run()=0;

    bool isRunning()const{return running;}
    bool isFinished()const{return finished;}
    bool operator!()const{return running;}

    void start(){
        running=true;
        finished=false;
        startTime=std::chrono::high_resolution_clock::now();
        _this_thread= std::make_shared< std::thread> (&Thread::_run_internal,this);
        _pid=_this_thread->get_id();
    }

    float runningTimeMs(){
        auto now=std::chrono::high_resolution_clock::now()    ;
        return double(std::chrono::duration_cast<std::chrono::nanoseconds>(now-startTime).count())/1e6;
    }

    int join(){
        if (_this_thread!=0)
            _this_thread->join();
        return exit_code;
    }
    bool joinable()const{
        if (_this_thread!=0)
            return  _this_thread->joinable();
        else return false;
    }

    void detach(){
        if (_this_thread!=0)
            _this_thread->detach();

    }

    int getExitCode()const {return exit_code;}

    void kill(){ if (running){
            detach();
//            std::stringstream cmd;cmd<<"kill -9 "<<_pid;
//            system(cmd.str().c_str());
            _this_thread.reset();
            killed=true;
            running=false;
            exit_code=-1000;
        }
    }

    bool hasBeenKilled()const{return killed;}
private:
    void _run_internal()  {
         exit_code=run();
        running=false;
        finished=true;

    }
    bool running=false;
    bool finished=false;
    bool killed=false;
    int exit_code;
    std::shared_ptr<std::thread> _this_thread;
    std::thread::id _pid;
    std::chrono::high_resolution_clock::time_point startTime;
};


class ThreadSet: public std::list<std::shared_ptr< Thread> >
{
public:


    ~ThreadSet(){
        //detach running threads
         for(auto thre: *this)
            if (thre->isRunning()) thre->detach();
        clean_finished();
    }

//    //returns the pair thread-pos in the list, return value
    void clean_finished(){
        auto it=this->begin();
        while(it!= this->end()){
            if (!(*it)->isRunning() ){
                if ((*it)->joinable())                (*it)->join();
                it=this->erase(it);
            }
            else it++;
        }
    }


////    void kill_all(){
////        std::list<std::shared_ptr< Thread> >::iterator it=this->begin();
////        while(it!= this->end())
////            (*it)->kill();

////    }

//    //returns the threads values
//    std::vector<int> join_all(){
//        std::vector<std::shared_ptr< Thread> >::iterator it=this->begin();
//        std::vector<int> return_values;
//        while(it!= this->end()){
//            int res=std::numeric_limits<int>::lowest();//not joinable
//            if ((*it)->joinable())
//                res=(*it)->join();
//            it++;
//            return_values.push_back(res);
//        }
//        return return_values;
//    }

    size_t nRunningThreads()const{
        size_t nr=0;
        for(auto thre:*this)
            if (thre->isRunning()) nr++;
        return nr;
    }
};

class RunCommand:public Thread{
    std::string _cmd;
public:

    RunCommand(std::string cmd):_cmd(cmd){}
    int run(){
        std::cerr<<"Running : "<<_cmd<<std::endl;
        int exit_code= system(_cmd.c_str());
         return exit_code;
    }
};



class RunCommandSet {
    std::vector<std::string> commands;
    std::vector<int> commands_res;

public:
    size_t _max_threads;
    RunCommandSet(size_t maxThreads):_max_threads(maxThreads){}
    void addCommand(std::string cmd){commands.push_back(cmd);}


    //returns the result of each command
    //if when leave there are running processes,  check the result. Values == numeric_limits<int>::lowest() indicates that it has not finished
    std::vector<int> synch_exec(int maxTimeMilliscsPerThread=-1,int maxTotalTimeMillisecs=-1){

        ThreadSet RunningThreads;
        ThreadSet FinishedThreads;
        auto startTime=std::chrono::high_resolution_clock::now();
        size_t lastAddedThread=0;


        //add all commands first, without running them
        for( ;lastAddedThread< _max_threads && lastAddedThread<commands.size();lastAddedThread++ ){
            RunningThreads.push_back(   std::make_shared<RunCommand>(commands[lastAddedThread]));
            RunningThreads.back()->start();
            RunningThreads.back()->id=lastAddedThread;
        }

        //wait until time finishes or all finished

        while( RunningThreads.nRunningThreads()!=0 || lastAddedThread<commands.size()){

            auto now=std::chrono::high_resolution_clock::now()    ;
            auto totalTimeMs=double(std::chrono::duration_cast<std::chrono::nanoseconds>(now-startTime).count())/1e6;
            //Has expired total time?
            if (maxTotalTimeMillisecs>0 && totalTimeMs>maxTotalTimeMillisecs){//kill all
                for(auto th: RunningThreads)
                    th->kill();
                break; //exit the loop
            }
            else{
                //check if the time of any thread has expired
                if(maxTimeMilliscsPerThread>0){
                    for(auto th: RunningThreads)
                        if(th->runningTimeMs()>maxTimeMilliscsPerThread)
                            th->kill();
             }
            }
            //remove finished threads
            for(auto th=RunningThreads.begin();th!=RunningThreads.end();){
                if(!(*th)->isRunning()){
                    FinishedThreads.push_back((*th));
                    th=RunningThreads.erase(th);
                }
                else th++;
            }

            //add more
            while (RunningThreads.size()<_max_threads && lastAddedThread<commands.size()){
                RunningThreads.push_back(   std::make_shared<RunCommand>(commands[lastAddedThread]));
                RunningThreads.back()->start();
                RunningThreads.back()->id=lastAddedThread;
                lastAddedThread++;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        //move any remaining thread to finised list
        for(auto th: RunningThreads)
            FinishedThreads.push_back(th);
        RunningThreads.clear();

        //get the results and put them into res vector
        std::vector<int> res(commands.size());
        for(auto th:FinishedThreads)
            res[ th->id]=th->getExitCode();
        return res;
    }

        //    //returns the result of each command
        //    //if when leave there are running processes,  check the result. Values == numeric_limits<int>::lowest() indicates that it has not finished
        //    std::vector<int> synch_exec(int maxTimeMillisecs=-1){
        //        ThreadSet Tset;
        //        commands_res.resize(commands.size());
        //        //add all commands first, without running them
        //        for(int i=0;i< commands.size();i++ ){
        //            Tset.push_back(   std::make_shared<RunCommand>(commands[i]));
        //            Tset.back()->start();
        //        }
        //        //wait until time finishes or all finished


        //        int totalTime=0;
        //        while( Tset.nRunningThreads()!=0 && (maxTimeMillisecs==-1 || totalTime<maxTimeMillisecs)){
        //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //            totalTime+=10;
        //        }

        //        std::vector<int> res;
        //        for(int i=0;i<Tset.size();i++){
        //            res.push_back(Tset[i]->getExitCode());
        //        }

        //        return res;
        //    }
};

#endif


