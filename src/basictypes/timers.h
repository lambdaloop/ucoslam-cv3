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
#ifndef UCOSLAM_TIMERS_H
#define UCOSLAM_TIMERS_H


#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include "debug.h"
namespace ucoslam{

//timer

struct ScopedTimerEvents
{
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    std::vector<std::chrono::high_resolution_clock::time_point> vtimes;
    std::vector<std::string> names;
    std::string _name;

 inline   ScopedTimerEvents(const std::string &name="",bool start=true,SCALE _sc=MSEC){
         if(start)
             add("start");
         sc=_sc;
         _name=name;
     }

   inline void add(const std::string &name){
         vtimes.push_back(std::chrono::high_resolution_clock::now());
        names.push_back(name);
     }
    inline void addspaces(std::vector<std::string> &str ){
        //get max size
        size_t m=0;
        for(auto &s:str)m=(std::max)(size_t(s.size()),m);
        for(auto &s:str){
            while(s.size()<m) s.push_back(' ');
        }
    }

   inline ~ScopedTimerEvents(){
         if (!debug::Debug::showTimer()) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };

        add("total");
        addspaces(names);
        for(size_t i=1;i<vtimes.size();i++){
            std::cout<<"Time("<<_name<<")-"<<names[i]<<" "<< double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[i-1]).count())/fact<<str<<" "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[0]).count())/fact<<str<<std::endl;
        }
     }
};

inline std::string methodName(  std::string  prettyFunction)
{
    std::string   res;
    res.reserve(prettyFunction.size());
    bool spaceFound=false;
    for(auto c:prettyFunction){
        if(c==' '  && !spaceFound)spaceFound=true;
        else if(c!='(' && spaceFound) res.push_back(c);
        else if (c=='(' &&spaceFound) break;
        }
    return res;
}

#ifdef USE_TIMERS

#define __UCOSLAM_ADDTIMER__ ScopedTimerEvents XTIMER_X(methodName(__PRETTY_FUNCTION__));
#define __UCOSLAM_TIMER_EVENT__(Y) XTIMER_X.add(Y);
#else
#define __UCOSLAM_ADDTIMER__
#define __UCOSLAM_TIMER_EVENT__(Y)
#endif

struct Timer{
    enum SCALE {NSEC,MSEC,SEC};

    std::chrono::high_resolution_clock::time_point _s;
    double sum=0,n=0;
    std::string _name;
    Timer(){}

    Timer(const std::string &name):_name(name){}
    void setName(std::string name){_name=name;}
   inline void start(){_s=std::chrono::high_resolution_clock::now();}
   inline void end()
    {
#ifdef USE_TIMERS
        auto e=std::chrono::high_resolution_clock::now();
        sum+=double(std::chrono::duration_cast<std::chrono::nanoseconds>(e-_s).count());
        n++;
#endif
    }

   inline void print(SCALE sc=MSEC){
#ifdef USE_TIMERS
       if (!debug::Debug::showTimer()) return;
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };
        std::cout<<"Time("<<_name<<")= "<< ( sum/n)/fact<<str<<std::endl;
#endif
    }

};


struct TimerAvrg{
    std::vector<double> times;
    size_t curr=0,n;
    std::chrono::high_resolution_clock::time_point begin,end;

    TimerAvrg(int _n=30)
    {
        n=_n;
        times.reserve(n);
    }
    inline void start(){
        begin= std::chrono::high_resolution_clock::now();

    }

    inline void stop(){
        end= std::chrono::high_resolution_clock::now();

        double duration=double(std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count())*1e-6;
        if ( times.size()<n) times.push_back(duration);
        else{
            times[curr]=duration;
            curr++;
            if (curr>=times.size()) curr=0;
        }
    }

    void reset(){
        times.clear();
        curr=0;
    }
//returns time in seconds
   inline double getAvrg(){
        double sum=0;
        for(auto t:times) sum+=t;
        return sum/double(times.size());
    }
};

}


#endif
