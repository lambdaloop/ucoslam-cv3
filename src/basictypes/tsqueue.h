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
#ifndef ucoslam_TSQueue_H
#define ucoslam_TSQueue_H
#include <vector>
#include <mutex>
#include <condition_variable>
namespace ucoslam{
//A thread safe queue to implement producer consumer

template<typename T>
class TSQueue
{
public:
    void push(T val) {
        while (true) {
            std::unique_lock<std::mutex> locker(mu);
            cond.wait(locker, [this](){return buffer_.size() < size_;});
            if (buffer_.size()==1)buffer_[0]=val;
            else buffer_.push_back(val);
            locker.unlock();
            cond.notify_all();
            return;
        }
    }
    void  pop(T&v) {
        while (true)
        {
            std::unique_lock<std::mutex> locker(mu);
            cond.wait(locker, [this](){return buffer_.size() > 0;});
            v= buffer_.back();
            buffer_.pop_back();
            locker.unlock();
            cond.notify_all();
            return ;
        }
    }

    bool empty(){
        std::unique_lock<std::mutex> locker(mu);
        return buffer_.size()==0;
    }
    void clear(){
        std::unique_lock<std::mutex> locker(mu);
        buffer_.clear();
    }

    size_t size(){
        std::unique_lock<std::mutex> locker(mu);
        return buffer_.size();

    }
    TSQueue() {}


public:
   // Add them as member variables here
    std::mutex mu;
    std::condition_variable cond;

   // Your normal variables here
    std::vector<T> buffer_;
    const unsigned int size_ = 10;
};
}
#endif
