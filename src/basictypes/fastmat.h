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
#ifndef    UCOSLAM_H_FASTMATH
#define    UCOSLAM_H_FASTMATH
//A matrix faster and simpler than cv::Mat
template<typename T >
class FastMat{
public:

    FastMat(int NRows,int NCols)
    {
        ncols=NCols;
        size=NRows*ncols;
        data=new T[size ];

    }
    ~FastMat(){
        delete data;
    }
    inline void setTo(const T &val){
        for(int i=0;i<size;i++)
            data[i]=val;
    }

    inline T& operator()(int rows,int col){
        return  data[ rows*ncols+col];
    }

    inline T* ptr(int r){
        return data+r*ncols;
    }
private:
    T *data;
    int size,ncols;

};
#endif
