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
#include "io_utils.h"
namespace ucoslam{
void   toStream__ ( const  cv::Mat &m,std::ostream &str ) {

    int r=0,c=0,t=0;
    if (!m.empty()){
        r=m.rows;
        c=m.cols;
        t=m.type();
    }

    str.write ( ( char* ) &r,sizeof ( int ) );
    str.write ( ( char* ) &c,sizeof ( int ) );
    str.write ( ( char* ) &t,sizeof ( int ) );

    //write data row by row
    for ( int y=0; y<m.rows; y++ )
        str.write ( m.ptr<char> ( y ),m.cols *m.elemSize() );
}
/**
 */

void  fromStream__ ( cv::Mat &m,std::istream &str ) {
    int r,c,t;
    str.read ( ( char* ) &r,sizeof ( int ) );
    str.read ( ( char* ) &c,sizeof ( int ) );
    str.read ( ( char* ) &t,sizeof ( int ) );
    if (r*c>0){
        m.create ( r,c,t );
        for ( int y=0; y<m.rows; y++ )
            str.read ( m.ptr<char> ( y ),m.cols *m.elemSize() );
    }
    else m=cv::Mat();
}

void   toStream__ ( const  std::string &m,std::ostream &str ) {
    uint32_t s=m.size();
    str.write((char*)&s,sizeof(s));
    str.write(&m[0],m.size());
}

void  fromStream__ ( std::string &m,std::istream &str ) {
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    m.resize(s);
    str.read(&m[0],m.size());
}

}
