#ifndef GENERICIMAGEREADER_H
#define GENERICIMAGEREADER_H

#include <opencv2/highgui/highgui.hpp>
#include <QStringList>
#include <iostream>
#include <QFile>
using namespace std;

class GenericImageReader{
public:
    enum TYPES : int {TYPE_NONE,TYPE_IMAGE,TYPE_VIDEO,TYPE_LIVE};
    virtual ~GenericImageReader(){}
     virtual bool isOpened()const=0;
    virtual bool grab()=0;
    virtual void retrieve(cv::Mat &im)=0;
    virtual double get(int prop)=0;
    virtual bool set(int prop,double val)=0;
    virtual std::string getInfo()const=0;
    virtual QStringList getSource()=0;
 };


class _CameraPlayer:public GenericImageReader{
    cv::VideoCapture vcap;
    int fidx=0;
    int index,_h,_w;
public:

      _CameraPlayer(int idx=0,int w=-1,int h=-1){
          if(w!=-1)
              vcap.set(CV_CAP_PROP_FRAME_WIDTH,w);
          if(h!=-1)
              vcap.set(CV_CAP_PROP_FRAME_HEIGHT,h);
        vcap.open(idx);
        if(w!=-1)
            vcap.set(CV_CAP_PROP_FRAME_WIDTH,w);
        if(h!=-1)
            vcap.set(CV_CAP_PROP_FRAME_HEIGHT,h);
        fidx=0;
        index=idx;
        _h=h;
        _w=w;
    }
    bool isOpened()const{return vcap.isOpened();}
    bool grab(){fidx++;return vcap.grab();}
    void retrieve(cv::Mat &im){vcap.retrieve(im);}
    double get(int prop){return vcap.get(prop);}
    bool set(int prop,double val){return vcap.set(prop,val);}
    std::string getInfo()const{ return std::to_string(fidx);}
    virtual QStringList getSource(){return { std::to_string(index).c_str()};}
};

class _ImagePlayer:public GenericImageReader{
    std::vector<std::string> tokenize(const std::string& in, const std::string& delims )
    {
        std::vector<std::string> tokens;
        std::string::size_type pos_begin  , pos_end  = 0;
        std::string input = in;

        input.erase(std::remove_if(input.begin(),
                                  input.end(),
                                  [](char x){return std::isspace(x);}),input.end());

        while ((pos_begin = input.find_first_not_of(delims,pos_end)) != std::string::npos)
        {
            pos_end = input.find_first_of(delims,pos_begin);
            if (pos_end == std::string::npos) pos_end = input.length();

            tokens.push_back( input.substr(pos_begin,pos_end-pos_begin) );
        }
        return tokens;
    }
public:
    std::vector<std::string> files;
    int curFile=-1;
    std::string info;
    cv::Mat image;
    QStringList sources;
    _ImagePlayer(const QStringList &strlist){
        files.clear();
        sources=strlist;
        //divide by ;
        for(int i=0;i<strlist.size();i++){
            files.push_back( strlist.at(i).toLocal8Bit().constData());
        }
        curFile=0;
    }
    virtual bool isOpened()const{return files.size()!=0;}
    virtual bool grab(){
        if (size_t(curFile)>=files.size())return false;
        image=cv::imread(files[curFile]);
        info=files[curFile];
        curFile++;
        return true;
    }
    virtual void retrieve(cv::Mat &im){
        image.copyTo(im);
    }
    virtual double get(int prop){
        switch (prop){
        case CV_CAP_PROP_FRAME_COUNT:return files.size();break;
        case CV_CAP_PROP_FPS:return 1;break;
        case CV_CAP_PROP_POS_FRAMES:return curFile;break;
        };
        return 0;
    }
    virtual bool set(int prop,double val){
        switch (prop){
        case CV_CAP_PROP_POS_FRAMES:
            if(val>=files.size()){
                std::cerr<<"_ImagePlayer can not set:CV_CAP_PROP_POS_FRAMES"<<std::endl;
                return  false;
            }
            else {
                curFile=val;
                return  true;
            }
            break;
        };

    }
    virtual std::string getInfo()const{
            return info;
    }
    virtual QStringList getSource(){
        return sources;
    }

};

class _VideoPlayer:public GenericImageReader{
    cv::VideoCapture videoReader;
    QString fpath;
    std::string info;
    QStringList source;
public:

      _VideoPlayer(const QString & filePath){videoReader.open(filePath.toStdString());
                                                fpath=QFile(filePath).fileName();
                                            source<<filePath;
                                            }
    virtual bool isOpened()const{return videoReader.isOpened();}
    virtual bool grab(){
          info=QString::number(videoReader.get(CV_CAP_PROP_POS_FRAMES)).toStdString();
          return videoReader.grab();
      }
    virtual void retrieve(cv::Mat &im){videoReader.retrieve(im);}
    virtual double get(int prop){return videoReader.get(prop);}
    virtual bool set(int prop,double val){return videoReader.set(prop,val);}
    virtual std::string getInfo()const{
        return info;
    }
      virtual QStringList getSource(){
          return  source;
      }

};

#endif // GENERICIMAGEREADER_H
