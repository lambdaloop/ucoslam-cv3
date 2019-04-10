

///---------------------------------------------------------------------------------------------
#ifndef _SLAM_SGL_OpenCV_VIewer_H
#define _SLAM_SGL_OpenCV_VIewer_H
#include "basictypes/sgl.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <thread>
#include "ucoslam.h"
#include "map.h"
namespace ucoslam{
//Class using an opencv window to render and manipulate a sgl scene



class MapDrawer  {
public:

    bool _showNumbers=false;
    bool _showKeyFrames=true;
    bool _showKeyPoints=true;
    int _mode=1;//mode 0:big 3d, mode 1: big image
    int _PtSize=2;
    bool _showPointNormals=false;
    int _showCovisGraph=0;
    //do not use it
    sgl::Scene _Scene;
    cv::Rect _small3dRect;//in mode 1, the region of the image where the 3D view is placed
    sgl::Matrix44 viewMatrixNoFollow,viewMatrixFollowCamera;

private:

    float _f;
    float _subw_nsize=0.3;
    cv::Mat _cameraImage;
    bool _followCamera=false;

    struct marker_pi{
        Se3Transform pose;
        vector<sgl::Point3> points;
    };
    std::map<uint32_t,marker_pi> marker_points;

    vector<pair<cv::Point3f,bool> > mapPoints;
    vector< cv::Point3f  > mapPointsNormals;
    std::map<uint32_t,Marker> map_markers;

    std::map<uint32_t,Se3Transform> _FSetPoses;
    int64_t curKF=-1;
    CovisGraph CVGraph;

    cv::Mat _resizedInImage;
    cv::Mat _camPose;
    string userMessage;
    std::mutex drawImageMutex;

    sgl::Point3 centerOfMovement=sgl::Point3(0,0,0);

public:


    MapDrawer( );

    //Sets the focal lenght f(1 means width), and redering buffer dimensions
    void setParams(float f  );
    //draws in the output image the map. If the camera Image is available will show it.
    void draw(cv::Mat &image,   std::shared_ptr<Map> map,const cv::Mat &cameraImage=cv::Mat(),const cv::Mat &cameraPose_f2g=cv::Mat(),string additional_msg="",int currentKF=-1);


    void setSubWindowSize(float normsize=0.3){_subw_nsize=normsize;}

    bool isFollowingCamera()const{return _followCamera;}

    void followCamera(bool v);


    void zoom(float value){
        _Scene.zoom(value);
    }

    void rotate(float x,float z){
        _Scene.rotate(x,z,centerOfMovement.x,centerOfMovement.y,centerOfMovement.z);
    }

    void translate(float x,float y){
        _Scene.translate(x,y);
    }

    void setCenterOfMovements(int x,int y){

        if(_mode==1){//small image( translate position to start of small image)
            if (_small3dRect.contains(cv::Point(x,y))){//if into limits of small image
                x-=_small3dRect.x;
                y-=_small3dRect.y;

            }
            else return;//not within limits
        }
        //big 3d image
        auto val=_Scene.getNearestProjection(x,y);
        if(!isnan(val.x))
            centerOfMovement=val;

    }


protected:
    void draw(cv::Mat &image  ) ;
    void updateImage(  );
    void drawScene(int ptSize);

private:
    void  blending(const cv::Mat &a,float f,cv::Mat &io);
    vector<sgl::Point3> getMarkerIdPcd(ucoslam::Marker &minfo , float perct);
};

MapDrawer::MapDrawer( ){setParams(1  );}


void MapDrawer::setParams(float f){
    _f=f;

    sgl::Matrix44 cam;
    cam.translate({0,6,0});
    cam.rotateX(3.1415/2.);
    _Scene.setViewMatrix(cam);

}

void MapDrawer::followCamera(bool v){
    if ( v==false && _followCamera)
        _Scene.setViewMatrix(viewMatrixFollowCamera);
    else if(v==true && !_followCamera)
        _Scene.setViewMatrix(viewMatrixNoFollow);
    _followCamera=v;
}

void MapDrawer::draw(cv::Mat &image, std::shared_ptr<Map> map,const cv::Mat &cameraImage ,const cv::Mat &cameraPose_f2g ,string additional_msg ,int currKeyFrame){

    auto areEqual=[](const Se3Transform &a,const Se3Transform &b){
      cv::Mat M=a*b.inv();
      float sum=0;
      for(int i=0;i<4;i++)
          for(int j=0;j<4;j++)
              sum+=M.at<float>(i,j);
      return (sum<1e-2);
    };
    if (image.size().area()==0)throw std::runtime_error("input image is not resized");
    assert(cameraPose_f2g.total()==16 || cameraPose_f2g.total()==0);
    curKF=currKeyFrame;
    if (!cameraPose_f2g.empty())
        _camPose=cameraPose_f2g.inv();
    else _camPose=cameraPose_f2g;
    userMessage=additional_msg;
    _cameraImage=cameraImage;
    _FSetPoses.clear();
    mapPoints.clear();
    mapPointsNormals.clear();
    if (map){
        map->lock(__FUNCTION__,__FILE__,__LINE__);
        for(auto &mp:map->map_points){
            mapPoints.push_back({mp.getCoordinates(),mp.isStable()});
            mapPointsNormals.push_back( mp.getNormal());
        }
        map_markers=map->map_markers;
        CVGraph=map->TheKpGraph;
        map->unlock(__FUNCTION__,__FILE__,__LINE__);

        for(auto fs:map->keyframes) _FSetPoses.insert({fs.idx,fs.pose_f2g.inv()});

        //set the marker points
        for(auto m:map_markers){
            bool redopoints=false;
            if(!m.second.pose_g2m.isValid())continue;
            if(marker_points.count(m.first)==0) redopoints=true;
            else if(!areEqual(marker_points[m.first].pose, m.second.pose_g2m)) redopoints=true;
            if (redopoints ){
                marker_points[m.first].points=getMarkerIdPcd(m.second,0.5);
                marker_points[m.first].pose=m.second.pose_g2m;
            }
        }
    }
    //first creation of the image
      draw(image);
}
void MapDrawer::drawScene(int ptSize ){

    auto join=[](uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };
    auto drawMarker=[](sgl::Scene &Scn, const ucoslam::Marker &m , int width=1){
        Scn.setLineSize(width);
        auto points=m.get3DPoints();
        Scn.drawLine((sgl::Point3*)&points[0],(sgl::Point3*)&points[1],{0,0,255});
        Scn.drawLine((sgl::Point3*)&points[1],(sgl::Point3*)&points[2],{0,255,0});
        Scn.drawLine((sgl::Point3*)&points[2],(sgl::Point3*)&points[3],{255,0,0});
        Scn.drawLine((sgl::Point3*)&points[3],(sgl::Point3*)&points[0],{155,0,155});
    };


    auto  drawPyramid=[](sgl::Scene &Scn,float w,float h,float z,const sgl::Color &color,int width=1){
        Scn.setLineSize(width);
        Scn.drawLine( {0,0,0}, {w,h,z},color);
        Scn.drawLine( {0,0,0}, {w,-h,z},color);
        Scn.drawLine( {0,0,0}, {-w,-h,z},color);
        Scn.drawLine( {0,0,0}, {-w,h,z},color);
        Scn.drawLine( {w,h,z}, {w,-h,z},color);
        Scn.drawLine( {-w,h,z}, {-w,-h,z},color);
        Scn.drawLine( {-w,h,z}, {w,h,z},color);
        Scn.drawLine( {-w,-h,z}, {w,-h,z},color);
    };

    auto  drawFrame=[&](sgl::Scene &Scn,Se3Transform &frame,const sgl::Color &color,int width=1){
        Scn.pushModelMatrix(sgl::Matrix44(frame.ptr<float>(0)));
        drawPyramid(Scn,0.1,0.05,0.04,color,width);
        Scn.popModelMatrix();

    };

    _Scene.clear(sgl::Color(255,255,255));


    if(_followCamera && !_camPose.empty()){
        viewMatrixNoFollow=_Scene.getViewMatrix();
        cv::Mat  invcp=_camPose.inv();
        viewMatrixFollowCamera=viewMatrixNoFollow*sgl::Matrix44(invcp.ptr<float>(0));
        _Scene.setViewMatrix(viewMatrixFollowCamera);
    }

    sgl::Color stablePointsColor(0,0,0);
    sgl::Color unStablePointsColor(0,0,255);
    sgl::Color normalColor(255,0,0);


    _Scene.setPointSize(ptSize);
    _Scene.setLineSize(1);


    if (_showKeyPoints){
        for(size_t i=0;i< mapPoints.size();i++){
            const auto &point=mapPoints[i];
            _Scene.drawPoint( (sgl::Point3*)&point.first, point.second?stablePointsColor:unStablePointsColor);
            const auto &normal=mapPointsNormals[i];
            //find the point at disatnce d from origin in direction of normal
            if (_showPointNormals){
                float d=0.01;
                cv::Point3f pend=point.first + d*normal;
                _Scene.drawLine( (sgl::Point3*)&point.first, (sgl::Point3*)&pend,normalColor);
            }

        }
    }



    for(const auto &m:map_markers){
        if (!m.second.pose_g2m.isValid())continue;
        drawMarker(_Scene,m.second);
        if(_showNumbers){
            for(const auto &p:marker_points[m.first].points)
                _Scene.drawPoint(p,{125,0,255});
        }
    }

    //draw frames
    if (_showKeyFrames){
        for(auto kf:_FSetPoses){
            if (curKF==kf.first)
                drawFrame(_Scene,kf.second,sgl::Color (125,0,200),2);
            else drawFrame(_Scene,kf.second,sgl::Color(255,0,0),1);
            // _Scene.pushModelMatrix(sgl::Matrix44(kf.second.ptr<float>(0)));
            // drawPyramid(_Scene,0.1,0.05,0.04,color);
            //                    _Scene.popModelMatrix();
        }


        //draw covis graph
        if (_showCovisGraph==1 && curKF!=-1){//show only connected to current keyframe
            if (CVGraph.isNode(curKF)){
                cv::Point3f p1=_FSetPoses[curKF].getT();

                //shows only the links of the current keyframe
                auto neigh=CVGraph.getNeighbors(curKF);
                for(auto n:neigh ){
                    auto color=sgl::Color(155,155,155);
                    if (CVGraph.getWeight(curKF,n)>25)
                        color=sgl::Color(125,255,0);
                    //draw a line between them
                    cv::Point3f p2=_FSetPoses[n].getT();
                    _Scene. drawLine( (sgl::Point3*)&p1,(sgl::Point3*) &p2,color  );
                    drawFrame(_Scene,_FSetPoses[n],color);

                }
            }
        }
        else if(_showCovisGraph==2){//show all
            std::set<uint64_t> connectionsDrawn;
            for(auto kf:_FSetPoses){
                cv::Point3f p1=_FSetPoses[kf.first].getT();
                auto neigh=CVGraph.getNeighbors(kf.first);
                for(auto n:neigh ){
                    if (!connectionsDrawn.count(join(kf.first,n))){
                        auto color=sgl::Color(155,155,155);
                        if (CVGraph.getWeight(kf.first,n)>25)
                            color=sgl::Color(0,255,0);
                        //draw a line between them
                        cv::Point3f p2=_FSetPoses[n].getT();
                        _Scene. drawLine( (sgl::Point3*)&p1,(sgl::Point3*) &p2,color  );
                        connectionsDrawn.insert(join(kf.first,n));
                    }
                }
            }

        }

    }
    //draw camera if it is possible
    if (!_camPose.empty()){
        _Scene.pushModelMatrix(sgl::Matrix44(_camPose.ptr<float>(0)));
        drawPyramid(_Scene,0.1,0.05,0.04,{0,125,255},2);
        _Scene.popModelMatrix();
    }

    //restore
    if(_followCamera && !_camPose.empty())
      _Scene.setViewMatrix(viewMatrixNoFollow);
}

void MapDrawer::blending(const cv::Mat &a,float f,cv::Mat &io){


    if (a.type()==CV_8UC3){
        for(int y=0;y<a.rows;y++){
            const cv::Vec3b *ptra=a.ptr<cv::Vec3b>(y);
            cv::Vec3b *ptrio=io.ptr<cv::Vec3b>(y);
            for(int x=0;x<a.cols;x++){
                ptrio[x][0]= ptra[x][0]*f+(1.-f)*ptrio[x][0];
                ptrio[x][1]= ptra[x][1]*f+(1.-f)*ptrio[x][1];
                ptrio[x][2]= ptra[x][2]*f+(1.-f)*ptrio[x][2];
            }
        }
    }
    else if(a.type()==CV_8UC4){
        for(int y=0;y<a.rows;y++){
            const cv::Vec4b *ptra=a.ptr<cv::Vec4b>(y);
            cv::Vec4b *ptrio=io.ptr<cv::Vec4b>(y);
            for(int x=0;x<a.cols;x++){
                ptrio[x][0]= ptra[x][0]*f+(1.-f)*ptrio[x][0];
                ptrio[x][1]= ptra[x][1]*f+(1.-f)*ptrio[x][1];
                ptrio[x][2]= ptra[x][2]*f+(1.-f)*ptrio[x][2];
                ptrio[x][3]= ptra[x][3]*f+(1.-f)*ptrio[x][3];
            }
        }

    }
}

void MapDrawer::draw(cv::Mat &image ) {

     std::unique_lock<std::mutex> lock(drawImageMutex);
    int _h=image.rows;
    int _w=image.cols;
    int nchannels=3;
    if ( image.type()==CV_8UC4) nchannels=4;

    _small3dRect.width= _w*_subw_nsize;
    _small3dRect.height=_h*_subw_nsize;
    _small3dRect.x=_w-_small3dRect.width;
    _small3dRect.y=_h-_small3dRect.height;

    int ptSize=1;
    if (_mode==0){
        _Scene.setCameraParams(_f,_w,_h,nchannels,image.ptr<uchar>(0));
        ptSize=2;
    }
    else {
        _Scene.setCameraParams(_f,_small3dRect.width,_small3dRect.height,nchannels);
        ptSize=1;
    }



    drawScene(ptSize);


    //copy 3d image and color image
    if (_mode==0){

        if (!_cameraImage.empty()){
            auto subrect=image(_small3dRect);//cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
            cv::resize( _cameraImage,_resizedInImage,_small3dRect.size());
            blending(_resizedInImage,0.8,subrect);
        }

    }
    else{

        if (!_cameraImage.empty()){
            if (_cameraImage.ptr<float>(0)==image.ptr<float>(0))
                image=_cameraImage;
            else cv::resize( _cameraImage,image, cv::Size(_w,_h));
        }
        else image.setTo(cv::Scalar::all(0));
         auto subrect=image(_small3dRect);//cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
        cv::Mat im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),image.type(),_Scene.getBuffer());
        im3d.copyTo(subrect);
//        blending(im3d,0.5,subrect);

    }


}




vector<sgl::Point3> MapDrawer::getMarkerIdPcd(ucoslam::Marker &minfo,float perct=1 )
{
    auto  mult=[](const cv::Mat& m, cv::Point3f p)
    {
        assert(m.isContinuous());
        assert(m.type()==CV_32F);
        cv::Point3f res;
        const float* ptr = m.ptr<float>(0);
        res.x = ptr[0] * p.x + ptr[1] * p.y + ptr[2] * p.z + ptr[3];
        res.y = ptr[4] * p.x + ptr[5] * p.y + ptr[6] * p.z + ptr[7];
        res.z = ptr[8] * p.x + ptr[9] * p.y + ptr[10] * p.z + ptr[11];
        return res;
    };

    int id = minfo.id;
    // marker id as a set of points
    stringstream sstr;sstr<<id;
    string text = sstr.str();
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 2;
    int thickness = 3;
    int baseline = 0;
    float markerSize_2 = minfo.size / 2.f;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
    cv::Mat img(textSize + cv::Size(0, baseline / 2), CV_8UC1, cv::Scalar::all(0));
    // center the text
    // then put the text itself
    cv::putText(img, text, cv::Point(0, textSize.height + baseline / 4), fontFace, fontScale, cv::Scalar::all(255),
                thickness, 8);
    // raster 2d points as 3d points
    vector<cv::Point3f> points_id;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            if (img.at<uchar>(y, x) != 0)
                points_id.push_back(cv::Point3f((float(x) / float(img.cols)) - 0.5f, (float(img.rows - y) / float(img.rows)) - 0.5f, 0.f));

    // now,scale
    for (auto& p : points_id)
        p *= markerSize_2;
    // finally, translate
    for (auto& p : points_id)
        p = mult(minfo.pose_g2m, p);


    //select only a fraction of them number of them
    vector<sgl::Point3> s_points_id;
    if(perct!=1){
        int notused=float(points_id.size())*(1-perct);
        vector<char> used(points_id.size(),true);
        for(int i=0;i<notused;i++) used[i]=false;
        std::random_shuffle(used.begin(),used.end());
        //copy only the selected
        s_points_id.reserve(points_id.size());
        for(size_t i=0;i<points_id.size();i++)
            if ( used[i]) s_points_id.push_back(sgl::Point3(points_id[i].x,points_id[i].y,points_id[i].z));
    }
    else
        memcpy(&s_points_id[0],&points_id[0],points_id.size()*sizeof(cv::Point3f));

    return s_points_id;
}



//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

//////////////////////////////////////////////
/// \brief The MapViewer class
///
//////////////////////////////////////////////
class MapViewer{
    MapDrawer _mapDrawer;
    std::string _wname;
    int curKF=-1;
    int snapShotIndex=0;
    int waitKeyTime=0;
    bool canLeave=false;
    bool bExitOnUnUnsedKey=true;
    bool showingHelp=false;
    cv::Mat _imshow;
    cv::Mat _cameraImage,_cameraPose_f2g;
    std::string _additional_msg;
    std::shared_ptr<Map> TheMap;
    std::mutex drawImageMutex;
    bool isWindowCreated=false;
public:
    MapViewer(){
        setParams(1,1280,720, "xSLAM");
    }
    void setParams(float f,int width,int height,std::string wname){
        _imshow.create(height,width,CV_8UC4);
        _mapDrawer.setParams(f);
        _wname=wname;
    };

    void set(const string &_param,const string & val){

        string param=_param;
        for(auto &c:param) c=std::toupper(c);
        if(param=="SHOWNUMBERS")
            _mapDrawer._showNumbers=std::stoi(val);
        else if (param=="SHOWCOVISGRAPH")
            _mapDrawer._showCovisGraph=std::stoi(val);
        else if(param=="CANLEAVE")
            canLeave=std::stoi(val);
        else if(param=="MODE")
            _mapDrawer._mode=std::stoi(val);
        else if(param=="FOLLOWCAMERA")
            _mapDrawer.followCamera(stoi(val));
        else if(param=="MODELMATRIX"){
            stringstream sstr;sstr<<val;
            sgl::Matrix44 m;
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    sstr>>m(i,j);
            _mapDrawer._Scene.setModelMatrix(m);
        }
        else if(param=="VIEWMATRIX"){
            stringstream sstr;sstr<<val;
            sgl::Matrix44 m;
            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                    sstr>>m(i,j);
            _mapDrawer._Scene.setViewMatrix(m);
        }

    }

    cv::Mat draw(std::shared_ptr<Map> map,const cv::Mat &cameraImage=cv::Mat(),const cv::Mat &cameraPose_f2g=cv::Mat(),string additional_msg="",int curKeyFrame=-1){
        TheMap=map;
        _cameraImage=cameraImage;
        _cameraPose_f2g=cameraPose_f2g;
        _additional_msg=additional_msg;
        curKF=curKeyFrame;
        draw();
        return _imshow;
    }

    int show(std::shared_ptr<Map> map,const cv::Mat &cameraImage=cv::Mat(),const cv::Mat &cameraPose_f2g=cv::Mat(),string additional_msg="",int curKeyFrame=-1){
        if(!isWindowCreated){

            cv::namedWindow(_wname,cv::WINDOW_AUTOSIZE);
            cv::resizeWindow(_wname,_imshow.cols,_imshow.rows);
            cv::setMouseCallback(_wname, &MapViewer::mouseCallBackFunc , this);
            isWindowCreated=true;
        }
        TheMap=map;
        _cameraImage=cameraImage;
        _cameraPose_f2g=cameraPose_f2g;
        _additional_msg=additional_msg;
        curKF=curKeyFrame;
        draw();
        int key,leaveNow=false;
        do{
            cv::imshow(_wname,_imshow);
            if (canLeave)waitKeyTime=2;
            else waitKeyTime=0;
            key=cv::waitKey(waitKeyTime);
            //            if (k!=255) cout<<"wkh="<<k<<endl;
            bool update=false,create=false;
            //change mode
            if (key=='m'){
                _mapDrawer._mode=_mapDrawer._mode==0?1:0;
                create=true;
            }
            else if(key=='h'){
                showingHelp=!showingHelp;
                update=true;
            }
            else if(key=='n'){
                _mapDrawer._showNumbers=!_mapDrawer._showNumbers;
                update=true;
            }
            else if(key=='k'){
                _mapDrawer._showKeyFrames=!_mapDrawer._showKeyFrames;
                update=true;
            }
            else if(key=='p'){
                _mapDrawer._showKeyPoints=!_mapDrawer._showKeyPoints;
                update=true;
            }
            else if (key=='w'){
                string name="ucoslam-"+std::to_string(snapShotIndex++)+".png";
                cv::imwrite(name,_imshow);
                std::cerr<<"Image saved to "<<name<<endl;

            }
            else if (key=='x'){
                _mapDrawer._showPointNormals=!_mapDrawer._showPointNormals;
                update=true;

            }
            else if (key=='s' ) canLeave=!canLeave;
            else if( key=='g') {
                _mapDrawer._showCovisGraph =(_mapDrawer._showCovisGraph+1) %3;
                update=true;
            }
            else if (key=='f'){
                _mapDrawer.followCamera(!_mapDrawer.isFollowingCamera());
                update=true;
            }

            //shift and ctrl keys
            else if (key==227 || key==225) {leaveNow=false;}
            else  if ( bExitOnUnUnsedKey  && key!=255 ) leaveNow=true;

            if (create|| update) draw();

        } while( (!canLeave && !leaveNow) && key!=27);
        return key;
    }

    cv::Mat getImage( ) {return _imshow;}

    void getImage(cv::Mat &image) {
        std::unique_lock<std::mutex> lock(drawImageMutex);
        _imshow.copyTo(image);
    }

    struct mouseInfo{
        sgl::Point2 pos;
        bool isTranslating=false,isZooming=false,isRotating=false;
    }mi;


    static   void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata){
        MapViewer *Sv=(MapViewer*)userdata;
        bool bRedraw=false;

        if  ( event == cv::EVENT_LBUTTONDOWN ){
            //select nearest 3d point
             Sv->_mapDrawer.setCenterOfMovements(x,y);

             Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false;
            if ( flags&cv::EVENT_FLAG_CTRLKEY)
                Sv->mi.isZooming=true;
            else if ( flags&cv::EVENT_FLAG_SHIFTKEY) Sv->mi.isTranslating=true;
            else Sv->mi.isRotating=true;
        }
        else if  ( event == cv::EVENT_MBUTTONDOWN ) Sv->mi.isTranslating=true;
        else if ( event == cv::EVENT_LBUTTONUP ) { Sv->mi.isRotating=Sv->mi.isTranslating=Sv->mi.isZooming=false; }
        else if ( event == cv::EVENT_MBUTTONUP ) Sv->mi.isTranslating=false;
        else if ( event == cv::EVENT_MOUSEMOVE )
        {
            sgl::Point2  dif(Sv->    mi.pos.x-x,Sv->   mi.pos.y-y);

            if (Sv->mi.isRotating){
                Sv->_mapDrawer.rotate(-float(dif.y)/100.   , -float(dif.x)/100.);
                bRedraw=true;
            }
            else if (Sv->mi.isZooming){
                bRedraw=true;
                Sv->_mapDrawer.zoom(-dif.y*0.01);
            }
            else if (Sv->mi.isTranslating){
                Sv->_mapDrawer.translate(float(-dif.x)/100., float(-dif.y)/100);
                bRedraw=true;
            }
        }
        Sv->mi.pos=sgl::Point2(x,y);
        if (bRedraw){
            Sv->draw();
            cv::imshow(Sv->_wname,Sv->_imshow);
        }

    }

    void draw(  ) {

        _mapDrawer.draw(_imshow,TheMap,_cameraImage,_cameraPose_f2g,_additional_msg,curKF);
        drawText(_imshow,_additional_msg);

        //        cv::Size subrectsize(_w*_subw_nsize,_h*_subw_nsize);
        //        auto subrect=_imshow(cv::Range(_h-subrectsize.height,_h),cv::Range(_w-subrectsize.width,_w));
        //        //copy 3d image and color image
        //        if (mode==0){
        //            if (!_cameraImage.empty()) {
        //                blending(_resizedInImage,0.8,subrect);

        //            }
        //        }
        //        else{
        //            cv::Mat    im3d=cv::Mat(_Scene.getHeight(),_Scene.getWidth(),CV_8UC3,_Scene.getBuffer());
        //            blending(im3d,0.5,subrect);
        //        }
        //         drawText();
    }

    void drawText(cv::Mat &image,const string &userMessage){

        auto putText=[](cv::Mat &im,string text,cv::Point p ){
            float fact=float(im.cols)/float(1280);
            cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(0,0,0),3*fact);
            cv::putText(im,text,p,cv::FONT_HERSHEY_SIMPLEX, 0.5*fact,cv::Scalar(125,255,255),1*fact);
        };

        int col=20;
        if (!userMessage.empty()) {
            putText(image,userMessage,cv::Point(30,col));
            col+=20;
        }
        //print help commands
        if(!showingHelp){
            putText(image, "'h' showhelp", cv::Point(30,col));
            putText(image, "'s' start/stop video", cv::Point(30,20+col));
        }
        else{
            vector<string> messages={ "'h' hide help",
                                      "'s' start/stop video",
                                      "'m' change view mode",
                                      "'RightButtonMouse: Rotate' ",
                                      "'RightButtonMouse+CRTL: Zoom' ",
                                      "'RightButtonMouse+SHFT: Translate' ",
                                      "'n' show/hide marker numbers",
                                      "'x' show/hide point normals ",
                                      "'f' follow camera mode on/off",
                                      "'g' covis graph change mode",
                                      "'w' take a snapshot",
                                      "'k' show/hide keyframes",
                                      "'p' show/hide keypoints"
                                    };

            for(auto msg:messages){
                putText(image,msg,cv::Point(30,col));
                col+=20;
            }

        }
    }

};

}
#endif
