#ifndef _MapDrawer_HH
#define _MapDrawer_HH
#include "qtgl/object.h"
#include <QtGui/QOpenGLFunctions>
#include "map.h"

class MapDrawer :public qtgl::Object{
    GLuint _pointCloudList=-1,_KeyFramesList=-1, CovisGraph_Global=-1,_MarkersList=-1;
    float _Size=1.5;
    bool _showKf=true;
    bool _showCvGraph=true;
    bool _showPoints=true;
    bool _showMarkers=true;
    cv::Mat _cameraPose;
    float _scale=1.;
    float _keyFrameSizeScale=1;
public:
    ~MapDrawer(){
        reset();
    }

    void setScaleFactor(float scale){_scale=scale;}
    void setKeyFrameScaleFactor(float scale){_keyFrameSizeScale=scale;}
    void showKeyFrames(bool v){_showKf=v;}
    void showGraph(bool v){_showCvGraph=v;}
    void showMarkers(bool v){_showMarkers=v;}
    void showPoints(bool v){_showPoints=v;}
    void set(std::shared_ptr<ucoslam::Map> map,const cv::Mat &campose=cv::Mat()){
        reset();
        if(!map)return;
        if(!campose.empty()){
            campose.convertTo(_cameraPose,CV_32F);
            _cameraPose=_cameraPose.inv();
            _cameraPose.at<float>(0,3)*=_scale;
            _cameraPose.at<float>(1,3)*=_scale;
            _cameraPose.at<float>(2,3)*=_scale;
        }
        else _cameraPose=cv::Mat();

        map->lock(__FUNCTION__,__FILE__,__LINE__);

        //keypoints
        _pointCloudList = glGenLists ( 1 );
        glNewList ( _pointCloudList, GL_COMPILE );
        glPointSize(_Size);
        glBegin ( GL_POINTS );
        for(auto &mp: map->map_points){
            auto p=_scale*mp.getCoordinates();
            if(mp.isStable())
                glColor3f ( 0,0,0 );
            else
                glColor3f ( 1,0,0 );
            glVertex3f ( p.x,p.y,p.z);
        }
        glEnd();
        glEndList();

        //keyframes
        glLineWidth(_Size);
        _KeyFramesList = glGenLists ( 1 );
        glNewList ( _KeyFramesList , GL_COMPILE );
        for(auto &kf:map->keyframes){
            //scale
            cv::Mat aux;
            aux=kf.pose_f2g.inv();
            aux.at<float>(0,3)*=_scale;
            aux.at<float>(1,3)*=_scale;
            aux.at<float>(2,3)*=_scale;
            drawKeyFrame(aux,cv::Scalar (0,0,1));
        }
        glEndList();

        //markers


        //covisgraph
        CovisGraph_Global=glGenLists(1);
        glNewList ( CovisGraph_Global , GL_COMPILE );
        glBegin(GL_LINES);
        std::set<uint64_t> connectionsDrawn;

        cv::Scalar color;
        for(auto &kf:map->keyframes)
        {
            for(auto neigh:map->TheKpGraph.getNeighbors(kf.idx)){
                if (!connectionsDrawn.count(join(kf.idx,neigh))){

                    if (map->TheKpGraph.getWeight(kf.idx,neigh)>25)
                        glColor3f(0,1,0);
                    else
                        glColor3f(0.8f,0.8f,0.8f);
                    //draw a line between them
                    auto kfpos=_scale*kf.getCameraCenter();
                    glVertex3f(kfpos.x, kfpos.y, kfpos.z);
                    kfpos=_scale*map->keyframes[neigh].getCameraCenter();
                    glVertex3f(kfpos.x, kfpos.y, kfpos.z);
                    connectionsDrawn.insert(join(kf.idx,neigh));
                }
            }
        }
        glEnd();
        glEndList();

        _MarkersList=glGenLists(1);
        glNewList ( _MarkersList , GL_COMPILE );
        for(const auto &m:map->map_markers){
            if(m.second.pose_g2m.isValid())
                drawMarker(m.second);
        }
        glEndList();

        map->unlock(__FUNCTION__,__FILE__,__LINE__);


    }

    void drawMarker(const ucoslam::Marker &marker){
            auto points=marker.get3DPoints();
            for(auto &p:points) p*=_scale;
            glBegin(GL_POLYGON);
            glColor3f(1.f,0.f,0.0f);
            glVertex3f(points[0].x,points[0].y,points[0].z);
            glVertex3f(points[1].x,points[1].y,points[1].z);
            glVertex3f(points[2].x,points[2].y,points[2].z);
            glVertex3f(points[3].x,points[3].y,points[3].z);
            glEnd();

    }

    void drawKeyFrame(const cv::Mat &pose,cv::Scalar color){
        glPointSize(_Size+1);
        glPushMatrix();
        cv::Mat tr=pose.t();
        glMultMatrixf( tr.ptr<float>());

        drawPyramid(_Size,_scale*_keyFrameSizeScale*0.1,_scale*_keyFrameSizeScale*0.05,_scale*_keyFrameSizeScale*0.04,color);

        glPopMatrix();
        glPointSize(_Size-1);

    }

    void drawPyramid(float lineSize,float w,float h,float z,const cv::Scalar &color){

        glLineWidth(lineSize);
        glColor3f(color[0],color[1],color[2]);

        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(w,h,z);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(w,-h,z);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(-w,-h,z);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(-w,h,z);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);


        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);


        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

    }

    void  _draw_impl(){
       if(_showPoints) glCallList ( _pointCloudList );
      if(_showKf)  glCallList(_KeyFramesList);
      if(_showCvGraph)  glCallList(CovisGraph_Global);
      if(_showMarkers) glCallList(_MarkersList);
      if(!_cameraPose.empty()) drawKeyFrame(_cameraPose,cv::Scalar(255,0,0));
    }
    std::string getType()const{return "Map";} //return an string indicating the type of the subclass
    uint64_t join(uint32_t a ,uint32_t b){
        if( a>b)swap(a,b);
        uint64_t a_b;
        uint32_t *_a_b_16=(uint32_t*)&a_b;
        _a_b_16[0]=b;
        _a_b_16[1]=a;
        return a_b;
    };

    void         reset(){
        if(_pointCloudList!=-1)         glDeleteLists(_pointCloudList,1);
        if(_KeyFramesList!=-1)          glDeleteLists(_KeyFramesList,1);
        if(CovisGraph_Global!=-1)       glDeleteLists(CovisGraph_Global,1);
        if(_MarkersList!=-1)            glDeleteLists(_MarkersList,1);
    }

};

#endif
