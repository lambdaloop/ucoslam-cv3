#include "ucoslamtypes.h"
#include "ucoslam.h"
#include "basictypes/debug.h"
#include "basictypes/timers.h"
#include "optimization/globaloptimizer.h"
#include <Eigen/Geometry>
#include <cctype>
#include "mapviewer.h"
using namespace std;
void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp);

ucoslam::ImageParams getRGBDImageParams() {

    ucoslam::ImageParams  IP;
    IP.CameraMatrix= (cv::Mat_<float>(3,3) <<  525.0,0,319.5,0,525.0,239.5,0,0,1);
    IP.Distorsion=cv::Mat::zeros(1,5,CV_32F);
    IP.Distorsion= (cv::Mat_<float>(1,5) << 0.262383,-0.953104,-0.005358,0.002628,1.163314);
    IP.CamSize=cv::Size(640,480);
    return IP;
}


//given a gfull file name, removes the dirpath and returns only the filename without .png
string getFileName(string fullImagePath){
    std::size_t found = fullImagePath.find_last_of("/\\");
    string fn= fullImagePath.substr(found+1);
    if (fn.size()>4){
        fn.resize(fn.size()-4);
        return fn;
    }
    else return fullImagePath;
}

class CmdLineParser{int argc;const char **argv;public:
CmdLineParser(int _argc,const char **_argv):argc(_argc),argv(_argv){}

string toUpper(const string &str){string out(str);for(auto &s:out) s=std::toupper(s);return out;}
bool operator[] ( string param ) {int idx=-1; param=toUpper(param);  for ( int i=0; i<argc && idx==-1; i++ ) if ( toUpper ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
string operator()(string param,string defvalue=""){int idx=-1; param=toUpper(param);   for ( int i=0; i<argc && idx==-1; i++ ) if ( toUpper ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
};


string getFileBaseName(string fileName){
    auto pos=fileName.find_last_of('/')+1;
    string baseName=std::string(fileName.begin()+pos,fileName.end());
    return baseName;
}

std::set<int> parseSaveFrames(string line){

    if (line.empty())return{};
    std::replace( line.begin(), line.end(), ',', ' '); // replace all
    stringstream sstr;sstr<<line;
    std::set<int>frames;
    while(!sstr.eof()){
        int frame;
        if(sstr>>frame)frames.insert(frame);
    }
    return frames;
}

void updateParamsFromCommandLine(  CmdLineParser &cml,ucoslam::Params &ucoslam_params){
    if(cml["-params"])
        ucoslam_params.readFromYMLFile(cml("-params"));
    ucoslam_params.runSequential=!cml["-realtime"];
    ucoslam_params.detectMarkers=!cml["-nomarkers"];
    if(cml["-maxFeatures"])         ucoslam_params.maxFeatures=stof(cml("-maxFeatures"));
    if (cml["-KFMinConfidence"])    ucoslam_params.KFMinConfidence= stof(cml("-KFMinConfidence"));
    if (cml["-KFCulling"])          ucoslam_params.KFCulling=stof(cml("-KFCulling"));
    ucoslam_params.KPNonMaximaSuppresion=cml["-KPNonMaxima"];


    ucoslam_params.aruco_markerSize=stof(cml("-aruco-markerSize","1"));
    ucoslam_params.aruco_Dictionary=cml("-aruco-dict","ARUCO_MIP_36h12");
    ucoslam_params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");
//    ucoslam_params.kpDescriptorType= ucoslam::DescriptorTypes::fromString("akaze");

    if(cml["-nokeypoints"])
        ucoslam_params.detectKeyPoints=false;
    ucoslam_params.kptImageScaleFactor=stof(cml("-scale","1.0"));

    if(cml["-autoadjustKpSensitivity"])
    ucoslam_params.autoAdjustKpSensitivity=true;



    ucoslam_params.extraParams=cml("-extra_params");
    if(cml["-nokploopclosure"]) ucoslam_params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) ucoslam_params.inPlaneMarkers=true;
}

bool grab(vector<cv::VideoCapture> &vcap){
    bool b=true;
    for(auto &v:vcap)
        b&=v.grab();
    return b;
}

void retrieve(vector<cv::VideoCapture> &vcap, vector<cv::Mat> &images)
{
 if(images.size()<vcap.size())throw std::runtime_error("retrieve images size incorrect");
    for(size_t i=0;i<vcap.size();i++)
         vcap[i].retrieve(images[i]);
}


int get(vector<cv::VideoCapture> &vcap, int prop)
{
    return vcap[0].get(prop);
}

void set_(vector<cv::VideoCapture> &vcap, int prop,double val)
{
    for(auto &v:vcap)
        v.set(prop,val);
}

void open(vector<cv::VideoCapture> &vcap,int argc,const char **argv){
    CmdLineParser cml(argc,argv);
    vcap[0].open(argv[1]);
    if(!vcap[0].isOpened())             throw std::runtime_error("Could not open video");
    if(cml["-stereo"]) {
        vcap[1].open(cml("-stereo"));
        if(!vcap[1].isOpened())             throw std::runtime_error("Could not open right image video");
    }

}


void skip(vector<cv::VideoCapture> &vcap,int frame){
    for(auto &v:vcap){
    v.set(CV_CAP_PROP_POS_FRAMES,frame);
    v.grab();
    v.set(CV_CAP_PROP_POS_FRAMES,frame);
    }
}

void setView(std::shared_ptr<ucoslam::MapViewer >TheViewer,int frame){
    TheViewer->set("followCamera","1");
    TheViewer->set("VIEWMATRIX","1 0 0 0  0 1 0 0   0 0 1 3  0 0 0 1");
    if(frame<100)
        TheViewer->set("VIEWMATRIX","1 0 0 0  0 1 0 0   0 0 1 5  0 0 0 1");
    else if( frame<400)
        TheViewer->set("VIEWMATRIX","1 0 0 0  0 1 0 0   0 0 1 8  0 0 0 1");
    else if( frame<900)
        TheViewer->set("VIEWMATRIX","1 0 0 0  0 1 0 0   0 0 1 15  0 0 0 1");

    if(frame%100==0)
        TheViewer->set("mode","0");
    if(frame%200==0)
        TheViewer->set("mode","1");

}


int main(int argc,const char **argv){
    try {
        cv::VideoWriter videowriter;
        CmdLineParser cml(argc,argv);
        if (argc < 5 || cml["-h"] )
            throw std::runtime_error("Usage: video camera vocabulary outlog [-start n] [-end n] [-realtime] [-recovery] [-params <file>] [-noX] [-nomarkers] [-save <step>] [-KFMinConfidence <val(0,inf)>] [-KFCulling (0,1)] [-canLeave <0,1>] [-maxFeatures <val>(0,inf)] [-KPNonMaxima] [-debug level(0,10)] [-saveFrames frame1,frame2,... ] [-map MapFile] [-stereo  videoRight] [-saveMap out] [-dbg_str xxxx] [-extra_params <params>] [-timefile filename]");

        ucoslam::debug::Debug::setLevel(stoi(cml("-debug","0")));
        if (cml["-dbg_str"])
            ucoslam::debug::Debug::addString(cml("-dbg_str"),"");
        ucoslam::debug::Debug::showTimer(true);

        vector<cv::VideoCapture> vcap( cml["-stereo"]?2:1);
        cv::Size videoCapImageSize;
        int maxNFrames=-1;
        open(vcap,argc,argv);

        if (cml["-start"]){
            for(auto &v:vcap)
             v.set(CV_CAP_PROP_POS_FRAMES,std::stoi(cml("-start")));
        }
        int endFrame=std::numeric_limits<int>::max();
        if (cml["-end"])
            endFrame=stoi(cml("-end"));
        set<int> Frames2Save=parseSaveFrames(cml("-saveFrames"));
        maxNFrames=vcap[0].get(CV_CAP_PROP_FRAME_COUNT);

        if (cml["-skip"])
            skip(vcap,std::stoi(cml("-skip")));

        std::shared_ptr<ucoslam::UcoSlam> Slam=std::make_shared<ucoslam::UcoSlam>();
        ucoslam::ImageParams imageParams;
        imageParams.readFromXMLFile(argv[2]);
        std::shared_ptr<ucoslam::Map> TheMap=std::make_shared<ucoslam::Map>();
        if(cml["-map"]) TheMap->readFromFile(cml("-map"));

        ucoslam::Params ucoslam_params;
        updateParamsFromCommandLine(cml,ucoslam_params);
        if(cml["-desc"])
            ucoslam_params.kpDescriptorType=ucoslam::DescriptorTypes::fromString(cml("-desc"));
        string vocPath ;
        if( argv[3]!=string("none"))vocPath=argv[3];
         Slam->setParams(TheMap,ucoslam_params   ,vocPath);


        //Create the viewer to see the images and the 3D
        std::shared_ptr<ucoslam::MapViewer> TheViewer;
        bool createViewer=true;
        if( cml["-noX"] && !cml["-saveAvi"]) createViewer=false;
        int currViewMode=1;
         if(createViewer){
             TheViewer=std::make_shared<ucoslam::MapViewer>();
             TheViewer->set("canLeave",cml("-canLeave","1"));//starts inmediatly without waiting

         }


         if (cml["-slam"]){
             cv::Mat aux;
             Slam->readFromFile(cml("-slam"));
             for(auto &v:vcap){
                 v.set(CV_CAP_PROP_POS_FRAMES,Slam->getLastProcessedFrame()+1);
                 v.retrieve(aux);
             }
             TheMap=Slam->getMap();
             auto params=Slam->getParams();
             updateParamsFromCommandLine(cml,params);

         }
        bool allowSLAMRecovery=cml["-recovery"];

        ucoslam::TimerAvrg SystemFps,GlobalFps,Xfps;
        bool finish = false;
        vector<cv::Mat> in_image(vcap.size()+1);
        bool trackingStarted=false;
        int saveStep=stoi(cml("-save","100"));
        int recoveryLostFrame=std::numeric_limits<int>::max();
        float normalKFMinConfidence,normalKFCulling,normalProjDistance;

        if(cml["-timefile"]){
            string commd="date > "+cml("-timefile");
            system(commd.c_str());
        }
        while(grab(vcap) && !finish){
             GlobalFps.start();
            retrieve(vcap,in_image);

            int frameIdx=get( vcap, CV_CAP_PROP_POS_FRAMES)-1;

           // setView(TheViewer,frameIdx);
              SystemFps.start();
             cv::Mat pose_f2g;
             if(in_image[1].empty())
                  pose_f2g=Slam->process(in_image[0], imageParams,frameIdx);
             else
                 pose_f2g=Slam->processStereo(in_image[0],in_image[1],imageParams,frameIdx);
             SystemFps.stop();

             cout << "|@# Image " << frameIdx << "/"<< maxNFrames<<  " fps=" << 1./SystemFps.getAvrg()<<" Gfps="<<1./GlobalFps.getAvrg()<<" Xpfs="<<1./Xfps.getAvrg() <<" sig="<<Slam->getSignatureStr()<<endl;
             if(!pose_f2g.empty()){
                cout<<"|@# Tracking successfull"<<endl;
                trackingStarted=Slam->getMap()->keyframes.size()>=5;
                if(frameIdx>=recoveryLostFrame){
                    cout<<"|@# End Of Recovery"<<endl;
                    // if(!cml["-noX"])TheViewer->set("canLeave", "0");//starts inmediatly without waiting
                    Slam->getParams().KFMinConfidence=normalKFMinConfidence;
                    Slam->getParams().KFCulling=normalKFCulling;
                    Slam->getParams().projDistThr=normalProjDistance;
                    recoveryLostFrame=std::numeric_limits<int>::max();
                }
            }
            else if(pose_f2g.empty() && trackingStarted ){
                cout<<"|@# Tracking lost "<<endl;
                //go back 15 frames, and increase minConfidence
                if(allowSLAMRecovery ){
                    if(recoveryLostFrame==std::numeric_limits<int>::max()){
                        string number=std::to_string(frameIdx);
                        while(number.size()!=4) number="0"+number;
                        Slam->saveToFile("lost_track-"+number+".slm");

                        recoveryLostFrame=frameIdx+5;
                        cout<<"|@#Start recovery until "<<recoveryLostFrame<<endl;
                        set_(vcap,CV_CAP_PROP_POS_FRAMES,max(frameIdx-15,0));
                        Slam->waitForFinished();
                        Slam->resetTracker();
                        normalKFMinConfidence=Slam->getParams().KFMinConfidence;
                        normalKFCulling=Slam->getParams().KFCulling;
                        normalProjDistance=Slam->getParams().projDistThr;
                        Slam->getParams().KFMinConfidence=0.9;
                        Slam->getParams().KFCulling=0.9;
                        Slam->getParams().projDistThr=1.5*normalProjDistance;
                    }
                    else if(frameIdx>=recoveryLostFrame ){//could not recover from previous  problem. Go back to normal and wait relocalization
                        cout<<"|@# Could not Recover. Back to normal"<<endl;
                        Slam->getParams().KFMinConfidence=normalKFMinConfidence;
                        Slam->getParams().KFCulling=normalKFCulling;
                        recoveryLostFrame=std::numeric_limits<int>::max();
                        trackingStarted=false;

                    }
                }

            }


            if (cml["-saveAvi"]){
                Xfps.start();
                cv::Mat image=TheViewer->draw(TheMap,   in_image[0], pose_f2g,"SLAM #" + std::to_string(frameIdx)+"/"+std::to_string(maxNFrames)   ,Slam->getCurrentKeyFrameIndex());
                Xfps.stop();
                if(!videowriter.isOpened())
                    videowriter.open(cml("-saveAvi"), CV_FOURCC('X', '2', '6', '4'),  cml["-fps"]?stoi(cml("-fps")):30,image.size()  , image.channels()!=1);
                if(videowriter.isOpened()){videoCapImageSize=image.size();  videowriter<<image;}
                bool modeChanged=false;
                if ( currViewMode==0 && frameIdx%50==0 )
                        modeChanged=true;
                else if (currViewMode==1 && frameIdx%250==0 && frameIdx!=0)
                    modeChanged=true;
                if(modeChanged){
                    currViewMode=currViewMode==0?1:0;
                    TheViewer->set("mode",to_string(currViewMode));
                }

            }

            char key=0;
            if(!cml["-noX"]){
                Xfps.start();
                key = TheViewer->show(TheMap,   in_image[0], pose_f2g,"SLAM #" + std::to_string(frameIdx)+"/"+std::to_string(maxNFrames) + " fps=" + to_string(1./SystemFps.getAvrg())  ,Slam->getCurrentKeyFrameIndex());
                Xfps.stop();
             }
            if (int(key) == 27)finish = true;//pressed ESC
            //write the current map
            if (key=='e'){
                Slam->saveToFile("Slam.slm");
            }


            if (cml["-save"] && frameIdx%saveStep==0 && !cml["-slam"]){
                string number=std::to_string(frameIdx);
                while(number.size()<5)number="0"+number;
                Slam->saveToFile("slam-"+number+".slm");
            }

            if ( frameIdx>=endFrame){
                Slam->saveToFile("slam"+std::to_string(frameIdx)+".slm");
                finish=true;
            }
            else if( Frames2Save.count(frameIdx)!=0){//is this a restore point
                Slam->saveToFile("slam"+std::to_string(frameIdx)+".slm");
            }
            GlobalFps.stop();

        }


        Slam->waitForFinished();
        Slam->globalOptimization();
        if(cml["-saveMap"])
        TheMap->saveToFile(cml("-saveMap"));

         Slam=std::make_shared<ucoslam::UcoSlam>();
        Slam->setParams(TheMap,ucoslam_params  );
        Slam->setMode(ucoslam::MODE_LOCALIZATION);


        for(auto &v:vcap) v.release();
        open(vcap,argc,argv);
        if (cml["-skip"])
            skip(vcap,std::stoi(cml("-skip")));





        //add some empty frames to see the separation
        if (cml["-saveAvi"]){
            cv::Mat image(videoCapImageSize,CV_8UC3);
            image.setTo(cv::Scalar(0,0,0));
            cv::putText(image,"Tracking the created map", cv::Point( image.cols/3,image.rows/2),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255),2);
            int fps= cml["-fps"]?stoi(cml("-fps")):30;
         for(int i=0;i<3*fps;i++)
                if(videowriter.isOpened()) videowriter.write(image);
        }
        //the second time, evaluation of  precision
        std::map<int,cv::Mat> poses;

        finish=false;
        SystemFps.reset();
        GlobalFps.reset();
        if(cml["-timefile"]){
            string commd="date >> "+cml("-timefile");
            system(commd.c_str());
        }

        while(grab(vcap)&& !finish){
            GlobalFps.start();
            retrieve(vcap,in_image);
            int frameIdx=get(vcap,CV_CAP_PROP_POS_FRAMES)-1;

            SystemFps.start();
            cv::Mat pose_f2g=Slam->process(in_image[0], imageParams,frameIdx);
            SystemFps.stop();
            cout << "Second round. Image " << frameIdx << "/"<< maxNFrames<<" fps=" << 1./SystemFps.getAvrg()<<" Gfps="<<1./GlobalFps.getAvrg()<<endl;

            if (!pose_f2g.empty()){
                poses.insert( { frameIdx,pose_f2g});
                cout<<"Tracking successfull"<<endl;
            }
            else
                cout<<"Tracking lost"<<endl;


            if (cml["-saveAvi"]){
                cv::Mat image=TheViewer->draw(TheMap,   in_image[0], pose_f2g,"TRACKING #" + std::to_string(frameIdx) +"/"+std::to_string(maxNFrames)  ,Slam->getCurrentKeyFrameIndex());
                if(!videowriter.isOpened())
                    videowriter.open(cml("-saveAvi"), CV_FOURCC('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
                if(videowriter.isOpened())  videowriter.write(image);
            }
            char key=0;
            if(!cml["-noX"])
                key = TheViewer->show(TheMap,   in_image[0], pose_f2g,"TRACKING #" + std::to_string(frameIdx)+"/"+std::to_string(maxNFrames) + " fps=" + to_string(1./SystemFps.getAvrg()) );
            if (int(key) == 27)finish = true;//pressed ESC
            if ( frameIdx>=endFrame) break;
            GlobalFps.stop();
        }
        if(cml["-timefile"]){
            string commd="date >> "+cml("-timefile");
            system(commd.c_str());
        }

        if(cml["-saveMap"]){
            TheMap->removeUnUsedKeyPoints();
            TheMap->saveToFile(cml("-saveMap"));
        }

        savePosesToFile(argv[4],poses);
    }
    catch (std::exception &ex) {
        cerr << ex.what() << endl;
        return -1;
    }
}



void savePosesToFile(string filename, const std::map<int,cv::Mat>& fmp)
{

    auto  getQuaternionAndTranslationfromMatrix44=[](const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
        //get the 3d part of matrix and get quaternion
        assert(M_in.total()==16);
        cv::Mat M;
        M_in.convertTo(M,CV_64F);
        cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
        //use now eigen
        Eigen::Matrix3f e_r33;
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                e_r33(i,j)=M.at<double>(i,j);

        //now, move to a angle axis
        Eigen::Quaternionf q(e_r33);
        qx=q.x();
        qy=q.y();
        qz=q.z();
        qw=q.w();


        tx=M.at<double>(0,3);
        ty=M.at<double>(1,3);
        tz=M.at<double>(2,3);
    };


    std::ofstream file(filename);
    double qx, qy, qz, qw, tx, ty, tz;
    for (auto frame : fmp)
    {
        if (!frame.second.empty())
        {
            cv::Mat minv=frame.second.inv();
            getQuaternionAndTranslationfromMatrix44(minv, qx, qy, qz, qw, tx, ty, tz);
            file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                 << qw << endl;
        }
    }
}
