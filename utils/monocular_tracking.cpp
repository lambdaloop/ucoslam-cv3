#include "slam.h"
#include "debug.h"
#include "mapviewer.h"
#include "timers.h"
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};


cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret;
    cv::resize(in,ret,size);
    return ret;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  vcap.get(CV_CAP_PROP_POS_FRAMES);
}
void savePosesToFile(string filename, const std::map<uint32_t, ucoslam::se3> &fmp);

int main(int argc,char **argv){
    CmdLineParser cml(argc,argv);
    if (argc<3 || cml["-h"]){
        cerr<<"Usage: (video|live[:cameraIndex(0,1...)])    world    [-st starts the processing stopped ]  [-outposes outfile] [-skip n: skips the first n frames of video] [-debug level]"<<endl;return -1;
    }
    ucoslam::debug::Debug::setLevel( stoi(cml("-debug","0")));
    int waitTime=2;
    if (cml["-st"]) waitTime=0;

    bool liveVideo=false;
    cv::VideoCapture vcap;
    string TheInputVideo=string(argv[1]);
    if ( TheInputVideo.find( "live")!=std::string::npos)
    {
        int vIdx = 0;
        // check if the :idx is here
        char cad[100];
        if (TheInputVideo.find(":") != string::npos)
        {
            std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
            sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
        }
        cout << "Opening camera index " << vIdx << endl;
        vcap.open(vIdx);
        liveVideo=true;

    }
    else vcap.open (argv[1]);

    if (!vcap.isOpened()) throw std::runtime_error("Video not opened");
    if(cml["-skip"]){
        int n=stoi(cml("-skip"));
        for(int i=0;i<n;i++) vcap.grab();

    }


    auto TViewer = ucoslam::MapViewer::create(cml["-noX"] ? "" : "Cv");
    ucoslam::Slam Slam;
    cv::Size vsize(0,0);
    cv::Mat in_image;


    Slam.readFromFile( argv[2]);
    Slam.setMode(ucoslam::Slam::MODE_LOCALIZATION);
    Slam.resetTracker();//reset the location and remove previous stored locations

    ucoslam::TimerAvrg Fps;

    while(in_image.empty())vcap>>in_image;
    bool finished=false;
    while(  !finished && !in_image.empty()){
        in_image=resize(in_image,vsize);
        int currentFrameIndex=getCurrentFrameIndex( vcap,liveVideo);
        Fps.start();
        Slam.process(in_image,currentFrameIndex);
        Fps.stop();
        int k = TViewer->show(ucoslam::Map::singleton(), Slam.getCurrentPose_f2g(), Slam.getShowImage(),"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
        if (k==27)finished=true;
        vcap>>in_image;
    }
    if (cml["-outposes"]){
        savePosesToFile(cml("-outposes"),Slam.getFramePoses());
    }

}



void getQuaternionAndTranslationfromMatrix44(const cv::Mat& M_in, float& qx, float& qy, float& qz, float& qw, float& tx,
                                             float& ty, float& tz)
{

    auto SIGN=[](float x)
    {
        return (x >= 0.0f) ? +1.0f : -1.0f;
    };

    auto NORM=[](double a, double b, double c, double d)
    {
        return sqrt(a * a + b * b + c * c + d * d);
    };
    // get the 3d part of matrix and get quaternion
    assert(M_in.total() == 16);
    cv::Mat M;
    M_in.convertTo(M, CV_32F);
    // use now eigen
    float r11 = M.at<float>(0, 0);
    float r12 = M.at<float>(0, 1);
    float r13 = M.at<float>(0, 2);
    float r21 = M.at<float>(1, 0);
    float r22 = M.at<float>(1, 1);
    float r23 = M.at<float>(1, 2);
    float r31 = M.at<float>(2, 0);
    float r32 = M.at<float>(2, 1);
    float r33 = M.at<float>(2, 2);

    double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if (q0 < 0.0f)
        q0 = 0.0f;
    if (q1 < 0.0f)
        q1 = 0.0f;
    if (q2 < 0.0f)
        q2 = 0.0f;
    if (q3 < 0.0f)
        q3 = 0.0f;
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if (q0 >= q1 && q0 >= q2 && q0 >= q3)
    {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3)
    {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3)
    {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2)
    {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    }
    else
    {
        cerr << "Coding error" << endl;
    }
    double r = NORM(q0, q1, q2, q3);
    qx = static_cast<float>(q0 / r);
    qy = static_cast<float>(q1 / r);
    qz = static_cast<float>(q2 / r);
    qw = static_cast<float>(q3 / r);

    tx = M.at<float>(0, 3);
    ty = M.at<float>(1, 3);
    tz = M.at<float>(2, 3);
}
void savePosesToFile(string filename, const std::map<uint32_t, ucoslam::se3>& fmp)
{
    std::ofstream file(filename);
    float qx, qy, qz, qw, tx, ty, tz;
    for (auto framePose : fmp)
    {
        if (framePose.second.isValid())
        {
            cv::Mat mpose=framePose.second;
            getQuaternionAndTranslationfromMatrix44(mpose, qx, qy, qz, qw, tx, ty, tz);
            file << framePose.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                 << qw << endl;
        }
    }
}
