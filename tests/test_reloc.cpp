#include "utils/system.h"
#include "utils/frameextractor.h"
#include "basictypes/misc.h"
#include "optimization/pnpsolver.h"
#include "map.h"
#include "utils/framematcher.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace  std;
using namespace  ucoslam;

namespace ucoslam {

class     DebugTest {
public:


    ucoslam::Params _params;
    std::shared_ptr<ucoslam::System> system;
    ucoslam::ImageParams imageparams;
    struct kp_reloc_solution{
        se3 pose;
        int nmatches=0;
        uint32_t fidx=std::numeric_limits<uint32_t>::max();
        std::vector<uint32_t> ids;
        std::vector<cv::DMatch> matches;
    };
    void readFromFile(string path){
        system=std::make_shared<ucoslam::System>();
        system->readFromFile(path);
        imageparams=system->TheMap->keyframes.front().imageParams;
    }
    vector<uint32_t> relocalizeCandidatesError(cv::Mat &im,uint32_t fseqid){
        Frame cFrame;
        system->fextractor->process(im,imageparams,cFrame,fseqid);
        //find the id of the nearest element
        Frame &nearest=system->TheMap->keyframes[0];
        for(auto &kf:system->TheMap->keyframes){
            if( std::fabs( int64_t(nearest.fseq_idx) - int64_t(fseqid)) > std::fabs(int64_t(kf.fseq_idx)-int64_t(fseqid)))
                nearest=kf;
        }

        //get the neighbors and remove them in the relocalization
        auto neigh=system->TheMap->TheKpGraph.getNeighbors(nearest.idx,true);
        se3 pose;
        auto res=relocalization_withkeypoints_(cFrame,pose,neigh);
        cout<<fseqid<<" "<<res.size()<<" ";
        cerr<<fseqid<<" "<<res.size()<<" "<<endl;
        vector<uint32_t> errorFrames;
        for(auto r:res){
            cout<<r.fidx  <<" ";
  //            cout<<system->TheMap->keyframes[  r.fidx ].fseq_idx<<" ";
            errorFrames.push_back(system->TheMap->keyframes[  r.fidx ].fseq_idx);
        }
        cout<<endl;


        return errorFrames;

    }
    std::vector<kp_reloc_solution>  relocalization_withkeypoints_( Frame &curFrame,se3 &pose_f2g_out ,const std::set<uint32_t> &excluded={} ){
        if (curFrame.ids.size()==0)return { };
        if (system->TheMap->TheKFDataBase.isEmpty())return  {};
        vector<uint32_t> kfcandidates=system->TheMap->relocalizationCandidates(curFrame,excluded);

        if (kfcandidates.size()==0)return  {};

        vector<kp_reloc_solution> Solutions(kfcandidates.size());

        FrameMatcher FMatcher;
        FMatcher.setParams(curFrame,FrameMatcher::MODE_ALL,_params.maxDescDistance*2);

#pragma omp parallel for
        for(int cf=0;cf<kfcandidates.size();cf++){

            auto kf=kfcandidates[cf];
            auto &KFrame=system->TheMap->keyframes[kf];
            Solutions[cf].matches=FMatcher.match(KFrame,FrameMatcher::MODE_ASSIGNED);
            Solutions[cf].fidx=kf;


            //change trainIdx and queryIdx to match the  solvePnpRansac requeriments
            for(auto &m:Solutions[cf].matches){
                std::swap(m.queryIdx,m.trainIdx);
                m.trainIdx= KFrame.ids[m.trainIdx];
            }
            //remove bad point matches
            for(int i=0;i<Solutions[cf].matches.size();i++){
                auto &mp=Solutions[cf].matches[i].trainIdx;
                if( !system->TheMap->map_points.is(mp))
                    Solutions[cf].matches[i].trainIdx=-1;
                if( system->TheMap->map_points[mp].isBad()  )
                    Solutions[cf].matches[i].trainIdx=-1;
            }

            remove_unused_matches(Solutions[cf].matches);
            if (Solutions[cf].matches.size()<25)continue;
            Solutions[cf].pose=KFrame.pose_f2g;
            //estimate initial position
            PnPSolver::solvePnPRansac(curFrame,system->TheMap,Solutions[cf].matches,Solutions[cf].pose);
            if (Solutions[cf].matches.size()<25) continue;
            //go to the map looking for more matches
            Solutions[cf].matches= system->TheMap->matchFrameToMapPoints ( system->TheMap->TheKpGraph.getNeighborsVLevel2( kf,true) , curFrame,  Solutions[cf].pose ,_params.maxDescDistance*2, 2.5,true);
            if (Solutions[cf].matches.size()<30) continue;

            //now refine
            PnPSolver::solvePnp(curFrame,system->TheMap,Solutions[cf].matches,Solutions[cf].pose);
            if (Solutions[cf].matches.size()<50) continue;
            Solutions[cf]. ids=curFrame.ids;
            for(auto match: Solutions[cf].matches)
                Solutions[cf].ids[ match.queryIdx]=match.trainIdx;
        }

        //take the solution with more matches
        Solutions.erase( std::remove_if(Solutions.begin(),Solutions.end(),[](const kp_reloc_solution &a){return a.matches.size()<=50;}),Solutions.end());
        std::sort( Solutions.begin(),Solutions.end(),[](const kp_reloc_solution &a,const kp_reloc_solution &b){return a.matches.size()>b.matches.size();});

        return Solutions;


    }
};
}

cv::Mat resize(const cv::Mat &im,cv::Size res){
    cv::Mat im2;
    cv::resize(im,im2,res);
    return im2;
}
int main(int argc,char **argv){
    try {
        if(argc==1)throw  std::runtime_error("Usage: map video");

        DebugTest dtest;
        cerr<<"Loading map"<<endl;
        dtest.readFromFile(argv[1]);

        cv::VideoCapture vcap;
        cerr<<"open video"<<endl;
        vcap.open(argv[2]);
        if(!vcap.isOpened()) throw  std::runtime_error("Could not open video ");


        cv::Mat im;
        bool show=false;
        while(vcap.grab()){
            vcap.retrieve(im);
            auto errorFrames=dtest.relocalizeCandidatesError(im,vcap.get(CV_CAP_PROP_POS_FRAMES));
            if( (errorFrames.size()>0 && show) /* || errorFrames.size()==5*/){

                int curF=vcap.get(CV_CAP_PROP_POS_FRAMES);
                cv::imshow("in",resize(im,cv::Size(800,600)));
                for(int i=0;i< errorFrames.size();i++){
                    vcap.set(CV_CAP_PROP_POS_FRAMES, errorFrames[i]);
                    cv::Mat imerror;
                    vcap.grab();
                    vcap.retrieve(imerror);
                    vcap.set(CV_CAP_PROP_POS_FRAMES,curF);
                    cv::imshow("error-"+std::to_string(i),resize(imerror,cv::Size(800,600)));
                }
                int k=cv::waitKey(0);
                for(int i=0;i< errorFrames.size();i++)                 cv::destroyWindow("error-"+std::to_string(i));

                if(k=='s')                  vcap.set(CV_CAP_PROP_POS_FRAMES,curF+100);

            }

        }

    } catch (std::exception &ex) {
        cerr<<ex.what()<<endl;
    }{}
}
