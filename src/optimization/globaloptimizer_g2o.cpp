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
#include "globaloptimizer_g2o.h"
#include "typesg2o.h"

#include "basictypes/misc.h"
#include "basictypes/timers.h"
#include <chrono>
#include "map_types/marker.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/base_binary_edge.h"
#include <iostream>

//#define ORIGINAL_G2O_SBA
namespace ucoslam{


class  MarkerEdgeX: public  g2o::BaseBinaryEdge<4, Vector8D,  VertexSE3Expmap,  VertexSE3Expmap>
{

    int _ref,_other;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MarkerEdgeX(int ref,int other){
      _ref=ref;
      _other=other;
  }
  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}
  inline void computeError()  {
      const  VertexSE3Expmap* m1g2m= static_cast<const  VertexSE3Expmap*>(_vertices[0]);//marker1
      const  VertexSE3Expmap* m2g2m = static_cast<const  VertexSE3Expmap*>(_vertices[1]);//marker12
      Eigen::Matrix<double,4,4> eM12=m1g2m->estimate().to_homogeneous_matrix().inverse()*  m2g2m->estimate().to_homogeneous_matrix();
      _error.resize(4);
      _error(0)=10.*eM12(0,2);
      _error(1)=10.* eM12(1,2);
      _error(2)=10.*(1-eM12(2,2));
      _error(3)=10.*eM12(2,3);

  }

};
g2o::RobustKernel *createKernel(double delta,double weight=1){
 #if 1
    g2o::RobustKernelHuber* rk = new  g2o::RobustKernelHuber();
    rk->setDelta(delta);
    return rk;
    return nullptr;
#else
    WeightedHubberRobustKernel* rk = new  WeightedHubberRobustKernel();
    rk->set(delta,weight);
     return rk;
#endif

}
void GlobalOptimizerG2O::setParams(std::shared_ptr<Map> map, const ParamSet &xp ){


    auto  toSE3Quat=[](const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    };




    _params=xp;
    _InvScaleFactors.reserve(map->keyframes.front().scaleFactors.size());
    for(auto f:map->keyframes.front().scaleFactors) _InvScaleFactors.push_back(1./f);


    //Find the frames and points that will be used
    isFixedFrame.resize(map->keyframes.capacity());
    usedFramesIdOpt.resize(map->keyframes.capacity());
    usedPointsIdOpt.resize(map->map_points.capacity());
    for(auto &v:usedFramesIdOpt)v=INVALID_IDX;
    for(auto &v:usedPointsIdOpt)v=INVALID_IDX;
    for(auto &v:isFixedFrame)v=UNFIXED;


    uint32_t optiD=0;//optimizers vertex id
    if (_params.used_frames.size()==0){     //if non indicated, use all
        for(auto f:map->keyframes)
            usedFramesIdOpt[f.idx]=optiD++;
    }
    else {//else, set the required ones
        for(auto f:_params.used_frames)
            usedFramesIdOpt[f]=optiD++;
    }


    if(_params.fixFirstFrame ){
        if ( usedFramesIdOpt[map->keyframes.front().idx]!=INVALID_IDX )
            isFixedFrame[map->keyframes.front().idx]=FIXED_WITHPOINTS;
    }

    //add also the fixed ones (in case they are not in the used_frames map
    for(auto f:_params.fixed_frames) {
        if (usedFramesIdOpt[f]!=INVALID_IDX)//add it if not yet
            isFixedFrame[f]=FIXED_WITHPOINTS;
    }

    usedMapPoints.clear();

    //now, go thru points assigning ids
    for( uint32_t f=0;f<usedFramesIdOpt.size();f++){

        if ( usedFramesIdOpt[f]==INVALID_IDX)continue;
        if (  isFixedFrame[f]==FIXED_WITHOUTPOINTS)continue;

        for(auto point_id:map->keyframes[f].ids){
            if ( point_id==INVALID_IDX) continue;
            if (usedPointsIdOpt[point_id]==INVALID_IDX){
                if( (map->map_points[point_id].frames.size()<2 && !map->map_points[point_id].isStereo() ) || map->map_points[point_id].isBad() )
                    usedPointsIdOpt[point_id]=INVALID_VISITED_IDX;
                else{
                    usedPointsIdOpt[point_id]=optiD++;
                    assert( std::find(usedMapPoints.begin(),usedMapPoints.end(),point_id)== usedMapPoints.end());
                    usedMapPoints.push_back(point_id);
                    for( const auto &f_info:map->map_points[point_id].frames)
                        if (usedFramesIdOpt[f_info.first]==INVALID_IDX){
                            usedFramesIdOpt[f_info.first]=optiD++;
                            isFixedFrame[f_info.first]=FIXED_WITHOUTPOINTS;
                        }
                }
            }
        }
        //now, the markers

        for(auto marker:map->keyframes[f].markers){
            if ( marker_info.count(marker.id)!=0 ) continue;//marker alreday added
            auto &map_marker=map->map_markers[marker.id];
            if (!map_marker.pose_g2m.isValid()) continue;//invalid pose yet
                 marker_info[marker.id].IdOptmz=(optiD++);//set a id for the optimizer
                //add all its frames if not yet
                for(auto frames_id:map_marker.frames)
                    if (usedFramesIdOpt[frames_id]==INVALID_IDX){
                        usedFramesIdOpt[frames_id]=optiD++;
                        isFixedFrame[frames_id]=FIXED_WITHOUTPOINTS;
                    }

        }

    }

    //finally, add

    Optimizer=std::make_shared<g2o::SparseOptimizer>();
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    Optimizer->setAlgorithm(solver);


    uint32_t totalVars=0;


    ///////////////////////////////////////
    /// Add KeyFrames as vertices of the graph
    ///////////////////////////////////////

     for( uint32_t fid=0;fid<usedFramesIdOpt.size();fid++){
         if (usedFramesIdOpt[fid]!=INVALID_IDX){
            VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
            vSE3->setEstimate(toSE3Quat(map->keyframes[fid].pose_f2g));
            vSE3->setId(usedFramesIdOpt.at(fid));

            if (isFixedFrame[fid])   vSE3->setFixed(true);

            totalVars+=6;
            Optimizer->addVertex(vSE3);
        }
    }

    ///////////////////////////////////////
    /// ADD MAP POINTS AND LINKS TO KEYFRAMES
    ///////////////////////////////////////
    point_edges_frameId.resize(map->map_points.capacity());
     for(auto mp_id:usedMapPoints){


        MapPoint &mp=map->map_points[mp_id];
        VertexSBAPointXYZ* vPoint = new VertexSBAPointXYZ();
        cv::Point3f p3d=mp.getCoordinates();
        Eigen::Matrix<double,3,1> vp3d;
        vp3d << p3d.x , p3d.y ,p3d.z;
        vPoint->setEstimate(vp3d);
        vPoint->setId(usedPointsIdOpt.at(mp_id));
        vPoint->setMarginalized(true);
        Optimizer->addVertex(vPoint);
        point_edges_frameId[mp_id].reserve(mp.frames.size());
        float edge_weight=1;

        //SET EDGES
         for( const auto &f_info:mp.frames)
        {
            if ( usedFramesIdOpt.at(f_info.first)==INVALID_IDX) continue;
             const auto &Frame=map->keyframes[f_info.first];
            const cv::KeyPoint &kpUn =Frame.und_kpts[f_info.second];
            float depth=Frame.getDepth(f_info.second);

            if ( depth<=0){
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;
                EdgeSE3ProjectXYZ* e = new  EdgeSE3ProjectXYZ();
                e->fx = Frame.imageParams.fx();
                e->fy = Frame.imageParams.fy();
                e->cx =  Frame.imageParams.cx();
                e->cy =  Frame.imageParams.cy();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedPointsIdOpt.at(mp_id))));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(f_info.first))));
                e->setMeasurement(obs);
                //try a different confidence based on the robustness (n times seen)
                e->setInformation(Eigen::Matrix2d::Identity()*_InvScaleFactors[kpUn.octave]);
                e->setRobustKernel(createKernel(thHuber2D,edge_weight));
                Optimizer->addEdge(e);
                point_edges_frameId[mp_id].push_back(edge_frameId_stereo((void*)e,f_info.first,false));
                frame_kpOptWeight[f_info.first]+=2*_InvScaleFactors[kpUn.octave];
            }
            else{//stereo observation
                Eigen::Matrix<double,3,1> obs;
                //compute the right proyection difference
                float mbf=Frame.imageParams.bl*Frame.imageParams.fx();
                const float kp_ur = kpUn.pt.x - mbf/depth;
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                EdgeStereoSE3ProjectXYZ* e = new  EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedPointsIdOpt.at(mp_id))));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(f_info.first))));
                e->setMeasurement(obs);
                e->setInformation(Eigen::Matrix3d::Identity()*double(_InvScaleFactors[kpUn.octave]));
                e->setRobustKernel(createKernel(thHuber3D,edge_weight));
                e->fx = Frame.imageParams.fx();
                e->fy = Frame.imageParams.fy();
                e->cx =  Frame.imageParams.cx();
                e->cy =  Frame.imageParams.cy();
                e->bf = mbf;
                Optimizer->addEdge(e);
                point_edges_frameId[mp_id].push_back(edge_frameId_stereo((void*)e,f_info.first,true));
                frame_kpOptWeight[f_info.first]+=3*_InvScaleFactors[kpUn.octave];
            }
        }
    }


      ///////////////////////////////////////////
     ///compute the weight for markers-keyframes
     ///////////////////////////////////////////
     double totalKpWeight=0;
     for( uint32_t fid=0;fid<usedFramesIdOpt.size();fid++){
         if (usedFramesIdOpt[fid]==INVALID_IDX)continue;
         Frame &frame=map->keyframes.at(fid);
         //compute the weight of this marker in this frame
         double kpw= frame_kpOptWeight[fid];
         totalKpWeight+=kpw;

         double weight_per_error=1;
         frame_MarkerWeight[fid].val=weight_per_error;

         if( kpw>40 && frame.markers.size()>0){//at least 20 keypoints visible
             double marker_perct= _params.markersOptWeight* std::min(1.,double(frame.markers.size())/_params.minMarkersForMaxWeight);
             double total_w=marker_perct*kpw;
             //divide so that the sum is the value required
             weight_per_error= total_w/double(frame.markers.size()*8);
             frame_MarkerWeight[fid].val=weight_per_error;

         }
     }


    ///////////////////////////////////////
    ///  ADD MARKERS Edges
    ///////////////////////////////////////


    for(auto &m:marker_info){
        ucoslam::Marker &marker= map->map_markers.at(m.first);
        VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
        vSE3->setEstimate(toSE3Quat( marker.pose_g2m));
        vSE3->setId(m.second.IdOptmz);
        Optimizer->addVertex(vSE3);
        m.second.vertex=vSE3;
        totalVars+=6;
    }



    double totalMarkerWeight=0;
    for(const auto &m:marker_info){
        VertexSE3Expmap * vSE3 =  (VertexSE3Expmap *)m.second.vertex;
        ucoslam::Marker &marker= map->map_markers.at(m.first);

        //Now add the links between markers and frames
        for(auto fid:marker.frames){
            assert(usedFramesIdOpt[fid]!=INVALID_IDX);



            Frame &frame=map->keyframes.at(fid);
            MarkerEdge * e = new MarkerEdge(marker.size,marker.id,fid);
            Eigen::Matrix<double,8,1> obs;
            auto mcorners=frame.getMarker(marker.id).und_corners;
            for(int i=0;i<4;i++){
                obs(i*2)=mcorners[i].x;
                obs(i*2+1)=mcorners[i].y;
            }
            e->setMeasurement(obs);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSE3));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(Optimizer->vertex(usedFramesIdOpt.at(fid))));
            e->fx=frame.imageParams.fx();
            e->fy=frame.imageParams.fy();
            e->cx=frame.imageParams.cx();
            e->cy=frame.imageParams.cy();
            double fmw=frame_MarkerWeight[fid].val;
            e->setInformation( Eigen::Matrix< double, 8, 8 >::Identity()* fmw);
            totalMarkerWeight+=fmw*8;
            Optimizer->addEdge(e);
            marker_edges.push_back(e);

        }
    }
    ////
    /// The Planar case:
    ///  create relationship between markers if planar case
    //check if at least two markers in the map with valid pose
    int nValidMarkers=0;
    for(const auto &m:map->map_markers) if(m.second.pose_g2m.isValid())nValidMarkers++;
    if(_params.InPlaneMarkers  && nValidMarkers>=2){
        //select one as reference marker (the one with more views)
        std::pair<uint32_t,uint32_t> best_nframes(std::numeric_limits<uint32_t>::max(),0);

         for(const auto &m:map->map_markers){
            if( !m.second.pose_g2m.isValid()) continue;
            if (  map->map_markers[m.first].frames.size()>best_nframes.second)
                best_nframes={m.first,m.second.frames.size()};            
        }
        auto ref_marker_id=best_nframes.first;

        if( ref_marker_id!=std::numeric_limits<uint32_t>::max()){
            //if the reference marker not in the optimization, add it as fixed
            if(marker_info.count(ref_marker_id)==0){
                 VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
                marker_info[ref_marker_id].IdOptmz=std::numeric_limits<int>::max();
                vSE3->setEstimate(toSE3Quat( map->map_markers.at(ref_marker_id).pose_g2m));
                vSE3->setId(marker_info[ref_marker_id].IdOptmz);
                vSE3->setFixed(true);
                Optimizer->addVertex(vSE3);
                marker_info[ref_marker_id].vertex=vSE3;
            }

            auto refMarkerPtr= marker_info[ref_marker_id].vertex;
            double totalPlanarWeight= 0.33*(totalMarkerWeight+totalKpWeight) ;
            double planar_weight_w= totalPlanarWeight / double(4*(marker_info.size()-1));
            for(auto m:marker_info){
                if(m.first==ref_marker_id) continue;
                MarkerEdgeX * e = new MarkerEdgeX( ref_marker_id,m.first);
                e->setVertex(0,(g2o::OptimizableGraph::Vertex*) refMarkerPtr);
                e->setVertex(1, (g2o::OptimizableGraph::Vertex*) m.second.vertex);
                e->setInformation(planar_weight_w* Eigen::Matrix< double, 4, 4 >::Identity());
                Optimizer->addEdge(e);
            }
        }
        else{
         //   cerr<<"No reference frame for InPlane"<<endl;
        }

    }


}

void GlobalOptimizerG2O::optimize(std::shared_ptr<Map> map, const ParamSet &p ) {
    __UCOSLAM_ADDTIMER__
        setParams(map,p);
    __UCOSLAM_TIMER_EVENT__("Setparams");
    optimize();
    __UCOSLAM_TIMER_EVENT__("Optmize");
    getResults(map);
    __UCOSLAM_TIMER_EVENT__("getResults");

}





void GlobalOptimizerG2O::optimize(bool *stopASAP ){


    __UCOSLAM_ADDTIMER__
        Optimizer->initializeOptimization();
    Optimizer->setForceStopFlag(stopASAP);
    Optimizer->setVerbose( _params.verbose);
#pragma message "warning: change this to consider maximum interstep error"

    Optimizer->optimize(_params.nIters,1);
//    Optimizer->optimize(_params.nIters,1);
    bool continueProcessing=true;
    if(stopASAP!=nullptr)
        if(*stopASAP) continueProcessing=false;
    if( continueProcessing)
    {
        for( auto &mp_id:usedMapPoints)
        {
            for(auto e_fix:point_edges_frameId.at (mp_id)){
                if (e_fix.isStereo){
                    if( ((EdgeStereoSE3ProjectXYZ*)e_fix.first)->chi2()>Chi3D || !((EdgeStereoSE3ProjectXYZ*)e_fix.first)->isDepthPositive())
                        ((EdgeStereoSE3ProjectXYZ*)e_fix.first)->setLevel(1);
                    ((EdgeStereoSE3ProjectXYZ*)e_fix.first)->setRobustKernel(0);
                }


                else{
                    if( ((EdgeSE3ProjectXYZ*)e_fix.first)->chi2()>Chi2D|| !((EdgeSE3ProjectXYZ*)e_fix.first)->isDepthPositive())
                        ((EdgeSE3ProjectXYZ*)e_fix.first)->setLevel(1);
                    ((EdgeSE3ProjectXYZ*)e_fix.first)->setRobustKernel(0);
                }
            }
        }
        for(auto me:marker_edges){
            MarkerEdge *medge=(MarkerEdge *)me;
            if (medge->chi2()>Chi8D)  medge->setLevel(0);
            medge->setRobustKernel(0);
        }
        // Optimize again without the outliers

        Optimizer->initializeOptimization();
        Optimizer->optimize(_params.nIters*2,1);

    }


}

void GlobalOptimizerG2O::getResults(std::shared_ptr<Map>  map){
__UCOSLAM_ADDTIMER__

    auto toCvMat=[](const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=eigMat(i,j);

        return cvMat.clone();

    };

    //Keyframes
    for(uint32_t fid=0;fid<usedFramesIdOpt.size();fid++)
    {
        if (usedFramesIdOpt[fid]==INVALID_IDX ) continue;
        if (isFixedFrame[fid])continue;

        Frame &frame=map->keyframes[fid];
        VertexSE3Expmap* vSE3 = static_cast<VertexSE3Expmap*>(Optimizer->vertex(usedFramesIdOpt[fid]));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        frame.pose_f2g= toCvMat(SE3quat);
    }
    __UCOSLAM_TIMER_EVENT__("Frames updated");

    _badAssociations.clear();
    _badAssociations.reserve(usedMapPoints.size());
    //Points
    for( auto &mp_id:usedMapPoints)
    {
        VertexSBAPointXYZ* vPoint = static_cast<VertexSBAPointXYZ*>(Optimizer->vertex( usedPointsIdOpt[mp_id]));
        auto est=vPoint->estimate();
        MapPoint &mp=map->map_points[mp_id];
        assert(!mp.isBad());
        mp.setCoordinates(cv::Point3f( est(0),est(1),est(2)));

        //check each projection
        for(auto e_fix:point_edges_frameId.at (mp.id)){
            bool isBad=false;
            if(e_fix.isStereo){
                if( ((EdgeStereoSE3ProjectXYZ*)e_fix.first)->chi2()>Chi3D || !((EdgeStereoSE3ProjectXYZ*)e_fix.first)->isDepthPositive())
                    isBad=true;
            }
            else{
                if( ((EdgeSE3ProjectXYZ*)e_fix.first)->chi2()>Chi2D)  isBad=true;
            }
            //check no point is behind a camera
            if(!isBad) {
                cv::Point3f pincam=map->keyframes[e_fix.second].pose_f2g* mp.getCoordinates();
                if( pincam.z<0) isBad=true;
            }
            if(isBad)
                _badAssociations.push_back(std::make_pair(mp.id,e_fix.second));
        }
    }
    __UCOSLAM_TIMER_EVENT__("Detected bad points");

    for(const auto &marker_idop:marker_info)
        map->map_markers[marker_idop.first].pose_g2m= toCvMat(  static_cast<VertexSE3Expmap*>(marker_idop.second.vertex)->estimate());

    __UCOSLAM_TIMER_EVENT__("Updated markers");

    //now, update normals and other values of the mappoints

    for( auto &mp_id:usedMapPoints)
        map->updatePointNormalAndDistances(mp_id);
    __UCOSLAM_TIMER_EVENT__("Updated points and normal distances");

}




}
