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
#include "typesg2o.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include "basictypes/debug.h"
#include "basictypes/misc.h"
#include "basictypes/timers.h"
#include "map_types/marker.h"
#include "map.h"
#include "pnpsolver.h"
namespace ucoslam{
int g2o_solvePnp( const Frame &frame, std::shared_ptr<Map> TheMap, std::vector<cv::DMatch> &map_matches, cv::Mat  &pose_io ,int64_t currentKeyFrame);

//matches_io: trainIdx must be mappoints ids and queryIdx must be frame ids
bool PnPSolver::solvePnPRansac( const Frame &frame, std::shared_ptr<Map> map, std::vector<cv::DMatch> &matches_io, se3 &posef2g_io ,int maxIters)
{

    if(matches_io.size()<4)return false;

    float maxErr=5.99;
    //get the 3d and 2d matches
    vector<cv::Point2d> p2d; p2d.reserve(matches_io.size());
    vector<cv::Point3d> p3d; p3d.reserve(matches_io.size());
    for(cv::DMatch m: matches_io){
        assert( map->map_points.is(m.trainIdx));
        assert (!map->map_points[m.trainIdx].isBad());
        p2d.push_back(frame.und_kpts[m.queryIdx].pt);
        p3d.push_back( map->map_points[m.trainIdx].getCoordinates());
    }

    std::vector<int> indices(matches_io.size());
    for(size_t i=0;i<indices.size();i++) indices[i]=i;

    int nSamples= 4;
    vector<cv::Point2d> _p2d(nSamples);
    vector<cv::Point3d> _p3d(nSamples);
    vector<cv::Point2d> reprojected(matches_io.size());
    vector<int> inliers;inliers.reserve(matches_io.size());
    vector<int> inliers_Best;
    const float *cam=frame.imageParams.CameraMatrix.ptr<float>(0);
    for(int iter=0;iter<maxIters;iter++){
        inliers.clear();
        std::random_shuffle(indices.begin(),indices.end());
        for(int j=0;j<nSamples;j++){
            _p2d[j]=p2d[  indices[j] ];
            _p3d[j]=p3d[  indices[j] ];
        }
        cv::Mat rv,tv;
        if(!cv::solvePnP(_p3d,_p2d, frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),rv,tv,false,cv::SOLVEPNP_P3P)) continue;
        if( rv.total()!=3 || tv.total()!=3) continue;
        //now, count number of inliers of the solution
        //let us project, create the projection matrix
        Se3Transform f2g_(rv,tv);
        for(size_t j=0;j<p3d.size();j++){
            cv::Point3f pointWRTFrame=f2g_*p3d[j];
            pointWRTFrame.z=1./pointWRTFrame.z;
            reprojected[j].x= ((cam[0]*pointWRTFrame.x)*pointWRTFrame.z)+cam[2];
            reprojected[j].y= ((cam[4]*pointWRTFrame.y)*pointWRTFrame.z)+cam[5];
        }

        //  cv::projectPoints(p3d,rv,tv,frame.imageParams.CameraMatrix,cv::Mat::zeros(1,5,CV_32F),reprojected);

        auto g2f_=f2g_.inv();
        cv::Point3f camCenter= g2f_*cv::Point3f(0,0,0);

        int nInliers=0;
        for(size_t j=0;j<p3d.size();j++)
        {
            inliers[j]=0;
            float distX = p2d[j].x-reprojected[j].x;
            float distY = p2d[j].y-reprojected[j].y;
            if(distX*distX+distY*distY<maxErr){
                //check visual angle with camera centre
                //take the point and see the angle
                const ucoslam::MapPoint &mapPoint=map->map_points[ matches_io[j].trainIdx];
                if (mapPoint.getViewCos(camCenter)<0.5) continue;
                nInliers++;
                inliers.push_back(j);
            }
        }
        if(inliers.size()>inliers_Best.size()){
            inliers_Best=inliers;
            posef2g_io.setRT(rv,tv);
        }
    }

    if(inliers_Best.size()<4) return false;
    std::vector<cv::DMatch> new_matches_io;new_matches_io.reserve(inliers_Best.size());
    for(auto i:inliers_Best)
        new_matches_io.push_back( matches_io[i]);
    matches_io=new_matches_io;
    return true;
}

int PnPSolver::solvePnp( const Frame &frame, std::shared_ptr<Map> TheMap, std::vector<cv::DMatch> &map_matches, se3 &estimatedPose ,
              int64_t currentKeyFrame){
    cv::Mat pose_io=estimatedPose.convert();


    //useful functions
    auto   toSE3Quat=[](const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    };
    auto  toCvMat=[](const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=eigMat(i,j);

        return cvMat.clone();

    };


    //Check if can START
    //no keypoints and no markers
    if (map_matches.size()==0 && frame.markers.size()==0)
        return 0;

    //only markers, then check if map has any valid marker already
    if (frame.markers.size()!=0 && map_matches.size()==0){
        //if no valid markers in map, go
        int nvalidmarkers=0;
        for(const auto &m:TheMap->map_markers)
            if(m.second.pose_g2m.isValid()) nvalidmarkers++;
        if( nvalidmarkers==0) return 0; //no
    }

    //Create the solvers
   // vBadMatches.resize( p3d.size());

    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver=g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


    optimizer.setAlgorithm(solver);

    // Set Frame vertex
    VertexSE3Expmap * G2oVertexCamera = new VertexSE3Expmap();
    G2oVertexCamera->setEstimate(toSE3Quat(pose_io));
    G2oVertexCamera->setId(0);//first vertex, camera
    G2oVertexCamera->setFixed(false);
    optimizer.addVertex(G2oVertexCamera);
    // Set MapPoint vertices

    const float Chi2D =  5.99;
    const float Chi3D =  7.815;
    const float Chi8D = 15.507;
    const float thHuber2D =  sqrt(5.99);
    const float thHuber3D= sqrt(7.815);
    const float thHuber8D = sqrt(15.507);


    float fx=frame.imageParams.fx();
    float fy=frame.imageParams.fy();
    float cx=frame.imageParams.cx();
    float cy=frame.imageParams.cy();

    vector<float> invScaleFactor(frame.scaleFactors);
    for(auto &v:invScaleFactor)v=1./v;


    struct edgeinfo{
        float MaxChi=0;
        void *ptr;
    };
    std::vector<edgeinfo> edgesInfo(map_matches.size());


    vector<bool> vBadMatches(map_matches.size(),false);
    double             KpWeightSum=0;
    for(size_t i=0;i<map_matches.size();i++){

        auto kpt=frame.und_kpts[ map_matches[i].queryIdx] ;
        auto &mp=TheMap->map_points[ map_matches[i].trainIdx];
        auto p3d=mp.getCoordinates();
        float edge_weight=1;
        if( !mp.isStable()) edge_weight=0.5;


        float depth=frame.getDepth(map_matches[i].queryIdx);
        //Monocular point
        if(depth<=0  ){


            Eigen::Matrix<double,2,1> obs;
            obs << kpt.pt.x , kpt.pt.y;

            EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose(p3d.x,p3d.y,p3d.z,fx,fy,cx,cy);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(G2oVertexCamera));
            e->setMeasurement(obs);

            const float invSigma2 = invScaleFactor[kpt.octave];
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            WeightedHubberRobustKernel* rk = new WeightedHubberRobustKernel;
            rk->set(thHuber2D,edge_weight);
            e->setRobustKernel(rk);

            optimizer.addEdge(e);

            edgesInfo[i].ptr=(void*)e;
            edgesInfo[i].MaxChi=Chi2D;
        }
        //stereo point
        else{

            //SET EDGE
            Eigen::Matrix<double,3,1> obs;
            //compute the right proyection difference
            float mbf=frame.imageParams.bl*frame.imageParams.fx();
            const float kp_ur = kpt.pt.x - mbf/depth;
            obs << kpt.pt.x, kpt.pt.y, kp_ur;

            EdgeStereoSE3ProjectXYZOnlyPose* e = new  EdgeStereoSE3ProjectXYZOnlyPose();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(G2oVertexCamera));
            e->setMeasurement(obs);
            const float invSigma2 = 1./frame.scaleFactors[kpt.octave];
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
            e->setInformation(Info);


            edge_weight*=2;
            WeightedHubberRobustKernel* rk = new WeightedHubberRobustKernel;
            rk->set(thHuber3D,edge_weight);
            e->setRobustKernel(rk);

            e->fx = fx;
            e->fy = fy;
            e->cx = cx;
            e->cy = cy;
            e->bf = mbf;
            e->Xw[0] = p3d.x;
            e->Xw[1] = p3d.y;
            e->Xw[2] = p3d.z;

            optimizer.addEdge(e);

            edgesInfo[i].ptr=(void*)e;
            edgesInfo[i].MaxChi=Chi3D;
        }
        KpWeightSum+=edge_weight;

    }

        //now, markers
        std::vector<std::pair<ucoslam::Marker,ucoslam::MarkerObservation> > marker_poses;
        if ( frame.markers.size()!=0 && currentKeyFrame!=-1){
            //get all neighbors
            auto neigh=TheMap->getNeighborKeyFrames(currentKeyFrame,true);
            //get all the valid markers in the neighbors
            std::set<uint32_t> markerInNeighbors;
            for(auto n:neigh){
                for(const auto &m:TheMap->keyframes[n].markers){
                    if (TheMap->map_markers[m.id].pose_g2m.isValid())
                        markerInNeighbors.insert(m.id);
                }
            }

            //create the vector with marker poses
            for(auto m:frame.markers){
                if ( markerInNeighbors.count(m.id)==0)continue;
                marker_poses.push_back({TheMap->map_markers[m.id],m});
            }
        }

        //Let us add the markers
        vector<MarkerEdgeOnlyProject*> marker_edges;
        marker_edges.reserve(marker_poses.size());

        //compute the weight of markers considering that w_markers+w_points must be 1.
        //the total sum of poits weigh is so far totalNEdges.
        //So first, count nunmber of marker edges
        float w_markers=0.3;
        int totalNEdges=map_matches.size()+ marker_poses.size();
        double weight_marker= ((w_markers *totalNEdges)/ (1.- w_markers))/float(KpWeightSum);

        uint32_t vid=1;


        for(auto mpose: marker_poses){
            //add first the markers
            VertexSE3Expmap * G2oVertexMarker= new VertexSE3Expmap();
            G2oVertexMarker->setEstimate(toSE3Quat(mpose.first.pose_g2m));
            G2oVertexMarker->setFixed(true);
            G2oVertexMarker->setId(vid++);
            optimizer.addVertex(G2oVertexMarker);
            //now, the edge

            MarkerEdgeOnlyProject* e = new MarkerEdgeOnlyProject(mpose.first.size);
            Eigen::Matrix<double,8,1> obs;
            auto mcorners=mpose.second.und_corners;
            for(int i=0;i<4;i++){
                obs(i*2)=mcorners[i].x;
                obs(i*2+1)=mcorners[i].y;
            }
            e->setMeasurement(obs);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(G2oVertexMarker));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( G2oVertexCamera));
            e->fx=fx;
            e->fy=fy;
            e->cx=cx;
            e->cy=cy;

            e->setInformation( Eigen::Matrix< double, 8, 8 >::Identity());
            WeightedHubberRobustKernel* rk = new  WeightedHubberRobustKernel;
            e->setRobustKernel(rk);
            rk->set(thHuber8D,weight_marker);


            optimizer.addEdge(e);
            marker_edges.push_back(e);
        }


    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.

    std::vector<int> its={10,10,10,10};

    for(size_t it=0; it<its.size(); it++)
    {

        G2oVertexCamera->setEstimate(toSE3Quat(pose_io));
        optimizer.initializeOptimization(0);
        optimizer.setVerbose(debug::Debug::getLevel()>=11);
        optimizer.optimize(its[it]);
        int nGoodMatches=0;

        for(size_t i=0, iend=edgesInfo.size(); i<iend; i++)
        {
            EdgeSE3ProjectXYZOnlyPose* e = (EdgeSE3ProjectXYZOnlyPose*) edgesInfo[i].ptr;

            if(vBadMatches[i]) e->computeError();
            vBadMatches[i]=e->chi2()>edgesInfo[i].MaxChi;

            e->setLevel(vBadMatches[i]?1:0);
            if(it>=2) e->setRobustKernel(nullptr);
            if(!vBadMatches[i])nGoodMatches++;
        }

        for(auto me:marker_edges){
                me->computeError();
                if (me->chi2()>Chi8D || it>=2)
                    me->setRobustKernel(nullptr);
        }

        //if few keypoints, stop
        if( nGoodMatches<  10 &&marker_poses.size()==0){
            break;
        }
    }

    // Recover optimized pose and return number of inliers
    VertexSE3Expmap* vSE3_recov = static_cast<VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    pose_io=  toCvMat(SE3quat_recov);



    //set bad matches to -1 so that they will not be used

    int nbadmatches=0;
    for(size_t i=0;i<vBadMatches.size();i++)
        if ( vBadMatches[i]) {
            nbadmatches++;
            map_matches[i].imgIdx=-1;//set this to indicate that the element is outlier
        }
        else map_matches[i].imgIdx=1;//set to indicate that is not a bad match
    int retval= map_matches.size()-nbadmatches;


    estimatedPose=pose_io;
    return retval ;
}
}
