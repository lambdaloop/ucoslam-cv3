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
#ifndef UCOSLAM_TYPES_G2O_
#define UCOSLAM_TYPES_G2O_
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/slam3d/se3quat.h"
#include  "g2o/types/sim3/sim3.h"
#include "map_types/marker.h"
namespace ucoslam{

typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>    Vector8D;


/**
 * \brief Point vertex, XYZ
 */
 class   VertexSBAPointXYZ : public g2o::BaseVertex<3, g2o::Vector3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSBAPointXYZ() : BaseVertex<3, g2o::Vector3>()
    {
    }

    virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

    virtual void setToOriginImpl() {
      _estimate.fill(0);
    }

    virtual void oplusImpl(const number_t* update)
    {
      Eigen::Map<const g2o::Vector3> v(update);
      _estimate += v;
    }
};

 /**
  * \brief SE3 Vertex parameterized internally with a transformation matrix
  and externally with its exponential map
  */
 class   VertexSE3Expmap : public g2o::BaseVertex<6, g2o::SE3Quat>{
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   VertexSE3Expmap(){}

   bool read(std::istream& is){throw std::runtime_error("Not implemented");}

   bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

   virtual void setToOriginImpl() {
     _estimate = g2o::SE3Quat();
   }

   virtual void oplusImpl(const number_t* update_)  {
     Eigen::Map<const g2o::Vector6> update(update_);
     setEstimate(g2o::SE3Quat::exp(update)*estimate());
   }
 };

class WeightedHubberRobustKernel : public g2o::RobustKernel
{
    double Weight,Delta,PureRho;
  public:

    void set(double delta,double weight=1){
        Weight=weight;
        Delta=delta;
    }

    virtual void robustify(double e2, g2o::Vector3& rho) const{
        double dsqr = Delta * Delta;
        if (e2 <= dsqr) { // inlier
          rho[0] = Weight*e2;
          rho[1] = 1.;
          rho[2] = 0.;
        } else { // outlier
          double sqrte = sqrt(e2); // absolut value of the error
          rho[0] = Weight*(2*sqrte*Delta - dsqr); // rho(e)   = 2 * delta * e^(1/2) - delta^2
          rho[1] = Delta / sqrte;        // rho'(e)  = delta / sqrt(e)
          rho[2] = - 0.5 * rho[1] / e2;    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
        }
    }
};


class  MarkerEdge: public  g2o::BaseBinaryEdge<8, Vector8D,  VertexSE3Expmap,  VertexSE3Expmap>
{
    g2o::Vector3 points[4];

    uint32_t marker_id,frame_id;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double fx, fy, cx, cy;

  MarkerEdge(double size,uint32_t markerid,uint32_t frameid){
      marker_id=frameid;
      frame_id=frameid;
     _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
      auto pointsA=ucoslam::Marker::get3DPointsLocalRefSystem(size);
      for(int i=0;i<4;i++){
          auto &p=pointsA[i];
          points[i]=g2o::Vector3(p.x,p.y,p.z);
      }
  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}

  inline void computeError()  {
      //marker
      const  VertexSE3Expmap* g2m= static_cast<const  VertexSE3Expmap*>(_vertices[0]);//marker
    //camera
      const  VertexSE3Expmap*  c2g = static_cast<const  VertexSE3Expmap*>(_vertices[1]);//camera

      //std::cout<<" ------------------------"<<std::endl;
    //first, move the points to the global reference system, and then to the camera
    auto Transform_C2M=c2g->estimate()*  g2m->estimate();
    //std::cout<<"g2m="<<g2m->estimate()<<" c2g="<<c2g->estimate()<< " Transform_C2M="<<Transform_C2M<<std::endl;
    g2o::Vector3 points2[4];
    for(int i=0;i<4;i++){
        points2[i]=Transform_C2M.map(points[i]);//3d rigid transform
  //      std::cout<<"p="<<points2[i]<<std::endl;
    }


    //now, project
     _error.resize(8);
    Vector8D obs(_measurement);
//    std::cout<<"obs="<<obs<<std::endl;
    assert(_error.size()==obs.size());
    int idx=0;
    for(int i=0;i<4;i++){
        float projx=( points2[i](0)/points2[i](2)) *fx +cx;
        _error(idx)=obs(idx)-projx;
         idx++;
        float projy=( points2[i](1)/points2[i](2)) *fy +cy;
        _error(idx)=obs(idx)-projy;
         idx++;
    }
//     std::cout<<"E(="<<marker_id<<"-"<< frame_id<<"):"<<_error.transpose()<<std::endl;
  }

};

typedef Eigen::Matrix<double,12,1,Eigen::ColMajor>    Vector12D;

class  MarkerEdgePlanar: public  g2o::BaseBinaryEdge<12, Vector8D,  VertexSE3Expmap,  VertexSE3Expmap>
{
    g2o::Vector3 points[4];

    uint32_t marker_id,frame_id;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double fx, fy, cx, cy;

  MarkerEdgePlanar(double size,uint32_t markerid,uint32_t frameid){
      marker_id=frameid;
      frame_id=frameid;
     _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
      auto pointsA=ucoslam::Marker::get3DPointsLocalRefSystem(size);
      for(int i=0;i<4;i++){
          auto &p=pointsA[i];
          points[i]=g2o::Vector3(p.x,p.y,p.z);
      }
  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}

  inline void computeError()  {
      auto toCvMat=[](const g2o::SE3Quat &SE3)
      {
          Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
          cv::Mat cvMat(4,4,CV_32F);
          for(int i=0;i<4;i++)
              for(int j=0; j<4; j++)
                  cvMat.at<float>(i,j)=eigMat(i,j);

          return cvMat.clone();

      };
      //marker
      const  VertexSE3Expmap* g2m= static_cast<const  VertexSE3Expmap*>(_vertices[0]);//marker
    //camera
      const  VertexSE3Expmap*  c2g = static_cast<const  VertexSE3Expmap*>(_vertices[1]);//camera

      //std::cout<<" ------------------------"<<std::endl;
    //first, move the points to the global reference system, and then to the camera
    auto Transform_C2M=c2g->estimate()*  g2m->estimate();
    //std::cout<<"g2m="<<g2m->estimate()<<" c2g="<<c2g->estimate()<< " Transform_C2M="<<Transform_C2M<<std::endl;
    g2o::Vector3 points2[4];
    for(int i=0;i<4;i++){
        points2[i]=Transform_C2M.map(points[i]);//3d rigid transform
  //      std::cout<<"p="<<points2[i]<<std::endl;
    }


    //now, project
     _error.resize(12);
    Vector8D obs(_measurement);
//    std::cout<<"obs="<<obs<<std::endl;
//    assert(_error.size()==obs.size());
    int idx=0;
    for(int i=0;i<4;i++){
        float projx=( points2[i](0)/points2[i](2)) *fx +cx;
        _error(idx)=obs(idx)-projx;
         idx++;
        float projy=( points2[i](1)/points2[i](2)) *fy +cy;
        _error(idx)=obs(idx)-projy;
         idx++;
    }

     cv::Mat m44=toCvMat(g2m->estimate());
    _error(idx++)=10.*m44.at<float>(0,2);
    _error(idx++)=10.*m44.at<float>(1,2);
    _error(idx++)=10.*(1-m44.at<float>(2,2));
    _error(idx++)=double(10.)*m44.at<float>(2,3);

//     std::cout<<"E(="<<marker_id<<"-"<< frame_id<<"):"<<_error.transpose()<<std::endl;
  }

};
class  EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ(){}


  virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
  virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

  inline void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }


  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void  linearizeOplus() {

      VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    Eigen::Matrix<double,2,3> tmp;
    tmp(0,0) = fx;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*fx;

    tmp(1,0) = 0;
    tmp(1,1) = fy;
    tmp(1,2) = -y/z*fy;

    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
    _jacobianOplusXj(0,2) = y/z *fx;
    _jacobianOplusXj(0,3) = -1./z *fx;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2 *fx;

    _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
    _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
    _jacobianOplusXj(1,2) = -x/z *fy;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *fy;
    _jacobianOplusXj(1,5) = y/z_2 *fy;


  }



  inline Eigen::Vector2d  cam_project(const Eigen::Vector3d & trans_xyz) const{
    return Eigen::Vector2d  ( (trans_xyz(0)/trans_xyz(2)) *fx + cx, (trans_xyz(1)/trans_xyz(2))*fy + cy);
  }
  double fx, fy, cx, cy;

};



class  EdgeStereoSE3ProjectXYZ: public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZ(){}


  virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
  virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}


  inline void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
  }

 inline  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


 virtual void linearizeOplus(){
     VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
     g2o::SE3Quat T(vj->estimate());
     VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
     Eigen::Vector3d xyz = vi->estimate();
     Eigen::Vector3d xyz_trans = T.map(xyz);

     const Eigen::Matrix3d R =  T.rotation().toRotationMatrix();

     double x = xyz_trans[0];
     double y = xyz_trans[1];
     double z = xyz_trans[2];
     double z_2 = z*z;

     _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
     _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
     _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

     _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
     _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
     _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

     _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
     _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
     _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

     _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
     _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
     _jacobianOplusXj(0,2) = y/z *fx;
     _jacobianOplusXj(0,3) = -1./z *fx;
     _jacobianOplusXj(0,4) = 0;
     _jacobianOplusXj(0,5) = x/z_2 *fx;

     _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
     _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
     _jacobianOplusXj(1,2) = -x/z *fy;
     _jacobianOplusXj(1,3) = 0;
     _jacobianOplusXj(1,4) = -1./z *fy;
     _jacobianOplusXj(1,5) = y/z_2 *fy;

     _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
     _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
     _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
     _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
     _jacobianOplusXj(2,4) = 0;
     _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;}

 inline Eigen::Vector3d cam_project(const Eigen::Vector3d & trans_xyz, const float &bf) const{
     const float invz = 1.0f/trans_xyz[2];
     Eigen::Vector3d res;
     res[0] = trans_xyz[0]*invz*fx + cx;
     res[1] = trans_xyz[1]*invz*fy + cy;
     res[2] = res[0] - bf*invz;
     return res;
 }

  double fx, fy, cx, cy, bf;
};


typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>                               Vector8D;

class  MarkerEdgeOnlyProject: public  g2o::BaseBinaryEdge<8, Vector8D, VertexSE3Expmap, VertexSE3Expmap>
{
    g2o::Vector3 points[4];

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double fx, fy, cx, cy;

  MarkerEdgeOnlyProject(double size){
     _delta_der=1e-4;//set the delta increment to compute the partial derivative for Jacobians
      auto pointsA=ucoslam::Marker::get3DPointsLocalRefSystem(size);
      for(int i=0;i<4;i++){
          auto &p=pointsA[i];
          points[i]=g2o::Vector3(p.x,p.y,p.z);
      }
  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}
inline Vector8D getError(){
 computeError();
 return _error;
}

  inline void computeError()  {
      //marker
      const VertexSE3Expmap* g2m= static_cast<const VertexSE3Expmap*>(_vertices[0]);//marker
    //camera
      const VertexSE3Expmap*  c2g = static_cast<const VertexSE3Expmap*>(_vertices[1]);//camera

   // std::cout<<"g2m="<<g2m->estimate()<<" c2g="<<c2g->estimate()<<std::endl;
    //first, move the points to the global reference system, and then to the camera
    auto Transform_C2M=c2g->estimate()*  g2m->estimate();
    g2o::Vector3 points2[4];
     for(int i=0;i<4;i++){
        points2[i]=Transform_C2M.map(points[i]);//3d rigid transform
     }

    //now, project
     _error.resize(8);
    Vector8D obs(_measurement);
  //  std::cout<<obs<<std::endl;
    assert(_error.size()==obs.size());
    int idx=0;
    for(int i=0;i<4;i++){
        float projx=( points2[i](0)/points2[i](2)) *fx +cx;
        _error(idx)=obs(idx)-projx;
         idx++;
        float projy=( points2[i](1)/points2[i](2)) *fy +cy;
        _error(idx)=obs(idx)-projy;
         idx++;
    }
 //   std::cout<<"E="<<_error.transpose()<<std::endl;
  }

};


typedef Eigen::Matrix<double,4,1,Eigen::ColMajor>                               Vector4D;


class  MarkerForceInPlace: public  g2o::BaseUnaryEdge<4, Vector4D, VertexSE3Expmap>
{


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  MarkerForceInPlace(void){

  }

  bool read(std::istream& is){assert(false);return false;}

  bool write(std::ostream& os) const{assert(false);return false;}
  inline Vector4D getError(){
    computeError();
    return _error;
}

  inline void computeError()  {
      //marker
      auto toCvMat=[](const g2o::SE3Quat &SE3)
      {
          Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
          cv::Mat cvMat(4,4,CV_32F);
          for(int i=0;i<4;i++)
              for(int j=0; j<4; j++)
                  cvMat.at<float>(i,j)=eigMat(i,j);

          return cvMat.clone();

      };
      const VertexSE3Expmap* g2m= static_cast<const VertexSE3Expmap*>(_vertices[0]);//marker
      cv::Mat m44=toCvMat(g2m->estimate());
      _error(0)=m44.at<float>(0,2);
      _error(1)=m44.at<float>(1,2);
      _error(2)=1-m44.at<float>(2,2);
      _error(3)=m44.at<float>(2,3);
   }

};


class  EdgeStereoSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZOnlyPose(){}


  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus(){
      VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
      Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

      double x = xyz_trans[0];
      double y = xyz_trans[1];
      double invz = 1.0/xyz_trans[2];
      double invz_2 = invz*invz;

      _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
      _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
      _jacobianOplusXi(0,2) = y*invz *fx;
      _jacobianOplusXi(0,3) = -invz *fx;
      _jacobianOplusXi(0,4) = 0;
      _jacobianOplusXi(0,5) = x*invz_2 *fx;

      _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
      _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
      _jacobianOplusXi(1,2) = -x*invz *fy;
      _jacobianOplusXi(1,3) = 0;
      _jacobianOplusXi(1,4) = -invz *fy;
      _jacobianOplusXi(1,5) = y*invz_2 *fy;

      _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
      _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
      _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
      _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
      _jacobianOplusXi(2,4) = 0;
      _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
    }


  Eigen::Vector3d cam_project(const Eigen::Vector3d & trans_xyz) const{
      const float invz = 1.0f/trans_xyz[2];
      Eigen::Vector3d res;
      res[0] = trans_xyz[0]*invz*fx + cx;
      res[1] = trans_xyz[1]*invz*fy + cy;
      res[2] = res[0] - bf*invz;
      return res;

  }

  Eigen::Vector3d Xw;
  double fx, fy, cx, cy, bf;

  virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
  virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

};

class  EdgeSE3ProjectXYZOnlyPose: public  g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose(){}
    EdgeSE3ProjectXYZOnlyPose(float x,float y,float z,float Fx,float Fy,float Cx,float Cy){
        fx = Fx;
        fy = Fy;
        cx = Cx;
        cy = Cy;
        Xw[0] = x;
        Xw[1] = y;
        Xw[2] = z;
    }

    bool read(std::istream& is){return false;}

    bool write(std::ostream& os) const{return false;}

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(Xw));
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        return (v1->estimate().map(Xw))(2)>0.0;
    }


    virtual void linearizeOplus(){
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
        _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
        _jacobianOplusXi(0,2) = y*invz *fx;
        _jacobianOplusXi(0,3) = -invz *fx;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = x*invz_2 *fx;

        _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
        _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
        _jacobianOplusXi(1,2) = -x*invz *fy;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -invz *fy;
        _jacobianOplusXi(1,5) = y*invz_2 *fy;
    }
    Eigen::Vector2d project2d(const  Eigen::Vector3d& v)const  {
        Eigen::Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }
    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const{
        Eigen::Vector2d proj = project2d(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Eigen::Vector3d Xw;
    double fx, fy, cx, cy;
};





/**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 *
 * Will represent relative transformation between two cameras
*/
class   VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
  virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

  virtual void setToOriginImpl() {
    _estimate = g2o::Sim3();
  }

  virtual void oplusImpl(const number_t* update_)
  {
    Eigen::Map<g2o::Vector7> update(const_cast<number_t*>(update_));

    if (_fix_scale)
      update[6] = 0;

    g2o::Sim3 s(update);
    setEstimate(s*estimate());
  }

  g2o::Vector2 _principle_point1, _principle_point2;
  g2o::Vector2 _focal_length1, _focal_length2;

  g2o::Vector2 cam_map1(const g2o::Vector2 &v) const {
    g2o::Vector2 res;
    res[0] = v[0] * _focal_length1[0] + _principle_point1[0];
    res[1] = v[1] * _focal_length1[1] + _principle_point1[1];
    return res;
  }

  g2o::Vector2 cam_map2(const g2o::Vector2 &v) const {
    g2o::Vector2 res;
    res[0] = v[0] * _focal_length2[0] + _principle_point2[0];
    res[1] = v[1] * _focal_length2[1] + _principle_point2[1];
    return res;
  }

  bool _fix_scale;


 protected:
};


/**
* \brief 7D edge between two Vertex7
*/
class   EdgeSim3 : public g2o::BaseBinaryEdge<7, g2o::Sim3,  VertexSim3Expmap,  VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3(){}  virtual bool read(std::istream& is){throw std::runtime_error("Not implemented");}
    virtual bool write(std::ostream& os) const{throw std::runtime_error("Not implemented");}

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

        g2o::Sim3 C(_measurement);
        g2o::Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
        _error = error_.log();
    }

    virtual number_t initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
    {
        VertexSim3Expmap* v1 = static_cast<VertexSim3Expmap*>(_vertices[0]);
        VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
        if (from.count(v1) > 0)
            v2->setEstimate(measurement()*v1->estimate());
        else
            v1->setEstimate(measurement().inverse()*v2->estimate());
    }
};
}

#endif
