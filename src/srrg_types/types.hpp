#pragma once

#include <iostream>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>

/**
 * @file   types.h
 * @Author Giorgio Grisetti
 * @date   December 2014
 * @brief  This file contains some useful defines about 
 * Eigen types, common Mat opencv specializations, and transforms mappings
 */

namespace srrg_core {

  //!a vector of Vector2f with alignment
  typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;

  //!a vector of Vector3f with alignment
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;

  //!a vector of Vector2f with alignment
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

  //!a vector of Matrix3f with alignment
  typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;

  //!a vector of Matrix2f with alignment
  typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;

  //!a 4x3 float matrix
  typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;

  //!a 4x6 float matrix
  typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;

  //!a 2x6 float matrix
  typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;

  //!a 3x6 float matrix
  typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;


  //!a 5x5 float matrix
  typedef Eigen::Matrix<float, 5, 5> Matrix5f;

  //!a 6x6 float matrix
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;

  //!a 6   float vector
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  
  //!a 9x6 float matrix 
  typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;

  //!a 9x9 float matrix 
  typedef Eigen::Matrix<float, 9, 9> Matrix9f;

  //!a 9 float vector
  typedef Eigen::Matrix<float, 9, 1> Vector9f;

  //!a 6x3 float matrix
  typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;

  //!a 5 float cv vector
  typedef cv::Vec<float, 5> Vec5f;

  //!a 7 float cv vector
  typedef cv::Vec<float, 7> Vec7f;
  
  //!check if an Eigen type contains a nan element
  //!@returns true if at least one element of
  //!the argument is null
  template <class T> 
  bool isNan(const T& m){
    for (int i=0; i< m.rows(); i++) {
      for (int j=0; j< m.cols(); j++) {
	float v = m(i,j);
	if ( std::isnan( v ) )
	  return true;
      }
    }
    return false;
  }

  //!converts from 6 vector to isometry
  //!@param t: a vector (tx, ty, tz, qx, qy, qz) reptesenting the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized queternion, with qw>0.
  //!@returns the isometry corresponding to the transform described by t
  inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      Eigen::Vector3f q=t.block<3,1>(3,0);
      q.normalize();
      T.linear()=Eigen::Quaternionf(0, q(0), q(1), q(2)).toRotationMatrix();
    }
    return T;
  }
  inline Eigen::Isometry3d v2t(const Eigen::Matrix<double, 6, 1>& t){
    Eigen::Isometry3d T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaterniond(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      Eigen::Vector3d q=t.block<3,1>(3,0);
      q.normalize();
      T.linear()=Eigen::Quaterniond(0, q(0), q(1), q(2)).toRotationMatrix();
    }
    return T;
  }

  //!converts from isometry to 6 vector                                                                   
  //!@param t: an isometry
  //!@returns a vector (tx, ty, tz, qx, qy, qz) reptesenting the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized queternion, with qw>0.
  inline Vector6f t2v(const Eigen::Isometry3f& t){
    Vector6f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v(3) = q.x();
    v(4) = q.y();
    v(5) = q.z();
    if (q.w()<0)
      v.block<3,1>(3,0) *= -1.0f;
    return v;
  }

  //!converts from isometry2f to isometry3f                                                                   
  //!@param t: an isometry2f
  //!@returns an isometry3f
  inline Eigen::Isometry3f toIsometry3f(const Eigen::Isometry2f& isometry2f){
    Eigen::Isometry3f isometry3f;
    isometry3f.linear().block<2,2>(0,0) = isometry2f.linear();
    isometry3f.translation().head<2>() = isometry2f.translation();
    return isometry3f;
  }

  //!computes the cross product matrix of the vector argument
  //!@param p: the vector
  //!@returns a 3x3 matrix 
  inline Eigen::Matrix3f skew(const Eigen::Vector3f& p){
    Eigen::Matrix3f s;
    s << 
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(), 
      -p.y(), p.x(), 0;
    return s;
  }
  inline Eigen::Matrix3d skew(const Eigen::Vector3d& p){
    Eigen::Matrix3d s;
    s <<
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(),
      -p.y(), p.x(), 0;
    return s;
  }

  inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
    Eigen::Isometry2f T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
  }

  inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
  }


  inline Eigen::Matrix3f Rx(float rot_x){
    float c=cos(rot_x);
    float s=sin(rot_x);
    Eigen::Matrix3f R;
    R << 1,  0, 0,
      0,  c,  -s,
      0,  s,  c;
    return R;
  }
  inline Eigen::Matrix3d Rx(const double& rot_x){
    const double c=cos(rot_x);
    const double s=sin(rot_x);
    Eigen::Matrix3d R;
    R << 1,  0, 0,
      0,  c,  -s,
      0,  s,  c;
    return R;
  }
  
  inline Eigen::Matrix3f Ry(float rot_y){
    float c=cos(rot_y);
    float s=sin(rot_y);
    Eigen::Matrix3f R;
    R << c,  0,  s,
      0 , 1,  0,
      -s,  0, c;
    return R;
  }
  inline Eigen::Matrix3d Ry(const double& rot_y){
    const double c=cos(rot_y);
    const double s=sin(rot_y);
    Eigen::Matrix3d R;
    R << c,  0,  s,
      0 , 1,  0,
      -s,  0, c;
    return R;
  }

  inline Eigen::Matrix3f Rz(float rot_z){
    float c=cos(rot_z);
    float s=sin(rot_z);
    Eigen::Matrix3f R;
    R << c,  -s,  0,
      s,  c,  0,
      0,  0,  1;
    return R;
  }
  inline Eigen::Matrix3d Rz(const double& rot_z){
    const double c=cos(rot_z);
    const double s=sin(rot_z);
    Eigen::Matrix3d R;
    R << c,  -s,  0,
      s,  c,  0,
      0,  0,  1;
    return R;
  }

  
  inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
    Eigen::Isometry3f T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
  }
  inline Eigen::Isometry3d v2tEuler(const Eigen::Matrix<double, 6, 1>& v){
    Eigen::Isometry3d T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
  }


  inline Eigen::Matrix2f skew(const Eigen::Vector2f& p){
    Eigen::Matrix2f s;
    s << 
      0,  -p.y(),
      p.x(), 0;
    return s;
  }

  /** \typedef UnsignedCharImage
   * \brief An unsigned char cv::Mat.
   */
  typedef cv::Mat_<unsigned char> UnsignedCharImage;
  
  /** \typedef CharImage
   * \brief A char cv::Mat.
   */
  typedef cv::Mat_<char> CharImage;

  /** \typedef UnsignedShortImage
   * \brief An unsigned short cv::Mat.
   */
  typedef cv::Mat_<unsigned short> UnsignedShortImage;
  
  /** \typedef UnsignedIntImage
   * \brief An unsigned int cv::Mat.
   */
  typedef cv::Mat_<unsigned int> UnsignedIntImage;
  
  /** \typedef IntImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<int> IntImage;

  /** \typedef Int4Image
   * \brief A 4D int cv::Mat.
   */
  typedef cv::Mat_<cv::Vec4i> Int4Image;

  /** \typedef IntervalImage
   * \brief A 4D int cv::Mat used to store intervals.
   */
  typedef Int4Image IntervalImage;

  /** \typedef FloatImage
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<float> FloatImage;

  /** \typedef Float3Image
   * \brief A 3D float cv::Mat.
   */
  typedef cv::Mat_<cv::Vec3f> Float3Image;

  /** \typedef Float5Image
   * \brief A 5D float cv::Mat.
   */
  typedef cv::Mat_<Vec5f> Float5Image;

  /** \typedef Float5Image
   * \brief A 7D float cv::Mat.
   */
  typedef cv::Mat_<Vec7f> Float7Image;
  
  /** \typedef DoubleImage
   * \brief A double cv::Mat.
   */
  typedef cv::Mat_<double> DoubleImage;
  
  /** \typedef RawDepthImage
   * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
   */
  typedef UnsignedShortImage RawDepthImage;
  
  /** \typedef IndexImage
   * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
   */
  typedef IntImage IndexImage;
  
  /** \typedef DepthImage
   * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
   */
  typedef cv::Mat_<cv::Vec3b> RGBImage;

  /** used to represent rgb values
   */
  typedef std::vector<cv::Vec3b> RGBVector;

  
  typedef std::vector<int> IntVector;
  
  typedef std::vector<float> FloatVector;

  typedef std::vector<std::pair<int, int> > IntPairVector;

  //ds overloaded opencv/eigen converters: double
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 3, 3>& matrix_eigen_) {
    cv::Mat_<double> matrix_opencv(3, 3);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 3; ++v) {
        matrix_opencv.at<double>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 3, 4>& matrix_eigen_) {
    cv::Mat_<double> matrix_opencv(3, 4);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 4; ++v) {
        matrix_opencv.at<double>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 5, 1>& vector_eigen_) {
    cv::Mat_<double> vector_opencv(5,1);
    for(uint32_t u = 0; u < 5; ++u) {
      vector_opencv.at<double>(u) = vector_eigen_(u);
    }
    return vector_opencv;
  }
  inline Eigen::Matrix<double, 3, 1> fromCv(const cv::Vec<double, 3>& vector_opencv_) {
    Eigen::Matrix<double, 3, 1> vector_eigen;
    for(uint32_t u = 0; u < 3; ++u) {
      vector_eigen(u) = vector_opencv_(u);
    }
    return vector_eigen;
  }

  //ds overloaded opencv/eigen converters: float
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 3, 3>& matrix_eigen_) {
    cv::Mat_<float> matrix_opencv(3, 3);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 3; ++v) {
        matrix_opencv.at<float>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 3, 4>& matrix_eigen_) {
    cv::Mat_<float> matrix_opencv(3, 4);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 4; ++v) {
        matrix_opencv.at<float>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 5, 1>& vector_eigen_) {
    cv::Mat_<float> vector_opencv(5,1);
    for(uint32_t u = 0; u < 5; ++u) {
      vector_opencv.at<float>(u) = vector_eigen_(u);
    }
    return vector_opencv;
  }
  inline Eigen::Matrix<float, 3, 1> fromCv(const cv::Vec<float, 3>& vector_opencv_) {
    Eigen::Matrix<float, 3, 1> vector_eigen;
    for(uint32_t u = 0; u < 3; ++u) {
      vector_eigen(u) = vector_opencv_(u);
    }
    return vector_eigen;
  }

  //ml std vector to Eigen converters
  inline Eigen::Matrix<float, 3, 1> fromFloatVector3f(const FloatVector& float_vector_){
    Eigen::Matrix<float, 3, 1> vector_eigen;
    assert(float_vector_.size() == 3);
    for (size_t i=0; i<3; i++)
      vector_eigen[i] = float_vector_[i];
    
    return vector_eigen;
  }
  inline FloatVector toFloatVector3f(const Eigen::Matrix<float, 3, 1>& vector_eigen_){
    FloatVector float_vector;
    float_vector.resize(3);
    for (size_t i=0; i<3; i++)
      float_vector[i] = vector_eigen_[i];
    return float_vector;
  }
}
