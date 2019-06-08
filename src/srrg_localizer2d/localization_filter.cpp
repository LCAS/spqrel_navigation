#include "localization_filter.h"
#include "srrg_path_map/distance_map_path_search.h"
#include <cmath>
#include <iostream>

namespace srrg_localizer2d {
  using namespace srrg_core;
  using namespace std;


  Particle::Particle(){
    _pose.setZero();
    _weight = 1.0f;
  }


  LocalizationFilter::LocalizationFilter(){
    _min_update_translation=0.1;
    _min_update_rotation=0.02;

    // transition_model
    _noise_coeffs << 
      0.01, 0.0005, 0.0002,
      0.0005, 0.0001, 0.0001,
      0.001, 0.00001, 0.05;

    // observation model
    _min_valid_points = 30;
    _min_weight=0.1;
    _particle_resetting = true;
    _likelihood_gain = 10;
    _force_update = false; 
  }


  void LocalizationFilter::setMap(const UnsignedCharImage& m, 
             float resolution,
             unsigned char occ_threshold,
             unsigned char free_threshold) {
    _resolution=resolution;
    _inverse_resolution = 1./resolution;
    _map=m.clone();

    // creates an integer matrix where the cells are either
    // -1 (occupied)
    // -2 (unknown)
    // k>0 an unique integer that represent an obstacle in the map.
    _int_map.create(m.rows, m.cols);
    int k = 0;
    int free_count = 0, occ_count = 0, unknown_count = 0;
  
    for (int r = 0; r<m.rows; r++){
      unsigned char* src_ptr = _map.ptr<unsigned char>(r);
      int* dest_ptr = _int_map.ptr<int>(r);
      for (int c = 0; c<_map.cols; c++) {
        unsigned char cell=*src_ptr;
        int v=-1;
        if (cell<occ_threshold) {
          occ_count++;
          v=k++;
          *src_ptr=0;
        } else if (cell>free_threshold) {
          free_count++;
          v=-1;
          *src_ptr=255;
        } else {
          unknown_count++;
          v=-2;
          *src_ptr=127;
        }
        *dest_ptr=v;
        dest_ptr++;
        src_ptr++;
      }
    }
    cerr << "free: " << free_count << endl;
    cerr << "unknown: " << unknown_count << endl;
    cerr << "occupied: " << occ_count << endl;
    init();
  }



  void LocalizationFilter::init(int num_particles,
               float dmax, 
               float robot_radius,
               float min_weight,
               int min_valid_points){
    _particles.resize(num_particles);
    _robot_radius = robot_radius;
    _min_valid_points = min_valid_points;

    // computes the distance map
    // _int_map is the integer map set through setMap
    // _distances are floats representing the distances (in meters)
    // between a cell and the closest obstacles
    // The distances are computed also for the unknown cells
    // but stored as  their opposite.
    // This allows doscriminating between free and unknown cells
    // when computing the likelihood
    DistanceMapPathSearch dmap_calculator;
    dmap_calculator.setMaxDistance(dmax*_inverse_resolution);
    dmap_calculator.setOutputPathMap(_distance_map);
    dmap_calculator.setIndicesImage(_int_map);
    dmap_calculator.init();
    dmap_calculator.compute();
    _assoc_map = dmap_calculator.indicesMap();
    _distances = dmap_calculator.distanceImage();
    _distances*=_resolution;

    _free_cells.resize(_map.rows*_map.cols);
    int k=0;
    for (int r = 0; r<_map.rows; ++r){
      const unsigned char *map_cell = _map.ptr<const unsigned char>(r);
      const float* distance=_distances.ptr<const float>(r);
      for (int c = 0; c<_map.cols; ++c, ++map_cell, ++distance) {
        //cerr << *distance <<  " ";
        if (*distance>_robot_radius) {
          _free_cells[k]=(Eigen::Vector2f(_resolution*r, _resolution*c));
          k++;
        }
      }
    }
    _free_cells.resize(k);
  }


  Eigen::Vector3f LocalizationFilter::sampleFromFreeSpace() {
    int r=drand48()*(_free_cells.size()-1);
    float theta = (drand48()-0.5)*2*M_PI;
    return Eigen::Vector3f(_free_cells[r].x(), _free_cells[r].y(), theta);
  }

  void LocalizationFilter::startGlobal() {
    for (size_t i = 0; i<_particles.size(); i++){
      _particles[i]._pose = sampleFromFreeSpace();
      _particles[i]._weight = 1;
    }
  }

  void LocalizationFilter::setPose(const Eigen::Vector3f pose, Eigen::Vector3f standard_deviations) {
    Eigen::Isometry2f pose_transform = v2t(pose);
    for (size_t i = 0; i<_particles.size(); i++){
      Eigen::Vector3f noise;
      for (int k = 0; k<3; k++)
        noise[k] = standard_deviations[k]*_normal_generator(_random_generator);
      _particles[i]._pose = t2v(pose_transform*v2t(noise));
      _particles[i]._weight = 1;
    }
    _force_update = true;
  }

  void LocalizationFilter::predict(const Eigen::Vector3f control) {
    if (control.squaredNorm()==0)
      return;
    prepareSampling(control);
    for (size_t i = 0; i<_particles.size(); i++)
      _particles[i]._pose = sample(_particles[i]._pose);
    _cumulative_translation+=control.head<2>().norm();
    _cumulative_rotation+=fabs(control[2]);
  }


  //! uniform resampling algorithm
  //! indices: the indices of the particles that survive after resampling
  //! weights: the weights of the particles as input
  //! n: the number of particles
  void resample_uniform(int* indices, const double* weights, int n){
    double acc=0;
    const double* w = weights;
    for (int i=0; i<n; i++, w++) {
      acc+= *w;
    }
    double inverse_acc = 1./acc;
    double cumulative_value=0;
    double step = 1./n;
    double threshold = step * drand48();
    int* idx = indices;
    
    w=weights;
    int k=0;
    for (int i=0; i<n; i++, w++){
      cumulative_value += (*w) *inverse_acc;
      while(cumulative_value>threshold){
        *idx = i;
        idx++;
        k++;
        threshold += step;
      }
    }
  }

  bool LocalizationFilter::update(const Vector2fVector& observation){
    // refresh the last endpoints
    _last_endpoints = observation; 
 
    // if the platform did not move enough, do nothing
    if (!_force_update && (_cumulative_rotation<_min_update_rotation &&
    _cumulative_translation < _min_update_translation))
      return 0;

    _force_update = false;

    //update
    _cumulative_translation = 0;
    _cumulative_rotation = 0;
    _cumulative_likelihood = 0;
    for (size_t i = 0; i<_particles.size(); i++) {
      // compute the weight of each particle
      float w = likelihood(_particles[i]._pose, observation);
      // if the weight is 0 and replace the particle with a random one,
      // otherwise assign a weight to a particle, based on the likelihood
      if (w==0 && _particle_resetting) {
        w=_min_weight;
        _particles[i]._pose = sampleFromFreeSpace();
      } 
      _particles[i]._weight = w;
      _cumulative_likelihood += w;;
    }
    
    if (_cumulative_likelihood < 0)
      return false;

    
    // resample
    int indices[_particles.size()];
    double weights[_particles.size()];
    for (size_t i = 0 ; i<_particles.size(); i++)
      weights[i]=_particles[i]._weight;
    resample_uniform(indices, weights, _particles.size());
    if (_cumulative_likelihood==0) {
       return true;
    }
    ParticleVector aux(_particles.size());
    int* idx = indices;
    for (size_t i=0; i<_particles.size(); i++){
      aux[i]=_particles[*idx];
      aux[i]._weight=1;
      idx++;
    }
    _particles=aux;
    return true;
  }

  void LocalizationFilter::paintState(RGBImage& img, bool use_distance_map){
    if (! use_distance_map) {
      cvtColor(_map, img, CV_GRAY2BGR);
    } else {
      UnsignedCharImage dest;
      _distance_map.toImage(dest);
      cvtColor(dest, img , CV_GRAY2BGR);
    }

    float ires=1./_resolution;
    int count=0;
    for (size_t i=0; i<_particles.size(); i++){
      int r = _particles[i]._pose.x()*ires;
      int c = _particles[i]._pose.y()*ires;
      if (! _distance_map.inside(r,c))
        continue;
      count++;
      img.at<cv::Vec3b>(r,c)=cv::Vec3b(0,0,255);
    }

    computeStats();
    Eigen::Isometry2f mean_transform=v2t(_mean);

    for (size_t i=0; i<_last_endpoints.size(); i++){
      Eigen::Vector2f ep=mean_transform*_last_endpoints[i];
      int r = ep.x()*ires;
      int c = ep.y()*ires;
      if (! _distance_map.inside(r,c))
        continue;
      cv::Scalar color(255,0,0);
      if (_endpoint_distances[i]<0)
        color=cv::Scalar(0,255,255);
      else if (_endpoint_distances[i]<0.3)
        color=cv::Scalar(0,255,0);
      cv::circle(img, cv::Point(c,r), 2, color);
    }
  }

  void LocalizationFilter::prepareSampling(const Eigen::Vector3f& control) {
    Eigen::Vector3f scales(fabs(control[0]),fabs(control[1]),fabs(control[2])); 
    _std_deviations=_noise_coeffs*scales;
    for (size_t i = 0; i<3; i++){
      _std_deviations[i]=sqrt(_std_deviations[i]);
    }
    _last_control=control;
  }

  Eigen::Vector3f LocalizationFilter::sample(const Eigen::Vector3f& old_state) {
    Eigen::Vector3f noise;
    for (int i = 0; i<3; i++)
      noise[i] = _std_deviations[i]*_normal_generator(_random_generator);
    
    noise += _last_control;
    Eigen::Isometry2f transform=v2t(old_state) * v2t(noise);
    return t2v(transform);
  }


  void LocalizationFilter::computeStats(){
    Eigen::Vector2f translational_mean;;
    Eigen::Vector2f angular_mean;
    translational_mean.setZero();
    angular_mean.setZero();
    // computes the mean. To calculate the angular component sum
    // the vectors (cos(theta), sin(theta) and recover the global
    // orientation
    for (size_t i = 0; i<_particles.size(); i++) {
      translational_mean+=_particles[i]._pose.head<2>();
      float theta=_particles[i]._pose[2];
      angular_mean+=Eigen::Vector2f(cos(theta), sin(theta));
    }
    _mean.head<2>()=translational_mean * (1./_particles.size());
    _mean[2]=atan2(angular_mean.y(),angular_mean.x());

    _covariance.setZero();
    for (size_t i = 0; i<_particles.size(); i++) {
      Eigen::Vector3f dp = _particles[i]._pose-_mean;
      dp[2]=fmod(dp[2], 2*M_PI);
      if (dp[2]>M_PI)
        dp[2]-=M_PI;
      _covariance+=dp*dp.transpose();
    }
    _covariance*=1./_particles.size();

    _endpoint_distances.resize(_last_endpoints.size());
    double l = likelihood(_mean, _last_endpoints, &_endpoint_distances[0]);
    if (l<_min_weight) {
      std::fill(_endpoint_distances.begin(), _endpoint_distances.end(), -1);
    }
  }

  double LocalizationFilter::likelihood(const Eigen::Vector3f& pose, const Vector2fVector& observation, float* endpoint_distances){
    Eigen::Isometry2f iso=v2t(pose);
  
    // handle the robot out of the map
    Eigen::Vector2i p = world2grid(iso.translation());
    if(p.x()<0 || p.x()>=_distances.rows || p.y() <0 || p.y()>= _distances.cols)
      return 0;

    
    // handle the robot in the unknown
    float d=_distances.at<float>(p.x(),p.y());
    if (d<_robot_radius)
      return 0;
  
    // handle the beams
    float cumulative_distance = 0;
    int valid_points=0;
    float local_endpoint_distances[observation.size()];
    float* dist_ptr=local_endpoint_distances;
    for (size_t i =0; i<observation.size(); i++, dist_ptr++) {
      p = world2grid(iso*observation[i]);
      *dist_ptr=-1;
      if (! _distance_map.inside(p.x(), p.y()))
        continue;
      float distance = fabs(_distances.at<float>(p.x(),p.y()));
      if (distance!=distance)
        throw std::runtime_error("nan detected");
      *dist_ptr=distance;
      cumulative_distance+=fabs(_distances.at<float>(p.x(),p.y()));
      valid_points++;
    }

    if (endpoint_distances) {
      memcpy(endpoint_distances, local_endpoint_distances, observation.size()*sizeof(float));
    }
    // if too less beams are good, ignore
    if (valid_points< _min_valid_points)
      return _min_weight;

    // heuristic but effective likelihood
    cumulative_distance/=valid_points;
    cumulative_distance*=_likelihood_gain;
    return exp(-cumulative_distance)+_min_weight;
  }

}

