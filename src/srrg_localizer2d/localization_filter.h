#pragma once
//#include "srrg_types/defs.h"
#include "srrg_types/types.hpp"
#include "srrg_path_map/path_map.h"
#include <tr1/random>

namespace srrg_localizer2d {

  /*!
    simple 2D particle class for localization;
   */
  class Particle {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Particle();
    Eigen::Vector3f _pose;
    double _weight;
  };

  typedef std::vector<Particle, Eigen::aligned_allocator<Particle> > ParticleVector;

  /**
     Simple localization filter that implements a markov localization algorithm on a 2D gridmap.
     The map is represented through an 8 bit grayscale image, and a resolution meters/pixel.


     Typical usage 
     \code{.cpp}
     LocalizationFilter my_filter;
     // load the map as a grayscale image. White means free, black means occupied, something in between means unknown;

     UnsignedCharImage  my_map= cv::imread(...);

     my_filter.setMap(my_map, 0.05, 10, 200);
     \endcode
     The latter instruction loads in the localizer
     a map from the image my_map, with a resolution of 5 cm/pixel.
     The pixels whose value is lower than 10 are regarded as occupied, and the cells
     whose value is above 200 are free.

     Once you set up this maravellios thing, you can use it
     \code{.cpp}
     my_filter.init();  // this initializes the filter with some default parameters.

     while (have_data) {
     
        // if the data is an odometry (incremental motion since the last step), do a predict
        if (next_data.type==odometry) {
	    lmy_filter.predict(next_data.odometry);
	} else if (next_data.type==laser) {
	    my_filter.update(next_data.laser);
	} 

	my_filter.computeStats(); // update the internal statistics;
	
	cerr << "Robot is in " <<< my_filter.mean() << endl;
     }
     
     \endcode
     
  */

  class LocalizationFilter {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    LocalizationFilter();
  
    //! call this to load the map in the localizer
    //! @param m: an 8 bit grayscale image
    //! @param resolution: size in meters of a map pixel
    //! @param occ_threshold: values of a pixel values below this are considered occupies
    //! @param free_threshold: values of a pixel above this are free space
    void setMap(const srrg_core::UnsignedCharImage& m,
		float resolution,
		unsigned char occ_threshold,
		unsigned char free_threshold);
	      
    //! initializes the localizer (after setting a map)
    //! @param num_particles: the higher the more powerful the filter, you pay with computation
    //! @param dmax: max distance to compute the likelihood field. 
    //!              High values result in larger convergence basin (good for global localization), 
    //!              but poorer position tracking
    //! @param min_weight: the lower, the more faster the filter converges
    //! @param min_valid_points: minimum number of good endpoints to do the update
    void init(int num_particles=2000, 
	      float dmax=1.0f, 
	      float robot_radius=0.2,
	      float min_weight=0.01,
	      int min_valid_points=30);

    //! starts the global localization
    void startGlobal();

    //! sets all particles in a position
    void setPose(const Eigen::Vector3f pose, 
		 Eigen::Vector3f standard_deviations=Eigen::Vector3f(0.2, 0.2, 0.2));
  
    //! integrates a motion
    //! param@ control: the motion
    void predict(const Eigen::Vector3f control);

    //! integrates an observation
    //! param@ observation: the <x,y> endpoints of the valid laser beams (no maxrange)
    //! returns true if the update was not performed (the robot has not moved enough)
    bool update(const srrg_core::Vector2fVector& observation);


    //! returns the minimum translation of the robot between updates (meters)
    inline float minUpdateTranslation() const {return _min_update_translation;}

    //! sets the minimum translation of the robot between updates (meters)
    inline void setMinUpdateTranslation(float v) { _min_update_translation=v;}

    //! returns the minimum rotation of the robot between updates (radians)
    inline float minUpdateRotation() const {return _min_update_rotation;}

    //! sets the minimum rotation of the robot between updates (radians)
    inline void setMinUpdateRotation(float v) { _min_update_rotation=v;}


    //! paints the state in a rgb image map + particles(red) + endpoints at the mean
    void paintState(srrg_core::RGBImage& img, bool use_distance_map=false);

    //! computes the stats of the filter (mean and covariance matrix)
    void computeStats();
    
    //! returns the distance between each beam endpoint and the closest occupied cell in the map
    //! its result is valid only after calling computeStats()
    inline const std::vector<float>& endpointDistances() const {return _endpoint_distances;}

    //! returns the mean of the particles. 
    //! call computeStats of the filter after a predict or an update
    //! to make this method returning a valid value
    inline const  Eigen::Vector3f& mean() const {return _mean;}

    inline const  Eigen::Matrix3f& covariance() const {return _covariance;}
    //! returns the covariance of the particles. 
    //! call computeStats of the filter after a predict or an update
    //! to make this method returning a valid value


    //! Gets the noise coefficients for the translation model.
    //! the standard deviation of the noise to inject is computed
    //! by multiplying this matrix to the absolute values of the control input
    //! thus the noise coeffs represent the contribution of the mixing factors
    //! of the motion components
    //! If the robot does not move the control is 0 and the standard deviations are 0
    inline const Eigen::Matrix3f& noiseCoeffs() const {return _noise_coeffs;}

    //! sets the noise coefficients of the translation model
    inline void setNoiseCoeffs(const Eigen::Matrix3f& nc) { _noise_coeffs = nc;}

    //! read only accessor to the particles
    const ParticleVector& particles() const { return _particles; }

    //! returns the particle resetting status
    //! when on the particles ending up in the unknown are
    //! replaced by samples drawn from the free space.
    //! Good to be on during global localization, bad for tracking
    inline bool particleResetting() const { return _particle_resetting; }
    
    //! enables/disables particle resetting
    inline void setParticleResetting(bool pr)  { _particle_resetting = pr; }

    //! gets the gain for the beam likelihood
    //! the higher this value the more the distance between endpoint and obstacle
    //! will result in a low likelihood;
    inline float likelihoodGain() const {return _likelihood_gain; }
    
    //! sets the likelihood gain
    inline void setLikelihoodGain(float lg)  {_likelihood_gain = lg; }

    //! forces an update of the particles
    inline void forceUpdate()  { _force_update = true; }

    //! returns the cumulative likelihood, is a measure of how well the measurements
    //! fit the map
    inline float cumulativeLikelihood() const { return _cumulative_likelihood; }

  protected:
    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
      return Eigen::Vector2i(p.x()*_inverse_resolution, p.y()*_inverse_resolution);
    }

    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) {
      return Eigen::Vector2f(p.x()*_resolution, p.y()*_resolution);
    }

    //! samples a pose from the free cells, considering the radius of the robot
    Eigen::Vector3f sampleFromFreeSpace();

    // global params
    ParticleVector _particles;
    srrg_core::Vector2fVector _free_cells;
    Eigen::Matrix3f _covariance;
    Eigen::Vector3f _mean;
    float _min_update_translation, _cumulative_translation;
    float _min_update_rotation, _cumulative_rotation;
    bool _particle_resetting;
    double _cumulative_likelihood;
    // map and lookups
    srrg_core::UnsignedCharImage _map;
    float _resolution, _inverse_resolution;
    // lookups extracted from the map (stored for debug)
    srrg_core::IntImage _int_map;
    srrg_core::IntImage _assoc_map;
    srrg_core::PathMap _distance_map;
    srrg_core::FloatImage _distances;

    //last laser valid endpoints
    srrg_core::Vector2fVector _last_endpoints;

    //observation model configuration   
    int _min_valid_points;
    float _min_weight;

    //robot configuration
    float _robot_radius;


    // transition model

    //! prepares the sampling to generate noise for a control
    void prepareSampling(const Eigen::Vector3f& control);

    //! draws a sample from the transition model.
    //! call prepareSampling before to set the control
    //! @param old_state: x_{t-1}
    Eigen::Vector3f sample(const Eigen::Vector3f& old_state);

    Eigen::Matrix3f _noise_coeffs;
    Eigen::Vector3f _last_control;
    Eigen::Vector3f _std_deviations;
    std::tr1::ranlux64_base_01 _random_generator;
    std::tr1::normal_distribution<double> _normal_generator;
    
    // observation_model
    //! computes the likelihood of an observation, given a particle
    //! if endpoint_distances is !=0, it will contain the distance between each endpoint and the closest occupied cell
    double likelihood(const Eigen::Vector3f& pose, const srrg_core::Vector2fVector& observation, float*endpoint_distances=0);
    std::vector<float> _endpoint_distances;
    float _likelihood_gain; 

    // variable to force one step of update of the particles
    bool _force_update;
  };

}
