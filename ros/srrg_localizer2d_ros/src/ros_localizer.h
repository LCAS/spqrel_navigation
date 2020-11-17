#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/time.h>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
//#include "srrg_types/defs.h"
#include "spqrel_navigation/LocalizerRanges.h"
#include "localization_filter.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace srrg_localizer2d_ros{

  using namespace std;

  /*!
     class that implements a ROS localization node.
     It uses a particle filter with likelihood function based on a distance map
   */
  class ROSLocalizer : public srrg_localizer2d::LocalizationFilter{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //! initializes the localizer on the node handle provided
    //! if the listener is not given, it will create a transform
    //! listener on its own
    ROSLocalizer(ros::NodeHandle& nh, tf::TransformListener* listener=0);

    //! call this after construction if you want to have a gui
    void initGUI();

    //!setters fot common parameters
    inline const std::string& baseFrameId() const { return _base_frame_id;}
    inline void setBaseFrameId(const std::string fid) {_base_frame_id=fid;}
    inline const std::string& globalFrameId() const { return _global_frame_id;}
    inline void setGlobalFrameId(const std::string fid) {_global_frame_id=fid;}
    inline const std::string& odomFrameId() const { return _odom_frame_id;}
    inline void setOdomFrameId(const std::string fid) {_odom_frame_id=fid;}
    inline const std::string& laserTopic() const {return _laser_topic;}
    inline void setStaticMapService(const std::string static_map_service) {_static_map_service=static_map_service;}
    inline const std::string& staticMapService() const {return _static_map_service;}

    //! sets/gets the maximum range of a laser, ranges faerther than this are ignored
    inline float forcedMaxRange() const {return _forced_max_range;}
    inline void setForcedMaxRange( float mr) {_forced_max_range=mr;}

    //! endpoints that are closer between each other than this threshold are suppressed
    inline float squaredEndpointDistance() const {return _squared_endpoint_distance;}
    inline void setSquaredEndpointDistance(float sd) { _squared_endpoint_distance=sd;}

    //! call this when all is in place, AND the map is loaded (either via direct setMap function or
    //! by calling the requestMap service)
    void subscribeCallbacks(const std::string& laser_topic="/base_scan", const std::string& odom_topic="/odom", const std::string& gps_topic="/gps");

    //! requests a map via rpc
    void requestMap();

    //! Set initial pose
    void setInitialPose(float x, float y,float theta);

    //! Set flag to check for tf timing
    inline void setTFTimeCheck(bool tf_timecheck) {_tf_timecheck = tf_timecheck;}
    //! Set flag to publish tf between globalFrameId -> baseFrameId
    inline void setPublishTF(bool publish_tf) {_publish_tf = publish_tf;}

    inline void setInvertedLaser(const bool inverted_laser) {_inverted_laser=inverted_laser;}
    inline void setUseOdomTopic(const bool use_odom_topic) {_use_odom_topic=use_odom_topic;}
    inline void setUseOdomCov(const bool use_odom_cov) {_use_odom_cov=use_odom_cov;}
    inline void setCovScaleFactor(const float cov_scale_factor) {_cov_scale_factor=cov_scale_factor;}
    inline void setUseGPSTopic(const bool use_gps_topic) {_use_gps_topic=use_gps_topic;}

  protected:

    tf::TransformListener* _listener; //< listener object uset do compute the relative laser transform and to listen to odom
    tf::TransformBroadcaster* _broadcaster; //< used to send the transform map->odom
    std::string _laser_topic;   //< laser topic to be used for localization
    std::string _odom_topic;    //< odom topic to be used for motion estimation
    std::string _odom_frame_id; //< odom frame id, used to compute the control of the filter
    std::string _base_frame_id; //< frame of the robot
    std::string _global_frame_id;  //< frame of the map
    std::string _static_map_service;  //< name of the static_map service
    std::string _gps_topic;    //< gps topic to be used for estimation update
    
    Eigen::Vector3f _old_odom_pose; //< stores the previous odom pose, used to compute the control
    Eigen::Matrix3f _old_odom_cov; //< stores the previous odom pose, used to compute the control
    float _cov_scale_factor;
    Eigen::Vector3f _laser_pose; //< stores the laser pose from the system
    bool _has_laser_pose;
    bool _restarted; //<
    bool _inverted_laser;
    bool _use_odom_topic;
    bool _use_odom_cov; //< use odometry covariance for particle propagation (only used if use_odom_topic is true)
    bool _use_gps_topic;
    bool _new_gps_pose;
    
    std::vector<double> _timers; //< holds the update time of the last n cycles
    size_t _last_timer_slot; // the current update cycle

    float _forced_max_range; //< max range to use in localization
    float _squared_endpoint_distance; //< max distance betweem endpoints in the laser. The ones that are closer are suppressed
    ros::Subscriber _laser_sub; //< subscriber for laser messages
    message_filters::Subscriber<sensor_msgs::LaserScan> _laser_sync_sub;
    message_filters::Subscriber<nav_msgs::Odometry> _odom_sync_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,nav_msgs::Odometry> LaserOdomSyncPolicy;
    typedef message_filters::Synchronizer<LaserOdomSyncPolicy> Sync;
    boost::shared_ptr<Sync> _sync;
    void syncLaserOdomCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg, const nav_msgs::Odometry::ConstPtr &odom_msg);
    ros::Subscriber _gps_sub;
    void gpsCallback(const nav_msgs::Odometry::ConstPtr &gps_msg);
    Eigen::Vector3f gps_pose_;


    ros::Publisher _pose_pub;   //< publisher for the amcl_pose
    ros::Publisher _particlecloud_pub; //< publisher for the particles
    ros::Publisher _ranges_pub; //< localizer_ranges_publisher
    ros::Subscriber _initial_pose_sub; //< subscriber for the set_pose
    ros::ServiceServer _global_loc_srv; //< service for global localization
    ros::Time _last_observation_time;   //< stores the last time an observation has been processed,
                                        //<to keep the timestamps of the transforms consistent

    ros::NodeHandle& _nh;             //< global node handle


    //! converts a range scan into a vector of regularized endpoints
    //! @param endpoints: a vector2fvector containing the cartesian
    //!                   written by the method coordinares of the regularized endpoints
    //! @paeam laser_pose: pose of the laser on the robot
    //! @param msg: the laser message to be processed
    void rangesToEndpoints(srrg_core::Vector2fVector& endpoints,
			   const Eigen::Vector3f& laser_pose,
			   const sensor_msgs::LaserScan::ConstPtr& msg);

    //! publishes the particles
    void publishParticles();

    //! publishes the statistics (amcl_pose) and the transform (tf)
    void publishPose();

    //! publishes the distance between each laser endpoint and the closest point in the map
    void publishRanges();

    //! handles a laser scan and updates the filter
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    //! handles the set-pose-estimate message from outside
    //! puts all partcles in the neighborhood of the pose in msg
    void setPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    //! triggers global localization
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);

    // called when the map is updated. causes some overhead
    void mapMessageCallback(const nav_msgs::OccupancyGrid& msg);

    // GUI stuff
    //! callback fot the mouse, used to set the robot pose
    //! when in set_pose mode, left click sets the pose
    //! right click sets the orientation
    static void onMouse( int event, int x, int y, int, void* );

    //! handles the keyboard
    //!  "s": toggles setPose mode
    //!  "d": toggles map mode (distance/occupancy)
    //!  "r": toggles particle resetting (spread the particles when enterning in unknown)
    //!  "g": triggers global localization
    void handleGUIInput();

    //! computes the average computation time of the last n updates
    double cycleLatency() const;

    //! shows the gui stuff
    void handleGUIDisplay();
    bool _use_gui;           //! handled internally
    bool _show_distance_map; //! if 1 in the gui shows the distance map. (to be toggled with "d")
    bool _force_redisplay;   //! if toggled to one forces the display and sends out all messages
    bool _set_pose;          //! set_pose mode (to be toggled with "s")

    bool _tf_timecheck;      //! if tf times are checked
    bool _publish_tf;

    int _cnt_not_updated;    //! how monay cycles without updates
  };


}
