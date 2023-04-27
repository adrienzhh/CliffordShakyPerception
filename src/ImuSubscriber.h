#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <ros/package.h>
#include <ros/ros.h>

using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::L;

class ImuSubscriber {
public:
  

// private:
  ros::Subscriber subImu;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  std::deque<double> imu_times;
  std::deque<gtsam::Vector3> imu_linaccs;
  std::deque<gtsam::Vector3> imu_angvel;

  int frame;
  bool accel_init_done;
  bool estimatorInit;


  //Initilize dummy IMU Variables. Need to be read in by .yaml data
  double initX = 0.0, initY = 0.0, initZ = 0.0;
  double initYaw = 0.0, initPitch = 0.0, initRoll = 0.0;
  double gravMag = 9.8;

  // //State of Robot
  gtsam::Pose3 priorPose; 
  gtsam::Vector3 priorVelocity = {0.5, 0, 0};
  gtsam::imuBias::ConstantBias priorBias; 
  gtsam::NavState prev_state;
  gtsam::NavState prop_state;
  gtsam::imuBias::ConstantBias prev_bias;
  gtsam::Pose3 gtPose = gtsam::Pose3();

  //Initilize GTSAM Variables
  gtsam::IncrementalFixedLagSmoother smootherISAM2;
  gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;
  gtsam::Values currentEstimate;

  nav_msgs::Path pathGT;
  nav_msgs::Path pathOPTI;
  ros::Publisher pathOPTI_pub;
  ros::Publisher pathGT_pub;
  ros::Publisher pose_pub;


  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> IMUparams;
  std::shared_ptr<gtsam::PreintegrationType> preintegrated;


  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();

  ImuSubscriber(ros::NodeHandle& nh);
  CombinedImuFactor create_imu_factor(double updatetime);
  void initializeFactorGraph();
  void do_accel_init();
  void do_nominal_init();
  void imuCallback(const sensor_msgs::Imu &msg);
  void sendTfs(double timestep);
  



};