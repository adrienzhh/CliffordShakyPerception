/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor
 * navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in
 * conjunction with GPS
 *  - imuFactor is used by default. You can test combinedImuFactor by
 *  appending a `-c` flag at the end (see below for example command).
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  (body frame - Forward, Right, Down)
 *  linAccX, linAccY, linAccZ, angVelX, angVelY, angVelX
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 * Note that for GPS correction, we're only using the position not the
 * rotation. The rotation is provided in the file for ground truth comparison.
 *
 *  See usage: ./ImuFactorsExample --help
 */

#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>


// ROS and Gazebo includes
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <livox_laser_simulation/IMU_ISAM2.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/Imu.h>
#include "ImuSubscriber.h"
#include "parameters.h"


#include <cstring>
#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace po = boost::program_options;

po::variables_map parseOptions(int argc, char* argv[]) {
  po::options_description desc;
  desc.add_options()("help,h", "produce help message")(
      "data_csv_path", po::value<string>()->default_value("imuAndGPSdata.csv"),
      "path to the CSV file with the IMU data")(
      "output_filename",
      po::value<string>()->default_value("imuFactorExampleResults.csv"),
      "path to the result file to use")("use_isam", po::bool_switch(),
                                        "use ISAM as the optimizer");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    exit(1);
  }

  return vm;
}


std::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}


std::shared_ptr<PreintegratedCombinedMeasurements::Params> ImuSubscriber::imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}


ImuSubscriber::ImuSubscriber(ros::NodeHandle& nh) {
    // Subscribe to the IMU topic
    subImu = nh.subscribe("/clifford/imu_gazebo", 10, &ImuSubscriber::imuCallback, this);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("imu_odom/pathOPTI", 1);
    pathGT_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_odom/pathGT", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("imu_odom/pose", 1);
    initializeFactorGraph();

}


void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{

    int link_index = 39;
    const geometry_msgs::Pose& pose = msg->pose[link_index];
    const geometry_msgs::Twist& twist = msg->twist[link_index];
    ROS_INFO("Pose of link: [%f, %f, %f]", pose.position.x, pose.position.y, pose.position.z);

    // // Create a new Odometry message
    // nav_msgs::Odometry odom;
    // odom.header.stamp = ros::Time::now();
    // odom.header.frame_id = "/odom";
    // odom.child_frame_id = "/base_footprint";
    // odom.pose.pose = pose;
    // odom.twist.twist = twist;
    // odom.pose.covariance[0] = 0.1;
    // odom.pose.covariance[7] = 0.1;
    // odom.pose.covariance[14] = 99999;
    // odom.pose.covariance[21] = 99999;
    // odom.pose.covariance[28] = 99999;
    // odom.pose.covariance[35] = 0.1;

    // static ros::Publisher pub = ros::NodeHandle().advertise<nav_msgs::Odometry>("/gazebo_link_states_odom", 50);
    // pub.publish(odom);

}



//// for test imu z acceleration ////////////////////////////////////


// void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   // Print the orientation of the IMU
//   ROS_INFO("Quaternion: [%f, %f, %f, %f]", msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//   ROS_INFO("Acceleration: [%f, %f, %f]", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
// }   





void ImuSubscriber::imuCallback(const sensor_msgs::Imu &msg) {
    // Extract the orientation quaternion and linear acceleration vector from the message
    // orientation.x() = msg->orientation.x;
    // orientation.y() = msg->orientation.y;
    // orientation.z() = msg->orientation.z;
    // orientation.w() = msg->orientation.w;

    // linear_acceleration.x() = msg->linear_acceleration.x;
    // linear_acceleration.y() = msg->linear_acceleration.y;
    // linear_acceleration.z() = msg->linear_acceleration.z;

    // angular_velocity.x() = msg->angular_velocity.x;
    // angular_velocity.y() = msg->angular_velocity.y;
    // angular_velocity.z() = msg->angular_velocity.z;


    geometry_msgs::Vector3 aV = msg.angular_velocity;
    geometry_msgs::Vector3 lA = msg.linear_acceleration;
    Vector3 linear_acceleration(lA.x,lA.y,lA.z - gravMag);
    Vector3 angular_velocity(aV.x,aV.y,aV.z);

    // ROS_INFO("Quaternion: [%f, %f, %f, %f]", msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    ROS_INFO("Acceleration: [%f, %f, %f]", msg.linear_acceleration.x, msg.linear_acceleration.y, linear_acceleration.z());
    // ROS_INFO("Angular Velocity: [%f, %f, %f]", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);









    if (!accel_init_done){
        // cout << "yo" << endl;
        do_nominal_init();
    }


    double timestep = msg.header.stamp.toSec();
    imu_times.push_back(timestep);
    imu_linaccs.push_back(linear_acceleration);
    imu_angvel.push_back(angular_velocity);



    // double timestep = msg.header.stamp.toSec();
    if (frame == 0){
        newTimestamps[X(0)] = timestep;
        newTimestamps[V(0)] = timestep;
        newTimestamps[B(0)] = timestep;
        }
    else {
        // cout << "Add IMU Factor" << endl;
        //Add Imu Factor
        CombinedImuFactor imufac = create_imu_factor(timestep);
        // cout << "created imufac" <<endl;
        graph.add(imufac);
        // cout << frame << endl;
        prop_state = preintegrated->predict(prev_state, prev_bias);
        initialEstimate.insert(X(frame), prop_state.pose());
        initialEstimate.insert(V(frame), prop_state.v());
        initialEstimate.insert(B(frame), prev_bias);   
        // cout << "Added IMU Factor" << endl;
        newTimestamps[X(frame)] = timestep;
        newTimestamps[V(frame)] = timestep;
        newTimestamps[B(frame)] = timestep;

        // auto diff  = visualPose.translation() - prop_state.pose().translation();

        // cout << diff << endl;
        noiseModel::Isotropic::shared_ptr pose_correction_noise = noiseModel::Isotropic::Sigma(6, 0.5);

        // pose factor from visual odometry. Maybe replaced by link state. 

        // gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(frame), visualPose,
        //                                             pose_correction_noise);

        // graph.add(pose_factor);

    }

    // graph.print("\nFactor Graph:\n");  // print
    // GraphUtils::writeGnuplot(graph, "mygraph.gp");
    smootherISAM2.update(graph, initialEstimate,newTimestamps);
        // for(size_t i = 1; i < 7; ++i) { // Optionally perform multiple iSAM2 iterations
        //     smootherISAM2.update();
        // }
    // cout << "curr estimate" << endl;

    // Ps: Aother option is LevenbergMarquardtOptimizer

    // gtsam::Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
    currentEstimate = smootherISAM2.calculateEstimate();
    currentEstimate.print("\nCurrent Estimate:\n");
    prev_state =
        gtsam::NavState(currentEstimate.at<Pose3>(X(frame)), currentEstimate.at<Vector3>(V(frame)));
    prev_bias = currentEstimate.at<imuBias::ConstantBias>(B(frame));

    // cout << "resize " << endl;
    graph.resize(0);
    initialEstimate.clear();
    newTimestamps.clear();
    preintegrated->resetIntegrationAndSetBias(prev_bias);


    

    
    // cout << "in VIO callback" << endl;
    sendTfs(timestep);
    
    frame ++;
}




void ImuSubscriber::initializeFactorGraph(){
    ROS_INFO("Initializing Factor Graph");
        
    //SET ISAM2 PARAMS
    ISAM2Params parameters;
    double lag = 2.0;
    parameters.relinearizeThreshold = 0.003; // Set the relin threshold to zero such that the batch estimate is recovered
    parameters.relinearizeSkip = 1; // Relinearize every time
    smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);
    IMUparams = imuParams(); // Defined in last function
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(IMUparams, priorBias);

    priorPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));
    cout << "priorPose" << priorPose << endl;

    accel_init_done = false;
    estimatorInit = false;
    frame = 0;

}

void ImuSubscriber::do_accel_init() {
    // ROS_INFO("Accel Init");
    gtsam::Vector3 acc_avg;

    // cout << "imu_size 2: " << imu_times.size() << endl;
    
    if (imu_times.size() < 30){
        return;
    } else {
        for (auto &accel : imu_linaccs) {
            acc_avg += accel;
            // cout << "accel" << accel << endl; 
        }
        acc_avg /= imu_times.size();

        // acc_avg << 0.0, 0.0, gravMag ;
    }
    
    // cout << "Gravity-aligning with accel. vector:\n" << acc_avg << endl;

    gtsam::Vector3 gravity_vec;
    gravity_vec << 0.0, 0.0, gravMag;
    auto initial_att = gtsam::Rot3(
        Eigen::Quaterniond().setFromTwoVectors(acc_avg, gravity_vec));

    cout << "gravMag" << gravMag << endl;

    // cout << "init att" << initial_att << endl;
    gtsam::Pose3 initial_pose_(initial_att, gtsam::Point3());
    // cout <<  "Gravity vector after alignment: " << (initial_pose_ * acc_avg) << endl;
    priorPose = priorPose*initial_pose_;

    // cout << "prior velocity " << priorVelocity << endl;
    // cout << "prior Pose " << priorPose << endl;

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
    
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    cout << "prev_state" << prev_state << endl;
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}

// MAKE SURE TO CHECK NOISE
void ImuSubscriber::do_nominal_init() {
    ROS_INFO("Nominal Init");

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
     
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    // cout << "prev_state" << prev_state << endl;
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}


CombinedImuFactor ImuSubscriber::create_imu_factor(double updatetime) {
    // cout << "create imu factor " << endl;
    int imucompound = 0;
    // This will add to the pre-integration through each time step until it reaches the timestep of the camera
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0); // new dt
        if (dt >= 0) {
            // Preintegrate this measurement!
            preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt); // adds to the preintegration object
        }
        // erases beginning of the vectors, to then integrate the next two points
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }
    // Checking the timing work out, if not, perform one more integration
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Preintegrate this measurement!
        preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Follows documentation for imu_integration in gtsam
    auto preint_imu_combined =
            dynamic_cast<const PreintegratedCombinedMeasurements&>(
                *preintegrated);


    // cout << "got down here" << endl;
    // cout << "frame" << frame << endl;
    // Creates factor given gtsam documentation, returns it
    CombinedImuFactor imufac(X(frame - 1), V(frame - 1), X(frame),
                                V(frame), B(frame - 1), B(frame),
                                preint_imu_combined);
    // cout << "got here too" << endl;
    return imufac;

}


void ImuSubscriber::sendTfs(double timestep){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    Point3 t;  
    Rot3 r;

    //Send gtsam tf
    t = prev_state.pose().translation();
    r = prev_state.pose().rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "Body_IMU"));

    // //Send ground truth tf
    // t = gtPose.translation();
    // r = gtPose.rotation();
    // transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    // q.setRPY(r.roll(), r.pitch(), r.yaw());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "True_Pose"));
 
    //Publish GT Trajectory
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="/world";
    poseStamped.header.stamp = ros::Time(timestep);
    poseStamped.pose.position.x =  gtPose.x();
    poseStamped.pose.position.y = gtPose.y();
    poseStamped.pose.position.z = gtPose.z();
    pathGT.header.frame_id = "world";
    r = gtPose.rotation();
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();
    // pathGT.poses.push_back(poseStamped);
    pathGT.header.stamp = poseStamped.header.stamp;
    pathGT_pub.publish(poseStamped);
 
    gtsam::Pose3 pose = prev_state.pose(); 
    poseStamped.pose.position.x =  pose.x();
    poseStamped.pose.position.y = pose.y();
    poseStamped.pose.position.z = pose.z();
    pathOPTI.header.frame_id = "world";
    pathOPTI.poses.push_back(poseStamped);
    pathOPTI.header.stamp = poseStamped.header.stamp;
    pathOPTI_pub.publish(pathOPTI); 

    //Publish SLAM Trajectory
    // gtsam::Pose3 pose;
    // for (int i = 0; i <= frame-1; i ++){
         
    //     if (!currentEstimate.exists(X(i))) {continue;}
    //     pose = currentEstimate.at<Pose3>(X(i)); 
    //     // cout << pose.x() << endl;  
    //     poseStamped.pose.position.x =  pose.x();
    //     poseStamped.pose.position.y = pose.y();
    //     poseStamped.pose.position.z = pose.z();
    //     pathOPTI.header.frame_id = "world";
    //     pathOPTI.poses.push_back(poseStamped);
    //     pathOPTI.header.stamp = poseStamped.header.stamp;
    //     pathOPTI_pub.publish(pathOPTI); 
    // }
     
    
    //Publish Pose
    r = pose.rotation();
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();
    pose_pub.publish(poseStamped);
    //pathOPTI.poses.clear();
    
}

int main(int argc, char* argv[]) {
    string data_filename, output_filename;

    bool use_isam = false;



    // enable ros

    ros::init(argc, argv, "loc");
    ros::NodeHandle nh("~");    
    // readParameters(nh); 
    ros::Duration(1.5).sleep();



    // cout << "fx " << fx << endl;

    // bool gazebo_ready = false;
    ImuSubscriber imu_subscriber(nh);
    
    ros::Rate loop_rate(100);
    ros::spin();


    // while (!gazebo_ready) {
    //     // gazebo_msgs::ModelStates model_states;
    //     // ros::ServiceClient get_model_states_client = nh.serviceClient<sensor_msgs::ModelStates>("/clifford/imu_gazebo");
    //     // get_model_states_client.call(model_states);
    //     gazebo_ready = imu_subscriber.linear_acceleration.size() > 0;
    //     cout << "waiting for gazebo" << endl;
    //     ros::spinOnce();
    // }


    // ros::Subscriber subGazeboLinkStates = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, linkStatesCallback); // subsribe to link state

    // ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/clifford/imu_gazebo", 10, imuCallback);
    // ImuSubscriber imu_subscriber(nh);



    // Vector3 linear_acceleration = imu_subscriber.linear_acceleration;
    // Vector3 angular_velocity = imu_subscriber.angular_velocity;     

    // double z_acc = imu_subscriber.linear_acceleration.z(); // get z acceleration
    // if (true) {
    //     printf("linear_acceleration z \n");
    //     cout << z_acc << "\n";
    // }


    // ros::Rate rate(1);  // Set the loop rate to 10 Hz

    // while (ros::ok()) {


        // double z_acc = imu_subscriber.linear_acceleration.z();  // Access the z acceleration from the ImuSubscriber instance
        // ROS_INFO("Z acceleration: %f", z_acc);




        // po::variables_map var_map = parseOptions(argc, argv);

        // data_filename = findExampleDataFile(var_map["data_csv_path"].as<string>());
        // output_filename = var_map["output_filename"].as<string>();
        // use_isam = var_map["use_isam"].as<bool>();

        // ISAM2* isam2 = 0;
        // if (use_isam) {
        // printf("Using ISAM2\n");
        // ISAM2Params parameters;
        // parameters.relinearizeThreshold = 0.01;
        // parameters.relinearizeSkip = 1;
        // isam2 = new ISAM2(parameters);

        // } else {
        // printf("Using Levenberg Marquardt Optimizer\n");
        // }

        // // Set up output file for plotting errors
        // FILE* fp_out = fopen(output_filename.c_str(), "w+");
        // fprintf(fp_out,
        //         "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,"
        //         "gt_qy,gt_qz,gt_qw\n");

        // // Begin parsing the CSV file.  Input the first line for initialization.
        // // From there, we'll iterate through the file and we'll preintegrate the IMU
        // // or add in the GPS given the input.
        // // start reading ImuAndGPS.csv

        // // Read the first line of the CSV file// 
        // ifstream file(data_filename.c_str());
        // string value;

        // // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
        // Vector10 initial_state;
        // getline(file, value, ',');  // i
        // for (int i = 0; i < 9; i++) {
        // getline(file, value, ',');
        // initial_state(i) = stof(value.c_str());
        // }
        // getline(file, value, '\n');
        // initial_state(9) = stof(value.c_str());
        // cout << "initial state:\n" << initial_state.transpose() << "\n\n";

        // // Assemble initial quaternion through GTSAM constructor
        // // ::quaternion(w,x,y,z);
        // Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
        //                                         initial_state(4), initial_state(5));
        // Point3 prior_point(initial_state.head<3>());
        // Pose3 prior_pose(prior_rotation, prior_point);
        
        // // Prior velocity for clifford x = 0.5 m/s
        // // Vector3 prior_velocity(initial_state.tail<3>());
        // Vector3 prior_velocity(0.5, 0.0, 0.0);


        // cout << "initial velo:\n" << prior_velocity.transpose() << "\n\n";


        // imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

        // Values initial_values;
        // int correction_count = 0;
        // initial_values.insert(X(correction_count), prior_pose);
        // initial_values.insert(V(correction_count), prior_velocity);
        // initial_values.insert(B(correction_count), prior_imu_bias);

        // // Assemble prior noise model and add it the graph.`
        // auto pose_noise_model = noiseModel::Diagonal::Sigmas(
        //     (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
        //         .finished());  // rad,rad,rad,m, m, m
        // auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
        // auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

        // // Add all prior factors (pose, velocity, bias) to the graph.
        // NonlinearFactorGraph* graph = new NonlinearFactorGraph();
        // graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
        // graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
        // graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

        // auto p = imuParams();

        // std::shared_ptr<PreintegrationType> preintegrated =
        //     std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

        // assert(preintegrated);

        // // Store previous state for imu integration and latest predicted outcome.
        // NavState prev_state(prior_pose, prior_velocity);
        // NavState prop_state = prev_state;
        // imuBias::ConstantBias prev_bias = prior_imu_bias;

        // // Keep track of total error over the entire run as simple performance metric.
        // double current_position_error = 0.0, current_orientation_error = 0.0;

        // double output_time = 0.0;
        // double dt = 0.005;  // The real system has noise, but here, results are nearly
        //                     // exactly the same, so keeping this for simplicity.


        // ////////All priors have been set up, now iterate through the data file.///////////////////////////////////////////////////////////////
        // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        // ROS_INFO("Z acceleration: %f", imu_subscriber.linear_acceleration.z());
        // preintegrated->integrateMeasurement(imu_subscriber.linear_acceleration, imu_subscriber.angular_velocity, dt);

        // auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
        // ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
        //                     X(correction_count), V(correction_count),
        //                     B(correction_count - 1), preint_imu);
        // graph->add(imu_factor);


        // imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        // graph->add(BetweenFactor<imuBias::ConstantBias>(
        //   B(correction_count - 1), B(correction_count), zero_bias,
        //   bias_noise_model));

        // while (file.good()) {
        //     // Parse out first value
        //     getline(file, value, ',');
        //     int type = stoi(value.c_str());
        //     int counter;

        //     // printf("linear_acceleration z \n");
        //     // cout << linear_acceleration.z() << "\n";
        //     if (type == 0) {  // IMU measurement
        //         // Vector6 imu;
        //         // for (int i = 0; i < 5; ++i) {
        //         // getline(file, value, ',');
        //         // imu(i) = stof(value.c_str());
        //         // }
        //         // getline(file, value, '\n');
        //         // imu(5) = stof(value.c_str());
        //         // printf("Adding IMU factor\n");
        //         // Adding the IMU preintegration.
        //         ROS_INFO("Z acceleration: %f", imu_subscriber.linear_acceleration.z());
        //         preintegrated->integrateMeasurement(imu_subscriber.linear_acceleration, imu_subscriber.angular_velocity, dt);
        //         // printf("linear_acceleration z \n");
        //         // cout << linear_acceleration.z() << "\n";

                



        //     } else if (type == 1) {  // GPS measurement
        //         Vector7 gps;
        //         for (int i = 0; i < 6; ++i) {
        //         getline(file, value, ',');
        //         gps(i) = stof(value.c_str());
        //         }
        //         getline(file, value, '\n');
        //         gps(6) = stof(value.c_str());

        //         correction_count++;

        //         // Adding IMU factor and GPS factor and optimizing.
        //         auto preint_imu =
        //             dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
        //         ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
        //                             X(correction_count), V(correction_count),
        //                             B(correction_count - 1), preint_imu);
        //         graph->add(imu_factor);
        //         imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        //         graph->add(BetweenFactor<imuBias::ConstantBias>(
        //             B(correction_count - 1), B(correction_count), zero_bias,
        //             bias_noise_model));

        //         auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
        //         GPSFactor gps_factor(X(correction_count),
        //                             Point3(gps(0),   // N,
        //                                     gps(1),   // E,
        //                                     gps(2)),  // D,
        //                             correction_noise);
        //         graph->add(gps_factor);

        //         // Now optimize and compare results.
        //         prop_state = preintegrated->predict(prev_state, prev_bias);
        //         initial_values.insert(X(correction_count), prop_state.pose());
        //         initial_values.insert(V(correction_count), prop_state.v());
        //         initial_values.insert(B(correction_count), prev_bias);

        //         Values result;

        //         if (use_isam) {
        //         isam2->update(*graph, initial_values);
        //         result = isam2->calculateEstimate();

        //         // reset the graph
        //         graph->resize(0);
        //         initial_values.clear();

        //         } else {
        //         LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
        //         result = optimizer.optimize();
        //         }

        //         // Overwrite the beginning of the preintegration for the next step.
        //         prev_state = NavState(result.at<Pose3>(X(correction_count)),
        //                             result.at<Vector3>(V(correction_count)));
        //         prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

        //         // Reset the preintegration object.
        //         preintegrated->resetIntegrationAndSetBias(prev_bias);

        //         // Print out the position and orientation error for comparison.
        //         Vector3 gtsam_position = prev_state.pose().translation();
        //         Vector3 position_error = gtsam_position - gps.head<3>();
        //         current_position_error = position_error.norm();

        //         Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
        //         Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
        //         Quaternion quat_error = gtsam_quat * gps_quat.inverse();
        //         quat_error.normalize();
        //         Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2,
        //                                 quat_error.z() * 2);
        //         current_orientation_error = euler_angle_error.norm();

        //         // display statistics
        //         cout << "Position error:" << current_position_error << "\t "
        //             << "Angular error:" << current_orientation_error << "\n";

        //         fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        //                 output_time, gtsam_position(0), gtsam_position(1),
        //                 gtsam_position(2), gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(),
        //                 gtsam_quat.w(), gps(0), gps(1), gps(2), gps_quat.x(),
        //                 gps_quat.y(), gps_quat.z(), gps_quat.w());

        //         output_time += 1.0;

        //     } else {
        //         cerr << "ERROR parsing file\n";
        //         return 1;
        //     }

        // // cout << "COunter:" << counter << "\n";
        // }
        // fclose(fp_out);
        // cout << "Complete, results written to " << output_filename << "\n\n";








        // rate.sleep();  // Sleep to maintain the loop rate
        // ros::spinOnce();  // Process any incoming ROS messages

    // }
    











    // po::variables_map var_map = parseOptions(argc, argv);

    // data_filename = findExampleDataFile(var_map["data_csv_path"].as<string>());
    // output_filename = var_map["output_filename"].as<string>();
    // use_isam = var_map["use_isam"].as<bool>();

    // ISAM2* isam2 = 0;
    // if (use_isam) {
    // printf("Using ISAM2\n");
    // ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // isam2 = new ISAM2(parameters);

    // } else {
    // printf("Using Levenberg Marquardt Optimizer\n");
    // }

    // // Set up output file for plotting errors
    // FILE* fp_out = fopen(output_filename.c_str(), "w+");
    // fprintf(fp_out,
    //         "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,"
    //         "gt_qy,gt_qz,gt_qw\n");

    // // Begin parsing the CSV file.  Input the first line for initialization.
    // // From there, we'll iterate through the file and we'll preintegrate the IMU
    // // or add in the GPS given the input.
    // // start reading ImuAndGPS.csv



    // ifstream file(data_filename.c_str());
    // string value;

    // // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
    // Vector10 initial_state;
    // getline(file, value, ',');  // i
    // for (int i = 0; i < 9; i++) {
    // getline(file, value, ',');
    // initial_state(i) = stof(value.c_str());
    // }
    // getline(file, value, '\n');
    // initial_state(9) = stof(value.c_str());
    // cout << "initial state:\n" << initial_state.transpose() << "\n\n";

    // // Assemble initial quaternion through GTSAM constructor
    // // ::quaternion(w,x,y,z);
    // Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
    //                                         initial_state(4), initial_state(5));
    // Point3 prior_point(initial_state.head<3>());
    // Pose3 prior_pose(prior_rotation, prior_point);
    
    // // Prior velocity for clifford x = 0.5 m/s
    // // Vector3 prior_velocity(initial_state.tail<3>());
    // Vector3 prior_velocity(0.5, 0.0, 0.0);


    // cout << "initial velo:\n" << prior_velocity.transpose() << "\n\n";


    // imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

    // Values initial_values;
    // int correction_count = 0;
    // initial_values.insert(X(correction_count), prior_pose);
    // initial_values.insert(V(correction_count), prior_velocity);
    // initial_values.insert(B(correction_count), prior_imu_bias);

    // // Assemble prior noise model and add it the graph.`
    // auto pose_noise_model = noiseModel::Diagonal::Sigmas(
    //     (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
    //         .finished());  // rad,rad,rad,m, m, m
    // auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
    // auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // // Add all prior factors (pose, velocity, bias) to the graph.
    // NonlinearFactorGraph* graph = new NonlinearFactorGraph();
    // graph->addPrior(X(correction_count), prior_pose, pose_noise_model);
    // graph->addPrior(V(correction_count), prior_velocity, velocity_noise_model);
    // graph->addPrior(B(correction_count), prior_imu_bias, bias_noise_model);

    // auto p = imuParams();

    // std::shared_ptr<PreintegrationType> preintegrated =
    //     std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

    // assert(preintegrated);

    // // Store previous state for imu integration and latest predicted outcome.
    // NavState prev_state(prior_pose, prior_velocity);
    // NavState prop_state = prev_state;
    // imuBias::ConstantBias prev_bias = prior_imu_bias;

    // // Keep track of total error over the entire run as simple performance metric.
    // double current_position_error = 0.0, current_orientation_error = 0.0;

    // double output_time = 0.0;
    // double dt = 0.005;  // The real system has noise, but here, results are nearly
    //                     // exactly the same, so keeping this for simplicity.

    
    // ////////All priors have been set up, now iterate through the data file.///////////////////////////////////////////////////////////////
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // while (file.good()) {
    // // Parse out first value
    // getline(file, value, ',');
    // int type = stoi(value.c_str());
    // int counter;

    // // printf("linear_acceleration z \n");
    // // cout << linear_acceleration.z() << "\n";
    // if (type == 0) {  // IMU measurement
    //     // Vector6 imu;
    //     // for (int i = 0; i < 5; ++i) {
    //     // getline(file, value, ',');
    //     // imu(i) = stof(value.c_str());
    //     // }
    //     // getline(file, value, '\n');
    //     // imu(5) = stof(value.c_str());
    //     // printf("Adding IMU factor\n");
    //     // Adding the IMU preintegration.
        
    //     preintegrated->integrateMeasurement(linear_acceleration, angular_velocity, dt);
    //     // printf("linear_acceleration z \n");
    //     // cout << linear_acceleration.z() << "\n";
    //     counter++;
        



    // } else if (type == 1) {  // GPS measurement
    //     Vector7 gps;
    //     for (int i = 0; i < 6; ++i) {
    //     getline(file, value, ',');
    //     gps(i) = stof(value.c_str());
    //     }
    //     getline(file, value, '\n');
    //     gps(6) = stof(value.c_str());

    //     correction_count++;

    //     // Adding IMU factor and GPS factor and optimizing.
    //     auto preint_imu =
    //         dynamic_cast<const PreintegratedImuMeasurements&>(*preintegrated);
    //     ImuFactor imu_factor(X(correction_count - 1), V(correction_count - 1),
    //                         X(correction_count), V(correction_count),
    //                         B(correction_count - 1), preint_imu);
    //     graph->add(imu_factor);
    //     imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    //     graph->add(BetweenFactor<imuBias::ConstantBias>(
    //         B(correction_count - 1), B(correction_count), zero_bias,
    //         bias_noise_model));

    //     auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
    //     GPSFactor gps_factor(X(correction_count),
    //                         Point3(gps(0),   // N,
    //                                 gps(1),   // E,
    //                                 gps(2)),  // D,
    //                         correction_noise);
    //     graph->add(gps_factor);

    //     // Now optimize and compare results.
    //     prop_state = preintegrated->predict(prev_state, prev_bias);
    //     initial_values.insert(X(correction_count), prop_state.pose());
    //     initial_values.insert(V(correction_count), prop_state.v());
    //     initial_values.insert(B(correction_count), prev_bias);

    //     Values result;

    //     if (use_isam) {
    //     isam2->update(*graph, initial_values);
    //     result = isam2->calculateEstimate();

    //     // reset the graph
    //     graph->resize(0);
    //     initial_values.clear();

    //     } else {
    //     LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    //     result = optimizer.optimize();
    //     }

    //     // Overwrite the beginning of the preintegration for the next step.
    //     prev_state = NavState(result.at<Pose3>(X(correction_count)),
    //                         result.at<Vector3>(V(correction_count)));
    //     prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

    //     // Reset the preintegration object.
    //     preintegrated->resetIntegrationAndSetBias(prev_bias);

    //     // Print out the position and orientation error for comparison.
    //     Vector3 gtsam_position = prev_state.pose().translation();
    //     Vector3 position_error = gtsam_position - gps.head<3>();
    //     current_position_error = position_error.norm();

    //     Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
    //     Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
    //     Quaternion quat_error = gtsam_quat * gps_quat.inverse();
    //     quat_error.normalize();
    //     Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2,
    //                             quat_error.z() * 2);
    //     current_orientation_error = euler_angle_error.norm();

    //     // display statistics
    //     cout << "Position error:" << current_position_error << "\t "
    //         << "Angular error:" << current_orientation_error << "\n";

    //     fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    //             output_time, gtsam_position(0), gtsam_position(1),
    //             gtsam_position(2), gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(),
    //             gtsam_quat.w(), gps(0), gps(1), gps(2), gps_quat.x(),
    //             gps_quat.y(), gps_quat.z(), gps_quat.w());

    //     output_time += 1.0;

    // } else {
    //     cerr << "ERROR parsing file\n";
    //     return 1;
    // }

    // // cout << "COunter:" << counter << "\n";
    // }
    // fclose(fp_out);
    // cout << "Complete, results written to " << output_filename << "\n\n";








    
//     ros::Rate loop_rate(100);
//     ros::spin();

  return 0;
}
