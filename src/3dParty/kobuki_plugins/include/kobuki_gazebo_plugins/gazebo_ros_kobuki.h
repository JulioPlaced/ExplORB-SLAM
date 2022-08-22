/*
 * Copyright (c) 2013, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Marcus Liebhardt
 *
 * This work has been inspired by Nate Koenig's Gazebo plugin for the iRobot Create.
 */

#ifndef GAZEBO_ROS_KOBUKI_H
#define GAZEBO_ROS_KOBUKI_H

#include <cmath>
#include <cstring>
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#if GAZEBO_MAJOR_VERSION >= 9
  // #include <ignition/math.hh>
  #include <ignition/math/Vector3.hh>
  #include <ignition/math/Quaternion.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/BumperEvent.h>

namespace gazebo
{

enum {LEFT= 0, RIGHT=1};

class GazeboRosKobuki : public ModelPlugin
{
public:
  /// Constructor
  GazeboRosKobuki();
  /// Destructor
  ~GazeboRosKobuki();
  /// Called when plugin is loaded
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  /// Called by the world update start event
  void OnUpdate();

private:
  /*
   * Methods
   */
  /// Callback for incoming velocity commands
  void cmdVelCB(const geometry_msgs::TwistConstPtr &msg);
  /// Callback for incoming velocity commands
  void motorPowerCB(const kobuki_msgs::MotorPowerPtr &msg);
  /// Callback for resetting the odometry data
  void resetOdomCB(const std_msgs::EmptyConstPtr &msg);
  /// Spin method for the spinner thread
  void spin();
  //  void OnContact(const std::string &name, const physics::Contact &contact); necessary?


  // internal functions for load
  void prepareMotorPower();
  bool prepareJointState();
  void preparePublishTf();
  bool prepareWheelAndTorque();
  void prepareOdom();
  bool prepareVelocityCommand();
  bool prepareCliffSensor();
  bool prepareBumper();
  bool prepareIMU();
  void setupRosApi(std::string& model_name);

  // internal functions for update
  void updateJointState();
  void updateOdometry(common::Time& step_time);
  void updateIMU();
  void propagateVelocityCommands();
  void updateCliffSensor();
  void updateBumper();


  /*
   *  Parameters
   */
  /// ROS node handles (relative & private)
  ros::NodeHandle nh_, nh_priv_;
  /// node name
  std::string node_name_;

  /// TF Prefix
  std::string tf_prefix_;
  /// extra thread for triggering ROS callbacks
//  boost::shared_ptr<boost::thread> ros_spinner_thread_; necessary?
  /// flag for shutting down the spinner thread
  bool shutdown_requested_;
  /// pointer to the model
  physics::ModelPtr model_;
  /// pointer to the gazebo ros node
  GazeboRosPtr gazebo_ros_;
  sdf::ElementPtr sdf_;
  /// pointer to simulated world
  physics::WorldPtr world_;
  /// pointer to the update event connection (triggers the OnUpdate callback when event update event is received)
  event::ConnectionPtr update_connection_;
  /// rate control
  double update_rate_;
  /// Simulation time on previous update
  common::Time prev_update_time_;
  /// ROS subscriber for motor power commands
  ros::Subscriber motor_power_sub_;
  /// Flag indicating if the motors are turned on or not
  bool motors_enabled_;
  /// Pointers to Gazebo's joints
  physics::JointPtr joints_[2];
  /// Left wheel's joint name
  std::string left_wheel_joint_name_;
  /// Right wheel's joint name
  std::string right_wheel_joint_name_;
  /// ROS publisher for joint state messages
  ros::Publisher joint_state_pub_;
  /// ROS message for joint sates
  sensor_msgs::JointState joint_state_;
  /// ROS subscriber for velocity commands
  ros::Subscriber cmd_vel_sub_;
  /// Simulation time of the last velocity command (used for time out)
  common::Time last_cmd_vel_time_;
  /// Time out for velocity commands in seconds
  double cmd_vel_timeout_;
  /// Speeds of the wheels
  double wheel_speed_cmd_[2];
  /// Max. torque applied to the wheels
  double torque_;
  /// Separation between the wheels
  double wheel_sep_;
  /// Diameter of the wheels
  double wheel_diam_;
  /// Vector for pose
  double odom_pose_[3];
  /// Vector for velocity
  double odom_vel_[3];
  /// Pointer to pose covariance matrix
  double *pose_cov_[36];
  /// Pointer to twist covariance matrix
  double *twist_cov_[36];
  /// ROS publisher for odometry messages
  ros::Publisher odom_pub_;
  /// ROS message for odometry data
  nav_msgs::Odometry odom_;
  /// Flag for (not) publish tf transform for odom -> robot
  bool publish_tf_;
  /// TF transform publisher for the odom frame
  tf::TransformBroadcaster tf_broadcaster_;
  /// TF transform for the odom frame
  geometry_msgs::TransformStamped odom_tf_;
  /// Pointer to left cliff sensor
  sensors::RaySensorPtr cliff_sensor_left_;
  /// Pointer to frontal cliff sensor
  sensors::RaySensorPtr cliff_sensor_center_;
  /// Pointer to left right sensor
  sensors::RaySensorPtr cliff_sensor_right_;
  /// ROS publisher for cliff detection events
  ros::Publisher cliff_event_pub_;
  /// Kobuki ROS message for cliff event
  kobuki_msgs::CliffEvent cliff_event_;
  /// Cliff flag for the left sensor
  bool cliff_detected_left_;
  /// Cliff flag for the center sensor
  bool cliff_detected_center_;
  /// Cliff flag for the right sensor
  bool cliff_detected_right_;
  /// measured distance in meter for detecting a cliff
  float cliff_detection_threshold_;
  /// Maximum distance to floor
  int floot_dist_;
  /// Pointer to bumper sensor simulating Kobuki's left, centre and right bumper sensors
  sensors::ContactSensorPtr bumper_;
  /// ROS publisher for bumper events
  ros::Publisher bumper_event_pub_;
  /// Kobuki ROS message for bumper event
  kobuki_msgs::BumperEvent bumper_event_;
  /// Flag for left bumper's last state
  bool bumper_left_was_pressed_;
  /// Flag for center bumper's last state
  bool bumper_center_was_pressed_;
  /// Flag for right bumper's last state
  bool bumper_right_was_pressed_;
  /// Flag for left bumper's current state
  bool bumper_left_is_pressed_;
  /// Flag for left bumper's current state
  bool bumper_center_is_pressed_;
  /// Flag for left bumper's current state
  bool bumper_right_is_pressed_;
  /// Pointer to IMU sensor model
  sensors::ImuSensorPtr imu_;
  /// Storage for the angular velocity reported by the IMU

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d vel_angular_;
  #else
    math::Vector3 vel_angular_;
  #endif

  /// ROS publisher for IMU data
  ros::Publisher imu_pub_;
  /// ROS message for publishing IMU data
  sensor_msgs::Imu imu_msg_;
  /// ROS subscriber for reseting the odometry data
  ros::Subscriber odom_reset_sub_;



};

} // namespace gazebo

#endif /* GAZEBO_ROS_KOBUKI_H */
