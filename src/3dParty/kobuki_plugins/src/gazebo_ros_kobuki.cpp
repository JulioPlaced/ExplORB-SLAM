
#include <cmath>
#include <cstring>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include "kobuki_gazebo_plugins/gazebo_ros_kobuki.h"
#if GAZEBO_MAJOR_VERSION >= 9
  // #include <ignition/math.hh>
  #include <ignition/math/Vector3.hh>
  #include <ignition/math/Quaternion.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif

namespace gazebo
{


GazeboRosKobuki::GazeboRosKobuki() : shutdown_requested_(false)
{
  // Initialise variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  cliff_detected_left_ = false;
  cliff_detected_center_ = false;
  cliff_detected_right_ = false;
}

GazeboRosKobuki::~GazeboRosKobuki()
{
//  rosnode_->shutdown();
  shutdown_requested_ = true;
  // Wait for spinner thread to end
//  ros_spinner_thread_->join();

  //  delete spinner_thread_;
//  delete rosnode_;
}

void GazeboRosKobuki::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;
  if (!model_)
  {
    ROS_ERROR_STREAM("Invalid model pointer! [" << node_name_ << "]");
    return;
  }

  gazebo_ros_ = GazeboRosPtr(new GazeboRos(model_, sdf, "Kobuki"));
  sdf_ = sdf;
  gazebo_ros_->getParameter(this->update_rate_, "update_rate", 0.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get then name of the parent model and use it as node name
  std::string model_name = sdf->GetParent()->Get<std::string>("name");
  gzdbg << "Plugin model name: " << model_name << "\n";
  node_name_ = model_name;
  world_ = parent->GetWorld();

  prepareMotorPower();
  preparePublishTf();

  if(prepareJointState() == false)
    return;
  if(prepareWheelAndTorque() == false)
    return;

  prepareOdom();

  if(prepareVelocityCommand() == false)
    return;
  if(prepareCliffSensor() == false)
    return;
  if(prepareBumper() == false)
    return;
  if(prepareIMU() == false)
    return;

  setupRosApi(model_name);

  #if GAZEBO_MAJOR_VERSION >= 9
    prev_update_time_ = world_->SimTime();
  #else
    prev_update_time_ = world_->GetSimTime();
  #endif

  ROS_INFO_STREAM("GazeboRosKobuki plugin ready to go! [" << node_name_ << "]");
  update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosKobuki::OnUpdate, this));

}


void GazeboRosKobuki::OnUpdate()
{
  /*
   * First process ROS callbacks
   */
  ros::spinOnce();

  /*
   * Update current time and time step
   */
  common::Time time_now;
  #if GAZEBO_MAJOR_VERSION >= 9
    time_now = world_->SimTime();
  #else
    time_now = world_->GetSimTime();
  #endif

  if (time_now < prev_update_time_) {
    ROS_WARN_NAMED("gazebo_ros_kobuki", "Negative update time difference detected.");
    prev_update_time_ = time_now;
  }

  common::Time step_time = time_now - prev_update_time_;

  // propagate wheel velocity commands at the full simulation rate
  propagateVelocityCommands();

  // publish rate control
  if (this->update_rate_ > 0 && step_time.Double() < (1.0 / this->update_rate_)) {
    return;
  }

  prev_update_time_ = time_now;

  updateJointState();
  updateOdometry(step_time);
  updateIMU();
  updateCliffSensor();
  updateBumper();
}

void GazeboRosKobuki::spin()
{
  ros::Rate r(10);
  while(ros::ok() && !shutdown_requested_)
  {
    r.sleep();
  }
}

void GazeboRosKobuki::motorPowerCB(const kobuki_msgs::MotorPowerPtr &msg)
{
  if ((msg->state == kobuki_msgs::MotorPower::ON) && (!motors_enabled_))
  {
    motors_enabled_ = true;
    ROS_INFO_STREAM("Motors fired up. [" << node_name_ << "]");
  }
  else if ((msg->state == kobuki_msgs::MotorPower::OFF) && (motors_enabled_))
  {
    motors_enabled_ = false;
    ROS_INFO_STREAM("Motors taking a rest. [" << node_name_ << "]");
  }
}

void GazeboRosKobuki::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
  #if GAZEBO_MAJOR_VERSION >= 9
    last_cmd_vel_time_ = world_->SimTime();
  #else
    last_cmd_vel_time_ = world_->GetSimTime();
  #endif

  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
}

void GazeboRosKobuki::resetOdomCB(const std_msgs::EmptyConstPtr &msg)
{
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosKobuki);

} // namespace gazebo
