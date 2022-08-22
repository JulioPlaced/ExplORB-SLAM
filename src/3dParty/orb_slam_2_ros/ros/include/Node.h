/**
* Modified by jplaced, jjgomez
* 2022, University of Zaragoza
*/

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_NODE_H_
#define ORBSLAM2_ROS_NODE_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include "tf_conversions/tf_eigen.h"

#include <dynamic_reconfigure/server.h>

#include <orb_slam2_ros/dynamic_reconfigureConfig.h>
#include "orb_slam2_ros/SaveMap.h"
#include <orb_slam2_ros/ORBState.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"

#include "System.h"
#include "Tracking.h"
#include "Map.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

class Node
{
  public:
    Node (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~Node ();
    void Init ();

  protected:
    void initializeParams(ros::NodeHandle &nh);
    void Update ();
    ORB_SLAM2::System* orb_slam_;
    ros::Time current_frame_time_;
    std::string camera_info_topic_;

  private:
    void PublishState(int trackingState);
    void PublishMapPoints (std::vector<ORB_SLAM2::MapPoint*> map_points);
    const char* stateDescription(orb_slam2_ros::ORBState orb_state);
    const orb_slam2_ros::ORBState toORBStateMessage(int trackingState);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishGBAStatus (bool gba_status);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);
    bool SaveMapSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
    void LoadOrbParameters (ORB_SLAM2::ORBParameters& parameters);

    bool getTfTransformMatrix(Eigen::Affine3d &transform_matrix, const std::string source_frame, const std::string target_frame);

    void publishVertices(std::list<float>& l);
    void publishEdges(std::list<float>& l);
    void publishPoints(std::list<float>& l);

    // Initialization Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    tf2::Transform TransformFromMat (cv::Mat position_mat);
    tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM2::MapPoint*> map_points);

    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher status_gba_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher state_desc_publisher_;

    ros::Publisher vertex_publisher_;
    ros::Publisher edge_publisher_;
    ros::Publisher point_publisher_;

    ros::Subscriber odom_subscriber_;

    ros::ServiceServer service_server_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM2::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string target_frame_id_param_;
    std::string map_file_name_param_;
    std::string voc_file_name_param_;
    bool load_map_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    bool was_lost_;
    int min_observations_per_point_;

    orb_slam2_ros::ORBState orb_state_;

    Eigen::Affine3d odometry_pose_;

    Eigen::Matrix3d cam_base_R_;
    Eigen::Vector3d cam_base_T_;
};

#endif //ORBSLAM2_ROS_NODE_H_
