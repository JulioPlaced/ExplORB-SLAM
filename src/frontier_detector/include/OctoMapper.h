//
// jplaced
// 2022, Universidad de Zaragoza
//

#ifndef EXPLORBSLAM_OCTOMAPPER_H
#define EXPLORBSLAM_OCTOMAPPER_H


#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Core>
#include <chrono>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include "tf_conversions/tf_eigen.h"

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>

#include <orb_slam2_ros/ORBState.h>
//#include <frontier_detector/GetOctomap.h>

class OctoMapper {
public:
    OctoMapper (ros::NodeHandle &node_handle);
    ~OctoMapper ();
    void Init();
    void Update ();

    int rate_hz_;
    std::string name_of_node_;

protected:
    void initializeParams(ros::NodeHandle& nh);
    ros::Time current_frame_time_;

private:
    void PublishOctoMap ();

    void MapCallback (const sensor_msgs::PointCloud2::ConstPtr& msg);
    void ORBStateCallback (const orb_slam2_ros::ORBState::ConstPtr& msg);
    void GBAStateCallback (const std_msgs::Bool::ConstPtr& msg);

    //bool GetOctomapSrv(frontier_detector::GetOctomap::Request &req, frontier_detector::GetOctomap::Response &res);

    bool getTfTransformMatrix(Eigen::Affine3d& transform_matrix, const std::string source_frame, const std::string target_frame);

    // Transform listener
    boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListener;

    // Publishers
    ros::Publisher octomap_publisher_;

    // Subscribers
    ros::Subscriber map_PC_subscriber_;
    ros::Subscriber ORBstate_subscriber_;
    ros::Subscriber GBAstate_subscriber_;

    // Services
    //ros::ServiceServer octoGetter_server_;

    ros::NodeHandle node_handle_;

    // Params
    bool publish_octomap_param_;

    bool octomap_rebuild_;

    bool update_octomap_;
    bool clear_octomap_;
    bool octomap_tf_based_;
    int lastBigMapChange_;
    int last_big_update_;
    int previous_cloud_size_;
    int since_last_big_update_;
    double max_height_param_;

    std::string map_frame_id_param_;
    std::string target_frame_id_param_;

    octomap::OcTree* octomap_ = new octomap::OcTree(0.10);
    octomap::Pointcloud* pointcloud_map_points_ = new octomap::Pointcloud;
    octomap::pose6d octo_frame_;
    octomap::point3d octo_origin_;

    Eigen::Affine3d T_optical_target_;

    bool gba_state_;
    int orb_state_;

};

#endif //EXPLORBSLAM_OCTOMAPPER_H
