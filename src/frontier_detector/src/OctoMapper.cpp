//
// jplaced
// 2022, Universidad de Zaragoza
//

#include "OctoMapper.h"

//#define OCTO_RESOLUTION_ 0.05
#define OCTO_MIN_POINTS_TH 1000
#define OCTO_CLEAR_TH_ 4

int main(int argc, char **argv) {

    ros::init(argc, argv, "Octomapper");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    OctoMapper OCTO_NODE (node_handle);

    OCTO_NODE.Init();

    ros::Rate r(int(OCTO_NODE.rate_hz_));

    while (ros::ok()) {

        // Update and publish OctoMap with extracted PC
        OCTO_NODE.Update ();

        // Spin ROS with rate r
        ros::spinOnce();
        r.sleep();

    }

    ros::shutdown();

    return 0;

}

OctoMapper::OctoMapper (ros::NodeHandle &node_handle) {

    name_of_node_ = ros::this_node::getName();
    node_handle_ = node_handle;
    previous_cloud_size_ = 0;
    since_last_big_update_ = 0;
    octo_origin_ = { 0.0, 0.0, 0.0 };
    octo_frame_ = octomap::pose6d(0, 0, 0, 0, 0, 0);
    update_octomap_ = false;
    clear_octomap_ = false;
    T_optical_target_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    gba_state_ = false;
    orb_state_ = 0; // Not initialized

}


OctoMapper::~OctoMapper () {

    delete octomap_;
    delete pointcloud_map_points_;

}


void OctoMapper::Init () {

    ROS_INFO("%s : Initializing.", name_of_node_.c_str());

    // Static ROS parameters
    initializeParams(node_handle_);

    // Initialization transformation listener
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

    // Enable publishing OctoMap
    octomap_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>(name_of_node_+"/octomap", 3);

    // PointCloud subscriber
    map_PC_subscriber_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/map_points", 10, &OctoMapper::MapCallback, this);
    ORBstate_subscriber_ = node_handle_.subscribe<orb_slam2_ros::ORBState>("/info/state", 10, &OctoMapper::ORBStateCallback, this);
    GBAstate_subscriber_ = node_handle_.subscribe<std_msgs::Bool> ("/gba_running", 10, &OctoMapper::GBAStateCallback, this);

    // Services
    //octoGetter_server_ = node_handle_.advertiseService(name_of_node_+"/get_octomap", &OctoMapper::GetOctomapSrv, this);

    getTfTransformMatrix(T_optical_target_, "camera_link_optical", target_frame_id_param_);

    ROS_INFO("%s : Initialized successfully.", name_of_node_.c_str());

}

void OctoMapper::initializeParams(ros::NodeHandle &nh){

    nh.param<int>(name_of_node_+ "/rate", rate_hz_, int(1));

    nh.param<bool>(name_of_node_+ "/octomap/publish_octomap", publish_octomap_param_, true);

    nh.param<std::string>(name_of_node_+ "/target_frame_id", target_frame_id_param_, "base_footprint");
    nh.param<std::string>(name_of_node_+ "/pointcloud_frame_id", map_frame_id_param_, "map");
    nh.param<double>(name_of_node_+ "/max_height", max_height_param_, 3.0);

    nh.param<bool>(name_of_node_+"/octomap/rebuild",octomap_rebuild_, false);

}


void OctoMapper::ORBStateCallback(const orb_slam2_ros::ORBState::ConstPtr &msg) {

    if (msg->state == 4) orb_state_ = 1; // OK
    else if (msg->state == 5) orb_state_ = 2; // LOST
    else orb_state_ = 0; // UNKNOWN, SYSTEM_NOT_READY, NOT_INITIALIZED

}


void OctoMapper::GBAStateCallback(const std_msgs::Bool::ConstPtr &msg) {

    gba_state_ = msg->data;

}


void OctoMapper::MapCallback (const sensor_msgs::PointCloud2::ConstPtr& msg) {
    /* This callback consumes little time, so pull all map points every time
     */

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    if (orb_state_ == 1) { // ORB-SLAM is OK
        // Convert input cloud from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
        sensor_msgs::PointCloud out_pointcloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);

        update_octomap_ = false; // Avoid OctoMap update at the same time points are being updated

        // Convert cloud from sensor_msgs::PointCloud to octomap::Pointcloud
        pointcloud_map_points_->clear();
        for (int i = 0; i < out_pointcloud.points.size(); i++) {
            if(out_pointcloud.points[i].z < max_height_param_) {
                Eigen::Vector3d map_pt(out_pointcloud.points[i].x,
                                       out_pointcloud.points[i].y,
                                       out_pointcloud.points[i].z);

                pointcloud_map_points_->push_back(map_pt[0], map_pt[1], map_pt[2]);
            }
        }

        update_octomap_ = true;
        ROS_DEBUG("%s: Pulled PC with %li points.", name_of_node_.c_str(), pointcloud_map_points_->size());

    } else if (orb_state_ == 2){
        update_octomap_ = false;
        ROS_WARN_THROTTLE(1,"%s: Not updating OctoMap if ORB-SLAM is LOST.", name_of_node_.c_str());
    } else {
        update_octomap_ = false;
        clear_octomap_ = true;
        ROS_WARN_THROTTLE(1,"%s: Not updating OctoMap if ORB-SLAM is UNKNOWN, NOT_READY, NOT_INITIALIZED.", name_of_node_.c_str());
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end - begin;
    ROS_DEBUG("%s: PointCloud reading took %f [s].", name_of_node_.c_str(), diff.count());
}


void OctoMapper::Update () {
    /* This update does consume time, so do it when necessary:
     * Only update if at least OCTO_MIN_POINTS_TH were added to the
     * PointCloud or if it has not been updated for OCTO_CLEAR_TH_ times
     */

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Not updating during GBAs or while ORB-SLAM not OK
    if (gba_state_) {
        ROS_WARN_THROTTLE(1, "%s: Not updating OctoMap during GBA.", name_of_node_.c_str());
    } else if (clear_octomap_){
        // TODO create clearing service?
        octomap_->clear();
        clear_octomap_ = false;
        PublishOctoMap();
        ROS_WARN("%s: Cleared OctoMap.", name_of_node_.c_str());
    } else if (update_octomap_){

        int points_difference = pointcloud_map_points_->size() - previous_cloud_size_;
        points_difference = abs(points_difference);

        // Add all points (slow update)
        if (points_difference >= OCTO_MIN_POINTS_TH || since_last_big_update_ >= OCTO_CLEAR_TH_) {

            octomap_->clear();
            octomap_->insertPointCloud(*pointcloud_map_points_, octo_origin_, octo_frame_);

            previous_cloud_size_ = pointcloud_map_points_->size();
            since_last_big_update_ = 0;

            //ROS_DEBUG("%s: OctoMap updated with %li points.", name_of_node_.c_str(), pointcloud_map_points_->size());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end - begin;
            ROS_DEBUG("%s: OctoMap full update took %f [s].", name_of_node_.c_str(), diff.count());

        // Add only recent points (fast update). Old points won't be updated nor deleted
        } else if (points_difference > 0) {
            since_last_big_update_++;
            octomap::Pointcloud pointcloud_map_points_reduced;

            int start = (int) previous_cloud_size_/1.1; // heuristic
            for (int i = start; i < pointcloud_map_points_->size(); i++) {
                pointcloud_map_points_reduced.push_back(pointcloud_map_points_->getPoint(i));
            }

            octomap_->insertPointCloud(pointcloud_map_points_reduced, octo_origin_, octo_frame_);
            previous_cloud_size_ = pointcloud_map_points_->size();

            //ROS_DEBUG("%s: OctoMap updated with %li points.", name_of_node_.c_str(), pointcloud_map_points_reduced.size());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = end - begin;
            ROS_DEBUG("%s: OctoMap incremental update took %f [s].", name_of_node_.c_str(), diff.count());
        }

        // Publishers
        PublishOctoMap();

    }

}


void OctoMapper::PublishOctoMap () {

    if (octomap_publisher_.getNumSubscribers() > 0){
        octomap_msgs::Octomap msgOctomap;
        msgOctomap.header.frame_id = map_frame_id_param_;
        msgOctomap.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(*octomap_, msgOctomap)) {
            octomap_publisher_.publish(msgOctomap);
        }
    }

}


bool OctoMapper::getTfTransformMatrix(Eigen::Affine3d& transform_matrix, const std::string source_frame, const std::string target_frame) {

    try {
        geometry_msgs::TransformStamped transform_to_robot = tfBuffer->lookupTransform(target_frame, source_frame,ros::Time::now(),ros::Duration(0.05));

        transform_matrix = tf2::transformToEigen(transform_to_robot);
        return true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
    }

}


/*
bool OctoMapper::GetOctomapSrv(frontier_detector::GetOctomap::Request  &req,
                               frontier_detector::GetOctomap::Response &res)
{
    //ros::WallTime startTime = ros::WallTime::now();
    //ROS_INFO("Sending binary map data on service request");

    res.map.header.frame_id = map_frame_id_param_;
    res.map.header.stamp = ros::Time::now();
    if (!octomap_msgs::binaryMapToMsg(*octomap_, res.map))
        return false;

    //double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    //ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
    return true;
}
 */
