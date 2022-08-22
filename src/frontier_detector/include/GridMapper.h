//
// jplaced
// 2022, Universidad de Zaragoza
//

#ifndef EXPLORBSLAM_GRIDMAPPER_H
#define EXPLORBSLAM_GRIDMAPPER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <unistd.h>

#include <chrono>
#include <Eigen/Core>

#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

#include <orb_slam2_ros/ORBState.h>
//#include <frontier_detector/GetOctomap.h>

class GridMapper {
public:
    GridMapper (ros::NodeHandle &node_handle);
    ~GridMapper ();
    void Init();
    void Update ();

    int rate_hz_;
    std::string name_of_node_;

protected:
    void initializeParams(ros::NodeHandle& nh);
    ros::Time current_frame_time_;

    inline std::vector<unsigned> point2Index (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p) {

        tf::Point pt;
        tf::Transform world_to_map;

        tf::pointMsgToTF(p, pt);
        tf::poseMsgToTF(info.origin, world_to_map);

        tf::Point p2 = world_to_map.inverse() * pt;

        unsigned i, j;
        i = floor(p2.x()/info.resolution);
        j = floor(p2.y()/info.resolution);

        return {i,j};
    }

private:
    void PublishOccupancyGrid ();

    void octoMapCallback (const octomap_msgs::Octomap::ConstPtr& msg);
    void GBAStateCallback (const std_msgs::Bool::ConstPtr& msg);

    void octomapToOccupancyGrid(const octomap::OcTree& octree, nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& map_rect, const double minZ_, const double maxZ_ );
    nav_msgs::OccupancyGrid rectifyMap(const nav_msgs::OccupancyGrid& map, cv::Mat matrix_OCC, cv::Mat matrix_FREE);

    // Publishers
    ros::Publisher occupancy_grid_publisher_;
    ros::Publisher occupancy_grid_rect_publisher_;

    // Subscribers
    ros::Subscriber octomap_subscriber_;
    ros::Subscriber GBAstate_subscriber_;

    // Services
    //ros::ServiceClient octoGetter_client_;

    ros::NodeHandle node_handle_;

    // Params
    bool publish_occupancy_grid_param_;
    bool publish_occupancy_grid_rect_param_;

    double projection_min_height_;
    double projection_max_height_;

    bool gba_state_;

    // Assign occupancy grid map values
    static const unsigned char MAT_UNKNOWN_ = 255;
    static const unsigned char MAT_KNOWN_ = 0; // UNOCCUPIED (0) OCCUPIED (255)

    std::string map_frame_id_param_;

    octomap::OcTree* octomap_ = new octomap::OcTree(0.05);

};

#endif //EXPLORBSLAM_GRIDMAPPER_H
