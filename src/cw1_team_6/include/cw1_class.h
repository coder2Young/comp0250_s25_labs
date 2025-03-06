/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_x/example.h"

typedef struct camera_info{
  int height;
  int width;
  float fx;
  float fy;
  float cx;
  float cy;
  bool info_received = false;
} camera_info;

typedef struct camera_image{
  sensor_msgs::Image latest_image;
  bool image_updated = false;
  ros::Time last_image_time;
} camera_image;

class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  // Util functions
  bool
  moveArm(geometry_msgs::PoseStamped target_pose);

  bool 
  moveGripper(float width, float wait_time = 0.0);

  void
  pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point);

  void 
  addCollisionBasket(geometry_msgs::Point centre);

  std::pair<int, int> 
  getLocOfCameraImage(geometry_msgs::PointStamped basket_loc);

  std::string
  colorMapping(float r, float g, float b);

  void
  cloudFiltering();

  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
  clusterPointclouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

  // Sensor callbacks
  void
  cameraImgCallback(const sensor_msgs::ImageConstPtr& msg);

  void
  cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void
  depthImgCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_img_;
  ros::Subscriber sub_img_info_;
  ros::Subscriber sub_depth_;

  /** \brief Define some useful constant values. */
  std::string base_frame_ = "panda_link0";
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;

  geometry_msgs::Pose scan_pose_;

  camera_info camera_info_;
  camera_image camera_image_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_;
};

#endif // end of include guard for CW1_CLASS_H_
