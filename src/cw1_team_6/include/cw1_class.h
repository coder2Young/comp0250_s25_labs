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
// Camera specific includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
// OpenCV specific includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/region_growing_rgb.h>

// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>

// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_x/example.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

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

typedef enum {
  red,
  purple,
  blue,
  none
} Color;

class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  void
  cw1Config();

  // Util functions
  bool
  moveArm(geometry_msgs::PoseStamped target_pose);

  bool 
  moveGripper(float width, float wait_time = 0.0);

  void
  pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point);

  void 
  addCollisionBasket(geometry_msgs::Point centre);

  void
  addCollitionGround();

  void
  clearCollisionObject();

  std::pair<int, int> 
  getLocOfCameraImage(geometry_msgs::PointStamped basket_loc);

  // Sensor callbacks
  void
  cameraImgCallback(const sensor_msgs::ImageConstPtr& msg);

  void
  cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void
  depthImgCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  std::vector<PointCPtr>
  regionGrowing(PointCPtr cloud);

  // Color Related Function
  static std::string
  colorMapping(float r, float g, float b);

  static std::string
  colorToString(Color color);

  static Color
  stringToColor(std::string color);

  PointCPtr
  filterCloudWithColor(PointCPtr cloud);

  PointCPtr
  transformCloudToBaseFrame(PointCPtr cloud_in);

  std::vector<PointCPtr> 
  mergeClusters(const std::vector<PointCPtr>& clusters);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);

  bool
  t1_process(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response);

  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);

  bool 
  t2_process(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);

  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  bool 
  t3_process(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  bool debug_ = false;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_img_;
  ros::Subscriber sub_img_info_;
  ros::Subscriber sub_depth_;
  ros::Publisher pub_filtered_cloud_;

  /** \brief Define some useful constant values. */
  std::string base_frame_ = "panda_link0";
  double gripper_open_;
  double gripper_closed_;
  float grasp_stanby_height_;
  float place_stanby_height_;

  float box_size_;
  float basket_size_;
  float hand_offset_;
  float ground_width_;
  float ground_length_;

  std::vector<moveit_msgs::CollisionObject> collision_object_vector_;

  geometry_msgs::Quaternion grasp_orientation_;

  geometry_msgs::Pose scan_pose_;

  camera_info camera_info_;
  camera_image camera_image_;

  std::string cloud_frame_id_;
  PointCPtr cloud_;
  float position_precision_;
  int box_basket_size_thresh_; // For distinguish from box and basket
  float cluster_color_thresh_ ;
  float cluster_dist_thresh_;
  int min_cluster_thresh_;
};

#endif // end of include guard for CW1_CLASS_H_
