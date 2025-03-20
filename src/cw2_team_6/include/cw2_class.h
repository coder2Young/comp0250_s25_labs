/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>

// // include any services created in this package
// #include "cw2_team_x/example.h"
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
// MoveIt specific includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// TF specific includes
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
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
// Add marker array for PCA visualization
#include <visualization_msgs/MarkerArray.h>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// Type definitions
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

// Structure for organizing orientation data from PCA analysis
struct ObjectOrientationData {
  Eigen::Vector3f primary_axis;    // Main orientation vector (largest variance for cross, normal for nought)
  Eigen::Vector3f secondary_axis;  // Secondary axis for nought shape (used for corner grasping)
  float grasp_angle;               // Computed angle for gripper rotation around Z
  bool is_valid;                   // Indicates if the orientation data is valid
};

class cw2
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw2(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
    cw2_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
    cw2_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
    cw2_world_spawner::Task3Service::Response &response);

  void
  cw2_config();

  bool
  moveArm(geometry_msgs::PoseStamped target_pose);

  bool 
  moveGripper(float width, float wait_time = 0.0);

  void
  pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point);

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

  float scan_height_;

  // Point cloud publishers for visualization
  ros::Publisher cloud_filtered_pub_;
  ros::Publisher cloud_object_pub_;
  ros::Publisher pca_axes_pub_;  // For visualizing PCA axes

  // Task 1 methods
  bool executeTask1(const cw2_world_spawner::Task1Service::Request &req, 
                    cw2_world_spawner::Task1Service::Response &res);
  
  // Methods for object orientation detection
  bool moveToScanPosition(const geometry_msgs::Point &target_point);
  PointCPtr getFilteredPointCloud();
  PointCPtr extractObjectPointCloud(
      PointCPtr cloud,
      const geometry_msgs::Point &object_center);
  ObjectOrientationData determineObjectOrientation(
      PointCPtr object_cloud,
      const std::string &shape_type);
  
  // Methods for grasp planning and execution
  bool planAndExecuteGrasp(
      const geometry_msgs::Point &object_point,
      const ObjectOrientationData &orientation_data,
      const std::string &shape_type);
  bool planAndExecutePlace(const geometry_msgs::Point &goal_point);
  
  // Debug visualization methods
  void publishPointCloud(
      const PointCPtr &cloud,
      const ros::Publisher &publisher);
      
  void visualizePCAAxes(
      const Eigen::Matrix3f &eigenvectors,
      const geometry_msgs::Point &center_point,
      const std::string &shape_type);

  // Constructor and destructor
  ~cw2();

  PointCPtr current_object_cloud_;  // Store the current object cloud

private:
  // Euclidean clustering parameters
  float cluster_tolerance_;   // Distance threshold for clustering
  int min_cluster_size_;      // Minimum number of points in a cluster
  int max_cluster_size_;      // Maximum number of points in a cluster
};

#endif // end of include guard for cw2_CLASS_H_
