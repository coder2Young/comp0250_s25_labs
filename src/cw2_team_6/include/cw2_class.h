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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// TF specific includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
// standard c++ library includes (std::string, std::vector)
#include <string>
#include <vector>
// Add marker array for PCA visualization
#include <visualization_msgs/MarkerArray.h>

// OctoMap specific includes
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

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
  Eigen::Vector3f edge_direction;  // Optimal edge direction for grasping nought objects
  float grasp_angle;               // Computed angle for gripper rotation around Z
  bool is_valid;                   // Indicates if the orientation data is valid
  float flatness_ratio;
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

  // Color filtering function for point clouds
  PointCPtr filterPointCloudByColor(const PointCPtr& input_cloud);

  // Add floor collision object to planning scene
  void addFloorCollisionObject();

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  bool debug_ = false;

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitor planning_scene_monitor_{"robot_description"};

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
  ros::Publisher filtered_cloud_for_octomap_pub_; // For OctoMap to subscribe to

  // OctoMap related members
  ros::Subscriber octomap_sub_;
  ros::ServiceClient octomap_client_;
  octomap_msgs::Octomap latest_octomap_;
  bool octomap_received_;
  
  // Task 1 methods
  bool executeTask1(const cw2_world_spawner::Task1Service::Request &req, 
                    cw2_world_spawner::Task1Service::Response &res);
  
  // Methods for object orientation detection
  bool moveToScanPosition(const geometry_msgs::Point &target_point);
  
  // New methods for scanning motion and octomap handling
  void octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg);
  bool performScanningMotion(const geometry_msgs::Point &target_point);
  PointCPtr extractPointCloudFromOctomap();
  
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
      const std::string &shape_type,
      const Eigen::Vector3f &grasp_direction,
      float grasp_angle);
      
  // Task 2 methods
  bool determineShapeType(
      PointCPtr object_cloud,
      const geometry_msgs::Point &center_point);

  // Constructor and destructor
  ~cw2();

  PointCPtr current_object_cloud_;  // Store the current object cloud
  
  // Scanning motion parameters
  int num_scan_poses_;
  float scan_radius_;
  float scan_height_offset_;

  // Point cloud processing for Task 2
  PointCPtr getLatestPointCloud(const std::string& topic, const std::string& target_frame);
  PointCPtr processPointCloud(const PointCPtr& input_cloud);
  bool determineShapeTypeFromCamera(PointCPtr object_cloud, const geometry_msgs::Point &center_point);

  // Publisher for center point visualization
  ros::Publisher center_point_marker_pub_;

  // New Task 1 parameters
  bool t1_downsample_;       // Whether to apply downsampling in Task 1
  bool t1_move_constraint_;  // Whether to apply path constraints for grasping
  float t1_scan_height_;     // Height above object for scanning

  // Grasp visualization
  void visualizeGraspPoint(const geometry_msgs::Point &grasp_point, const tf2::Quaternion &orientation);
  ros::Publisher grasp_marker_pub_;

  // Grasping parameters
  float pick_lift_offset_;  // Height to lift objects after picking

  // Current grasp orientation for place operation
  geometry_msgs::Quaternion current_grasp_orientation_;

  // Current lift height for horizontal movement to place
  double current_lift_height_;

  // 添加抓取姿态计算函数声明
  geometry_msgs::PoseStamped calculateGraspPose(
      const Eigen::Vector4f& centroid,
      bool is_cross,
      const ObjectOrientationData& orientation);

private:
  // Euclidean clustering parameters
  float cluster_tolerance_;   // Distance threshold for clustering
  int min_cluster_size_;      // Minimum number of points in a cluster
  int max_cluster_size_;      // Maximum number of points in a cluster
  
  // Shape determination radius for Task 2
  float t2_shape_determine_radius_;  // Radius to check for center points in Task 2
  float t2_shape_determine_z_offset_; // Z offset for center point in Task 2
  int t2_shape_determine_min_points_; // Minimum number of points to be confident in Task 2

  // Task 3 helper methods
  PointCPtr scanSceneFromMultipleViewpoints();
  PointCPtr filterOutGreenFloor(const PointCPtr& cloud);
  PointCPtr extractBrownBasket(const PointCPtr& cloud);
  geometry_msgs::Point findBasketCenter(const PointCPtr& basket_cloud);
  PointCPtr extractBlackObstacles(const PointCPtr& cloud);
  void addObstaclesToPlanningScene(const PointCPtr& obstacles_cloud);
  PointCPtr extractGraspableObjects(const PointCPtr& cloud);
  bool clusterAndClassifyObjects(
      const PointCPtr& objects_cloud,
      std::vector<PointCPtr>& object_clusters,
      std::vector<bool>& is_cross_shape,
      std::vector<ObjectOrientationData>& object_orientations);
  bool graspAndPlaceObjectsOfType(
      const std::vector<PointCPtr>& object_clusters,
      const std::vector<bool>& is_cross_shape,
      const std::vector<ObjectOrientationData>& object_orientations,
      bool grasp_cross_shape,
      const geometry_msgs::Point& basket_center);

  // PCA轴可视化辅助函数
  visualization_msgs::MarkerArray createPCAAxesMarkers(
      const Eigen::Vector4f& centroid,
      const Eigen::Vector3f& primary_axis,
      const Eigen::Vector3f& secondary_axis,
      int id_offset,
      const std::string& shape_type);
  
  // 补充的可视化发布者
  ros::Publisher clusters_pub_;           // 为聚类后的对象点云
  ros::Publisher obstacles_cloud_pub_;    // 为障碍物点云
  ros::Publisher all_pca_axes_pub_;       // 为所有物体的 PCA 轴

  // Task 3 scanning and grasping parameters
  float t3_scan_height_;          // Height for scanning the scene in Task 3
  float t3_grasp_height_offset_;  // Z-offset to adjust grasp points upward

  // Continuous scanning parameters and state
  std::vector<PointCPtr> collected_clouds_;
  bool is_collecting_clouds_;
  int cloud_frame_counter_;
  int t3_pointcloud_save_interval_;  // Save every Nth frame
  float t3_continuous_scan_voxel_size_; // Voxel filter size during continuous scanning
  
  // New cloud callback and processing methods
  void continuousScanCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool isGreenPoint(const PointT& point);
  PointCPtr mergeClouds(const std::vector<PointCPtr>& clouds);

  // New continuous scanning method (different name to avoid redefinition)
  PointCPtr continuousScanSceneFromMultipleViewpoints();

  // Helper method for Cartesian path execution
  bool moveAlongCartesianPath(
      const std::vector<geometry_msgs::Pose>& waypoints,
      double eef_step,
      double jump_threshold,
      double speed_factor);

  // Point cloud subscriber - always active
  ros::Subscriber cloud_sub_;
};

#endif // end of include guard for cw2_CLASS_H_
