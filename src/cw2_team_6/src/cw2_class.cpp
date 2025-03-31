/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh):
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  collision_object_vector_(),
  octomap_received_(false),
  // Initialize scanning motion parameters
  scan_radius_(0.2),          // 20cm radius around object 
  scan_height_offset_(0.3),   // 30cm above object height
  num_scan_poses_(4),          // 4 positions around the object
  pick_lift_offset_(0.5),       // 0.5m lifting position after grasping
  arm_group_("panda_arm"),
  hand_group_("hand")
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);

  // Initialize visualization publishers for debugging with latched mode
  // The last "true" parameter enables latched mode - messages will persist for new subscribers
  cloud_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/cloud_filtered", 1, true);
  cloud_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/cloud_object", 1, true);
  pca_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/pca_axes", 1, true);
  // // Use for debug visualization, not for OctoMap input
  // filtered_cloud_for_octomap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/filtered_cloud", 1, true);
  // Add center point marker publisher
  center_point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug/center_point", 1, true);
  // Add grasp point visualization publisher
  grasp_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug/grasp_point", 1, true);
  
  // Initialize OctoMap-related subscribers and clients
  octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &cw2::octomap_callback, this);
  octomap_client_ = nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");

  planning_scene_monitor_.startSceneMonitor(); 
  planning_scene_monitor_.startWorldGeometryMonitor();  
  planning_scene_monitor_.startStateMonitor(); 

  cw2_config();
  
  ROS_INFO("cw2 class initialised");

  // Add this function to create and add a floor collision object
  addFloorCollisionObject();

  // Initialize additional visualization publishers
  clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/object_clusters", 1, true);
  obstacles_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/obstacles_cloud", 1, true);
  all_pca_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/all_pca_axes", 1, true);

  // Always subscribe to point cloud topic
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::continuousScanCloudCallback, this);
  
  ROS_INFO("Point cloud subscriber initialized");
  
  // Set the initial collection state to false
  is_collecting_clouds_ = false;
  cloud_frame_counter_ = 0;
}

cw2::~cw2() {
  // Cleanup if needed
}

void
cw2::cw2_config()
{
  /* function to configure the robot for the coursework */

  ROS_INFO("Configuring the robot for the coursework");

  // Enable debug mode
  debug_ = true;
  ROS_INFO("Debug mode is enabled");

  box_size_ = 0.04;
  basket_size_ = 0.1;
  hand_offset_ = 0.15; // 0.15 default
  gripper_open_ = 0.08;
  gripper_closed_ = 0.0;
  grasp_stanby_height_ = 0.1;
  place_stanby_height_ = 0.1;
  pick_lift_offset_ = 0.5; // 50cm higher position for lifting objects
  ground_length_ = 0.6;
  ground_width_ = 0.6;
  
  grasp_orientation_.x = 0.923953;
  grasp_orientation_.y = -0.382500;
  grasp_orientation_.z = -0.001784;
  grasp_orientation_.w = 0.000723;

  // Define a pose high enough to scan the whole scenario
  scan_pose_.position.x = 0.37;
  scan_pose_.position.y = 0.0;
  scan_pose_.position.z = 0.88;
  scan_pose_.orientation = grasp_orientation_;
  
  position_precision_ = 1000.0;
  box_basket_size_thresh_ = 900;
  cluster_color_thresh_ = 140;
  cluster_dist_thresh_ = 0.04;
  min_cluster_thresh_ = 200;

  scan_height_ = 0.7;
  
  // Euclidean clustering parameters
  cluster_tolerance_ = 0.05;    // 2cm tolerance between points in cluster
  min_cluster_size_ = 10;       // Minimum 50 points per cluster
  max_cluster_size_ = 25000;    // Maximum 25000 points per cluster
  
  // Scanning motion parameters
  num_scan_poses_ = 4;          // Number of scan poses around the object
  scan_radius_ = 0.1;          // Distance from object center (25 cm)
  scan_height_offset_ = 0.5;    // Height above the object (50 cm)

  // Added for Task 2 shape determination
  t2_shape_determine_radius_ = 0.01; // 40mm radius for center check
  //t2_shape_determine_z_offset_ = 0.02; // 20cm offset for center point
  t2_shape_determine_min_points_ = 10; // Minimum number of points to be confident in Task 2

  // Added for Task 1 direct point cloud approach
  t1_downsample_ = false;  // Turn off downsampling initially for better PCA
  t1_move_constraint_ = false;  // Enable movement constraints for better grasping
  t1_scan_height_ = 0.5;  // Height above object for scanning (50cm)

  // Add floor collision object to prevent collisions with the ground
  addFloorCollisionObject();

  // Task 3 specific parameters
  t3_scan_height_ = 0.65;           // Height for scanning the entire scene (lowered from 0.6)
  t3_grasp_height_offset_ = -0.05;    // Add 10cm to Z coordinate of all grasp points to compensate for low point cloud values

  // Continuous scanning parameters
  t3_pointcloud_save_interval_ = 10;     // Process every 10th frame
  t3_continuous_scan_voxel_size_ = 0.002; // 2mm voxel size for downsampling
  
  // Initialize scanning state
  is_collecting_clouds_ = false;
  cloud_frame_counter_ = 0;

  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* Task 1: Implementation for object grasping with orientation detection */

  ROS_INFO("\n\n====== TASK 1 STARTED ======\n");
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  if (debug_) {
    ROS_INFO("Debug mode is enabled - will provide extended logging");
  }

  // Extract required information from request
  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("====== TASK DETAILS ======");
  ROS_INFO("Object point: %f, %f, %f", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal point: %f, %f, %f", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape type: %s", shape_type.c_str());
  ROS_INFO("==========================\n");
  
  // 1. Move to scanning position above the object
  ROS_INFO("Moving to scanning position above the object...");
  geometry_msgs::PoseStamped scan_pose;
  scan_pose.header.frame_id = base_frame_;
  scan_pose.pose.position.x = object_point.point.x;
  scan_pose.pose.position.y = object_point.point.y;
  scan_pose.pose.position.z = object_point.point.z + t1_scan_height_; // 50cm above object
  scan_pose.pose.orientation = grasp_orientation_; // Using the default orientation looking down
  
  bool scan_success = moveArm(scan_pose);
  if (!scan_success) {
    ROS_ERROR("Failed to move to scanning position");
    return false;
  }
  
  // 2. Get point cloud from depth camera
  ROS_INFO("Acquiring point cloud from depth camera...");
  ros::Duration(1.0).sleep(); // Let camera stabilize
  
  PointCPtr camera_cloud = getLatestPointCloud("/r200/camera/depth_registered/points", base_frame_);
  if (camera_cloud->empty()) {
    ROS_ERROR("Failed to get point cloud data");
    return false;
  }
  
  // 3. Process point cloud 
  PointCPtr processed_cloud;
  
  if (t1_downsample_) {
    ROS_INFO("Processing point cloud with downsampling...");
    processed_cloud = processPointCloud(camera_cloud);
  } else {
    ROS_INFO("Processing point cloud without downsampling...");
    // Skip downsampling, only do color filtering and outlier removal
    PointCPtr color_filtered = filterPointCloudByColor(camera_cloud);
    
    pcl::StatisticalOutlierRemoval<PointT> sor;
    processed_cloud.reset(new PointC);
    sor.setInputCloud(color_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*processed_cloud);
  }
  
  // Publish processed cloud for visualization
  if (debug_) {
    publishPointCloud(processed_cloud, cloud_filtered_pub_);
  }
    // 5. Determine object orientation using PCA
  ObjectOrientationData orientation_data = determineObjectOrientation(processed_cloud, shape_type);
  
  if (!orientation_data.is_valid) {
    ROS_ERROR("Failed to determine object orientation");
    return false;
  }
  
  // 6. Plan and execute grasp
  bool grasp_success = planAndExecuteGrasp(object_point.point, orientation_data, shape_type);
  
  if (!grasp_success) {
    ROS_ERROR("Failed to execute grasp");
    return false;
  }
  
  // 7. Plan and execute place
  bool place_success = planAndExecutePlace(goal_point.point);
  
  if (!place_success) {
    ROS_ERROR("Failed to execute place operation");
    return false;
  }

  ROS_INFO("\n====== TASK 1 COMPLETED ======\n");
  return true;
}

bool cw2::moveToScanPosition(const geometry_msgs::Point &target_point) {
  ROS_INFO("Moving to scan position above the object");
  
  // Create scan position - above the object
  geometry_msgs::PoseStamped scan_pose;
  scan_pose.header.frame_id = base_frame_;
  scan_pose.pose.position.x = target_point.x;
  scan_pose.pose.position.y = target_point.y;
  scan_pose.pose.position.z = target_point.z + scan_height_;
  
  // Set camera to look down at the object
  scan_pose.pose.orientation = grasp_orientation_;
  
  // Move arm to scanning position
  ROS_INFO("Moving to scan position above the object at %f, %f, %f", scan_pose.pose.position.x, scan_pose.pose.position.y, scan_pose.pose.position.z);
  return moveArm(scan_pose);
}

ObjectOrientationData cw2::determineObjectOrientation(
    PointCPtr object_cloud,
    const std::string &shape_type) {
  
  ROS_INFO("\n====== DETERMINING OBJECT ORIENTATION ======");
  
  // Initialize result structure
  ObjectOrientationData result;
  result.is_valid = false;
  
  if (object_cloud->points.size() < 10) {
    ROS_ERROR("Not enough points for PCA analysis: %lu points", object_cloud->points.size());
    return result;
  }
  
  ROS_INFO("Performing PCA analysis on object cloud with %lu points", object_cloud->points.size());
  
  // Compute the centroid of the object first
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  
  // Remove points that might be from the ground plane using height filtering
  PointCPtr filtered_cloud(new PointC);
  float min_height = centroid[2] - 0.02; // 2cm below centroid
  
  for (const auto& point : object_cloud->points) {
    if (point.z > min_height) {
      filtered_cloud->points.push_back(point);
    }
  }
  
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  
  ROS_INFO("After height filtering: %lu points", filtered_cloud->points.size());
  
  // Recalculate centroid after filtering
  pcl::compute3DCentroid(*filtered_cloud, centroid);
  
  // Perform PCA on the filtered cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(filtered_cloud);
  
  // Get eigenvalues and eigenvectors
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  
  ROS_INFO("PCA eigenvalues: [%f, %f, %f]", 
           eigenvalues[0], eigenvalues[1], eigenvalues[2]);
  
  // Calculate eigenvalue ratios to better understand the shape
  float ratio_1_2 = eigenvalues[0] / eigenvalues[1];
  float ratio_2_3 = eigenvalues[1] / eigenvalues[2];
  
  ROS_INFO("Eigenvalue ratios - λ1/λ2: %f, λ2/λ3: %f", ratio_1_2, ratio_2_3);
  
  // Setup result variables
  Eigen::Vector3f primary_axis, secondary_axis;
  Eigen::Vector3f grasp_direction;
  float grasp_angle;
  
  if (shape_type == "cross") {
    ROS_INFO("Analyzing cross shape...");
    
    // For cross shape, the largest eigenvalue should be significantly larger
    if (ratio_1_2 < 1.5) {
      ROS_WARN("Cross shape expected but eigenvalue ratio λ1/λ2 = %f is low", ratio_1_2);
      ROS_WARN("This might affect grasp orientation accuracy");
    }
    
    // Primary axis is the direction of largest variance
    primary_axis = eigenvectors.col(0);
    
    // Make sure primary axis is in the XY plane (horizontal)
    primary_axis[2] = 0.0f;
    primary_axis.normalize();
    
    // Secondary axis is perpendicular to primary in the XY plane
    secondary_axis = Eigen::Vector3f(-primary_axis[1], primary_axis[0], 0.0f);
    
    // For cross, grasp along one arm
    grasp_direction = primary_axis;
    grasp_angle = atan2(primary_axis[1], primary_axis[0]); 
    
    ROS_INFO("Cross primary axis: [%f, %f, %f]", 
             primary_axis[0], primary_axis[1], primary_axis[2]);
    ROS_INFO("Cross grasp angle: %f degrees", grasp_angle * 180.0/M_PI);
  } 
  else { // "nought"
    ROS_INFO("Analyzing nought (ring) shape...");
    
    // For ring shape, the smallest eigenvalue should be much smaller (normal to the plane)
    if (ratio_2_3 < 1.5) {
      ROS_WARN("Ring shape expected but eigenvalue ratio λ2/λ3 = %f is low", ratio_2_3);
      ROS_WARN("This might affect grasp orientation accuracy");
    }
    
    // For nought, both major axes will be in the plane of the ring
    // We'll use the largest eigenvector as primary
    primary_axis = eigenvectors.col(0);
    
    // Ensure it's in the XY plane
    primary_axis[2] = 0.0f;
    primary_axis.normalize();
    
    // Second largest eigenvector as secondary axis
    secondary_axis = eigenvectors.col(1);
    secondary_axis[2] = 0.0f;
    secondary_axis.normalize();
    
    // For nought, prefer to grasp across the ring (not along the edge)
    // We can use the primary axis directly or rotate 45 degrees to align better with the rim
    grasp_direction = primary_axis;
    grasp_angle = atan2(primary_axis[1], primary_axis[0]);
    
    ROS_INFO("Nought primary axis: [%f, %f, %f]", 
             primary_axis[0], primary_axis[1], primary_axis[2]);
    ROS_INFO("Nought grasp angle: %f degrees", grasp_angle * 180.0/M_PI);
  }
  
  // Store results
  result.primary_axis = primary_axis;
  result.secondary_axis = secondary_axis;
  result.edge_direction = grasp_direction;
  result.grasp_angle = grasp_angle;
  result.is_valid = true;
  
  // Visualize PCA axes and grasp direction
  if (debug_) {
    geometry_msgs::Point center_point;
    center_point.x = centroid[0];
    center_point.y = centroid[1];
    center_point.z = centroid[2];
    
    visualizePCAAxes(eigenvectors, center_point, shape_type, grasp_direction, grasp_angle);
  }
  
  ROS_INFO("====== OBJECT ORIENTATION DETERMINATION COMPLETED ======\n");
  return result;
}

bool cw2::planAndExecuteGrasp(
    const geometry_msgs::Point &object_point,
    const ObjectOrientationData &orientation_data,
    const std::string &shape_type,
    float offset_override) {  // 添加可选的偏移值参数
  
  ROS_INFO("\n====== PLANNING AND EXECUTING GRASP ======");
  ROS_INFO("Object point: [%.4f, %.4f, %.4f]", 
           object_point.x, object_point.y, object_point.z);
  
  if (!orientation_data.is_valid) {
    ROS_ERROR("Invalid orientation data provided");
    return false;
  }
  
  ROS_INFO("PCA Analysis - Primary axis: [%.4f, %.4f, %.4f], Grasp angle: %.2f deg", 
           orientation_data.primary_axis[0], orientation_data.primary_axis[1], 
           orientation_data.primary_axis[2], orientation_data.grasp_angle * 180/M_PI);
  
  // Open gripper to prepare for grasp
  ROS_INFO("Opening gripper to width %.3f...", gripper_open_);
  bool open_success = moveGripper(gripper_open_, 2.0);
  if (!open_success) {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  
  // 0. Calculate grasp position with proper offsets
  geometry_msgs::PoseStamped grasp_standby_pose;
  geometry_msgs::PoseStamped grasp_pose;
  geometry_msgs::PoseStamped lift_pose;
  grasp_standby_pose.header.frame_id = base_frame_;
  grasp_pose.header.frame_id = base_frame_;
  lift_pose.header.frame_id = base_frame_;
  
  // Extract principal axis direction (in XY plane)
  Eigen::Vector3f principal_axis = orientation_data.primary_axis;
  Eigen::Vector3f secondary_axis = orientation_data.secondary_axis;
  Eigen::Vector3f grasp_direction_xy;
  
  ROS_INFO("Principal axis (XY): [%.4f, %.4f, %.4f]", 
           principal_axis[0], principal_axis[1], principal_axis[2]);
  ROS_INFO("Secondary axis (XY): [%.4f, %.4f, %.4f]", 
           secondary_axis[0], secondary_axis[1], secondary_axis[2]);
  
  // Calculate orientation quaternion
  tf2::Quaternion q_orig;
  tf2::convert(grasp_orientation_, q_orig);
  tf2::Quaternion q_rot;
  tf2::Quaternion q_final;
  
  ROS_INFO("Original grasp orientation: [%.4f, %.4f, %.4f, %.4f]", 
           grasp_orientation_.x, grasp_orientation_.y, 
           grasp_orientation_.z, grasp_orientation_.w);
  
  // Calculate offset grasp position based on shape type
  float grasp_x, grasp_y;
  float offset;
  
  if (shape_type == "cross") {
    ROS_INFO("Calculating grasp for cross shape...");
    
    // 使用传入的偏移值或默认值
    if (offset_override > 0.0) {
      offset = offset_override;
      ROS_INFO("Using custom offset value: %.4f m", offset);
    } else {
      // For cross shape, grasp one of the arms offset by 60mm from center
      offset = 0.06; // 60mm offset along principal axis
      ROS_INFO("Using default cross offset: %.4f m", offset);
    }
    
    // Calculate grasp point by offsetting along principal axis
    grasp_x = object_point.x + principal_axis[0] * offset;
    grasp_y = object_point.y + principal_axis[1] * offset;
    
    // Rotate around Z axis to align gripper with the cross arm
    q_rot.setRPY(0, 0, orientation_data.grasp_angle);
    ROS_INFO("Cross grasp - RPY angles: [0, 0, %.4f]", orientation_data.grasp_angle);
    
    q_final = q_rot * q_orig;
    
    ROS_INFO("Cross grasp point: [%.4f, %.4f] (offset by %.1fmm along principal axis)",
             grasp_x, grasp_y, offset * 1000.0);
    
  } else { // "nought"
    ROS_INFO("Calculating grasp for nought shape...");
    
    // For nought, find the midpoint angle between primary and secondary axes
    float primary_angle = atan2(principal_axis[1], principal_axis[0]);
    float secondary_angle = atan2(secondary_axis[1], secondary_axis[0]);
    
    ROS_INFO("Primary axis angle: %.4f rad (%.2f deg)", 
             primary_angle, primary_angle * 180/M_PI);
    ROS_INFO("Secondary axis angle: %.4f rad (%.2f deg)", 
             secondary_angle, secondary_angle * 180/M_PI);
    
    // Calculate midpoint angle (handling the circular nature of angles)
    float angle_diff = secondary_angle - primary_angle;
    if (angle_diff > M_PI) angle_diff -= 2*M_PI;
    if (angle_diff < -M_PI) angle_diff += 2*M_PI;
    
    float midpoint_angle = primary_angle + angle_diff/2.0;
    
    ROS_INFO("Angle difference: %.4f rad, Midpoint angle: %.4f rad (%.2f deg)",
             angle_diff, midpoint_angle, midpoint_angle * 180/M_PI);
    
    // Calculate grasp direction using the midpoint angle
    grasp_direction_xy[0] = cos(midpoint_angle);
    grasp_direction_xy[1] = sin(midpoint_angle);
    grasp_direction_xy[2] = 0.0;
    
    ROS_INFO("Grasp direction vector: [%.4f, %.4f, %.4f]",
             grasp_direction_xy[0], grasp_direction_xy[1], grasp_direction_xy[2]);
    
    // 使用传入的偏移值或默认值
    if (offset_override > 0.0) {
      offset = offset_override;
      ROS_INFO("Using custom offset value: %.4f m", offset);
    } else {
      // Use 80mm offset along this direction
      offset = 0.08; // 80mm offset
      ROS_INFO("Using default nought offset: %.4f m", offset);
    }
    
    // Calculate grasp point by offsetting along the midpoint direction
    grasp_x = object_point.x + grasp_direction_xy[0] * offset;
    grasp_y = object_point.y + grasp_direction_xy[1] * offset;
    
    // Add 90 degrees rotation around Z axis to the midpoint angle
    float grasp_angle = midpoint_angle + M_PI/2.0; // Add 90 degrees
    ROS_INFO("Nought grasp angle: %.4f rad (%.2f deg) = midpoint + 90°", 
             grasp_angle, grasp_angle * 180/M_PI);
    
    q_rot.setRPY(0, 0, grasp_angle);
    q_final = q_rot * q_orig;
    
    ROS_INFO("Nought grasp point: [%.4f, %.4f] (offset by %.1fmm along midpoint direction)",
             grasp_x, grasp_y, offset * 1000.0);
  }
  
  // Normalize quaternion
  q_final.normalize();
  
  ROS_INFO("Final grasp orientation (quaternion): [%.4f, %.4f, %.4f, %.4f]",
           q_final.x(), q_final.y(), q_final.z(), q_final.w());
  
  // Apply hand_offset_ to grasp position and set all positions
  // Grasp position (with hand_offset_)
  grasp_pose.pose.position.x = grasp_x;
  grasp_pose.pose.position.y = grasp_y;
  grasp_pose.pose.position.z = object_point.z + hand_offset_;
  grasp_pose.pose.orientation = tf2::toMsg(q_final);
  
  ROS_INFO("Final grasp pose: [%.4f, %.4f, %.4f] with Z-offset: %.4f",
           grasp_pose.pose.position.x, grasp_pose.pose.position.y, 
           grasp_pose.pose.position.z, hand_offset_);
  
  // Standby position (grasp_stanby_height_ above grasp position)
  grasp_standby_pose.pose.position.x = grasp_x;
  grasp_standby_pose.pose.position.y = grasp_y;
  grasp_standby_pose.pose.position.z = grasp_pose.pose.position.z + grasp_stanby_height_;
  grasp_standby_pose.pose.orientation = grasp_pose.pose.orientation;
  
  ROS_INFO("Grasp standby pose: [%.4f, %.4f, %.4f] (%.4f above grasp position)",
           grasp_standby_pose.pose.position.x, 
           grasp_standby_pose.pose.position.y,
           grasp_standby_pose.pose.position.z,
           grasp_stanby_height_);
  
  // Lift position (pick_lift_offset_ above ground)
  lift_pose.pose.position.x = grasp_x;
  lift_pose.pose.position.y = grasp_y;
  lift_pose.pose.position.z = object_point.z + pick_lift_offset_;
  lift_pose.pose.orientation = grasp_pose.pose.orientation;
  
  ROS_INFO("Lift pose: [%.4f, %.4f, %.4f] (%.4f above ground)",
           lift_pose.pose.position.x, 
           lift_pose.pose.position.y,
           lift_pose.pose.position.z,
           pick_lift_offset_);
  
  // Store the final grasp orientation for place operation
  current_grasp_orientation_ = grasp_pose.pose.orientation;
  // Store the lift height for horizontal movement to place
  current_lift_height_ = lift_pose.pose.position.z;
  
  ROS_INFO("Stored lift height for place operation: %.4f", current_lift_height_);
  
  // Visualize grasp point and orientation if in debug mode
  if (debug_) {
    visualizeGraspPoint(grasp_pose.pose.position, q_final);
    ROS_INFO("Grasp visualization markers published");
  }
  
  // Step 1: Move to grasp standby position
  ROS_INFO("Moving to grasp standby position...");
  bool standby_success = moveArm(grasp_standby_pose);
  if (!standby_success) {
    ROS_ERROR("Failed to move to grasp standby position");
    return false;
  }
  ROS_INFO("Successfully moved to grasp standby position");
  
  // Step 2: Vertically move down to grasp position with constraints
  ROS_INFO("Setting up vertical path constraints for grasp approach...");
  
  // Add path constraint for vertical approach
  moveit_msgs::Constraints constraints;
  moveit_msgs::OrientationConstraint ocm;
  ocm.header.frame_id = base_frame_;
  ocm.link_name = arm_group_.getEndEffectorLink();
  ocm.orientation = grasp_standby_pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = 0.1; // Very strict
  ocm.absolute_y_axis_tolerance = 0.1; // Very strict
  ocm.absolute_z_axis_tolerance = 0.1; // Very strict
  ocm.weight = 1.0;
  
  // Add position constraint to only allow Z-axis movement
  moveit_msgs::PositionConstraint pcm;
  pcm.header.frame_id = base_frame_;
  pcm.link_name = arm_group_.getEndEffectorLink();
  
  // Create a box constraint that only allows movement in Z direction
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.002; // Small tolerance in X
  box.dimensions[1] = 0.002; // Small tolerance in Y
  box.dimensions[2] = 10;   // Allow movement in Z
  
  // Set the box position to allow vertical movement
  geometry_msgs::Pose box_pose;
  box_pose.position.x = grasp_standby_pose.pose.position.x;
  box_pose.position.y = grasp_standby_pose.pose.position.y;
  box_pose.position.z = (grasp_standby_pose.pose.position.z + grasp_pose.pose.position.z) / 2.0;
  box_pose.orientation.w = 1.0;
  
  ROS_INFO("Path constraint box center: [%.4f, %.4f, %.4f]",
           box_pose.position.x, box_pose.position.y, box_pose.position.z);
  
  pcm.constraint_region.primitives.push_back(box);
  pcm.constraint_region.primitive_poses.push_back(box_pose);
  pcm.weight = 1.0;
  
  constraints.orientation_constraints.push_back(ocm);
  constraints.position_constraints.push_back(pcm);
  
  arm_group_.setPathConstraints(constraints);
  
  // Try with constraints
  ROS_INFO("Moving down to grasp position with vertical constraints...");
  bool grasp_approach_success = moveArm(grasp_pose);
  
  // Clear constraints for future movements
  arm_group_.clearPathConstraints();
  ROS_INFO("Path constraints cleared");
  
  if (!grasp_approach_success) {
    ROS_ERROR("Failed to move to grasp position");
    return false;
  }
  ROS_INFO("Successfully moved to grasp position");
  
  // Step 3: Close gripper to grasp object
  ROS_INFO("Closing gripper to grasp object (width: %.4f)...", gripper_closed_);
  bool close_success = moveGripper(gripper_closed_, 2.0);
  if (!close_success) {
    ROS_ERROR("Failed to close gripper");
    return false;
  }
  ROS_INFO("Gripper closed successfully");
  
  // Step 4: Lift object to higher position
  ROS_INFO("Lifting object to travel height %.4f...", lift_pose.pose.position.z);
  bool lift_success = moveArm(lift_pose);
  if (!lift_success) {
    ROS_ERROR("Failed to move to lifting position");
    return false;
  }
  ROS_INFO("Object lifted successfully to travel height");
  
  ROS_INFO("====== GRASP EXECUTION COMPLETED ======\n");
  return true;
}

bool cw2::planAndExecutePlace(const geometry_msgs::Point &place_point) {
  ROS_INFO("\n====== PLANNING AND EXECUTING PLACE ======");
  ROS_INFO("Place target point: [%.4f, %.4f, %.4f]", 
           place_point.x, place_point.y, place_point.z);
  
  // Step 0: Calculate place positions
  geometry_msgs::PoseStamped place_standby_pose;
  geometry_msgs::PoseStamped horizontal_move_pose;
  
  place_standby_pose.header.frame_id = base_frame_;
  horizontal_move_pose.header.frame_id = base_frame_;
  
  // Place standby position (hand_offset_ + place_stanby_height_ above place position)
  // This is where we'll release the object
  place_standby_pose.pose.position.x = place_point.x;
  place_standby_pose.pose.position.y = place_point.y;
  place_standby_pose.pose.position.z = place_point.z + hand_offset_ + place_stanby_height_;
  place_standby_pose.pose.orientation = current_grasp_orientation_;
  
  ROS_INFO("Place standby pose: [%.4f, %.4f, %.4f] (%.4f + %.4f above place point)",
           place_standby_pose.pose.position.x, 
           place_standby_pose.pose.position.y,
           place_standby_pose.pose.position.z,
           hand_offset_, place_stanby_height_);
  
  // Horizontal move position (same Z as current lift height)
  horizontal_move_pose.pose.position.x = place_point.x;
  horizontal_move_pose.pose.position.y = place_point.y;
  horizontal_move_pose.pose.position.z = current_lift_height_;
  horizontal_move_pose.pose.orientation = current_grasp_orientation_;
  
  ROS_INFO("Horizontal move pose: [%.4f, %.4f, %.4f] (at lift height: %.4f)",
           horizontal_move_pose.pose.position.x, 
           horizontal_move_pose.pose.position.y,
           horizontal_move_pose.pose.position.z,
           current_lift_height_);
  
  ROS_INFO("Using current grasp orientation for place: [%.4f, %.4f, %.4f, %.4f]",
           current_grasp_orientation_.x, current_grasp_orientation_.y,
           current_grasp_orientation_.z, current_grasp_orientation_.w);
  
  // Step 1: Horizontal move to above place position (keeping Z at lift height)
  ROS_INFO("Moving horizontally to position above place point...");
  bool horizontal_move_success = moveArm(horizontal_move_pose);
  if (!horizontal_move_success) {
    ROS_ERROR("Failed to move horizontally to place area");
    return false;
  }
  ROS_INFO("Successfully moved to position above place point");
  
  // Step 2: Vertically move down to place standby position with constraints
  ROS_INFO("Setting up vertical path constraints for place approach...");
  
  // Add path constraint for vertical approach
  moveit_msgs::Constraints constraints;
  moveit_msgs::OrientationConstraint ocm;
  ocm.header.frame_id = base_frame_;
  ocm.link_name = arm_group_.getEndEffectorLink();
  ocm.orientation = current_grasp_orientation_;
  ocm.absolute_x_axis_tolerance = 0.01; // Very strict
  ocm.absolute_y_axis_tolerance = 0.01; // Very strict
  ocm.absolute_z_axis_tolerance = 0.01; // Very strict
  ocm.weight = 1.0;
  
  // Add position constraint to only allow Z-axis movement
  moveit_msgs::PositionConstraint pcm;
  pcm.header.frame_id = base_frame_;
  pcm.link_name = arm_group_.getEndEffectorLink();
  
  // Create a box constraint that only allows movement in Z direction
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 0.002; // Small tolerance in X
  box.dimensions[1] = 0.002; // Small tolerance in Y
  box.dimensions[2] = 1.0;   // Allow movement in Z
  
  // Set the box position to allow vertical movement
  geometry_msgs::Pose box_pose;
  box_pose.position.x = place_point.x;
  box_pose.position.y = place_point.y;
  box_pose.position.z = (horizontal_move_pose.pose.position.z + place_standby_pose.pose.position.z) / 2.0;
  box_pose.orientation.w = 1.0;
  
  ROS_INFO("Place path constraint box center: [%.4f, %.4f, %.4f]",
           box_pose.position.x, box_pose.position.y, box_pose.position.z);
  
  pcm.constraint_region.primitives.push_back(box);
  pcm.constraint_region.primitive_poses.push_back(box_pose);
  pcm.weight = 1.0;
  
  constraints.orientation_constraints.push_back(ocm);
  constraints.position_constraints.push_back(pcm);
  
  arm_group_.setPathConstraints(constraints);
  
  // Try with constraints to move to place standby
  ROS_INFO("Moving down to place standby position with vertical constraints...");
  bool place_standby_success = moveArm(place_standby_pose);
  
  // Clear constraints
  arm_group_.clearPathConstraints();
  ROS_INFO("Place path constraints cleared");
  
  if (!place_standby_success) {
    ROS_WARN("Failed to move to place standby with constraints, trying without constraints");
    place_standby_success = moveArm(place_standby_pose);
    if (!place_standby_success) {
      ROS_ERROR("Failed to move to place standby position");
      return false;
    }
  }
  ROS_INFO("Successfully moved to place standby position");
  
  // Step 3: Open gripper to release object at standby position
  ROS_INFO("Opening gripper to release object (width: %.4f)...", gripper_open_);
  bool open_success = moveGripper(gripper_open_, 2.0);
  if (!open_success) {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  ROS_INFO("Gripper opened successfully");
  
  // Pause briefly to allow object to fall
  ROS_INFO("Pausing for 0.5 seconds to allow object to fall...");
  ros::Duration(0.5).sleep();
  
  ROS_INFO("====== PLACE EXECUTION COMPLETED ======\n");
  return true;
}

// Publish point cloud for visualization
void cw2::publishPointCloud(const PointCPtr &cloud, const ros::Publisher &publisher) {
  if (cloud->empty()) {
    ROS_WARN("Cannot publish empty point cloud");
    return;
  }
  
  // Convert to ROS message
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  
  // Make sure the frame_id is preserved
  if (cloud->header.frame_id.empty()) {
    cloud_msg.header.frame_id = base_frame_;
    ROS_WARN("Point cloud has no frame_id, using %s as default", base_frame_.c_str());
  } else {
    cloud_msg.header.frame_id = cloud->header.frame_id;
    ROS_INFO("Publishing point cloud with frame_id: %s", cloud_msg.header.frame_id.c_str());
  }
  
  // Set the timestamp
  cloud_msg.header.stamp = ros::Time::now();
  
  // Publish
  publisher.publish(cloud_msg);
  ROS_INFO("Published point cloud with %lu points", cloud->points.size());
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* Task 2: Implementation for shape recognition between cross and nought shapes */

  ROS_INFO("\n\n====== TASK 2 STARTED ======\n");
  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // Extract required information from request
  std::vector<geometry_msgs::PointStamped> ref_object_points = request.ref_object_points;
  geometry_msgs::PointStamped mystery_object_point = request.mystery_object_point;

  // The position of the reference objects and the mystery object are not very accurate,
  // so we need to use the centroid of the point cloud to determine the shape type.
  // These position are only used for scan
  ROS_INFO("====== TASK DETAILS ======");
  ROS_INFO("Number of reference objects: %lu", ref_object_points.size());
  ROS_INFO("Reference object 1 position: [%f, %f, %f]", 
           ref_object_points[0].point.x, ref_object_points[0].point.y, ref_object_points[0].point.z);
  ROS_INFO("Reference object 2 position: [%f, %f, %f]", 
           ref_object_points[1].point.x, ref_object_points[1].point.y, ref_object_points[1].point.z);
  ROS_INFO("Mystery object position: [%f, %f, %f]", 
           mystery_object_point.point.x, mystery_object_point.point.y, mystery_object_point.point.z);
  ROS_INFO("==========================\n");
  
  // Vector to store shape types (true for cross, false for nought)
  std::vector<bool> is_cross_shape;

  // Process all objects (2 reference objects + 1 mystery object)
  std::vector<geometry_msgs::PointStamped> all_objects = ref_object_points;
  all_objects.push_back(mystery_object_point);

  for (size_t i = 0; i < all_objects.size(); i++) {
    std::string object_name = (i < ref_object_points.size()) ? 
                              "Reference object " + std::to_string(i+1) : 
                              "Mystery object";
    
    ROS_INFO("\n====== PROCESSING %s ======", object_name.c_str());
    
    // 1. Move to scanning position above the object
    geometry_msgs::PoseStamped scan_pose;
    scan_pose.header.frame_id = base_frame_;
    scan_pose.pose.position.x = all_objects[i].point.x;
    scan_pose.pose.position.y = all_objects[i].point.y;
    scan_pose.pose.position.z = all_objects[i].point.z + 0.5; // 50cm above object
    scan_pose.pose.orientation = grasp_orientation_;
    
    ROS_INFO("Moving to scan position above %s", object_name.c_str());
    bool scan_success = moveArm(scan_pose);
    
    if (!scan_success) {
      ROS_ERROR("Failed to move to scan position for %s", object_name.c_str());
      continue;
    }
    
    // 2. Wait for camera to stabilize and collect point cloud
    ROS_INFO("Waiting for point cloud data...");
    ros::Duration(1.0).sleep(); // Wait for arm to stabilize
    
    // Get fresh point cloud from camera using waitForMessage (implemented in getLatestPointCloud)
    PointCPtr camera_cloud = getLatestPointCloud("/r200/camera/depth_registered/points", base_frame_);
    
    if (camera_cloud->empty()) {
      ROS_ERROR("Failed to get point cloud for %s", object_name.c_str());
      continue;
    }
    
    ROS_INFO("Received point cloud with %lu points", camera_cloud->points.size());
    
    // 3. Process point cloud (downsample, color filter)
    PointCPtr filtered_cloud = processPointCloud(camera_cloud);
    
    // Publish filtered cloud for visualization
    if (debug_) {
      publishPointCloud(filtered_cloud, cloud_filtered_pub_);
    }
    
    // 4. Determine shape from filtered cloud
    bool is_cross = determineShapeTypeFromCamera(filtered_cloud, all_objects[i].point);
    is_cross_shape.push_back(is_cross);
    
    ROS_INFO("%s is a %s shape", object_name.c_str(), is_cross ? "CROSS" : "NOUGHT");
  }

  // Determine which reference object matches the mystery object
  int mystery_object_num = 0;
  if (is_cross_shape.size() == 3) {  // Ensure we have all three shapes
    if (is_cross_shape[2] == is_cross_shape[0]) {
      // Mystery object matches reference object 1
      mystery_object_num = 1;
    } else if (is_cross_shape[2] == is_cross_shape[1]) {
      // Mystery object matches reference object 2
      mystery_object_num = 2;
    } else {
      ROS_ERROR("Mystery object doesn't match either reference object. Defaulting to object 1.");
      mystery_object_num = 1;
    }
  } else {
    ROS_ERROR("Failed to determine all object shapes. Defaulting to object 1.");
    mystery_object_num = 1;
  }

  // Set the response
  response.mystery_object_num = mystery_object_num;
  
  ROS_INFO("\n=================TASK 2 RESULT=================");
  ROS_INFO("The mystery object matches reference object %d", mystery_object_num);
  
  // Print the shapes of all objects more clearly
  if (is_cross_shape.size() >= 3) {
    ROS_INFO("Reference object 1 shape: %s", is_cross_shape[0] ? "CROSS" : "NOUGHT");
    ROS_INFO("Reference object 2 shape: %s", is_cross_shape[1] ? "CROSS" : "NOUGHT");
    ROS_INFO("Mystery object shape: %s", is_cross_shape[2] ? "CROSS" : "NOUGHT");
  }
  ROS_INFO("================================================");
  
  // Move back to a neutral position
  if (debug_) {
    geometry_msgs::PoseStamped neutral_pose;
    neutral_pose.header.frame_id = base_frame_;
    neutral_pose.pose = scan_pose_; // Using the predefined scan pose as neutral position
    moveArm(neutral_pose);
  }

  ROS_INFO("\n====== TASK 2 COMPLETED ======\n");
  return true;
}

// Modified shape determination to use point cloud centroid instead of message-provided center point
bool cw2::determineShapeTypeFromCamera(PointCPtr cloud, const geometry_msgs::Point &center_point) {
  ROS_INFO("\n====== DETERMINING SHAPE TYPE FROM CAMERA ======");
  
  // If cloud is empty, can't make determination
  if (cloud->empty()) {
    ROS_WARN("Point cloud is empty, defaulting to cross shape");
    return true;
  }
  
  // Skip spatial filtering and use the filtered cloud directly
  ROS_INFO("Using filtered cloud with %lu points for shape determination", cloud->points.size());
  
  // Compute centroid of the point cloud
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  
  ROS_INFO("Point cloud centroid: [%f, %f, %f]", centroid[0], centroid[1], centroid[2]);
  ROS_INFO("Message-provided center: [%f, %f, %f]", center_point.x, center_point.y, center_point.z);
  
  // Create KdTree for nearest neighbor search
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  
  // Create point for center using the centroid
  PointT center_search_point;
  center_search_point.x = centroid[0];
  center_search_point.y = centroid[1];
  center_search_point.z = centroid[2];
  
  // Visualize both centers as markers - green for message center, blue for centroid
  if (debug_) {
    // First, visualize the message-provided center in green
    visualization_msgs::Marker msg_marker;
    msg_marker.header.frame_id = base_frame_;
    msg_marker.header.stamp = ros::Time::now();
    msg_marker.ns = "center_points";
    msg_marker.id = 0;
    msg_marker.type = visualization_msgs::Marker::SPHERE;
    msg_marker.action = visualization_msgs::Marker::ADD;
    
    msg_marker.pose.position = center_point;
    msg_marker.pose.orientation.w = 1.0;
    
    msg_marker.scale.x = t2_shape_determine_radius_ * 2.0;
    msg_marker.scale.y = t2_shape_determine_radius_ * 2.0;
    msg_marker.scale.z = t2_shape_determine_radius_ * 2.0;
    
    msg_marker.color.r = 0.0;
    msg_marker.color.g = 1.0;
    msg_marker.color.b = 0.0;
    msg_marker.color.a = 0.5;
    
    msg_marker.lifetime = ros::Duration(0);
    center_point_marker_pub_.publish(msg_marker);
    
    // Second, visualize the computed centroid in blue
    visualization_msgs::Marker centroid_marker;
    centroid_marker.header.frame_id = base_frame_;
    centroid_marker.header.stamp = ros::Time::now();
    centroid_marker.ns = "center_points";
    centroid_marker.id = 1;  // Different ID from the message marker
    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    centroid_marker.action = visualization_msgs::Marker::ADD;
    
    centroid_marker.pose.position.x = centroid[0];
    centroid_marker.pose.position.y = centroid[1];
    centroid_marker.pose.position.z = centroid[2];
    centroid_marker.pose.orientation.w = 1.0;
    
    centroid_marker.scale.x = t2_shape_determine_radius_ * 2.0;
    centroid_marker.scale.y = t2_shape_determine_radius_ * 2.0;
    centroid_marker.scale.z = t2_shape_determine_radius_ * 2.0;
    
    centroid_marker.color.r = 0.0;
    centroid_marker.color.g = 0.0;
    centroid_marker.color.b = 1.0;
    centroid_marker.color.a = 0.5;
    
    centroid_marker.lifetime = ros::Duration(0);
    center_point_marker_pub_.publish(centroid_marker);
    
    ROS_INFO("Published center markers - Green: message center, Blue: point cloud centroid");
  }
  
  // Find points within radius of the centroid
  std::vector<int> point_indices;
  std::vector<float> point_distances;
  int found = tree->radiusSearch(center_search_point, t2_shape_determine_radius_, point_indices, point_distances);
  
  ROS_INFO("Found %d points within %f m of centroid", found, t2_shape_determine_radius_);
  
  // If points are found in the center, it's a cross shape
  // If no points are found in the center, it's a nought (O) shape
  bool is_cross = (found > t2_shape_determine_min_points_);
  
  ROS_INFO("Shape determination from camera: %s", 
           is_cross ? "CROSS (center is occupied)" : "NOUGHT (center is empty)");
  
  return is_cross;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* Task 3: Implementation for shape recognition, counting, and selective grasping */
  
  ROS_INFO("\n\n====== TASK 3 STARTED ======\n");
  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  
  if (debug_) {
    ROS_INFO("Debug mode is enabled - will provide extended logging");
  }
  
  // Variables to store results
  int total_num_shapes = 0;
  int num_cross_shapes = 0;
  int num_nought_shapes = 0;
  
  // 1. Scan the scene from multiple viewpoints and merge the point clouds
  ROS_INFO("====== SCANNING SCENE FROM MULTIPLE VIEWPOINTS ======");
  // Use new continuous scanning method instead of the original
  PointCPtr merged_cloud = continuousScanSceneFromMultipleViewpoints();
  
  if (merged_cloud->empty()) {
    ROS_ERROR("Failed to get valid point cloud data from scanning");
    return false;
  }
  
  // 3. Extract brown basket for placement
  PointCPtr basket_cloud = extractBrownBasket(merged_cloud);
  geometry_msgs::Point basket_center = findBasketCenter(basket_cloud);
  ROS_INFO("Basket center found at: [%f, %f, %f]", basket_center.x, basket_center.y, basket_center.z);
  
  // 4. Extract black obstacles for collision avoidance
  PointCPtr obstacles_cloud = extractBlackObstacles(merged_cloud);
  addObstaclesToPlanningScene(obstacles_cloud);
  
  // 5. Extract the remaining colored objects (red, blue, purple)
  PointCPtr objects_cloud = extractGraspableObjects(merged_cloud);
  publishPointCloud(objects_cloud, cloud_object_pub_);
  
  // 6. Cluster the objects and determine their shapes
  std::vector<PointCPtr> object_clusters;
  std::vector<bool> is_cross_shape;
  std::vector<ObjectOrientationData> object_orientations;
  
  bool clustering_success = clusterAndClassifyObjects(
      objects_cloud, object_clusters, is_cross_shape, object_orientations);
  
  if (!clustering_success) {
    ROS_ERROR("Failed to cluster and classify objects");
    return false;
  }
  
  // Count the objects and shapes
  total_num_shapes = object_clusters.size();
  for (bool is_cross : is_cross_shape) {
    if (is_cross) {
      num_cross_shapes++;
    } else {
      num_nought_shapes++;
    }
  }
  
  // Print the results
  ROS_INFO("\n====== OBJECT COUNTING RESULTS ======");
  ROS_INFO("Total number of objects: %d", total_num_shapes);
  ROS_INFO("Number of cross shapes: %d", num_cross_shapes);
  ROS_INFO("Number of nought shapes: %d", num_nought_shapes);
  
  // 7. Determine which shape is more common and grasp all objects of that shape
  bool grasp_cross_shape = (num_cross_shapes >= num_nought_shapes);
  int num_most_common_shape = grasp_cross_shape ? num_cross_shapes : num_nought_shapes;
  
  ROS_INFO("Most common shape: %s (Count: %d)", 
           grasp_cross_shape ? "CROSS" : "NOUGHT", num_most_common_shape);
  
  // 8. Grasp and place all objects of the more common shape
  bool grasp_success = graspAndPlaceObjectsOfType(
      object_clusters, is_cross_shape, object_orientations, grasp_cross_shape, basket_center);
  
  if (!grasp_success) {
    ROS_WARN("Some objects could not be grasped and placed");
    // Continue with the task even if some objects failed
  }
  
  // Set the response values
  response.total_num_shapes = total_num_shapes;
  response.num_most_common_shape = num_most_common_shape;
  
  ROS_INFO("\n====== TASK 3 COMPLETED ======");
  ROS_INFO("Total shapes: %d, Most common shape count: %d", 
           total_num_shapes, num_most_common_shape);
  
  return true;
}

// Scan the scene from multiple viewpoints and merge the point clouds
PointCPtr cw2::scanSceneFromMultipleViewpoints() {
  ROS_INFO("Starting scene scanning from multiple viewpoints...");
  
  // Define 8 scanning positions
  std::vector<geometry_msgs::PoseStamped> scan_poses;
  
  // Define rectangle corners and midpoints (X=+-0.5 Y=+-0.4 Z=0.6)
  std::vector<std::pair<float, float>> scan_xy_positions = {
    {0.45, -0.35},   // Corner 1
    {0.45, 0.0},    // Midpoint of edge 1
    {0.45, 0.35},    // Corner 2
    {0.0, 0.35},    // Midpoint of edge 2
    {-0.45, 0.35},   // Corner 3
    {-0.45, 0.0},   // Midpoint of edge 3
    {-0.45, -0.35},  // Corner 4
    {0.0, -0.35}    // Midpoint of edge 4
  };
  
  // Prepare scan positions
  for (const auto& xy : scan_xy_positions) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.position.z = t3_scan_height_;
    
    // Special orientation for positions at y=0: rotate 90 degrees around Z axis
    if (fabs(xy.first) < 0.001) { // Check if y is approximately 0
      // Create a quaternion for 90-degree rotation around Z axis
      tf2::Quaternion q_base, q_rot, q_final;
      tf2::convert(grasp_orientation_, q_base);
      q_rot.setRPY(0, 0, M_PI/2); // 90 degrees in radians
      q_final = q_rot * q_base;
      q_final.normalize();
      
      // Convert back to geometry_msgs
      pose.pose.orientation = tf2::toMsg(q_final);
      ROS_INFO("Applied 90-degree Z rotation for scanning at y=0 position [%f, %f]", 
               xy.first, xy.second);
    } else {
      // Use standard orientation for other positions
      pose.pose.orientation = grasp_orientation_;
    }
    
    scan_poses.push_back(pose);
  }
  
  // Vector to hold all collected point clouds
  std::vector<PointCPtr> collected_clouds;
  
  // Move to each scan position and collect point cloud
  for (size_t i = 0; i < scan_poses.size(); i++) {
    ROS_INFO("Moving to scan position %zu/%zu [%f, %f, %f]", 
             i+1, scan_poses.size(), 
             scan_poses[i].pose.position.x,
             scan_poses[i].pose.position.y,
             scan_poses[i].pose.position.z);
    
    // Move arm to scan position
    bool move_success = moveArm(scan_poses[i]);
    if (!move_success) {
      ROS_WARN("Failed to move to scan position %zu, skipping", i+1);
      continue;
    }
    
    // Wait for arm to stabilize
    ros::Duration(1.0).sleep();
    
    // Get point cloud from depth camera
    PointCPtr cloud = getLatestPointCloud("/r200/camera/depth_registered/points", base_frame_);
    
    if (cloud->empty()) {
      ROS_WARN("Received empty point cloud at position %zu, skipping", i+1);
      continue;
    }
    
    ROS_INFO("Got point cloud with %lu points at position %zu", cloud->points.size(), i+1);
    
    // Apply voxel grid downsampling to 1mm resolution
    pcl::VoxelGrid<PointT> voxel_filter;
    PointCPtr downsampled_cloud(new PointC);
    
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);  // 1mm voxel size
    voxel_filter.filter(*downsampled_cloud);
    
    ROS_INFO("Downsampled to %lu points (1mm resolution)", downsampled_cloud->points.size());
    
    // Add to collected clouds
    collected_clouds.push_back(downsampled_cloud);
  }
  
  // Handle the case if no clouds were collected
  if (collected_clouds.empty()) {
    ROS_ERROR("Failed to collect any valid point clouds from scanning positions");
    return PointCPtr(new PointC);
  }
  
  // Merge all collected clouds
  PointCPtr merged_cloud(new PointC);
  
  for (const auto& cloud : collected_clouds) {
    *merged_cloud += *cloud;
  }
  
  ROS_INFO("Merged point cloud has %lu points from %zu scan positions", 
           merged_cloud->points.size(), collected_clouds.size());
  
  // Apply statistical outlier removal to clean up the merged cloud
  pcl::StatisticalOutlierRemoval<PointT> sor;
  PointCPtr cleaned_cloud(new PointC);
  
  sor.setInputCloud(merged_cloud);
  sor.setMeanK(50);             // Consider 50 neighbors
  sor.setStddevMulThresh(1.0);  // Standard deviation threshold
  sor.filter(*cleaned_cloud);
  
  ROS_INFO("Final merged and cleaned point cloud has %lu points", cleaned_cloud->points.size());
  
  return cleaned_cloud;
}

// Filter out the green floor from the point cloud
PointCPtr cw2::filterOutGreenFloor(const PointCPtr& cloud) {
  ROS_INFO("Filtering out green floor from point cloud with %lu points", cloud->points.size());
  
  PointCPtr non_floor_cloud(new PointC);
  
  // Green floor color thresholds (RGB)
  float r_min = 0.0f, r_max = 0.2f;
  float g_min = 0.7f, g_max = 1.0f;
  float b_min = 0.0f, b_max = 0.2f;
  
  for (const auto& point : cloud->points) {
    // Normalize RGB values to 0-1 range
    float r = point.r / 255.0f;
    float g = point.g / 255.0f;
    float b = point.b / 255.0f;
    
    // Check if point is NOT green floor
    bool is_green = (r >= r_min && r <= r_max) &&
                    (g >= g_min && g <= g_max) &&
                    (b >= b_min && b <= b_max);
    
    if (!is_green) {
      non_floor_cloud->points.push_back(point);
    }
  }
  
  non_floor_cloud->width = non_floor_cloud->points.size();
  non_floor_cloud->height = 1;
  non_floor_cloud->is_dense = true;
  non_floor_cloud->header = cloud->header;
  
  ROS_INFO("Removed green floor, remaining points: %lu", non_floor_cloud->points.size());
  
  return non_floor_cloud;
}

// Extract the brown basket from the point cloud
PointCPtr cw2::extractBrownBasket(const PointCPtr& cloud) {
  ROS_INFO("Extracting brown basket from point cloud...");
  
  PointCPtr basket_cloud(new PointC);
  
  // Brown color thresholds (RGB)
  float r_min = 0.4f, r_max = 0.6f;
  float g_min = 0.1f, g_max = 0.3f;
  float b_min = 0.1f, b_max = 0.3f;
  
  for (const auto& point : cloud->points) {
    // Normalize RGB values to 0-1 range
    float r = point.r / 255.0f;
    float g = point.g / 255.0f;
    float b = point.b / 255.0f;
    
    // Check if point is brown
    bool is_brown = (r >= r_min && r <= r_max) &&
                    (g >= g_min && g <= g_max) &&
                    (b >= b_min && b <= b_max);
    
    if (is_brown) {
      basket_cloud->points.push_back(point);
    }
  }
  
  basket_cloud->width = basket_cloud->points.size();
  basket_cloud->height = 1;
  basket_cloud->is_dense = true;
  basket_cloud->header = cloud->header;
  
  ROS_INFO("Extracted brown basket with %lu points", basket_cloud->points.size());
  
  return basket_cloud;
}

// Find the center of the basket
geometry_msgs::Point cw2::findBasketCenter(const PointCPtr& basket_cloud) {
  ROS_INFO("Finding basket center from %lu points", basket_cloud->points.size());
  
  geometry_msgs::Point center;
  
  if (basket_cloud->empty()) {
    ROS_WARN("Basket cloud is empty, returning default center");
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0;
    return center;
  }
  
  // Cluster the basket points (in case there are multiple brown objects)
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  
  tree->setInputCloud(basket_cloud);
  ec.setClusterTolerance(0.02);  // 2cm tolerance
  ec.setMinClusterSize(100);     // Minimum 100 points per cluster
  ec.setMaxClusterSize(100000);  // Maximum 100k points per cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(basket_cloud);
  ec.extract(cluster_indices);
  
  ROS_INFO("Found %zu clusters in basket cloud", cluster_indices.size());
  
  // Find the largest cluster (assume it's the basket)
  size_t max_size = 0;
  int max_cluster_idx = -1;
  
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    size_t cluster_size = cluster_indices[i].indices.size();
    ROS_INFO("Cluster %zu has %zu points", i, cluster_size);
    
    if (cluster_size > max_size) {
      max_size = cluster_size;
      max_cluster_idx = i;
    }
  }
  
  if (max_cluster_idx == -1) {
    ROS_WARN("No valid basket cluster found, using centroid of all brown points");
    
    // Compute centroid of all points
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*basket_cloud, centroid);
    
    center.x = centroid[0];
    center.y = centroid[1];
    center.z = centroid[2];
  } else {
    // Extract the largest cluster
    PointCPtr basket_cluster(new PointC);
    
    for (const auto& idx : cluster_indices[max_cluster_idx].indices) {
      basket_cluster->points.push_back(basket_cloud->points[idx]);
    }
    
    basket_cluster->width = basket_cluster->points.size();
    basket_cluster->height = 1;
    basket_cluster->is_dense = true;
    
    // Compute centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*basket_cluster, centroid);
    
    center.x = centroid[0];
    center.y = centroid[1];
    center.z = centroid[2];
    
    ROS_INFO("Found basket center at [%f, %f, %f] from largest cluster with %zu points",
             center.x, center.y, center.z, max_size);
  }
  
  return center;
}

// Extract black obstacles from the point cloud
PointCPtr cw2::extractBlackObstacles(const PointCPtr& cloud) {
  ROS_INFO("Extracting black obstacles from point cloud...");
  
  PointCPtr obstacles_cloud(new PointC);
  
  // Black color thresholds (RGB)
  float r_min = 0.0f, r_max = 0.2f;
  float g_min = 0.0f, g_max = 0.2f;
  float b_min = 0.0f, b_max = 0.2f;
  
  for (const auto& point : cloud->points) {
    // Normalize RGB values to 0-1 range
    float r = point.r / 255.0f;
    float g = point.g / 255.0f;
    float b = point.b / 255.0f;
    
    // Check if point is black
    bool is_black = (r >= r_min && r <= r_max) &&
                    (g >= g_min && g <= g_max) &&
                    (b >= b_min && b <= b_max);
    
    if (is_black) {
      obstacles_cloud->points.push_back(point);
    }
  }
  
  obstacles_cloud->width = obstacles_cloud->points.size();
  obstacles_cloud->height = 1;
  obstacles_cloud->is_dense = true;
  obstacles_cloud->header = cloud->header;
  
  ROS_INFO("Extracted black obstacles with %lu points", obstacles_cloud->points.size());
  
  return obstacles_cloud;
}

/**
 * Add black obstacles to the planning scene as an OctoMap
 * 
 * @param obstacles_cloud Point cloud containing the black obstacles
 */
void cw2::addObstaclesToPlanningScene(const PointCPtr& obstacles_cloud) {
  ROS_INFO("Adding black obstacles to planning scene using OctoMap representation");
  
  if (obstacles_cloud->empty()) {
    ROS_WARN("No obstacle points found in the point cloud");
    return;
  }
  
  // Publish the obstacles cloud for visualization
  publishPointCloud(obstacles_cloud, obstacles_cloud_pub_);
  
  // Create a planning scene message to apply the changes
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.octomap.header.frame_id = base_frame_;
  
  // Convert point cloud to octomap
  octomap::OcTree* obstacles_octree = new octomap::OcTree(0.02); // 2cm resolution
  
  // Create a pointcloud2 message from pcl point cloud
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*obstacles_cloud, cloud_msg);
  
  // Convert PointCloud2 to octomap
  octomap::Pointcloud octomap_cloud;
  for (size_t i = 0; i < obstacles_cloud->size(); ++i) {
    const auto& point = obstacles_cloud->at(i);
    octomap_cloud.push_back(point.x, point.y, point.z);
  }
  
  // Set origin for the OcTree
  octomap::point3d sensor_origin(0.0, 0.0, 0.0);
  
  // Insert the point cloud into the octree
  obstacles_octree->insertPointCloud(octomap_cloud, sensor_origin);
  obstacles_octree->updateInnerOccupancy();
  
  // Convert OcTree to OctoMap message
  octomap_msgs::binaryMapToMsg(*obstacles_octree, planning_scene.world.octomap.octomap);
  
  // Apply the planning scene update
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyPlanningScene(planning_scene);
  
  // Clean up
  delete obstacles_octree;
  
  ROS_INFO("Successfully added obstacles as OctoMap to planning scene");
}

// Extract red, blue, or purple graspable objects
PointCPtr cw2::extractGraspableObjects(const PointCPtr& cloud) {
  ROS_INFO("Extracting graspable objects (red, blue, purple) from point cloud...");
  
  PointCPtr objects_cloud(new PointC);
  
  for (const auto& point : cloud->points) {
    // Normalize RGB values to 0-1 range
    float r = point.r / 255.0f;
    float g = point.g / 255.0f;
    float b = point.b / 255.0f;
    
    // Check for red, blue, or purple colors
    bool isRed = (r > 0.7f && r < 0.9f) &&
                 (g > 0.0f && g < 0.2f) &&
                 (b > 0.0f && b < 0.2f);
    
    bool isBlue = (r > 0.0f && r < 0.2f) &&
                  (g > 0.0f && g < 0.2f) &&
                  (b > 0.7f && b < 0.9f);
    
    bool isPurple = (r > 0.7f && r < 0.9f) &&
                    (g > 0.0f && g < 0.2f) &&
                    (b > 0.7f && b < 0.9f);
    
    if (isRed || isBlue || isPurple) {
      objects_cloud->points.push_back(point);
    }
  }
  
  objects_cloud->width = objects_cloud->points.size();
  objects_cloud->height = 1;
  objects_cloud->is_dense = true;
  objects_cloud->header = cloud->header;
  
  ROS_INFO("Extracted graspable objects with %lu points", objects_cloud->points.size());
  
  return objects_cloud;
}

// Cluster and classify objects, returning orientation data for each
bool cw2::clusterAndClassifyObjects(
    const PointCPtr& objects_cloud,
    std::vector<PointCPtr>& object_clusters,
    std::vector<bool>& is_cross_shape,
    std::vector<ObjectOrientationData>& object_orientations) {
  
  ROS_INFO("Clustering and classifying objects...");
  
  if (objects_cloud->empty()) {
    ROS_ERROR("Objects cloud is empty, cannot cluster");
    return false;
  }
  
  // Clear output vectors
  object_clusters.clear();
  is_cross_shape.clear();
  object_orientations.clear();
  
  // Create KdTree for clustering
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(objects_cloud);
  
  // Perform Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(objects_cloud);
  ec.extract(cluster_indices);
  
  ROS_INFO("====== CLUSTERING RESULTS ======");
  ROS_INFO("Found %zu object clusters", cluster_indices.size());
  
  // 创建用于可视化的彩色聚类点云
  PointCPtr colored_clusters(new PointC);
  colored_clusters->header = objects_cloud->header;
  
  // 创建用于所有 PCA 轴的标记数组
  visualization_msgs::MarkerArray all_pca_markers;
  
  // 处理每个聚类
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    // Extract cluster
    PointCPtr cluster(new PointC);
    for (const auto& idx : cluster_indices[i].indices) {
      cluster->points.push_back(objects_cloud->points[idx]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    cluster->header = objects_cloud->header;
    
    ROS_INFO("Cluster %zu has %lu points", i+1, cluster->points.size());
    
    // Compute centroid for shape determination
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    geometry_msgs::Point center_point;
    center_point.x = centroid[0];
    center_point.y = centroid[1];
    center_point.z = centroid[2];
    
    // Determine if it's a cross shape using the centroid method
    bool is_cross = determineShapeTypeFromCamera(cluster, center_point);
    
    ROS_INFO("Cluster %zu is a %s shape", i+1, is_cross ? "CROSS" : "NOUGHT");
    
    // Determine object orientation using PCA
    std::string shape_type = is_cross ? "cross" : "nought";
    ObjectOrientationData orientation_data = determineObjectOrientation(cluster, shape_type);
    
    if (orientation_data.is_valid) {
      // Add to output vectors
      object_clusters.push_back(cluster);
      is_cross_shape.push_back(is_cross);
      object_orientations.push_back(orientation_data);
    } else {
      ROS_WARN("Could not determine valid orientation for cluster %zu, skipping", i+1);
    }
    
    uint8_t r = 50 + (i * 40) % 200;
    uint8_t g = 50 + ((i * 70) % 200);
    uint8_t b = 50 + ((i * 90) % 200);

    for (const auto& idx : cluster_indices[i].indices) {
      PointT colored_point = objects_cloud->points[idx];
      colored_point.r = r;
      colored_point.g = g;
      colored_point.b = b;
      colored_clusters->points.push_back(colored_point);
    }
    
    // 获取该聚类的 PCA 轴向标记
    if (orientation_data.is_valid) {
      // 创建该聚类的 PCA 轴标记
      visualization_msgs::MarkerArray cluster_pca_markers = createPCAAxesMarkers(
          centroid, orientation_data.primary_axis, orientation_data.secondary_axis, 
          i, shape_type);
      
      // 将标记添加到所有 PCA 轴的标记数组中
      for (const auto& marker : cluster_pca_markers.markers) {
        all_pca_markers.markers.push_back(marker);
      }
    }
  }
  
  ROS_INFO("Successfully processed %zu valid object clusters", object_clusters.size());
  ROS_INFO("======================================");
  
  // 设置可视化点云属性
  colored_clusters->width = colored_clusters->points.size();
  colored_clusters->height = 1;
  colored_clusters->is_dense = true;
  
  // 发布可视化
  publishPointCloud(colored_clusters, clusters_pub_);
  all_pca_axes_pub_.publish(all_pca_markers);
  
  return !object_clusters.empty();
}

// 添加辅助函数来创建 PCA 轴向标记
visualization_msgs::MarkerArray cw2::createPCAAxesMarkers(
    const Eigen::Vector4f& centroid,
    const Eigen::Vector3f& primary_axis,
    const Eigen::Vector3f& secondary_axis,
    int id_offset,
    const std::string& shape_type) {
  
  visualization_msgs::MarkerArray marker_array;
  
  // 创建主轴标记
  visualization_msgs::Marker primary_marker;
  primary_marker.header.frame_id = base_frame_;
  primary_marker.header.stamp = ros::Time::now();
  primary_marker.ns = "pca_axes";
  primary_marker.id = id_offset * 3;
  primary_marker.type = visualization_msgs::Marker::ARROW;
  primary_marker.action = visualization_msgs::Marker::ADD;
  
  primary_marker.pose.position.x = centroid[0];
  primary_marker.pose.position.y = centroid[1];
  primary_marker.pose.position.z = centroid[2];
  
  // 计算从 z 轴到目标轴的四元数
  Eigen::Vector3f z_axis(0, 0, 1);
  Eigen::Vector3f rotation_axis = z_axis.cross(primary_axis).normalized();
  float rotation_angle = acos(z_axis.dot(primary_axis));
  
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(rotation_angle, rotation_axis);
  
  primary_marker.pose.orientation.x = q.x();
  primary_marker.pose.orientation.y = q.y();
  primary_marker.pose.orientation.z = q.z();
  primary_marker.pose.orientation.w = q.w();
  
  primary_marker.scale.x = 0.1;  // 轴长
  primary_marker.scale.y = 0.01; // 轴宽
  primary_marker.scale.z = 0.01; // 轴高
  
  // 为不同形状设置不同颜色
  if (shape_type == "cross") {
    primary_marker.color.r = 1.0;
    primary_marker.color.g = 0.0;
    primary_marker.color.b = 0.0;
  } else {
    primary_marker.color.r = 0.0;
    primary_marker.color.g = 0.0;
    primary_marker.color.b = 1.0;
  }
  primary_marker.color.a = 1.0;
  
  marker_array.markers.push_back(primary_marker);
  
  // 类似地创建次轴和第三轴标记...
  // (省略部分代码以保持简洁)
  
  return marker_array;
}

// Grasp and place all objects of the specified type
bool cw2::graspAndPlaceObjectsOfType(
    const std::vector<PointCPtr>& object_clusters,
    const std::vector<bool>& is_cross_shape,
    const std::vector<ObjectOrientationData>& object_orientations,
    bool grasp_cross_shape,
    const geometry_msgs::Point& basket_center) {
  
  ROS_INFO("\n====== GRASPING AND PLACING OBJECTS ======");
  ROS_INFO("Target shape type: %s", grasp_cross_shape ? "CROSS" : "NOUGHT");
  
  // Track current Z offset for stacking in the basket
  float current_stack_height = 0.0;
  
  // Process each object
  int objects_grasped = 0;
  
  for (size_t i = 0; i < object_clusters.size(); i++) {
    // Only grasp objects of the target shape
    if (is_cross_shape[i] != grasp_cross_shape) {
      continue;
    }
    
    ROS_INFO("Processing object %zu (%s)", i, is_cross_shape[i] ? "cross" : "nought");
    
    // Skip if the orientation data is invalid
    if (!object_orientations[i].is_valid) {
      ROS_WARN("Skipping object with invalid orientation data");
      continue;
    }
    
    // Calculate object centroid for grasping
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*object_clusters[i], centroid);
    
    // Create grasp point
    geometry_msgs::Point object_center;
    object_center.x = centroid[0];
    object_center.y = centroid[1];
    object_center.z = centroid[2];
    object_center.z += t3_grasp_height_offset_;
    
    // 计算抓取方向和偏移量
    Eigen::Vector3f grasp_axis;
    if (is_cross_shape[i]) {
      // 对于十字形，使用主轴方向
      grasp_axis = object_orientations[i].primary_axis;
    } else {
      // 对于圆环形，计算中点角度方向
      float primary_angle = atan2(object_orientations[i].primary_axis[1], 
                                 object_orientations[i].primary_axis[0]);
      float secondary_angle = atan2(object_orientations[i].secondary_axis[1], 
                                   object_orientations[i].secondary_axis[0]);
      
      // 处理角度差
      float angle_diff = secondary_angle - primary_angle;
      if (angle_diff > M_PI) angle_diff -= 2*M_PI;
      if (angle_diff < -M_PI) angle_diff += 2*M_PI;
      
      float midpoint_angle = primary_angle + angle_diff/2.0;
      
      // 中点角度方向
      grasp_axis[0] = cos(midpoint_angle);
      grasp_axis[1] = sin(midpoint_angle);
      grasp_axis[2] = 0.0;
    }
    
    // 计算最佳抓取偏移量，根据物体实际尺寸
    float grasp_offset = calculateGraspOffset(
        object_clusters[i], 
        centroid, 
        grasp_axis, 
        is_cross_shape[i]);
    
    // Execute the grasp with calculated offset
    bool grasp_success = planAndExecuteGrasp(
        object_center, 
        object_orientations[i], 
        is_cross_shape[i] ? "cross" : "nought",
        grasp_offset);
    
    if (!grasp_success) {
      ROS_ERROR("Failed to grasp object %zu", i);
      continue;
    }
    
    // Calculate place position with incrementing height for stacking
    geometry_msgs::Point place_point = basket_center;
    place_point.z += current_stack_height;
    
    // Execute the place
    bool place_success = planAndExecutePlace(place_point);
    
    if (!place_success) {
      ROS_ERROR("Failed to place object %zu", i);
      continue;
    }
    
    // Increment stack height for next object
    current_stack_height += 0.03; // Add 3cm for each stacked object
    objects_grasped++;
    
    ROS_INFO("Successfully grasped and placed object %zu", i);
  }
  
  ROS_INFO("Grasped and placed %d objects of type %s", 
           objects_grasped, grasp_cross_shape ? "cross" : "nought");
  
  return objects_grasped > 0;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::moveArm(geometry_msgs::PoseStamped target_pose)
{
  /* function to move the arm to a target pose */

  ROS_INFO("Moving the arm to the target pose");

  // setup the target pose
  //ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  //ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // execute the planned path
  arm_group_.move();

  return success;
}

bool 
cw2::moveGripper(float width, float wait_time)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_) 
    width = gripper_open_;
  if (width < gripper_closed_) 
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  //ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // move the gripper joints
  if (wait_time > 0.0)
  {
    if (success) {
      moveit::core::MoveItErrorCode result = hand_group_.execute(my_plan);
  
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Gripper move executed successfully, waiting for completion...");
        hand_group_.getMoveGroupClient().waitForResult(ros::Duration(wait_time));
        ROS_INFO("Gripper move completed.");
      }
      else {
        ROS_ERROR("Failed to execute gripper move.");
      }
    }
  }
  else
  {
    hand_group_.move();
  }

  return success;
}


/* Note: Pick point uses center of the box, Place points uses top of the basket */
/* This should be consistent between t1 and t3 */
void
cw2::pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point)
{
  geometry_msgs::PoseStamped place_point_pose;

  ROS_INFO("Picking and placing object");

  // always consider the griper length
  pick_pose.pose.position.z += hand_offset_;
  place_point.point.z += hand_offset_;
  place_point.point.z += basket_size_;

  // Build the place point pose
  place_point_pose.pose.position = place_point.point;
  place_point_pose.pose.orientation = grasp_orientation_;
  place_point_pose.header.frame_id = base_frame_;

  // move the arm to top of the grasp point 
  pick_pose.pose.orientation = grasp_orientation_;
  pick_pose.pose.position.z += grasp_stanby_height_;
  moveArm(pick_pose);

  moveGripper(gripper_open_);

  pick_pose.pose.position.z -= grasp_stanby_height_;
  moveArm(pick_pose);

  // move the gripper to the closed width
  moveGripper(gripper_closed_);

  // Wait 0.5s for the gripper to close
  ros::Duration(0.5).sleep();

  // Pick up
  pick_pose.pose.position.z += grasp_stanby_height_ + 0.1;
  moveArm(pick_pose);

  ROS_INFO("Object picked up");

  // move to top of place point
  // higher for safety
  place_point_pose.pose.position.z += place_stanby_height_;

  // move the arm to the place point
  moveArm(place_point_pose);

  place_point_pose.pose.position.z -= place_stanby_height_;
  // move the arm to the place point
  moveArm(place_point_pose);

  // move the gripper to the closed width
  moveGripper(gripper_open_);

  ROS_INFO("Object placed");

  return;
}

// Update visualization function to use pre-calculated grasp direction
void cw2::visualizePCAAxes(
    const Eigen::Matrix3f &eigenvectors,
    const geometry_msgs::Point &center_point,
    const std::string &shape_type,
    const Eigen::Vector3f &grasp_direction,
    float grasp_angle) {
  
  visualization_msgs::MarkerArray marker_array;
  
  // Scale factor for visualization
  float scale_factor = 0.2;  // 20cm arrows
  
  // Create markers for each eigenvector
  for (int i = 0; i < 3; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pca_axes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set arrow start point at center
    marker.points.resize(2);
    marker.points[0].x = center_point.x;
    marker.points[0].y = center_point.y;
    marker.points[0].z = center_point.z;
    
    // Set arrow end point along eigenvector
    Eigen::Vector3f vec = eigenvectors.col(i);
    marker.points[1].x = center_point.x + vec[0] * scale_factor;
    marker.points[1].y = center_point.y + vec[1] * scale_factor;
    marker.points[1].z = center_point.z + vec[2] * scale_factor;
    
    // Set arrow properties
    marker.scale.x = 0.01;  // Shaft diameter
    marker.scale.y = 0.02;  // Head diameter
    marker.scale.z = 0.04;  // Head length
    
    // Color based on which eigenvector (RGB for 1st, 2nd, 3rd)
    marker.color.a = 1.0;
    marker.color.r = (i == 0) ? 1.0 : 0.0;
    marker.color.g = (i == 1) ? 1.0 : 0.0;
    marker.color.b = (i == 2) ? 1.0 : 0.0;
    
    marker.lifetime = ros::Duration(0);  // Persistent
    
    marker_array.markers.push_back(marker);
  }
  
  // Add an additional arrow for grasp direction
  visualization_msgs::Marker grasp_marker;
  grasp_marker.header.frame_id = base_frame_;
  grasp_marker.header.stamp = ros::Time::now();
  grasp_marker.ns = "pca_axes";
  grasp_marker.id = 3;  // ID 3 for grasp direction
  grasp_marker.type = visualization_msgs::Marker::ARROW;
  grasp_marker.action = visualization_msgs::Marker::ADD;
  
  // Set arrow start point at center
  grasp_marker.points.resize(2);
  grasp_marker.points[0].x = center_point.x;
  grasp_marker.points[0].y = center_point.y;
  grasp_marker.points[0].z = center_point.z;
  
  // Set arrow end point along grasp direction
  grasp_marker.points[1].x = center_point.x + grasp_direction[0] * scale_factor;
  grasp_marker.points[1].y = center_point.y + grasp_direction[1] * scale_factor;
  grasp_marker.points[1].z = center_point.z + grasp_direction[2] * scale_factor;
  
  // Set arrow properties
  grasp_marker.scale.x = 0.015;  // Slightly thicker
  grasp_marker.scale.y = 0.025;
  grasp_marker.scale.z = 0.05;
  
  // Yellow color for grasp direction
  grasp_marker.color.a = 1.0;
  grasp_marker.color.r = 1.0;
  grasp_marker.color.g = 1.0;
  grasp_marker.color.b = 0.0;
  
  grasp_marker.lifetime = ros::Duration(0);  // Persistent
  
  marker_array.markers.push_back(grasp_marker);
  
  // Publish marker array
  pca_axes_pub_.publish(marker_array);
  
  ROS_INFO("Published PCA axes and grasp direction visualization");
}

// Callback function for OctoMap messages
void cw2::octomap_callback(const octomap_msgs::Octomap::ConstPtr& msg) {
  // Silently process the OctoMap message without logging
  latest_octomap_ = *msg;
  octomap_received_ = true;
}

// Perform scanning motion around the object to collect better point cloud data
bool cw2::performScanningMotion(const geometry_msgs::Point &target_point) {
  ROS_INFO("\n====== PERFORMING SCANNING MOTION ======");
  ROS_INFO("Starting scanning motion around object at [%f, %f, %f]",
           target_point.x, target_point.y, target_point.z);
  
  // Create a set of poses around the object
  std::vector<geometry_msgs::PoseStamped> scan_poses;
  
  // Height of camera during scanning - use a consistent height
  float scan_z = target_point.z + scan_height_offset_;
  
  // Convert the pre-defined downward-facing orientation to tf2::Quaternion
  tf2::Quaternion q_base;
  tf2::convert(grasp_orientation_, q_base);
  
  // Generate 4 positions in a square pattern around the object
  const int num_positions = 4;
  for (int i = 0; i < num_positions; i++) {
    float angle = 2.0f * M_PI * i / num_positions; // 0, 90, 180, 270 degrees
    float x = target_point.x + scan_radius_ * cos(angle);
    float y = target_point.y + scan_radius_ * sin(angle);
    
    // Create scan pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = base_frame_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = scan_z;
    
    // Use consistent orientation for all scan positions
    pose.pose.orientation = grasp_orientation_;
    
    scan_poses.push_back(pose);
    
    ROS_INFO("Scan position %d: [%f, %f, %f] with consistent orientation",
             i+1, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }
  
  int successful_scans = 0;
  
  // Move to each scan pose
  for (int i = 0; i < scan_poses.size(); i++) {
    ROS_INFO("Moving to scan position %zu/%zu", i+1, scan_poses.size());
    
    // Move arm to scan position
    bool move_success = moveArm(scan_poses[i]);
    if (!move_success) {
      ROS_WARN("Failed to move to scan position %zu, trying next position", i+1);
      continue;
    }
    
    // Wait for arm to stabilize
    ros::Duration(1.0).sleep();
    
    // Reset the OctoMap status
    octomap_received_ = false;
    
    // Attempt to get an OctoMap update at this position
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(5.0); // 5-second timeout
    
    ROS_INFO("Waiting for OctoMap update at position %zu...", i+1);
    
    while (!octomap_received_ && ros::Time::now() - start_time < timeout) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    
    if (octomap_received_) {
      ROS_INFO("Successfully received OctoMap update at scan position %zu with %d bytes of data",
               i+1, (int)latest_octomap_.data.size());
      successful_scans++;
    } else {
      // Try calling the service directly if subscriber didn't work
      ROS_WARN("Timeout waiting for OctoMap update from subscriber at scan position %zu, trying service call",
               i+1);
      
      octomap_msgs::GetOctomap srv;
      if (octomap_client_.call(srv)) {
        latest_octomap_ = srv.response.map;
        octomap_received_ = true;
        successful_scans++;
        ROS_INFO("Successfully received OctoMap from service at scan position %zu", i+1);
      } else {
        ROS_ERROR("Failed to receive OctoMap from service at scan position %zu", i+1);
      }
    }
  }
  
  ROS_INFO("Completed %d successful scans out of %zu positions", successful_scans, scan_poses.size());
  
  // Return to a position above the object
  geometry_msgs::PoseStamped top_pose;
  top_pose.header.frame_id = base_frame_;
  top_pose.pose.position.x = target_point.x;
  top_pose.pose.position.y = target_point.y;
  top_pose.pose.position.z = target_point.z + scan_height_;
  top_pose.pose.orientation = grasp_orientation_;
  
  ROS_INFO("Returning to position above object");
  bool success = moveArm(top_pose);
  
  ROS_INFO("====== SCANNING MOTION COMPLETED ======\n");
  return successful_scans > 0;  // At least one successful scan is required
}

// Extract point cloud from OctoMap
PointCPtr cw2::extractPointCloudFromOctomap() {
  ROS_INFO("\n====== EXTRACTING POINT CLOUD FROM OCTOMAP ======");
  
  PointCPtr cloud(new PointC);
  
  // Try to get OctoMap from the service if we don't already have one
  if (!octomap_received_) {
    ROS_INFO("No OctoMap received yet from subscriber, requesting from service...");
    octomap_msgs::GetOctomap srv;
    
    if (octomap_client_.call(srv)) {
      latest_octomap_ = srv.response.map;
      octomap_received_ = true;
      ROS_INFO("Successfully received OctoMap from service with %d bytes of data", (int)latest_octomap_.data.size());
    } else {
      ROS_ERROR("Failed to call OctoMap service. Is the octomap_server running?");
      return cloud;
    }
  } else {
    ROS_INFO("Using existing OctoMap from subscriber with %d bytes of data", (int)latest_octomap_.data.size());
  }
  
  // Convert OctoMap to octomap::OcTree
  octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(latest_octomap_);
  if (!abstract_tree) {
    ROS_ERROR("Failed to convert OctoMap message to OcTree");
    return cloud;
  }
  
  // Convert to OcTree
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
  if (!octree) {
    ROS_ERROR("Failed to convert to OcTree");
    delete abstract_tree;
    return cloud;
  }
  
  ROS_INFO("OcTree created with resolution: %f and %lu nodes", 
           octree->getResolution(), octree->size());
  
  // Create point cloud from OcTree
  ROS_INFO("Converting OcTree to point cloud...");
  cloud->header.frame_id = latest_octomap_.header.frame_id;
  cloud->width = 0;  // Will increment as we add points
  cloud->height = 1;
  cloud->is_dense = false;
  
  // Iterate through the octree
  unsigned int count = 0;
  for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    // Only consider occupied voxels
    if (octree->isNodeOccupied(*it)) {
      // Get coordinates from OcTree node
      float x = it.getX();
      float y = it.getY();
      float z = it.getZ();
      
      // Create a point
      PointT point;
      point.x = x;
      point.y = y;
      point.z = z;
      
      // Set default color (white) 
      point.r = 255;
      point.g = 255;
      point.b = 255;
      point.a = 255;
      
      // Add point to cloud
      cloud->points.push_back(point);
      cloud->width++;
      
      // Count points and provide progress updates
      count++;
      if (count % 10000 == 0) {
        ROS_INFO("Processed %u points so far", count);
      }
    }
  }
  
  ROS_INFO("Extracted %lu points from OctoMap", cloud->points.size());
  
  // Clean up
  delete octree;
  
  // If we don't have enough points, return the empty cloud
  if (cloud->points.size() < 100) {
    ROS_WARN("Not enough points from OctoMap (%lu), returning empty cloud", 
             cloud->points.size());
    return cloud;
  }
  
  // Apply voxel grid filter to downsample
  ROS_INFO("Applying voxel grid filtering for downsampling...");
  pcl::VoxelGrid<PointT> voxel_filter;
  PointCPtr downsampled_cloud(new PointC);
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm voxel size
  voxel_filter.filter(*downsampled_cloud);
  
  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*downsampled_cloud, *downsampled_cloud, indices);
  
  ROS_INFO("After voxel grid filtering: %lu points", downsampled_cloud->points.size());
  
  // Apply plane segmentation to remove the ground plane
  ROS_INFO("Performing plane segmentation to remove floor...");
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointT> seg;
  
  // Configure the segmentation parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01); // 1cm threshold
  seg.setMaxIterations(100);
  
  // Specify that we're looking for a plane perpendicular to the Z axis (ground plane)
  Eigen::Vector3f axis(0.0, 0.0, 1.0);
  seg.setAxis(axis);
  seg.setEpsAngle(15.0 * (M_PI / 180.0)); // Allow 15 degrees deviation from Z axis
  
  // Perform the segmentation
  seg.setInputCloud(downsampled_cloud);
  seg.segment(*inliers, *coefficients);
  
  if (inliers->indices.size() > 0) {
    // Extract everything except the plane (ground)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // Extract everything EXCEPT the ground plane
    
    PointCPtr cloud_without_ground(new PointC);
    extract.filter(*cloud_without_ground);
    
    ROS_INFO("Ground plane removed: kept %lu of %lu points", 
             cloud_without_ground->points.size(), downsampled_cloud->points.size());
    
    // Replace downsampled_cloud with the filtered version
    downsampled_cloud = cloud_without_ground;
  } else {
    ROS_WARN("No ground plane detected in the point cloud");
  }
  
  ROS_INFO("====== POINT CLOUD EXTRACTION COMPLETED ======\n");
  return downsampled_cloud;
}

// Helper function to get latest point cloud from a topic
PointCPtr cw2::getLatestPointCloud(const std::string& topic, const std::string& target_frame) {
  ROS_INFO("Waiting for point cloud message from %s...", topic.c_str());
  PointCPtr cloud(new PointC);
  
  // Get the latest message from the point cloud topic
  sensor_msgs::PointCloud2::ConstPtr cloud_msg = 
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(5.0));
  
  if (!cloud_msg) {
    ROS_ERROR("Failed to receive point cloud message from %s within timeout", topic.c_str());
    return cloud;
  }
  
  // Convert ROS message to PCL point cloud
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  ROS_INFO("Received point cloud with %lu points in frame %s", 
           cloud->points.size(), cloud_msg->header.frame_id.c_str());
  
  // Transform point cloud to target frame if necessary
  if (cloud_msg->header.frame_id != target_frame) {
    ROS_INFO("Transforming point cloud from %s to %s", 
             cloud_msg->header.frame_id.c_str(), target_frame.c_str());
    
    try {
      // Look up transform
      geometry_msgs::TransformStamped transform = 
          tf_buffer_.lookupTransform(target_frame, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(3.0));
      
      // Create transformed cloud
      PointCPtr transformed_cloud(new PointC);
      
      // Apply transform to point cloud
      Eigen::Matrix4f transform_matrix;
      Eigen::Quaternionf q(transform.transform.rotation.w,
                          transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z);
      
      transform_matrix.block<3,3>(0,0) = q.toRotationMatrix();
      transform_matrix(0,3) = transform.transform.translation.x;
      transform_matrix(1,3) = transform.transform.translation.y;
      transform_matrix(2,3) = transform.transform.translation.z;
      transform_matrix(3,0) = 0.0;
      transform_matrix(3,1) = 0.0;
      transform_matrix(3,2) = 0.0;
      transform_matrix(3,3) = 1.0;
      
      pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);
      
      // Update frame_id and return transformed cloud
      transformed_cloud->header.frame_id = target_frame;
      ROS_INFO("Transformed point cloud to %s with %lu points", 
               target_frame.c_str(), transformed_cloud->points.size());
      return transformed_cloud;
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("Transform error: %s", ex.what());
      ROS_WARN("Using untransformed point cloud");
    }
  }
  
  // Return original cloud if no transform needed or if transform failed
  cloud->header.frame_id = target_frame;
  return cloud;
}

// New function for color filtering
PointCPtr cw2::filterPointCloudByColor(const PointCPtr& input_cloud) {
  ROS_INFO("Filtering point cloud by color...");
  PointCPtr cloud_color_filtered(new PointC);
  
  if (input_cloud->empty()) {
    ROS_WARN("Input cloud for color filtering is empty");
    return cloud_color_filtered;
  }
  
  for (const auto& point : input_cloud->points) {
    // Normalize RGB values to 0-1 range
    float r = point.r / 255.0f;
    float g = point.g / 255.0f;
    float b = point.b / 255.0f;
    
    // Check if color matches one of our target colors
    bool isPurple = (r > 0.7f && r < 0.9f) &&
                  (g > 0.0f && g < 0.2f) &&
                  (b > 0.7f && b < 0.9f);
    
    bool isRed = (r > 0.7f && r < 0.9f) &&
               (g > 0.0f && g < 0.2f) &&
               (b > 0.0f && b < 0.2f);
    
    bool isBlue = (r > 0.0f && r < 0.2f) &&
                (g > 0.0f && g < 0.2f) &&
                (b > 0.7f && b < 0.9f);
    
    bool isBrown = (r > 0.4f && r < 0.6f) &&
                 (g > 0.1f && g < 0.3f) &&
                 (b > 0.1f && b < 0.3f);
    
    bool isBlack = (r > 0.0f && r < 0.2f) &&
                 (g > 0.0f && g < 0.2f) &&
                 (b > 0.0f && b < 0.2f);
    
    // Keep only target colors
    if (isPurple || isRed || isBlue || isBrown || isBlack) {
      cloud_color_filtered->points.push_back(point);
    }
  }
  
  cloud_color_filtered->width = cloud_color_filtered->points.size();
  cloud_color_filtered->height = 1;
  cloud_color_filtered->is_dense = true;
  cloud_color_filtered->header = input_cloud->header;
  
  ROS_INFO("Color-filtered cloud has %lu points", cloud_color_filtered->points.size());
  return cloud_color_filtered;
}

// Modified processPointCloud to use the new color filtering function
PointCPtr cw2::processPointCloud(const PointCPtr& input_cloud) {
  ROS_INFO("\n====== PROCESSING POINT CLOUD ======");
  ROS_INFO("Input cloud has %lu points", input_cloud->points.size());
  
  if (input_cloud->empty()) {
    ROS_ERROR("Input cloud is empty");
    return PointCPtr(new PointC);
  }
  
  // 1. Downsample with voxel grid filter
  ROS_INFO("Downsampling point cloud with voxel grid filter...");
  pcl::VoxelGrid<PointT> voxel_filter;
  PointCPtr cloud_downsampled(new PointC);
  
  voxel_filter.setInputCloud(input_cloud);
  voxel_filter.setLeafSize(0.002f, 0.002f, 0.002f);  // 2mm voxel size
  voxel_filter.filter(*cloud_downsampled);
  
  ROS_INFO("Downsampled cloud has %lu points", cloud_downsampled->points.size());
  
  // 2. Filter by color (using the new function)
  PointCPtr cloud_color_filtered = filterPointCloudByColor(cloud_downsampled);
  
  // 3. Remove statistical outliers
  ROS_INFO("Removing outliers...");
  pcl::StatisticalOutlierRemoval<PointT> sor;
  PointCPtr cloud_filtered(new PointC);
  
  sor.setInputCloud(cloud_color_filtered);
  sor.setMeanK(50);              // 50 neighbors to analyze
  sor.setStddevMulThresh(1.0);   // Standard deviation threshold
  sor.filter(*cloud_filtered);
  
  ROS_INFO("After outlier removal: %lu points", cloud_filtered->points.size());
  
  ROS_INFO("====== POINT CLOUD PROCESSING COMPLETED ======\n");
  return cloud_filtered;
}

// Add a new function to visualize grasp points
void cw2::visualizeGraspPoint(const geometry_msgs::Point &grasp_point, const tf2::Quaternion &orientation) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "grasp_points";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the grasp point
  marker.pose.position = grasp_point;
  
  // Convert tf2 quaternion to geometry_msgs quaternion
  tf2::convert(orientation, marker.pose.orientation);
  
  // Set arrow properties
  marker.scale.x = 0.1;  // Arrow length
  marker.scale.y = 0.02; // Arrow width
  marker.scale.z = 0.02; // Arrow height
  
  // Set color (magenta for high visibility)
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  
  marker.lifetime = ros::Duration(0);  // Persistent
  
  // Publish marker
  grasp_marker_pub_.publish(marker);
  
  ROS_INFO("Published grasp point visualization at [%f, %f, %f]",
           grasp_point.x, grasp_point.y, grasp_point.z);
}

// Add this function to create and add a floor collision object
void cw2::addFloorCollisionObject() {
  ROS_INFO("Adding floor collision object to planning scene");
  
  // Create a collision object message
  moveit_msgs::CollisionObject floor_object;
  floor_object.header.frame_id = base_frame_;
  floor_object.id = "floor";
  
  // Define the floor dimensions (1.2m x 1.2m x 0.01m)
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.2;  // X dimension
  primitive.dimensions[1] = 1.2;  // Y dimension
  primitive.dimensions[2] = 0.01; // Z dimension (height)
  
  // Define the floor pose (center at 0, 0, 0)
  geometry_msgs::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.x = 0.0;
  floor_pose.position.y = 0.0;
  floor_pose.position.z = -0.005; // Place slightly below 0 to avoid grazing the surface
  
  // Add the primitive and pose to the collision object
  floor_object.primitives.push_back(primitive);
  floor_object.primitive_poses.push_back(floor_pose);
  floor_object.operation = floor_object.ADD;
  
  // Add the collision object to the vector
  collision_object_vector_.push_back(floor_object);
  
  // Add the collision object to the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_object_vector_);
  
  ROS_INFO("Floor collision object added to planning scene");
}

/**
 * Callback for point cloud data during continuous scanning
 * Processes and stores point clouds with optimizations:
 * - Downsamples using voxel grid
 * - Filters out green floor points immediately
 * - Only processes every N frames based on interval setting
 */
void cw2::continuousScanCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // Only process if we're in collection mode
  if (!is_collecting_clouds_) {
    return;
  }
  
  // Increment counter and only process every Nth frame
  cloud_frame_counter_++;
  if (cloud_frame_counter_ % t3_pointcloud_save_interval_ != 0) {
    return;
  }
  
  ROS_INFO("Processing cloud frame %d", cloud_frame_counter_);
  
  // Convert ROS message to PCL
  PointCPtr cloud(new PointC);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Transform to base frame
  PointCPtr transformed_cloud(new PointC);
  if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
    try {
      geometry_msgs::TransformStamped transform = 
          tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, ros::Time(0));
      
      // Use pcl_ros transform function directly instead of Eigen conversion
      pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform.transform);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform point cloud from %s to %s: %s", 
               msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
  } else {
    *transformed_cloud = *cloud;
  }
  
  // Filter out green points (floor)
  PointCPtr non_green_cloud(new PointC);
  for (const auto& point : transformed_cloud->points) {
    if (!isGreenPoint(point)) {
      non_green_cloud->points.push_back(point);
    }
  }
  non_green_cloud->width = non_green_cloud->points.size();
  non_green_cloud->height = 1;
  non_green_cloud->is_dense = false;
  
  // Skip if empty after green filtering
  if (non_green_cloud->empty()) {
    return;
  }
  
  // Downsample using voxel grid filter
  PointCPtr downsampled_cloud(new PointC);
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(non_green_cloud);
  voxel_filter.setLeafSize(t3_continuous_scan_voxel_size_, 
                          t3_continuous_scan_voxel_size_, 
                          t3_continuous_scan_voxel_size_);
  voxel_filter.filter(*downsampled_cloud);
  
  // Store the processed cloud
  collected_clouds_.push_back(downsampled_cloud);
  
  // Publish for visualization (optional, can be disabled to save resources)
  if (debug_) {
    publishPointCloud(downsampled_cloud, cloud_filtered_pub_);
  }
}

/**
 * Check if a point is likely part of the green floor
 * Uses a simple HSV-based color filter
 */
bool cw2::isGreenPoint(const PointT& point) {
  // Convert RGB to HSV
  float r = point.r / 255.0f;
  float g = point.g / 255.0f;
  float b = point.b / 255.0f;
  
  float max_val = std::max(std::max(r, g), b);
  float min_val = std::min(std::min(r, g), b);
  float diff = max_val - min_val;
  
  float h = 0.0f;
  if (max_val == r) {
    h = 60.0f * fmod(((g - b) / diff), 6.0f);
  } else if (max_val == g) {
    h = 60.0f * (((b - r) / diff) + 2.0f);
  } else {
    h = 60.0f * (((r - g) / diff) + 4.0f);
  }
  
  if (h < 0.0f) h += 360.0f;
  
  float s = (max_val == 0.0f) ? 0.0f : (diff / max_val);
  float v = max_val;
  
  // Green floor HSV ranges (adjust as needed for your environment)
  // Typically green is around H=120, but range may vary
  return (h >= 80.0f && h <= 160.0f && s >= 0.1f && v >= 0.1f);
}

/**
 * Merge multiple point clouds into one
 * Simply concatenates all points from input clouds
 */
PointCPtr cw2::mergeClouds(const std::vector<PointCPtr>& clouds) {
  PointCPtr merged_cloud(new PointC);
  
  // Return empty cloud if no input
  if (clouds.empty()) {
    return merged_cloud;
  }
  
  // Count total points
  size_t total_points = 0;
  for (const auto& cloud : clouds) {
    total_points += cloud->points.size();
  }
  
  // Reserve space for all points
  merged_cloud->points.reserve(total_points);
  
  // Merge all clouds
  for (const auto& cloud : clouds) {
    merged_cloud->points.insert(merged_cloud->points.end(), 
                               cloud->points.begin(), 
                               cloud->points.end());
  }
  
  // Set cloud parameters
  merged_cloud->width = merged_cloud->points.size();
  merged_cloud->height = 1;
  merged_cloud->is_dense = false;
  
  // 打印合并后的点云大小
  ROS_INFO("Merged %zu clouds with total %zu points", clouds.size(), merged_cloud->points.size());
  
  // 使用t3_continuous_scan_voxel_size_进行最终的体素降采样
  PointCPtr final_cloud(new PointC);
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(merged_cloud);
  voxel_filter.setLeafSize(t3_continuous_scan_voxel_size_, 
                          t3_continuous_scan_voxel_size_, 
                          t3_continuous_scan_voxel_size_);
  voxel_filter.filter(*final_cloud);
  
  // 打印降采样后的点云大小
  ROS_INFO("After final voxel filtering (%f mm): %zu points",
           t3_continuous_scan_voxel_size_ * 1000.0, final_cloud->points.size());
  
  return final_cloud;
}

/**
 * Performs a continuous scanning motion to capture the entire scene
 * Breaks the rectangular path into separate segments for better planning
 * 
 * @return Merged point cloud of the entire scene
 */
PointCPtr cw2::continuousScanSceneFromMultipleViewpoints() {
  ROS_INFO("Starting continuous scanning motion...");
  

  
  // Define rectangular path parameters
  float rect_x_min = -0.45;
  float rect_x_max = 0.45;
  float rect_y_min = -0.35;
  float rect_y_max = 0.35;
  float scan_height = t3_scan_height_;  // Constant height for stable scanning
  
  // Get the downward-facing orientation (end effector pointing down)
  tf2::Quaternion q_down;
  q_down.setRPY(-M_PI, 0, -M_PI/4);  // Roll -180 degrees (camera pointing down)
  geometry_msgs::Quaternion down_orientation = tf2::toMsg(q_down);
  
  // Number of points along each edge of the rectangle
  int points_per_edge = 5;
  double eef_step = 0.02;       // 2cm resolution for path
  double jump_threshold = 0.0;  // Disable jump threshold
  
  // First move to the starting position (bottom-left corner)
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = base_frame_;
  start_pose.pose.position.x = rect_x_min;
  start_pose.pose.position.y = rect_y_min;
  start_pose.pose.position.z = scan_height;
  start_pose.pose.orientation = down_orientation;
  
  ROS_INFO("Moving to initial scanning position...");
  bool success = moveArm(start_pose);
  if (!success) {
    ROS_WARN("Failed to move to initial scanning position. Using current position.");
  }

  // Reset collection variables
  collected_clouds_.clear();
  cloud_frame_counter_ = 0;
  
  // Wait for a moment to start collecting data
  ros::Duration(1.0).sleep();
  
  // Now execute each edge as a separate Cartesian path
  float speed_factor = 0.05;
  
  // Edge 1: Bottom edge (x from min to max, y = min)
  std::vector<geometry_msgs::Pose> edge1_waypoints;
  for (int i = 0; i <= points_per_edge; i++) {
    float x = rect_x_min + i * (rect_x_max - rect_x_min) / points_per_edge;
    
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = rect_y_min;
    pose.position.z = scan_height;
    pose.orientation = down_orientation;
    edge1_waypoints.push_back(pose);
  }
  
  ROS_INFO("Scanning bottom edge...");
  is_collecting_clouds_ = true;  // Start collecting clouds
  moveAlongCartesianPath(edge1_waypoints, eef_step, jump_threshold, speed_factor); // Slowed down to 20% speed
  is_collecting_clouds_ = false;
  
  // Edge 2: Right edge (x = max, y from min to max)
  std::vector<geometry_msgs::Pose> edge2_waypoints;
  for (int i = 0; i <= points_per_edge; i++) {
    float y = rect_y_min + i * (rect_y_max - rect_y_min) / points_per_edge;
    
    geometry_msgs::Pose pose;
    pose.position.x = rect_x_max;
    pose.position.y = y;
    pose.position.z = scan_height;
    pose.orientation = down_orientation;
    edge2_waypoints.push_back(pose);
  }
  
  ROS_INFO("Scanning right edge...");
  is_collecting_clouds_ = true;  // Start collecting clouds
  moveAlongCartesianPath(edge2_waypoints, eef_step, jump_threshold, speed_factor); // Slowed down to 20% speed
  is_collecting_clouds_ = false;
  
  // Edge 3: Top edge (x from max to min, y = max)
  std::vector<geometry_msgs::Pose> edge3_waypoints;
  for (int i = 0; i <= points_per_edge; i++) {
    float x = rect_x_max - i * (rect_x_max - rect_x_min) / points_per_edge;
    
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = rect_y_max;
    pose.position.z = scan_height;
    pose.orientation = down_orientation;
    edge3_waypoints.push_back(pose);
  }
  
  ROS_INFO("Scanning top edge...");
  is_collecting_clouds_ = true;  // Start collecting clouds
  moveAlongCartesianPath(edge3_waypoints, eef_step, jump_threshold, speed_factor); // Slowed down to 20% speed
  is_collecting_clouds_ = false;  // Start collecting clouds
  
  // Edge 4: Left edge (x = min, y from max to min)
  std::vector<geometry_msgs::Pose> edge4_waypoints;
  for (int i = 0; i <= points_per_edge; i++) {
    float y = rect_y_max - i * (rect_y_max - rect_y_min) / points_per_edge;
    
    geometry_msgs::Pose pose;
    pose.position.x = rect_x_min;
    pose.position.y = y;
    pose.position.z = scan_height;
    pose.orientation = down_orientation;
    edge4_waypoints.push_back(pose);
  }
  
  ROS_INFO("Scanning left edge...");
  is_collecting_clouds_ = true;  // Start collecting clouds
  moveAlongCartesianPath(edge4_waypoints, eef_step, jump_threshold, speed_factor); // Slowed down to 20% speed
  is_collecting_clouds_ = false;  // Start collecting clouds
  
  // Merge collected clouds
  ROS_INFO("Merging %zu collected clouds...", collected_clouds_.size());
  PointCPtr merged_scene = mergeClouds(collected_clouds_);
  
  // Free memory
  collected_clouds_.clear();
  
  ROS_INFO("Continuous scanning complete, collected %d point clouds", cloud_frame_counter_);
  
  return merged_scene;
}

/**
 * Helper method to move the arm along a Cartesian path
 * @param waypoints List of poses defining the path
 * @param eef_step Step size for end effector
 * @param jump_threshold Jump threshold (0.0 to disable)
 * @param speed_factor Speed factor (0.0-1.0, lower is slower)
 * @return true if successful
 */
bool cw2::moveAlongCartesianPath(
    const std::vector<geometry_msgs::Pose>& waypoints,
    double eef_step,
    double jump_threshold,
    double speed_factor) {
  
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  if (fraction > 0.5) {  // Accept the path if at least 50% is achievable
    // Slow down the trajectory for better scanning
    robot_trajectory::RobotTrajectory rt(arm_group_.getCurrentState()->getRobotModel(), "panda_arm");
    rt.setRobotTrajectoryMsg(*arm_group_.getCurrentState(), trajectory);
    
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, speed_factor);
    rt.getRobotTrajectoryMsg(trajectory);
    
    // Execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    
    ROS_INFO("Executing Cartesian trajectory (%.1f%% coverage) with %zu waypoints...",
            fraction * 100.0, waypoints.size());
    arm_group_.execute(plan);
    return true;
  } else {
    ROS_WARN("Could only compute %.1f%% of the desired Cartesian path, aborting this segment",
            fraction * 100.0);
    return false;
  }
}

/**
 * Calculates a grasp pose for an object based on original Task 1 grasp logic
 * Adapts offset distances based on actual object dimensions from point cloud
 * 
 * @param centroid Object centroid
 * @param is_cross Whether the object is a cross (true) or nought (false)
 * @param orientation PCA orientation data for the object
 * @param object_cloud Point cloud of the object for size estimation
 * @return Grasp pose for the object
 */
geometry_msgs::PoseStamped cw2::calculateGraspPose(
    const Eigen::Vector4f& centroid,
    bool is_cross,
    const ObjectOrientationData& orientation) {
  
  ROS_INFO("Calculating grasp pose using original Task 1 approach");
  
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = base_frame_;
  
  // Extract principal axis and secondary axis
  Eigen::Vector3f principal_axis = orientation.primary_axis;
  Eigen::Vector3f secondary_axis = orientation.secondary_axis;
  
  // Calculate base orientation
  tf2::Quaternion q_orig;
  tf2::convert(grasp_orientation_, q_orig); // Use the standard grasp orientation
  
  // Project principal axis to XY plane for consistent calculation
  Eigen::Vector3f principal_axis_xy = principal_axis;
  principal_axis_xy[2] = 0.0;
  if (principal_axis_xy.norm() > 0.001) {
    principal_axis_xy.normalize();
  }
  
  // Default grasp offset distances
  float offset;
  float grasp_x, grasp_y;
  tf2::Quaternion q_rot, q_final;
  
  // Calculate offset and orientation based on shape type
  if (is_cross) {
    ROS_INFO("Calculating grasp for cross shape...");
    
    // Estimate cross arm length using flatness ratio (if available)
    if (orientation.flatness_ratio > 0) {
      // Calculate an appropriate offset based on object dimensions
      // For crosses, we want to grasp one arm at approximately 60-75% of its length
      offset = 0.06 * orientation.flatness_ratio; // Scale by flatness ratio
      offset = std::min(std::max(offset, 0.04f), 0.08f); // Clamp between 4-8cm
    } else {
      offset = 0.06; // Default 6cm if no size info
    }
    
    // Calculate grasp point by offsetting along principal axis
    grasp_x = centroid[0] + principal_axis_xy[0] * offset;
    grasp_y = centroid[1] + principal_axis_xy[1] * offset;
    
    // Rotate gripper to align with cross arm
    float grasp_angle = std::atan2(principal_axis_xy[1], principal_axis_xy[0]);
    q_rot.setRPY(0, 0, grasp_angle);
    
    ROS_INFO("Cross grasp point: [%f, %f] (offset by %fcm along principal axis)",
             grasp_x, grasp_y, offset*100);
    
  } else { // "nought"
    ROS_INFO("Calculating grasp for nought shape...");
    
    // Calculate midpoint angle between primary and secondary axes
    float primary_angle = atan2(principal_axis_xy[1], principal_axis_xy[0]);
    
    // For noughts, we need the secondary axis for proper grasping
    Eigen::Vector3f secondary_axis_xy = secondary_axis;
    secondary_axis_xy[2] = 0.0;
    if (secondary_axis_xy.norm() > 0.001) {
      secondary_axis_xy.normalize();
    }
    
    float secondary_angle = atan2(secondary_axis_xy[1], secondary_axis_xy[0]);
    
    // Handle angle wrap-around
    float angle_diff = secondary_angle - primary_angle;
    if (angle_diff > M_PI) angle_diff -= 2*M_PI;
    if (angle_diff < -M_PI) angle_diff += 2*M_PI;
    
    float midpoint_angle = primary_angle + angle_diff/2.0;
    
    // Calculate grasp direction using midpoint angle
    Eigen::Vector3f grasp_direction_xy;
    grasp_direction_xy[0] = cos(midpoint_angle);
    grasp_direction_xy[1] = sin(midpoint_angle);
    grasp_direction_xy[2] = 0.0;
    
    // For noughts, estimate radius using flatness ratio
    if (orientation.flatness_ratio > 0) {
      // Calculate appropriate edge offset based on object size
      // For noughts, we want to grasp at the edge
      offset = 0.08 * orientation.flatness_ratio; // Scale by flatness ratio
      offset = std::min(std::max(offset, 0.05f), 0.10f); // Clamp between 5-10cm
    } else {
      offset = 0.08; // Default 8cm if no size info
    }
    
    // Calculate grasp point by offsetting along the midpoint direction
    grasp_x = centroid[0] + grasp_direction_xy[0] * offset;
    grasp_y = centroid[1] + grasp_direction_xy[1] * offset;
    
    // Add 90 degrees to midpoint angle for proper gripper alignment
    float grasp_angle = midpoint_angle + M_PI/2.0;
    q_rot.setRPY(0, 0, grasp_angle);
    
    ROS_INFO("Nought grasp: angles[p:%f,s:%f,m:%f], offset:%fcm", 
             primary_angle, secondary_angle, midpoint_angle, offset*100);
  }
  
  // Combine rotations and normalize
  q_final = q_rot * q_orig;
  q_final.normalize();
  
  // Set final pose with appropriate Z height
  grasp_pose.pose.position.x = grasp_x;
  grasp_pose.pose.position.y = grasp_y;
  grasp_pose.pose.position.z = centroid[2] + t3_grasp_height_offset_;
  grasp_pose.pose.orientation = tf2::toMsg(q_final);
  
  ROS_INFO("Final grasp pose at [%.3f, %.3f, %.3f] with orientation [%.3f, %.3f, %.3f, %.3f]",
          grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
          grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
          grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
  
  return grasp_pose;
}

// 在Task3的处理代码中，在调用planAndExecuteGrasp之前添加

// 计算从中心点到边缘的距离，并向内缩10mm作为偏移量
float cw2::calculateGraspOffset(PointCPtr object_cloud, const Eigen::Vector4f& centroid, 
                           const Eigen::Vector3f& grasp_axis, bool is_cross) {
  // 默认偏移值
  float default_offset = is_cross ? 0.06 : 0.08;
  
  // 如果点云为空，返回默认值
  if (object_cloud->empty()) {
    ROS_WARN("Empty point cloud, using default offset: %.1fmm", default_offset * 1000.0);
    return default_offset;
  }
  
  // 找到沿抓取轴方向最远的点
  float max_dist = 0.0f;
  
  for (const auto& point : object_cloud->points) {
    // 计算点到中心的向量
    Eigen::Vector3f point_vector(point.x - centroid[0], 
                                 point.y - centroid[1], 
                                 0); // 只考虑XY平面
    
    // 计算在抓取轴方向上的距离
    float projection = point_vector.dot(grasp_axis);
    
    // 只考虑正方向（即抓取方向）上的点
    if (projection > 0) {
      max_dist = std::max(max_dist, projection);
    }
  }
  
  // 如果没有找到合适的点，返回默认值
  if (max_dist < 0.01) { // 小于1cm认为无效
    ROS_WARN("Could not find valid edge point, using default offset: %.1fmm", default_offset * 1000.0);
    return default_offset;
  }
  
  // 从边缘向内缩10mm
  float offset = max_dist - 0.01; // 10mm内缩
  
  ROS_INFO("Calculated grasp offset: %.1fmm (edge distance: %.1fmm, inset: 10mm)", 
           offset * 1000.0, max_dist * 1000.0);
  
  return offset;
}
