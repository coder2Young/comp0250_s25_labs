/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh):
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  cloud_(new PointC),
  collision_object_vector_()
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
  
  ROS_INFO("cw2 class initialised");
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
  hand_offset_ = 0.16;
  gripper_open_ = 0.08;
  gripper_closed_ = 0.0;
  grasp_stanby_height_ = 0.1;
  place_stanby_height_ = 0.1;
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
  cluster_tolerance_ = 0.02;    // 2cm tolerance between points in cluster
  min_cluster_size_ = 50;       // Minimum 50 points per cluster
  max_cluster_size_ = 25000;    // Maximum 25000 points per cluster

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
    ROS_INFO("Debug mode is enabled - will provide extended logging and return to home after completion");
  }

  // Make sure the robot is configured
  cw2_config();

  // Extract required information from request
  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("====== TASK DETAILS ======");
  ROS_INFO("Object point: %f, %f, %f", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal point: %f, %f, %f", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape type: %s", shape_type.c_str());
  ROS_INFO("==========================\n");
  
  // 1. Move to scanning position
  bool scan_success = moveToScanPosition(object_point.point);
  
  // 2. Get filtered point cloud
  PointCPtr filtered_cloud = getFilteredPointCloud();
  
  // Publish filtered cloud only in debug mode
  if (debug_) {
    publishPointCloud(filtered_cloud, cloud_filtered_pub_);
  }
  
  // 3. Extract object from point cloud
  PointCPtr object_cloud = 
      extractObjectPointCloud(filtered_cloud, object_point.point);
  
  // Publish object cloud only in debug mode
  if (debug_) {
    publishPointCloud(object_cloud, cloud_object_pub_);
  }
  
  // 4. Determine object orientation
  ObjectOrientationData orientation_data = 
      determineObjectOrientation(object_cloud, shape_type);
  
  // 5. Plan and execute grasp
  bool grasp_success = 
      planAndExecuteGrasp(object_point.point, orientation_data, shape_type);
  
  // 6. Plan and execute place
  bool place_success = planAndExecutePlace(goal_point.point);

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

PointCPtr cw2::getFilteredPointCloud() {
  ROS_INFO("Acquiring and filtering point cloud data");
  
  // Wait for point cloud message
  sensor_msgs::PointCloud2ConstPtr cloud_msg = 
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/r200/camera/depth_registered/points", nh_, ros::Duration(5.0));
  
  if (!cloud_msg) {
    ROS_ERROR("Failed to receive point cloud message");
    return PointCPtr(new PointC);
  }
  
  // Save the original frame ID for transformation
  std::string camera_frame_id = cloud_msg->header.frame_id;
  ROS_INFO("Received point cloud in frame: %s", camera_frame_id.c_str());
  
  // Convert ROS message to PCL point cloud (still in camera frame)
  PointCPtr cloud_camera(new PointC);
  pcl::fromROSMsg(*cloud_msg, *cloud_camera);
  
  // Transform the point cloud from camera frame to base frame
  PointCPtr cloud_base(new PointC);
  try {
    // Look up transform from camera to base frame
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform(
      base_frame_, camera_frame_id, ros::Time(0), ros::Duration(1.0));
    
    // Apply transform to the point cloud
    pcl_ros::transformPointCloud(*cloud_camera, *cloud_base, transform_stamped.transform);
    ROS_INFO("Successfully transformed point cloud to %s frame", base_frame_.c_str());
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Transform lookup failed: %s", ex.what());
    ROS_WARN("Continuing with untransformed cloud - results may be incorrect!");
    cloud_base = cloud_camera; // Fallback to untransformed cloud
  }
  
  // Filter by color - only keep red/blue/purple/black objects
  PointCPtr color_filtered_cloud(new PointC);
  color_filtered_cloud->header = cloud_base->header;
  
  for (const auto& pt : cloud_base->points) {
    float r = static_cast<float>(pt.r) / 255.0f;
    float g = static_cast<float>(pt.g) / 255.0f;
    float b = static_cast<float>(pt.b) / 255.0f;

    bool isPurple = (r > 0.7f && r < 0.9f) &&
                   (g > 0.0f && g < 0.2f) &&
                   (b > 0.7f && b < 0.9f);

    bool isRed = (r > 0.7f && r < 0.9f) &&
                (g > 0.0f && g < 0.2f) &&
                (b > 0.0f && b < 0.2f);

    bool isBlue = (r > 0.0f && r < 0.2f) &&
                 (g > 0.0f && g < 0.2f) &&
                 (b > 0.7f && b < 0.9f);
                 
    bool isBlack = (r < 0.2f) && (g < 0.2f) && (b < 0.2f);

    if (isPurple || isRed || isBlue || isBlack) {
      color_filtered_cloud->points.push_back(pt);
    }
  }
  
  color_filtered_cloud->width = color_filtered_cloud->points.size();
  color_filtered_cloud->height = 1;
  
  ROS_INFO("Color filtering: kept %lu of %lu points", 
           color_filtered_cloud->points.size(), cloud_base->points.size());

  // Apply voxel grid filter to downsample
  pcl::VoxelGrid<PointT> voxel_filter;
  PointCPtr cloud_filtered(new PointC);
  voxel_filter.setInputCloud(color_filtered_cloud);
  voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm voxel size
  voxel_filter.filter(*cloud_filtered);
  
  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);
  
  ROS_INFO("After voxel grid filtering: %lu points", cloud_filtered->points.size());
  
  return cloud_filtered;
}

PointCPtr cw2::extractObjectPointCloud(
    PointCPtr cloud,
    const geometry_msgs::Point &object_center) {
  
  ROS_INFO("\n====== EXTRACTING OBJECT POINT CLOUD ======");
  ROS_INFO("Extracting object from point cloud using color filtering and clustering");
  
  // Create output cloud for color filtering
  PointCPtr color_filtered_cloud(new PointC);
  
  // Filter by color - keep points that are not green (ground)
  for (const auto& pt : cloud->points) {
    float r = static_cast<float>(pt.r) / 255.0f;
    float g = static_cast<float>(pt.g) / 255.0f;
    float b = static_cast<float>(pt.b) / 255.0f;
    
    bool isGreen = (g > 0.4f && g > r * 1.5 && g > b * 1.5) || 
                   (g > 0.5f && (r < 0.35f || b < 0.35f));
    
    if (!isGreen) {
      color_filtered_cloud->points.push_back(pt);
    }
  }
  
  color_filtered_cloud->width = color_filtered_cloud->points.size();
  color_filtered_cloud->height = 1;
  color_filtered_cloud->is_dense = true;
  
  ROS_INFO("After color filtering: %lu points", color_filtered_cloud->points.size());
  
  // If no points after color filtering, return empty cloud
  if (color_filtered_cloud->points.empty()) {
    ROS_ERROR("No points after color filtering");
    return color_filtered_cloud;
  }
  
  // Create KdTree for clustering
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(color_filtered_cloud);
  
  // Extract Euclidean clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(color_filtered_cloud);
  ec.extract(cluster_indices);
  
  ROS_INFO("Found %lu clusters", cluster_indices.size());
  
  // If no clusters found, return empty cloud
  if (cluster_indices.empty()) {
    ROS_ERROR("No clusters found");
    return PointCPtr(new PointC);
  }
  
  // Find the cluster closest to expected object position
  int best_cluster_idx = -1;
  float min_distance = std::numeric_limits<float>::max();
  
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    // Calculate centroid of cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*color_filtered_cloud, cluster_indices[i], centroid);
    
    // Calculate distance to expected object center
    float dx = centroid[0] - object_center.x;
    float dy = centroid[1] - object_center.y;
    float dz = centroid[2] - object_center.z;
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    ROS_INFO("Cluster %lu: %lu points, center [%f, %f, %f], distance to expected: %f", 
             i, cluster_indices[i].indices.size(), centroid[0], centroid[1], centroid[2], distance);
    
    if (distance < min_distance) {
      min_distance = distance;
      best_cluster_idx = i;
    }
  }
  
  // Extract the best cluster
  PointCPtr object_cloud(new PointC);
  if (best_cluster_idx >= 0) {
    for (const auto& idx : cluster_indices[best_cluster_idx].indices) {
      object_cloud->points.push_back(color_filtered_cloud->points[idx]);
    }
    object_cloud->width = object_cloud->points.size();
    object_cloud->height = 1;
    object_cloud->is_dense = true;
    
    ROS_INFO("Selected cluster %d with %lu points at distance %f", 
             best_cluster_idx, object_cloud->points.size(), min_distance);
  } else {
    ROS_ERROR("Failed to find suitable cluster");
  }
  
  current_object_cloud_ = object_cloud;  // Store for later use
  ROS_INFO("====== OBJECT EXTRACTION COMPLETED ======\n");
  
  return object_cloud;
}

ObjectOrientationData cw2::determineObjectOrientation(
    PointCPtr object_cloud,
    const std::string &shape_type) {
  
  ROS_INFO("\n====== DETERMINING OBJECT ORIENTATION ======");
  
  // 初始化结果结构
  ObjectOrientationData result;
  result.is_valid = false;
  
  if (object_cloud->points.size() < 3) {
    ROS_ERROR("Not enough points for PCA analysis");
    return result;
  }
  
  // Perform PCA on the object cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(object_cloud);
  
  // Get eigenvalues and eigenvectors
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  
  ROS_INFO("PCA eigenvalues: [%f, %f, %f]", 
           eigenvalues[0], eigenvalues[1], eigenvalues[2]);
  
  Eigen::Vector3f grasp_direction;
  float grasp_angle;

  if (shape_type == "cross") {
    Eigen::Vector3f primary_axis = eigenvectors.col(0);
    grasp_direction = primary_axis;
    grasp_angle = atan2(primary_axis[1], primary_axis[0]) + M_PI;
  } else { // "nought"
    Eigen::Vector3f primary_axis = eigenvectors.col(2);
    Eigen::Vector3f secondary_axis = eigenvectors.col(0);
    
    Eigen::Vector3f edge_direction;
    edge_direction[0] = primary_axis[0] + secondary_axis[0];
    edge_direction[1] = primary_axis[1] + secondary_axis[1];
    edge_direction[2] = 0.0f;
    edge_direction.normalize();
    
    grasp_direction = edge_direction;
    grasp_angle = atan2(edge_direction[1], edge_direction[0]) + M_PI/2;
  }

  // 在这里需要计算物体中心点，以用于可视化
  geometry_msgs::Point object_center;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*object_cloud, centroid);
  object_center.x = centroid[0];
  object_center.y = centroid[1];
  object_center.z = centroid[2];
  
  // 使用正确的变量名 object_center 替代 object_point
  visualizePCAAxes(eigenvectors, object_center, shape_type, grasp_direction, grasp_angle);
  
  result.is_valid = true;
  ROS_INFO("====== ORIENTATION DETERMINATION COMPLETED ======\n");
  
  return result;
}

bool cw2::planAndExecuteGrasp(
    const geometry_msgs::Point &object_point,
    const ObjectOrientationData &orientation_data,
    const std::string &shape_type) {
  
  ROS_INFO("\n====== PLANNING AND EXECUTING GRASP ======");
  
  if (!orientation_data.is_valid) {
    ROS_ERROR("Invalid orientation data for grasping");
    return false;
  }
  
  ROS_INFO("Planning grasp strategy for %s object", shape_type.c_str());
  
  // Calculate grasp pose based on object orientation
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = base_frame_;
  
  // Use the predefined downward-facing orientation as base
  tf2::Quaternion q_base;
  tf2::convert(grasp_orientation_, q_base);
  
  // Create quaternion for Z-axis rotation based on object orientation
  tf2::Quaternion q_z;
  
  if (shape_type == "cross") {
    // For cross objects, grasp along one arm, near the outer edge
    float arm_length = 0.08; // 80mm - move 2 cubes out from center
    
    // Set grasp angle perpendicular to the arm with extra 90-degree rotation for proper gripper alignment
    float grasp_angle = atan2(orientation_data.primary_axis[1], orientation_data.primary_axis[0]) + M_PI; // Full 180-degree rotation
    q_z.setRPY(0, 0, grasp_angle);
    
    // Move from center along arm direction
    grasp_pose.pose.position = object_point;
    grasp_pose.pose.position.x += arm_length * orientation_data.primary_axis[0];
    grasp_pose.pose.position.y += arm_length * orientation_data.primary_axis[1];
    grasp_pose.pose.position.z += hand_offset_;
    
    ROS_INFO("Cross grasp strategy:");
    ROS_INFO("- Grasping along arm at distance %.1f mm from center", arm_length * 1000);
    ROS_INFO("- Gripper perpendicular to arm at angle %.1f degrees", grasp_angle * 180.0/M_PI);
    ROS_INFO("- Grasp point: [%f, %f, %f]", 
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  } 
  else { // "nought"
    // For ring-shaped objects, grasp the middle of an edge
    // Use 45-degree angle between primary and secondary axes (diagonal directions)
    
    // Calculate the 45-degree direction between primary and secondary axes
    Eigen::Vector3f edge_direction;
    edge_direction[0] = orientation_data.primary_axis[0] + orientation_data.secondary_axis[0];
    edge_direction[1] = orientation_data.primary_axis[1] + orientation_data.secondary_axis[1];
    edge_direction[2] = 0.0f; // Keep it in the horizontal plane
    edge_direction.normalize();
    
    // Use 100mm distance to reach the edge from center
    float edge_distance = 0.1; // 100mm
    
    // Set grasp angle perpendicular to the edge for better grip (add 90 degrees)
    float grasp_angle = atan2(edge_direction[1], edge_direction[0]) + M_PI/2;
    q_z.setRPY(0, 0, grasp_angle);
    
    // Move from center along the 45-degree direction
    grasp_pose.pose.position = object_point;
    grasp_pose.pose.position.x += edge_distance * edge_direction[0];
    grasp_pose.pose.position.y += edge_distance * edge_direction[1];
    grasp_pose.pose.position.z += hand_offset_;
    
    ROS_INFO("Square ring grasp strategy:");
    ROS_INFO("- Using 45-degree direction between principal axes");
    ROS_INFO("- Direction vector: [%.2f, %.2f]", edge_direction[0], edge_direction[1]);
    ROS_INFO("- Grasping at distance %.1f mm from center", edge_distance * 1000);
    ROS_INFO("- Gripper perpendicular to edge at angle %.1f degrees", grasp_angle * 180.0/M_PI);
    ROS_INFO("- Grasp point: [%f, %f, %f]", 
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  }
  
  // Combine rotations (first apply base orientation, then Z rotation)
  tf2::Quaternion q_rot = q_z * q_base;
  q_rot.normalize();
  
  // Convert to geometry_msgs quaternion
  geometry_msgs::Quaternion q_msg;
  tf2::convert(q_rot, q_msg);
  grasp_pose.pose.orientation = q_msg;
  
  // Output final orientation for debugging
  tf2::Matrix3x3 m(q_rot);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Final gripper orientation - Roll: %.2f, Pitch: %.2f, Yaw: %.2f (degrees)",
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
  
  ROS_INFO("Executing grasp sequence...");
  
  // Make sure we open gripper to the specified width
  ROS_INFO("Opening gripper to maximum width...");
  moveGripper(gripper_open_);
  
  // Log the actual gripper opening for debugging
  ROS_INFO("Gripper opened to %.3f meters", gripper_open_);
  
  // Move to position above object
  ROS_INFO("Moving to pre-grasp position...");
  geometry_msgs::PoseStamped pregrasp_pose = grasp_pose;
  pregrasp_pose.pose.position.z += grasp_stanby_height_;
  
  // Log the pre-grasp position for debugging
  ROS_INFO("Pre-grasp position: [%f, %f, %f]", 
           pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z);
  
  bool success = moveArm(pregrasp_pose);
  if (!success) {
    ROS_ERROR("Failed to move to pre-grasp position");
    return false;
  }
  
  // Move to grasp position
  ROS_INFO("Moving to grasp position...");
  ROS_INFO("Grasp position: [%f, %f, %f]", 
           grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  success = moveArm(grasp_pose);
  
  // Close gripper
  ROS_INFO("Closing gripper to grasp object...");
  moveGripper(gripper_closed_);
  
  ROS_INFO("==== Grasp position: [%f, %f, %f] ====", 
           grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  ROS_INFO("==== Grasp orientation: [%f, %f, %f, %f] ====", 
           grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, 
           grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
  
  // Wait for gripper to close
  ros::Duration(0.5).sleep();
  
  // Lift object
  ROS_INFO("Lifting object...");
  pregrasp_pose.pose.position.z += 0.1; // Additional lift
  success = moveArm(pregrasp_pose);
  
  if (success) {
    ROS_INFO("Grasp and lift successful");
  } else {
    ROS_ERROR("Failed to lift object");
  }
  
  ROS_INFO("====== GRASP EXECUTION COMPLETED ======\n");
  return success;
}

bool cw2::planAndExecutePlace(const geometry_msgs::Point &goal_point) {
  ROS_INFO("\n====== PLANNING AND EXECUTING PLACE ======");
  ROS_INFO("Planning to place object at [%f, %f, %f]", 
           goal_point.x, goal_point.y, goal_point.z);
  
  // Create place pose
  geometry_msgs::PoseStamped place_pose;
  place_pose.header.frame_id = base_frame_;
  
  // Simplified approach: Move to 0.2m above goal point
  place_pose.pose.position = goal_point;
  place_pose.pose.position.z += 0.3;  // 20cm above goal point
  place_pose.pose.orientation = grasp_orientation_;
  
  ROS_INFO("Moving to place position (20cm above goal)...");
  bool success = moveArm(place_pose);
  if (!success) {
    ROS_ERROR("Failed to move to place position");
    return false;
  }
  
  // Open gripper to release object
  ROS_INFO("Opening gripper to release object...");
  moveGripper(gripper_open_);
  
  // Wait for release
  ros::Duration(0.5).sleep();
  
  // If debug mode is enabled, return to home position
  if (debug_) {
    ROS_INFO("Debug mode: Returning to home position...");
    
    // Create home pose
    geometry_msgs::PoseStamped home_pose;
    home_pose.header.frame_id = base_frame_;
    
    // Use a safe position as home
    home_pose.pose.position.x = 0.4;  // Forward
    home_pose.pose.position.y = 0.0;  // Center
    home_pose.pose.position.z = 0.6;  // Up
    home_pose.pose.orientation = grasp_orientation_;
    
    success = moveArm(home_pose);
    if (!success) {
      ROS_WARN("Failed to return to home position");
      // Continue anyway, not critical
    } else {
      ROS_INFO("Successfully returned to home position");
    }
  }
  
  ROS_INFO("====== PLACE EXECUTION COMPLETED ======\n");
  return true;  // Return true even if home position fails, as the main task succeeded
}

void cw2::publishPointCloud(
    const PointCPtr &cloud,
    const ros::Publisher &publisher) {
  
  // Only proceed if in debug mode
  if (!debug_) {
    return;
  }
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  
  // All point clouds are now in base_frame
  cloud_msg.header.frame_id = base_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  
  publisher.publish(cloud_msg);
  
  ROS_INFO("Published point cloud with %lu points in frame %s", 
           cloud->points.size(), cloud_msg.header.frame_id.c_str());
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

// Directly copied from cw1_class.cpp
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
    const Eigen::Vector3f &grasp_direction,  // Pass in the actual grasp direction
    float grasp_angle) {                     // Pass in the actual grasp angle
  
  // Skip visualization if not in debug mode
  if (!debug_) {
    return;
  }
  
  ROS_INFO("Visualizing PCA axes in RViz");
  
  visualization_msgs::MarkerArray marker_array;
  
  // Create arrow markers for three principal axes
  for (int i = 0; i < 3; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pca_axes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Set arrow starting point to object center
    marker.pose.position = center_point;
    
    // Calculate arrow direction quaternion
    Eigen::Vector3f axis = eigenvectors.col(i);
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f rotation_axis = z_axis.cross(axis);
    
    if (rotation_axis.norm() > 1e-6) {
      float angle = acos(z_axis.dot(axis) / axis.norm());
      rotation_axis.normalize();
      
      tf2::Quaternion q;
      q.setRotation(tf2::Vector3(rotation_axis[0], rotation_axis[1], rotation_axis[2]), angle);
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();
    }
    
    // Set arrow size
    float scale_factor = 0.15;  // Principal axis length
    marker.scale.x = scale_factor; // Arrow length
    marker.scale.y = 0.01;        // Arrow width
    marker.scale.z = 0.01;        // Arrow height
    
    // Set color - Red=first axis, Green=second axis, Blue=third axis
    marker.color.a = 1.0;  // Opacity
    if (i == 0) {
      marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    } else if (i == 1) {
      marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    }
    
    // Set duration (0 means permanent)
    marker.lifetime = ros::Duration(0);
    
    // Add to marker array
    marker_array.markers.push_back(marker);
  }
  
  // Add marker indicating grasp position and orientation
  visualization_msgs::Marker grasp_marker;
  grasp_marker.header.frame_id = base_frame_;
  grasp_marker.header.stamp = ros::Time::now();
  grasp_marker.ns = "grasp_direction";
  grasp_marker.id = 3;
  grasp_marker.type = visualization_msgs::Marker::ARROW;
  grasp_marker.action = visualization_msgs::Marker::ADD;
  
  // Use the passed-in grasp direction
  float distance = 0.08;
  
  grasp_marker.pose.position = center_point;
  grasp_marker.pose.position.x += distance * grasp_direction[0];
  grasp_marker.pose.position.y += distance * grasp_direction[1];
  
  // Set the grasp orientation using passed angle
  tf2::Quaternion q;
  q.setRPY(0, 0, grasp_angle);
  grasp_marker.pose.orientation.x = q.x();
  grasp_marker.pose.orientation.y = q.y();
  grasp_marker.pose.orientation.z = q.z();
  grasp_marker.pose.orientation.w = q.w();
  
  grasp_marker.scale.x = 0.12;    // Arrow length
  grasp_marker.scale.y = 0.02;    // Arrow width
  grasp_marker.scale.z = 0.02;    // Arrow height
  
  // Set color - Yellow for grasp direction
  grasp_marker.color.r = 1.0;
  grasp_marker.color.g = 1.0;
  grasp_marker.color.b = 0.0;
  grasp_marker.color.a = 1.0;
  
  grasp_marker.lifetime = ros::Duration(0);
  
  marker_array.markers.push_back(grasp_marker);
  
  // Publish marker array
  pca_axes_pub_.publish(marker_array);
  ROS_INFO("PCA axes and grasp direction visualization published");
}
