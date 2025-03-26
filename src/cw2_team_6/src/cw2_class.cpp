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
  collision_object_vector_(),
  octomap_received_(false),
  // Initialize scanning motion parameters
  scan_radius_(0.2),          // 20cm radius around object 
  scan_height_offset_(0.3),   // 30cm above object height
  num_scan_poses_(4)          // 4 positions around the object
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
  // Use for debug visualization, not for OctoMap input
  filtered_cloud_for_octomap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/filtered_cloud", 1, true);
  
  // Initialize OctoMap-related subscribers and clients
  octomap_sub_ = nh_.subscribe("/octomap_binary", 1, &cw2::octomap_callback, this);
  octomap_client_ = nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");

  cw2_config();
  
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
  hand_offset_ = 0.15;
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
  
  // Scanning motion parameters
  num_scan_poses_ = 4;          // Number of scan poses around the object
  scan_radius_ = 0.1;          // Distance from object center (25 cm)
  scan_height_offset_ = 0.5;    // Height above the object (50 cm)

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

  // Extract required information from request
  geometry_msgs::PointStamped object_point = request.object_point;
  geometry_msgs::PointStamped goal_point = request.goal_point;
  std::string shape_type = request.shape_type;

  ROS_INFO("====== TASK DETAILS ======");
  ROS_INFO("Object point: %f, %f, %f", object_point.point.x, object_point.point.y, object_point.point.z);
  ROS_INFO("Goal point: %f, %f, %f", goal_point.point.x, goal_point.point.y, goal_point.point.z);
  ROS_INFO("Shape type: %s", shape_type.c_str());
  ROS_INFO("==========================\n");
  
  // 1. Move to initial scanning position above the object
  bool scan_success = moveToScanPosition(object_point.point);
  
  // 2. Perform scanning motion around the object to collect better point cloud data
  bool motion_success = performScanningMotion(object_point.point);
  
  // 3. Extract point cloud from OctoMap
  PointCPtr point_cloud = extractPointCloudFromOctomap();
  
  // 4. Skip the camera point cloud, use only OctoMap data as per new strategy
  // Note: We're no longer combining OctoMap and camera point clouds
  PointCPtr filtered_cloud = point_cloud;
  
  // Publish filtered cloud only in debug mode
  if (debug_) {
    publishPointCloud(filtered_cloud, cloud_filtered_pub_);
  }
  
  // 5. Extract object from point cloud
  PointCPtr object_cloud = 
      extractObjectPointCloud(filtered_cloud, object_point.point);
  
  // Publish object cloud only in debug mode
  if (debug_) {
    publishPointCloud(object_cloud, cloud_object_pub_);
  }
  
  // 6. Determine object orientation
  ObjectOrientationData orientation_data = 
      determineObjectOrientation(object_cloud, shape_type);
  
  // 7. Plan and execute grasp
  bool grasp_success = 
      planAndExecuteGrasp(object_point.point, orientation_data, shape_type);
  
  // 8. Plan and execute place
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

PointCPtr cw2::extractObjectPointCloud(
    PointCPtr cloud,
    const geometry_msgs::Point &object_center) {
  
  ROS_INFO("\n====== EXTRACTING OBJECT POINT CLOUD ======");
  ROS_INFO("Extracting object from point cloud using clustering");
  
  // The input cloud is already color filtered, so we can proceed directly to clustering
  
  // If no points in the cloud, return empty cloud
  if (cloud->points.empty()) {
    ROS_ERROR("Input cloud has no points");
    return PointCPtr(new PointC);
  }
  
  // Create KdTree for clustering
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  
  // Extract Euclidean clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
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
    pcl::compute3DCentroid(*cloud, cluster_indices[i], centroid);
    
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
      object_cloud->points.push_back(cloud->points[idx]);
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
    grasp_angle = atan2(primary_axis[1], primary_axis[0]) + M_PI; // Full 180-degree rotation
    
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
    
    // Since we know the object is flat on the ground, enforce vertical normal
    primary_axis = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    
    // First principal axis in the XY plane (largest variance direction)
    secondary_axis = eigenvectors.col(0);
    secondary_axis[2] = 0.0f;  // Project onto XY plane
    secondary_axis.normalize();
    
    // For optimal grasping of a square ring, calculate an edge direction
    // Using a 45-degree angle between the principal axes in the plane gives good results
    Eigen::Vector3f edge_direction;
    edge_direction[0] = secondary_axis[0] + eigenvectors.col(1)[0];
    edge_direction[1] = secondary_axis[1] + eigenvectors.col(1)[1];
    edge_direction[2] = 0.0f; // Keep it in the horizontal plane
    edge_direction.normalize();
    
    // Store this edge direction for grasp point calculation
    grasp_direction = edge_direction;
    
    // Set grasp angle perpendicular to the edge for better grip
    grasp_angle = atan2(edge_direction[1], edge_direction[0]) + M_PI/2; // Add 90 degrees
    
    ROS_INFO("Ring orientation - Normal: [%f, %f, %f]", 
             primary_axis[0], primary_axis[1], primary_axis[2]);
    ROS_INFO("Ring in-plane principal axis: [%f, %f, %f]", 
             secondary_axis[0], secondary_axis[1], secondary_axis[2]);
    ROS_INFO("Ring edge direction (for grasping): [%f, %f, %f]", 
             edge_direction[0], edge_direction[1], edge_direction[2]);
    ROS_INFO("Ring grasp angle: %f degrees", grasp_angle * 180.0/M_PI);
  }
  
  // Set object center point for visualization
  geometry_msgs::Point object_center;
  object_center.x = centroid[0];
  object_center.y = centroid[1];
  object_center.z = centroid[2];
  
  // Visualize the PCA axes and grasp direction
  visualizePCAAxes(eigenvectors, object_center, shape_type, grasp_direction, grasp_angle);
  
  // Set result fields
  result.primary_axis = primary_axis;
  result.secondary_axis = secondary_axis;
  result.grasp_angle = grasp_angle;
  result.edge_direction = grasp_direction;
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
    // For cross objects, grasp along one arm
    // Use a smaller distance from center for more stability
    float arm_length = 0.06; // 60mm - just far enough to reach arm edge
    
    // Set grasp angle based on the computed orientation
    q_z.setRPY(0, 0, orientation_data.grasp_angle);
    
    // Move from center along primary axis direction
    grasp_pose.pose.position = object_point;
    grasp_pose.pose.position.x += arm_length * orientation_data.primary_axis[0];
    grasp_pose.pose.position.y += arm_length * orientation_data.primary_axis[1];
    
    // Adjust height slightly based on center point to ensure good grasp
    grasp_pose.pose.position.z = object_point.z + hand_offset_ + 0.01; // 1cm above object
    
    ROS_INFO("Cross grasp strategy:");
    ROS_INFO("- Grasping along arm at distance %.1f mm from center", arm_length * 1000);
    ROS_INFO("- Gripper angle: %.1f degrees", orientation_data.grasp_angle * 180.0/M_PI);
    ROS_INFO("- Grasp point: [%f, %f, %f]", 
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  } 
  else { // "nought"
    // For ring-shaped objects, grasp at an edge
    // The distance needs to be precise to hit the edge
    float edge_distance = 0.08; // 80mm - distance to the ring edge
    
    // Set grasp angle perpendicular to the edge
    q_z.setRPY(0, 0, orientation_data.grasp_angle);
    
    // Use the pre-calculated edge direction from orientation_data
    const Eigen::Vector3f& edge_direction = orientation_data.edge_direction;
    
    // Set grasp position by moving from center along the edge direction
    grasp_pose.pose.position = object_point;
    grasp_pose.pose.position.x += edge_distance * edge_direction[0];
    grasp_pose.pose.position.y += edge_distance * edge_direction[1];
    
    // Adjust height slightly based on center point
    grasp_pose.pose.position.z = object_point.z + hand_offset_ + 0.01; // 1cm above object
    
    ROS_INFO("Square ring grasp strategy:");
    ROS_INFO("- Grasping at edge, distance %.1f mm from center along optimal edge direction", edge_distance * 1000);
    ROS_INFO("- Edge direction: [%.2f, %.2f, %.2f]", 
             edge_direction[0], edge_direction[1], edge_direction[2]);
    ROS_INFO("- Gripper angle: %.1f degrees", orientation_data.grasp_angle * 180.0/M_PI);
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
  
  // Slow approach for better precision
  arm_group_.setMaxVelocityScalingFactor(0.3);
  
  // Move to grasp position
  ROS_INFO("Moving to grasp position...");
  ROS_INFO("Grasp position: [%f, %f, %f]", 
           grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  success = moveArm(grasp_pose);
  
  // Close gripper with increased force for secure grasp
  ROS_INFO("Closing gripper to grasp object...");
  moveGripper(gripper_closed_, 1.0); // 1 second wait time for firm grasp
  
  // Reset velocity scaling
  arm_group_.setMaxVelocityScalingFactor(1.0);
  
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
    bool scan_success = moveToScanPosition(all_objects[i].point);
    
    // 2. Perform scanning motion around the object
    bool motion_success = performScanningMotion(all_objects[i].point);
    
    // 3. Extract point cloud from OctoMap
    PointCPtr point_cloud = extractPointCloudFromOctomap();
    
    // 4. Extract object from point cloud
    PointCPtr object_cloud = extractObjectPointCloud(point_cloud, all_objects[i].point);
    
    // 5. Determine if object is cross or nought by checking if center has points
    bool is_cross = determineShapeType(object_cloud, all_objects[i].point);
    is_cross_shape.push_back(is_cross);
    
    ROS_INFO("%s is a %s shape", object_name.c_str(), is_cross ? "CROSS" : "NOUGHT");
  }

  // Determine which reference object matches the mystery object
  int mystery_object_num = 0;
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

  // Set the response
  response.mystery_object_num = mystery_object_num;
  
  ROS_INFO("\n=========task2 result=========");
  ROS_INFO("The mystery object matches reference object %d", mystery_object_num);
  
  // Print the shapes of all objects more clearly
  ROS_INFO("Reference object 1 shape: %s", is_cross_shape[0] ? "CROSS" : "NOUGHT");
  ROS_INFO("Reference object 2 shape: %s", is_cross_shape[1] ? "CROSS" : "NOUGHT");
  ROS_INFO("Mystery object shape: %s", is_cross_shape[2] ? "CROSS" : "NOUGHT");
  ROS_INFO("================================");
  
  // Move back to a neutral position
  geometry_msgs::PoseStamped neutral_pose;
  neutral_pose.header.frame_id = base_frame_;
  neutral_pose.pose = scan_pose_; // Using the predefined scan pose as neutral position
  moveArm(neutral_pose);

  ROS_INFO("\n====== TASK 2 COMPLETED ======\n");
  return true;
}

// Helper function to determine if an object is a cross (true) or nought (false)
bool cw2::determineShapeType(PointCPtr object_cloud, const geometry_msgs::Point &center_point) {
  ROS_INFO("Determining shape type (cross or nought) based on center occupancy");
  
  // If cloud is empty, can't make determination
  if (object_cloud->empty()) {
    ROS_WARN("Object cloud is empty, defaulting to cross shape");
    return true;
  }
  
  // Radius to check around center point (10mm as specified)
  const float center_check_radius = 0.05; // 10mm
  
  // Create KdTree for nearest neighbor search
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(object_cloud);
  
  // Create point for center
  PointT center_search_point;
  center_search_point.x = center_point.x;
  center_search_point.y = center_point.y;
  center_search_point.z = center_point.z;
  
  // Find points within radius
  std::vector<int> point_indices;
  std::vector<float> point_distances;
  int found = tree->radiusSearch(center_search_point, center_check_radius, point_indices, point_distances);
  
  ROS_INFO("Found %d points within %f m of center", found, center_check_radius);
  
  // If points are found in the center, it's a cross shape
  // If no points are found in the center, it's a nought (O) shape
  bool is_cross = (found > 0);
  
  ROS_INFO("Shape determination: %s", is_cross ? "CROSS (center is occupied)" : "NOUGHT (center is empty)");
  
  // Publish points for visualization in debug mode
  if (debug_) {
    publishPointCloud(object_cloud, cloud_object_pub_);
  }
  
  return is_cross;
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
    ROS_INFO("Moving to scan position %d/%lu", i+1, scan_poses.size());
    
    bool success = moveArm(scan_poses[i]);
    if (!success) {
      ROS_WARN("Failed to move to scan position %d, trying next position", i+1);
      continue;
    }
    
    // Wait a bit for the arm to stabilize
    ros::Duration(1.0).sleep();
    
    // Reset the OctoMap status
    octomap_received_ = false;
    
    // Attempt to get an OctoMap update at this position
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(5.0); // 5-second timeout
    
    ROS_INFO("Waiting for OctoMap update at position %d...", i+1);
    
    while (!octomap_received_ && ros::Time::now() - start_time < timeout) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    
    if (octomap_received_) {
      ROS_INFO("Successfully received OctoMap update at scan position %d with %d bytes of data",
               i+1, (int)latest_octomap_.data.size());
      successful_scans++;
    } else {
      // Try calling the service directly if subscriber didn't work
      ROS_WARN("Timeout waiting for OctoMap update from subscriber at scan position %d, trying service call",
               i+1);
      
      octomap_msgs::GetOctomap srv;
      if (octomap_client_.call(srv)) {
        latest_octomap_ = srv.response.map;
        octomap_received_ = true;
        successful_scans++;
        ROS_INFO("Successfully received OctoMap from service at scan position %d", i+1);
      } else {
        ROS_ERROR("Failed to receive OctoMap from service at scan position %d", i+1);
      }
    }
  }
  
  ROS_INFO("Completed %d successful scans out of %lu positions", successful_scans, scan_poses.size());
  
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
  
  // Publish the filtered cloud for visualization in debug mode
  if (debug_) {
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*downsampled_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = downsampled_cloud->header.frame_id.empty() ? base_frame_ : downsampled_cloud->header.frame_id;
    filtered_cloud_msg.header.stamp = ros::Time::now();
    filtered_cloud_for_octomap_pub_.publish(filtered_cloud_msg);
    ROS_INFO("Published filtered cloud for visualization");
  }
  
  ROS_INFO("====== POINT CLOUD EXTRACTION COMPLETED ======\n");
  return downsampled_cloud;
}
