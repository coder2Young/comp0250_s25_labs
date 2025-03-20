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

  box_size_ = 0.04;
  basket_size_ = 0.1;
  hand_offset_ = 0.11;
  gripper_open_ = box_size_ + 2e-2;
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

  scan_height_ = 0.6;
  
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
  
  // Publish filtered cloud for debugging
  publishPointCloud(filtered_cloud, cloud_filtered_pub_);
  
  // 3. Extract object from point cloud
  PointCPtr object_cloud = 
      extractObjectPointCloud(filtered_cloud, object_point.point);
  
  // Publish object cloud for debugging
  publishPointCloud(object_cloud, cloud_object_pub_);
  
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
  
  // Convert ROS message to PCL point cloud
  PointCPtr cloud(new PointC);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  
  // Apply voxel grid filter to downsample
  pcl::VoxelGrid<PointT> voxel_filter;
  PointCPtr cloud_filtered(new PointC);
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm voxel size
  voxel_filter.filter(*cloud_filtered);
  
  // Remove NaN points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);
  
  ROS_INFO("Point cloud filtered: %lu points", cloud_filtered->points.size());
  
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
  ROS_INFO("Determining object orientation for shape type: %s", shape_type.c_str());
  
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
  
  if (shape_type == "cross") {
    // For cross, use the principal component (largest variance)
    result.primary_axis = eigenvectors.col(0);
    // Calculate Z-axis rotation angle
    result.grasp_angle = atan2(result.primary_axis[1], result.primary_axis[0]);
    ROS_INFO("Cross shape: Detected primary axis [%f, %f, %f]", 
             result.primary_axis[0], result.primary_axis[1], result.primary_axis[2]);
    ROS_INFO("Computed grasp angle: %f radians (%.1f degrees)", 
             result.grasp_angle, result.grasp_angle * 180.0/M_PI);
  } else {  // "nought"
    // For nought, normal is the third principal component (smallest variance)
    result.primary_axis = eigenvectors.col(2);
    // For corner grasping, we need the first principal axis
    result.secondary_axis = eigenvectors.col(0);
    // Calculate Z-axis rotation angle from first principal axis
    result.grasp_angle = atan2(result.secondary_axis[1], result.secondary_axis[0]);
    ROS_INFO("Nought shape: Detected normal [%f, %f, %f]", 
             result.primary_axis[0], result.primary_axis[1], result.primary_axis[2]);
    ROS_INFO("Detected corner axis [%f, %f, %f]", 
             result.secondary_axis[0], result.secondary_axis[1], result.secondary_axis[2]);
    ROS_INFO("Computed grasp angle: %f radians (%.1f degrees)", 
             result.grasp_angle, result.grasp_angle * 180.0/M_PI);
  }
  
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
  
  // Keep gripper vertical, rotate only around Z axis
  tf2::Quaternion q_rot;
  q_rot.setRPY(0, 0, orientation_data.grasp_angle); // Use pre-computed angle
  
  if (shape_type == "cross") {
    // For cross objects, grasp at the center
    grasp_pose.pose.position = object_point;
    // Account for gripper length
    grasp_pose.pose.position.z += hand_offset_;
    
    ROS_INFO("Cross grasp strategy:");
    ROS_INFO("- Gripper aligned with arm direction at angle %.1f degrees", 
             orientation_data.grasp_angle * 180.0/M_PI);
    ROS_INFO("- Grasping at center point [%f, %f, %f]", 
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
  } 
  else { // "nought"
    // For ring-shaped objects, grasp at a corner
    float corner_offset = 0.1; // 100mm, half the square edge length
    // Move to the corner position using the pre-computed secondary axis
    grasp_pose.pose.position = object_point;
    grasp_pose.pose.position.x += corner_offset * orientation_data.secondary_axis[0];
    grasp_pose.pose.position.y += corner_offset * orientation_data.secondary_axis[1];
    grasp_pose.pose.position.z += hand_offset_; // Account for gripper length
    
    ROS_INFO("Square ring grasp strategy:");
    ROS_INFO("- Gripper aligned with diagonal at angle %.1f degrees", 
             orientation_data.grasp_angle * 180.0/M_PI);
    ROS_INFO("- Grasping at corner point [%f, %f, %f] (offset by %.2f m from center)", 
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
             corner_offset);
  }
  
  // Convert to geometry_msgs quaternion
  geometry_msgs::Quaternion q_msg;
  tf2::convert(q_rot, q_msg);
  grasp_pose.pose.orientation = q_msg;
  
  ROS_INFO("Executing grasp sequence...");
  
  // Execute grasp action
  // First open the gripper
  ROS_INFO("Opening gripper...");
  moveGripper(gripper_open_);
  
  // Move to position above object
  ROS_INFO("Moving to pre-grasp position...");
  geometry_msgs::PoseStamped pregrasp_pose = grasp_pose;
  pregrasp_pose.pose.position.z += grasp_stanby_height_;
  bool success = moveArm(pregrasp_pose);
  if (!success) {
    ROS_ERROR("Failed to move to pre-grasp position");
    return false;
  }
  
  // Move to grasp position
  ROS_INFO("Moving to grasp position...");
  success = moveArm(grasp_pose);
  if (!success) {
    ROS_ERROR("Failed to move to grasp position");
    return false;
  }
  
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
  
  // Move above basket
  place_pose.pose.position = goal_point;
  place_pose.pose.position.z += place_stanby_height_ + basket_size_;
  place_pose.pose.orientation = grasp_orientation_;
  
  // Move to pre-place position
  ROS_INFO("Moving to pre-place position...");
  bool success = moveArm(place_pose);
  if (!success) {
    ROS_ERROR("Failed to move to pre-place position");
    return false;
  }
  
  // Move down to place position
  ROS_INFO("Moving down to place position...");
  place_pose.pose.position.z = goal_point.z + 0.05;  // Just above basket bottom
  success = moveArm(place_pose);
  if (!success) {
    ROS_ERROR("Failed to move to place position");
    return false;
  }
  
  // Open gripper to release object
  ROS_INFO("Opening gripper to release object...");
  moveGripper(gripper_open_);
  
  // Wait for release
  ros::Duration(0.5).sleep();
  
  // Move up from basket
  ROS_INFO("Moving up from basket...");
  place_pose.pose.position.z += place_stanby_height_ + basket_size_;
  success = moveArm(place_pose);
  
  if (success) {
    ROS_INFO("Place operation successful");
  } else {
    ROS_ERROR("Failed to move up after placing");
  }
  
  ROS_INFO("====== PLACE EXECUTION COMPLETED ======\n");
  return success;
}

void cw2::publishPointCloud(
    const PointCPtr &cloud,
    const ros::Publisher &publisher) {
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = base_frame_;
  cloud_msg.header.stamp = ros::Time::now();
  publisher.publish(cloud_msg);
  
  ROS_INFO("Published point cloud with %lu points", cloud->points.size());
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
