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
  pick_lift_offset_(0.5)       // 0.5m lifting position after grasping
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
  cluster_tolerance_ = 0.02;    // 2cm tolerance between points in cluster
  min_cluster_size_ = 50;       // Minimum 50 points per cluster
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
    const std::string &shape_type) {
  
  ROS_INFO("\n====== PLANNING AND EXECUTING GRASP ======");
  
  if (!orientation_data.is_valid) {
    ROS_ERROR("Invalid orientation data provided");
    return false;
  }
  
  // Open gripper to prepare for grasp
  ROS_INFO("Opening gripper...");
  bool open_success = moveGripper(gripper_open_, 2.0);
  if (!open_success) {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  
  // Calculate grasp position
  geometry_msgs::PoseStamped grasp_standby_pose;
  geometry_msgs::PoseStamped grasp_pose;
  geometry_msgs::PoseStamped lift_pose; // Higher lifting pose
  grasp_standby_pose.header.frame_id = base_frame_;
  grasp_pose.header.frame_id = base_frame_;
  lift_pose.header.frame_id = base_frame_;
  
  // Extract principal axis direction (in XY plane)
  Eigen::Vector3f principal_axis = orientation_data.primary_axis;
  Eigen::Vector3f secondary_axis = orientation_data.secondary_axis;
  Eigen::Vector3f grasp_direction_xy;
  
  // Calculate orientation quaternion
  tf2::Quaternion q_orig;
  tf2::convert(grasp_orientation_, q_orig);
  tf2::Quaternion q_rot;
  tf2::Quaternion q_final;
  
  // Determine the appropriate grasp strategy based on shape
  if (shape_type == "cross") {
    ROS_INFO("Calculating grasp for cross shape...");
    
    // For cross shape, grasp one of the arms offset by 60mm from center
    float offset = 0.06; // 60mm offset along principal axis
    
    // Calculate grasp point by offsetting along principal axis
    grasp_pose.pose.position.x = object_point.x + principal_axis[0] * offset;
    grasp_pose.pose.position.y = object_point.y + principal_axis[1] * offset;
    grasp_pose.pose.position.z = object_point.z + hand_offset_; // Offset upward to prevent collision
    
    // Set standby position above grasp point
    grasp_standby_pose.pose.position.x = grasp_pose.pose.position.x;
    grasp_standby_pose.pose.position.y = grasp_pose.pose.position.y;
    grasp_standby_pose.pose.position.z = grasp_pose.pose.position.z + grasp_stanby_height_;
    
    // Set lift position (same X,Y but higher Z)
    lift_pose.pose.position.x = grasp_pose.pose.position.x;
    lift_pose.pose.position.y = grasp_pose.pose.position.y;
    lift_pose.pose.position.z = object_point.z + pick_lift_offset_; // Higher lifting position
    
    ROS_INFO("Cross grasp point: [%f, %f, %f] (offset by 60mm along principal axis)",
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
    
    // Rotate around Z axis to align gripper with the cross arm
    q_rot.setRPY(0, 0, orientation_data.grasp_angle);
    q_final = q_rot * q_orig;
    
  } else { // "nought"
    ROS_INFO("Calculating grasp for nought shape...");
    
    // For nought, find the midpoint angle between primary and secondary axes
    // First, get the angles of both axes in XY plane
    float primary_angle = atan2(principal_axis[1], principal_axis[0]);
    float secondary_angle = atan2(secondary_axis[1], secondary_axis[0]);
    
    // Calculate midpoint angle (handling the circular nature of angles)
    float angle_diff = secondary_angle - primary_angle;
    if (angle_diff > M_PI) angle_diff -= 2*M_PI;
    if (angle_diff < -M_PI) angle_diff += 2*M_PI;
    
    float midpoint_angle = primary_angle + angle_diff/2.0;
    
    ROS_INFO("Primary axis angle: %f, Secondary axis angle: %f, Midpoint angle: %f",
             primary_angle, secondary_angle, midpoint_angle);
    
    // Calculate grasp direction using the midpoint angle
    grasp_direction_xy[0] = cos(midpoint_angle);
    grasp_direction_xy[1] = sin(midpoint_angle);
    grasp_direction_xy[2] = 0.0;
    
    // Use 100mm offset along this direction
    float offset = 0.08; // 80mm offset - increased from previous 0.06
    
    // Calculate grasp point by offsetting along the midpoint direction
    grasp_pose.pose.position.x = object_point.x + grasp_direction_xy[0] * offset;
    grasp_pose.pose.position.y = object_point.y + grasp_direction_xy[1] * offset;
    grasp_pose.pose.position.z = object_point.z + hand_offset_; // Offset upward to prevent collision
    
    // Set standby position above grasp point
    grasp_standby_pose.pose.position.x = grasp_pose.pose.position.x;
    grasp_standby_pose.pose.position.y = grasp_pose.pose.position.y;
    grasp_standby_pose.pose.position.z = grasp_pose.pose.position.z + grasp_stanby_height_;
    
    // Set lift position (same X,Y but higher Z)
    lift_pose.pose.position.x = grasp_pose.pose.position.x;
    lift_pose.pose.position.y = grasp_pose.pose.position.y;
    lift_pose.pose.position.z = object_point.z + pick_lift_offset_; // Higher lifting position
    
    ROS_INFO("Nought grasp point: [%f, %f, %f] (offset by 100mm along midpoint axis)",
             grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z);
    
    // Add 90 degrees rotation around Z axis to the midpoint angle
    float grasp_angle = midpoint_angle + M_PI/2.0; // Add 90 degrees
    q_rot.setRPY(0, 0, grasp_angle);
    q_final = q_rot * q_orig;
  }
  
  // Normalize and apply quaternion to all poses
  q_final.normalize();
  tf2::convert(q_final, grasp_pose.pose.orientation);
  tf2::convert(q_final, grasp_standby_pose.pose.orientation);
  tf2::convert(q_final, lift_pose.pose.orientation);
  
  // Visualize grasp point and orientation if in debug mode
  if (debug_) {
    visualizeGraspPoint(grasp_pose.pose.position, q_final);
  }
  
  // Move to standby position first
  ROS_INFO("Moving to grasp standby position...");
  bool standby_success = false;
  
  if (t1_move_constraint_) {
    // Add path constraint for vertical approach
    moveit_msgs::Constraints constraints;
    moveit_msgs::OrientationConstraint ocm;
    ocm.header.frame_id = base_frame_;
    ocm.link_name = arm_group_.getEndEffectorLink();
    ocm.orientation = grasp_standby_pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1; // Relatively strict
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 2.0 * M_PI; // Allow rotation around Z
    ocm.weight = 1.0;
    
    constraints.orientation_constraints.push_back(ocm);
    arm_group_.setPathConstraints(constraints);
    
    // Try with constraints
    standby_success = moveArm(grasp_standby_pose);
    
    // If failed, retry without constraints
    if (!standby_success) {
      ROS_WARN("Failed to move to standby position with constraints, retrying without constraints");
      arm_group_.clearPathConstraints();
      standby_success = moveArm(grasp_standby_pose);
    }
    
    // Clear constraints for future movements
    arm_group_.clearPathConstraints();
  } else {
    // Move without constraints
    standby_success = moveArm(grasp_standby_pose);
  }
  
  if (!standby_success) {
    ROS_ERROR("Failed to move to grasp standby position");
    return false;
  }
  
  // Move down to grasp position
  ROS_INFO("Moving down to grasp position...");
  bool grasp_approach_success = moveArm(grasp_pose);
  if (!grasp_approach_success) {
    ROS_ERROR("Failed to move to grasp position");
    return false;
  }
  
  // Close gripper to grasp object
  ROS_INFO("Closing gripper to grasp object...");
  bool close_success = moveGripper(gripper_closed_, 2.0);
  if (!close_success) {
    ROS_ERROR("Failed to close gripper");
    return false;
  }
  
  // Move to higher lifting position for safer travel
  ROS_INFO("Lifting object to travel height...");
  bool lift_success = moveArm(lift_pose);
  if (!lift_success) {
    ROS_ERROR("Failed to move to lifting position");
    return false;
  }
  
  // Store the final grasp orientation for later use in place operation
  current_grasp_orientation_ = lift_pose.pose.orientation;
  
  ROS_INFO("====== GRASP EXECUTION COMPLETED ======\n");
  return true;
}

bool cw2::planAndExecutePlace(const geometry_msgs::Point &place_point) {
  ROS_INFO("\n====== PLANNING AND EXECUTING PLACE ======");
  
  // Calculate place position
  geometry_msgs::PoseStamped place_standby_pose;
  geometry_msgs::PoseStamped place_pose;
  place_standby_pose.header.frame_id = base_frame_;
  place_pose.header.frame_id = base_frame_;
  
  // Set place position
  place_pose.pose.position = place_point;
  place_standby_pose.pose.position = place_point;
  place_standby_pose.pose.position.z += place_stanby_height_;  // Standby position is above place point
  
  // Use the same orientation from the grasp operation
  place_pose.pose.orientation = current_grasp_orientation_;
  place_standby_pose.pose.orientation = current_grasp_orientation_;
  
  // Move to place standby position
  ROS_INFO("Moving to place standby position...");
  bool standby_success = moveArm(place_standby_pose);
  if (!standby_success) {
    ROS_ERROR("Failed to move to place standby position");
    return false;
  }
  
  // Move down to place position
  ROS_INFO("Moving down to place position...");
  bool place_approach_success = moveArm(place_pose);
  if (!place_approach_success) {
    ROS_ERROR("Failed to move to place position");
    return false;
  }
  
  // Open gripper to release object
  ROS_INFO("Opening gripper to release object...");
  bool open_success = moveGripper(gripper_open_, 2.0);
  if (!open_success) {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  
  // Move back up to standby position
  ROS_INFO("Moving back to place standby position...");
  bool retreat_success = moveArm(place_standby_pose);
  if (!retreat_success) {
    ROS_ERROR("Failed to retreat to place standby position");
    return false;
  }
  
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
