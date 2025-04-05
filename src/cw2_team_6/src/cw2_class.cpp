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
  arm_group_("panda_arm"),
  hand_group_("hand") // Not config, just instantiate
{
  /* class constructor */
  nh_ = nh;

  cw2_config();

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);

  // Sub for task3, continous scan for the whole scene
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::continuousScanCloudCallback, this);

  // Set the initial collection state to false
  // This is a switch, not a config
  is_collecting_clouds_ = false;
  cloud_frame_counter_ = 0;

  // Add floor collision object to prevent collisions with the ground
  addFloorCollisionObject();

  // Initialize visualization publishers for debugging with latched mode
  // The last "true" parameter enables latched mode - messages will persist for new subscribers
  if (debug_) {
    cloud_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/cloud_filtered", 1, true);
    cloud_object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/cloud_object", 1, true);
    pca_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/pca_axes", 1, true);
    center_point_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug/center_point", 1, true);
    grasp_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug/grasp_point", 1, true);
    clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/object_clusters", 1, true);
    obstacles_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/obstacles_cloud", 1, true);
    all_pca_axes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/debug/all_pca_axes", 1, true);
  }

  ROS_INFO("cw2 class initialised");
  return;
}

/* function to configure the robot for the coursework */
void
cw2::cw2_config()
{
  ROS_INFO("Configuring the robot for the coursework");

  // Enable debug mode
  debug_ = true;
  if (debug_){
    ROS_INFO("Debug mode enabled");
  }

  // Basic pick and place parameters
  hand_offset_ = 0.15; // 0.15 default
  gripper_open_ = 0.08; // 80mm
  gripper_closed_ = 0.0;
  grasp_stanby_height_ = 0.15;
  place_stanby_height_ = 0.1;
  pick_lift_offset_ = 0.4; // 40cm higher position for lifting objects
  
  // Default grasp_orientaion, heading down by roll:-M_PI
  tf2::Quaternion q_grasp;
  q_grasp.setRPY(-M_PI, 0, -M_PI/4);
  grasp_orientation_ = tf2::toMsg(q_grasp);

  /* Task1 */
  t1_downsample_ = false;  // Turn off downsampling initially for better PCA
  t1_scan_height_ = 0.55;  // Height above object for scanning

  /* Task 2*/
  t2_scan_height_ = t1_scan_height_; // Same as t1
  t2_shape_determine_radius_ = 0.01; // 10mm radius for center check
  t2_shape_determine_min_points_ = 10; // Minimum number of points to be confident in Task 2

  /* Task3 */
  t3_scan_height_ = 0.65;           // Height for scanning the entire scene, as hight as possible for avoiding obstacles
  t3_grasp_height_offset_ = -0.06;    // Add 6cm to Z coordinate of all grasp points to compensate for low point cloud values

  // Continuous scanning parameters
  t3_pointcloud_save_interval_ = 10;     // Process every 10th frame
  t3_continuous_scan_voxel_size_ = 0.001; // 2mm voxel size for downsampling
  t3_merge_voxel_size_ = 0.001; // 1mm voxel for merged cloud
  
  t3_cross_grasp_offset_base_ = 0.01;
  t3_nought_grasp_offset_base_ = -0.01;
  // Euclidean clustering parameters
  t3_cluster_tolerance_ = 0.002;    // 2mm tolerance between points in cluster
  t3_min_cluster_size_ = 500; 
  t3_max_cluster_size_ = 100000;

  return;
}

///////////////////////////////////////////////////////////////////////////////
// Callbacks
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
  scan_pose.pose.position.z = object_point.point.z + t1_scan_height_;
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
    scan_pose.pose.position.z = all_objects[i].point.z + t2_scan_height_;
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
    bool is_cross = determineObjectShape(filtered_cloud, all_objects[i].point);
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
      ROS_ERROR("ERROR! No matching object");
      mystery_object_num = 1;
    }
  } else {
    ROS_ERROR("ERROR! Failed to determine all object shapes. Defaulting to object 1.");
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

  ROS_INFO("\n====== TASK 2 COMPLETED ======\n");
  return true;
}


bool cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response) {
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
ROS_INFO("====== SCANNING SCENE  ======");
PointCPtr merged_cloud = continuousScanSceneFromMultipleViewpoints();

if (merged_cloud->empty()) {
ROS_ERROR("Failed to get valid point cloud data from scanning");
return false;
}

// 2. Extract brown basket for placement
PointCPtr basket_cloud = extractBrownBasket(merged_cloud);
geometry_msgs::Point basket_center = findBasketCenter(basket_cloud);
ROS_INFO("Basket center found at: [%f, %f, %f]", basket_center.x, basket_center.y, basket_center.z);

// 3. Extract black obstacles for collision avoidance
PointCPtr obstacles_cloud = extractBlackObstacles(merged_cloud);
addObstaclesToPlanningScene(obstacles_cloud);
addFloorCollisionObject();

// 4. Extract the remaining colored objects (red, blue, purple)
PointCPtr objects_cloud = extractGraspableObjects(merged_cloud);
publishPointCloud(objects_cloud, cloud_object_pub_);

// 5. Cluster the objects and determine their shapes
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

// 6. Determine which shape is more common
bool grasp_cross_shape = (num_cross_shapes >= num_nought_shapes);
int num_most_common_shape = grasp_cross_shape ? num_cross_shapes : num_nought_shapes;

ROS_INFO("Most common shape: %s (Count: %d)", 
grasp_cross_shape ? "CROSS" : "NOUGHT", num_most_common_shape);

// 7. Find the largest object of the most common shape
int largest_idx = -1;
float max_size = -1.0;

for (size_t i = 0; i < object_clusters.size(); i++) {
// 筛选出符合 most common shape 的物体
if (is_cross_shape[i] != grasp_cross_shape) continue;

// 计算物体尺寸（这里用点云的边界框对角线长度）
PointT min_pt, max_pt;
pcl::getMinMax3D(*object_clusters[i], min_pt, max_pt);
float size = sqrt(pow(max_pt.x - min_pt.x, 2) +
  pow(max_pt.y - min_pt.y, 2) +
  pow(max_pt.z - min_pt.z, 2));

ROS_INFO("Object %zu (%s): Size = %.3f", i, 
is_cross_shape[i] ? "CROSS" : "NOUGHT", size);

if (size > max_size) {
max_size = size;
largest_idx = i;
}
}

if (largest_idx == -1) {
ROS_ERROR("No objects of the most common shape found");
return false;
}

ROS_INFO("Largest object of most common shape: Index %d, Size %.3f", largest_idx, max_size);

// 8. Prepare data for grasping only the largest object
std::vector<PointCPtr> largest_cluster = {object_clusters[largest_idx]};
std::vector<bool> largest_is_cross = {is_cross_shape[largest_idx]};
std::vector<ObjectOrientationData> largest_orientation = {object_orientations[largest_idx]};

// 9. Grasp and place the largest object
bool grasp_success = graspAndPlaceObjectsOfType(
largest_cluster, largest_is_cross, largest_orientation, grasp_cross_shape, basket_center);

if (!grasp_success) {
ROS_WARN("Failed to grasp and place the largest object");
}

// Set the response values
response.total_num_shapes = total_num_shapes;
response.num_most_common_shape = num_most_common_shape;

ROS_INFO("\n====== TASK 3 COMPLETED ======");
ROS_INFO("Total shapes: %d, Most common shape count: %d", 
total_num_shapes, num_most_common_shape);

return true;
}

///////////////////////////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////////////////////////

// Get axes and grasp pose
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
  
  // Perform PCA on the filtered cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(object_cloud);
  
  // Get eigenvalues and eigenvectors
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  
  ROS_INFO("PCA eigenvalues: [%f, %f, %f]", 
           eigenvalues[0], eigenvalues[1], eigenvalues[2]);
  
  // Setup result variables
  Eigen::Vector3f primary_axis, secondary_axis;
  Eigen::Vector3f grasp_direction;
  float grasp_angle;
  
  if (shape_type == "cross") {
    ROS_INFO("Analyzing cross shape...");
    
    // Primary axis is the direction of largest variance
    primary_axis = eigenvectors.col(0);
    
    // Make sure primary axis is in the XY plane (horizontal)
    primary_axis[2] = 0.0f;
    primary_axis.normalize();
    
    // Secondary axis is perpendicular to primary in the XY plane
    secondary_axis = Eigen::Vector3f(-primary_axis[1], primary_axis[0], 0.0f);
    secondary_axis[2] = 0.0f;
    secondary_axis.normalize();
    
    // For cross, grasp along one arm
    grasp_direction = primary_axis;
    grasp_angle = atan2(primary_axis[1], primary_axis[0]); 
    
    ROS_INFO("Cross primary axis: [%f, %f, %f]", 
             primary_axis[0], primary_axis[1], primary_axis[2]);
    ROS_INFO("Cross grasp angle: %f degrees", grasp_angle * 180.0/M_PI);
  } 
  else { // "nought"
    ROS_INFO("Analyzing nought (ring) shape...");
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

// Grasp for 2 shapes
// offset_override - for Task3 any size, use any grasp offset
bool cw2::planAndExecuteGrasp(
    const geometry_msgs::Point &object_point,
    const ObjectOrientationData &orientation_data,
    const std::string &shape_type,
    float offset_override) {
  
  ROS_INFO("\n====== PLANNING AND EXECUTING GRASP ======");
  ROS_INFO("Object point: [%.4f, %.4f, %.4f]", 
           object_point.x, object_point.y, object_point.z);
  
  if (!orientation_data.is_valid) {
    ROS_ERROR("Invalid orientation data provided");
    return false;
  }
  
  if (debug_) {
    ROS_INFO("PCA Analysis - Primary axis: [%.4f, %.4f, %.4f], Grasp angle: %.2f deg", 
            orientation_data.primary_axis[0], orientation_data.primary_axis[1], 
            orientation_data.primary_axis[2], orientation_data.grasp_angle * 180/M_PI);
  }
  
  // Open gripper to prepare for grasp
  ROS_INFO("Opening gripper");
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
  
  if (debug_) {
    ROS_INFO("Principal axis (XY): [%.4f, %.4f, %.4f]", 
            principal_axis[0], principal_axis[1], principal_axis[2]);
    ROS_INFO("Secondary axis (XY): [%.4f, %.4f, %.4f]", 
            secondary_axis[0], secondary_axis[1], secondary_axis[2]);
  }
  
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
    
    if (offset_override > 0.0) {
      offset = offset_override;
      ROS_INFO("Using custom offset value: %.4f m", offset);
    } else {
      // For cross shape, grasp one of the arms offset by 60mm from center
      offset = 0.06; // 60mm offset along principal axis for task1
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
    
    if (offset_override > 0.0) {
      offset = offset_override;
      ROS_INFO("Using custom offset value: %.4f m", offset);
    } else {
      // Use 80mm offset along this direction for task1
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
  grasp_standby_pose.pose.position.z = object_point.z + hand_offset_ + grasp_stanby_height_;
  grasp_standby_pose.pose.orientation = tf2::toMsg(q_final);
  
  ROS_INFO("Grasp standby pose: [%.4f, %.4f, %.4f] (%.4f above grasp position)",
           grasp_standby_pose.pose.position.x, 
           grasp_standby_pose.pose.position.y,
           grasp_standby_pose.pose.position.z,
           grasp_stanby_height_);
  
  // Lift position (pick_lift_offset_ above grasp position)
  lift_pose.pose.position.x = grasp_x;
  lift_pose.pose.position.y = grasp_y;
  lift_pose.pose.position.z = object_point.z + hand_offset_ + pick_lift_offset_;
  lift_pose.pose.orientation = tf2::toMsg(q_final);
  
  ROS_INFO("Lift pose: [%.4f, %.4f, %.4f] (%.4f above ground)",
           lift_pose.pose.position.x, 
           lift_pose.pose.position.y,
           lift_pose.pose.position.z,
           pick_lift_offset_);
  
  // Store for place operation
  current_grasp_orientation_ = tf2::toMsg(q_final);
  current_lift_height_ = lift_pose.pose.position.z;
  
  ROS_INFO("Stored lift height for place operation: %.4f", current_lift_height_);
  
  // Visualize grasp point and orientation if in debug mode
  if (debug_) {
    visualizeGraspPoint(grasp_pose.pose.position, q_final);
    ROS_INFO("Grasp visualization markers published");
  }
  
  // Step 1: Move to standby position first 
  ROS_INFO("Moving to grasp standby position...");
  bool standby_success = moveArm(grasp_standby_pose);
  if (!standby_success) {
    ROS_ERROR("Failed to move to grasp standby position");
    return false;
  }
  
  // Step 2: Use Cartesian path to move vertically down
  ROS_INFO("Planning Cartesian path for vertical approach to grasp...");
  std::vector<geometry_msgs::Pose> vertical_waypoints;
  vertical_waypoints.push_back(grasp_standby_pose.pose); 
  vertical_waypoints.push_back(grasp_pose.pose); 
  
  double eef_step = 0.001;       // 1cm Step size
  double jump_threshold = 0.0; 
  double speed_factor = 0.05;   // Down to 5% of max speed
  
  ROS_INFO("Executing vertical approach using Cartesian path (speed: %.1f%%)", speed_factor * 100);
  bool cartesian_success = moveAlongCartesianPath(vertical_waypoints, eef_step, jump_threshold, speed_factor);
  
  if (!cartesian_success) {
    ROS_WARN("Failed to execute Cartesian approach to grasp, trying regular planning");
    bool grasp_approach_success = moveArm(grasp_pose);
    if (!grasp_approach_success) {
      ROS_ERROR("Failed to move to grasp position");
      return false;
    }
  }
  
  // Step 3: Close gripper to grasp object
  ROS_INFO("Closing gripper to grasp object...");
  bool close_success = moveGripper(gripper_closed_, 2.0);
  if (!close_success) {
    ROS_ERROR("Failed to close gripper");
    return false;
  }
  
  // Step 4: Lift object to higher position
  ROS_INFO("Lifting object to travel height...");
  bool lift_success = moveArm(lift_pose);
  if (!lift_success) {
    ROS_ERROR("Failed to move to lifting position");
    return false;
  }
  
  ROS_INFO("====== GRASP EXECUTION COMPLETED ======\n");
  return true;
}

bool cw2::planAndExecutePlace(const geometry_msgs::Point &place_point) {
  ROS_INFO("\n====== PLANNING AND EXECUTING PLACE ======");
  
  // Step 0: Calculate place positions
  geometry_msgs::PoseStamped place_standby_pose;
  geometry_msgs::PoseStamped horizontal_move_pose;
  
  place_standby_pose.header.frame_id = base_frame_;
  horizontal_move_pose.header.frame_id = base_frame_;
  
  // Place standby position (hand_offset_ + place_stanby_height_ above place position)
  place_standby_pose.pose.position.x = place_point.x;
  place_standby_pose.pose.position.y = place_point.y;
  place_standby_pose.pose.position.z = place_point.z + hand_offset_ + place_stanby_height_;
  place_standby_pose.pose.orientation = current_grasp_orientation_;
  
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
  
  // Step 1: Horizontal move to above place position (keeping Z at lift height)
  ROS_INFO("Moving horizontally to position above place point...");
  bool horizontal_move_success = moveArm(horizontal_move_pose);
  if (!horizontal_move_success) {
    ROS_ERROR("Failed to move horizontally to place area");
    return false;
  }
  
  // Step 2: Use Cartesian path to move vertically down
  ROS_INFO("Planning Cartesian path for vertical descent to place standby...");
  std::vector<geometry_msgs::Pose> vertical_waypoints;
  vertical_waypoints.push_back(horizontal_move_pose.pose); 
  vertical_waypoints.push_back(place_standby_pose.pose); 
  
  double eef_step = 0.001; 
  double jump_threshold = 0.0;
  double speed_factor = 0.05; 
  
  ROS_INFO("Executing vertical descent using Cartesian path (speed: %.1f%%)", speed_factor * 100);
  bool cartesian_success = moveAlongCartesianPath(vertical_waypoints, eef_step, jump_threshold, speed_factor);
  
  if (!cartesian_success) {
    ROS_WARN("Failed to execute Cartesian approach to place standby, trying regular planning");
    bool place_standby_success = moveArm(place_standby_pose);
    if (!place_standby_success) {
      ROS_ERROR("Failed to move to place standby position");
      return false;
    }
  }
  
  // Step 3: Open gripper to release object at standby position
  ROS_INFO("Opening gripper to release object (width: %.4f)...", gripper_open_);
  bool open_success = moveGripper(gripper_open_, 2.0);
  if (!open_success) {
    ROS_ERROR("Failed to open gripper");
    return false;
  }
  
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


// Modified shape determination to use point cloud centroid instead of message-provided center point
bool cw2::determineObjectShape(PointCPtr cloud, const geometry_msgs::Point &center_point) {
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

/**
 * Finds the center of the basket from a point cloud
 * @param basket_cloud Point cloud of the brown basket
 * @return 3D point representing the basket center
 */
geometry_msgs::Point cw2::findBasketCenter(const PointCPtr& basket_cloud) {
  ROS_INFO("Finding basket center from %zu points", basket_cloud->points.size());
  
  if (basket_cloud->empty()) {
    ROS_ERROR("Empty basket cloud provided");
    geometry_msgs::Point empty_point;
    empty_point.x = 0;
    empty_point.y = 0;
    empty_point.z = 0;
    return empty_point;
  }
  
  PointCPtr downsampled_cloud(new PointC);
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(basket_cloud);
  float voxel_size = 0.005; // 5mm voxel size
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.filter(*downsampled_cloud);
  
  ROS_INFO("Downsampled basket cloud from %zu to %zu points with 5mm voxel filter",
           basket_cloud->points.size(), downsampled_cloud->points.size());
  
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  
  tree->setInputCloud(downsampled_cloud);
  ec.setClusterTolerance(0.05);  // 5cm tolerance
  ec.setMinClusterSize(500);     // Minimum 500 points per cluster
  ec.setMaxClusterSize(100000);  // Maximum 100k points per cluster
  ec.setSearchMethod(tree);
  ec.setInputCloud(downsampled_cloud);
  ec.extract(cluster_indices);
  
  if (cluster_indices.empty()) {
    ROS_ERROR("No clusters found in basket cloud");
    geometry_msgs::Point empty_point;
    empty_point.x = 0;
    empty_point.y = 0;
    empty_point.z = 0;
    return empty_point;
  }
  
  // Find the largest cluster
  size_t max_size = 0;
  int max_idx = 0;
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    if (cluster_indices[i].indices.size() > max_size) {
      max_size = cluster_indices[i].indices.size();
      max_idx = i;
    }
  }
  
  ROS_INFO("Found %zu clusters in basket cloud, largest has %zu points",
           cluster_indices.size(), max_size);
  
  // Visualize clusters if in debug mode
  if (debug_) {
    PointCPtr colored_clusters(new PointC);
    
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      // Assign a color to each cluster
      // For simplicity, use a fixed color for all clusters
      uint8_t r = 50;
      uint8_t g = 50;
      uint8_t b = 50;
      
      if (i == max_idx) {
        r = 255;  // Red for the largest cluster
        g = 0;
        b = 0;
      }
      
      for (const auto& idx : cluster_indices[i].indices) {
        PointT colored_point = downsampled_cloud->points[idx];
        colored_point.r = r;
        colored_point.g = g;
        colored_point.b = b;
        colored_clusters->points.push_back(colored_point);
      }
    }
    
    colored_clusters->width = colored_clusters->points.size();
    colored_clusters->height = 1;
    colored_clusters->is_dense = false;
    
    publishPointCloud(colored_clusters, cloud_filtered_pub_);
  }
  
  // Calculate centroid of the largest cluster
  PointCPtr largest_cluster(new PointC);
  for (const auto& idx : cluster_indices[max_idx].indices) {
    largest_cluster->points.push_back(downsampled_cloud->points[idx]);
  }
  
  largest_cluster->width = largest_cluster->points.size();
  largest_cluster->height = 1;
  largest_cluster->is_dense = false;
  
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*largest_cluster, centroid);
  
  // Create a point for the basket center
  geometry_msgs::Point basket_center;
  basket_center.x = centroid[0];
  basket_center.y = centroid[1];
  basket_center.z = centroid[2];
  
  ROS_INFO("Basket center found at [%.4f, %.4f, %.4f]", 
           basket_center.x, basket_center.y, basket_center.z);
  
  return basket_center;
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
  octomap::OcTree* obstacles_octree = new octomap::OcTree(0.01); // 1cm resolution
  
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

  ROS_INFO("Clustering and classifying objects from %zu points", objects_cloud->points.size());
  
  // Clear output vectors
  object_clusters.clear();
  is_cross_shape.clear();
  object_orientations.clear();
  
  if (objects_cloud->empty()) {
    ROS_WARN("Empty cloud provided for clustering");
    return false;
  }
  
  // Create KdTree for searching
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(objects_cloud);
  
  // Perform Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(t3_cluster_tolerance_);
  ec.setMinClusterSize(t3_min_cluster_size_);
  ec.setMaxClusterSize(t3_max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(objects_cloud);
  ec.extract(cluster_indices);
  
  ROS_INFO("Found %zu clusters in the scene", cluster_indices.size());
  
  if (cluster_indices.empty()) {
    ROS_WARN("No object clusters found");
    return false;
  }
  
  // Create a point cloud to visualize all clusters
  PointCPtr all_clusters_cloud(new PointC);
  all_clusters_cloud->reserve(objects_cloud->points.size()); // Preallocate memory
  
  // Create a marker array for PCA axes visualization
  visualization_msgs::MarkerArray all_pca_axes;
  
  // Process each cluster
  for (size_t i = 0; i < cluster_indices.size(); i++) {
    ROS_INFO("Processing cluster %zu with %zu points", i, cluster_indices[i].indices.size());
    
    // Extract cluster points
    PointCPtr cluster_cloud(new PointC);
    for (const auto& idx : cluster_indices[i].indices) {
      cluster_cloud->points.push_back(objects_cloud->points[idx]);
    }
    cluster_cloud->width = cluster_cloud->points.size();
    cluster_cloud->height = 1;
    cluster_cloud->is_dense = false;
    
    // Compute cluster centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);
    
    // Create center point for shape determination
    geometry_msgs::Point center_point;
    center_point.x = centroid[0];
    center_point.y = centroid[1];
    center_point.z = centroid[2];
    
    // Determine if it's a cross shape
    bool is_cross = determineObjectShape(cluster_cloud, center_point);
    ROS_INFO("Cluster %zu classified as %s", i, is_cross ? "CROSS" : "NOUGHT");
    
    // Determine object orientation using PCA
    std::string shape_type = is_cross ? "cross" : "nought";
    ObjectOrientationData orientation_data = determineObjectOrientation(cluster_cloud, shape_type);
    
    if (orientation_data.is_valid) {
      // Add to output vectors
      object_clusters.push_back(cluster_cloud);
      is_cross_shape.push_back(is_cross);
      object_orientations.push_back(orientation_data);
    } else {
      ROS_WARN("Could not determine valid orientation for cluster %zu, skipping", i);
      continue; // Skip visualization for invalid orientation
    }
    
    // Add color-coded points to visualization cloud
    uint8_t r = 50 + (i * 40) % 200;
    uint8_t g = 50 + ((i * 70) % 200);
    uint8_t b = 50 + ((i * 90) % 200);
    
    // Brighten colors for cross shapes，darken for nought shapes
    if (is_cross) {
      r = std::min(255, int(r * 1.5));
      g = std::min(255, int(g * 1.5));
      b = std::min(255, int(b * 1.5));
    }

    for (const auto& idx : cluster_indices[i].indices) {
      PointT colored_point = objects_cloud->points[idx];
      colored_point.r = r;
      colored_point.g = g;
      colored_point.b = b;
      all_clusters_cloud->points.push_back(colored_point);
    }
    
    // Create PCA visualization markers for this cluster
    visualization_msgs::MarkerArray pca_markers = createPCAAxesMarkers(
        centroid, orientation_data.primary_axis, orientation_data.secondary_axis, i*10, 
        shape_type);
    
    // Add the markers to the global collection
    for (const auto& marker : pca_markers.markers) {
      all_pca_axes.markers.push_back(marker);
    }
  }
  
  // Set the header for the visualization cloud
  all_clusters_cloud->width = all_clusters_cloud->points.size();
  all_clusters_cloud->height = 1;
  all_clusters_cloud->is_dense = false;
  
  // Publish the clustered point cloud for visualization
  if (debug_) {
    // Publish the clustered point cloud
    publishPointCloud(all_clusters_cloud, clusters_pub_);
    
    // Publish the PCA axes markers
    all_pca_axes_pub_.publish(all_pca_axes);
    
    ROS_INFO("Published visualization of %zu clusters with PCA axes", cluster_indices.size());
  }
  
  return !object_clusters.empty();
}

visualization_msgs::MarkerArray cw2::createPCAAxesMarkers(
    const Eigen::Vector4f& centroid,
    const Eigen::Vector3f& primary_axis,
    const Eigen::Vector3f& secondary_axis,
    int id_offset,
    const std::string& shape_type) {
  
  visualization_msgs::MarkerArray marker_array;

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
  
  Eigen::Vector3f z_axis(0, 0, 1);
  Eigen::Vector3f rotation_axis = z_axis.cross(primary_axis).normalized();
  float rotation_angle = acos(z_axis.dot(primary_axis));
  
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(rotation_angle, rotation_axis);
  
  primary_marker.pose.orientation.x = q.x();
  primary_marker.pose.orientation.y = q.y();
  primary_marker.pose.orientation.z = q.z();
  primary_marker.pose.orientation.w = q.w();
  
  primary_marker.scale.x = 0.1;  
  primary_marker.scale.y = 0.01; 
  primary_marker.scale.z = 0.01; 
  
  // Set color based on shape type
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
  
  return marker_array;
}

/**
 * Calculates the optimal grasp offset based on object dimensions
 * @param object_cloud The point cloud of the object
 * @param centroid The object's centroid
 * @param orientation_data The pre-calculated orientation data from determineObjectOrientation
 * @param is_cross Whether the object is a cross (true) or nought (false)
 * @return The calculated grasp offset
 */
float cw2::calculateGraspOffset(PointCPtr object_cloud, const Eigen::Vector4f& centroid, 
                           const ObjectOrientationData& orientation_data, bool is_cross) {
  float default_offset = is_cross ? 0.06 : 0.08;
  
  if (object_cloud->empty()) {
    ROS_WARN("Empty point cloud, using default offset: %.1fmm", default_offset * 1000.0);
    return default_offset;
  }
  
  float max_dist = 0.0f;
  
  // Use the centroid to calculate the grasp offset
  Eigen::Vector3f grasp_axis;
  if (is_cross) {
    // Use primary axis for cross
    grasp_axis = orientation_data.primary_axis;
  } else {
    //  Use edge direction for nought
    if (orientation_data.edge_direction.norm() > 0.01) {
      grasp_axis = orientation_data.edge_direction;
    } else {
      // Calculate the angle between primary and secondary axes if edge direction is not valid
      float primary_angle = atan2(orientation_data.primary_axis[1], 
                                 orientation_data.primary_axis[0]);
      float secondary_angle = atan2(orientation_data.secondary_axis[1], 
                                   orientation_data.secondary_axis[0]);
      
      // Handle angle wrapping
      float angle_diff = secondary_angle - primary_angle;
      if (angle_diff > M_PI) angle_diff -= 2*M_PI;
      if (angle_diff < -M_PI) angle_diff += 2*M_PI;
      
      float midpoint_angle = primary_angle + angle_diff/2.0;
      
      // Calculate the grasp axis based on the midpoint angle
      grasp_axis[0] = cos(midpoint_angle);
      grasp_axis[1] = sin(midpoint_angle);
      grasp_axis[2] = 0.0;
    }
  }
  
  // Calculate the maximum distance from the centroid along the grasp axis
  for (const auto& point : object_cloud->points) {
    Eigen::Vector3f point_vector(point.x - centroid[0], 
                                 point.y - centroid[1], 
                                 0); 
    
    float projection = point_vector.dot(grasp_axis);
    
    if (projection > 0) {
      max_dist = std::max(max_dist, projection);
    }
  }
  
  if (max_dist < 0.01) { 
    ROS_WARN("Could not find valid edge point, using default offset: %.1fmm", default_offset * 1000.0);
    return default_offset;
  }
  
  float offset;

  if (is_cross){
    offset = max_dist / 2.0 + t3_cross_grasp_offset_base_;
  }
  else {
    offset = max_dist + t3_nought_grasp_offset_base_;
  }
  
  ROS_INFO("Calculated grasp offset: %.1fmm (edge distance: %.1fmm)", 
           offset * 1000.0, max_dist * 1000.0);
  
  return offset;
}

/**
 * Plan and execute grasping and placing of objects by type
 * @param object_clusters Vector of object point clouds
 * @param is_cross_shape Vector of booleans indicating if each object is a cross
 * @param object_orientations Vector of orientation data for each object
 * @param grasp_cross_shape Whether to grasp cross (true) or nought (false) objects
 * @param basket_center Position of the basket center for placing
 * @return true if at least one object was successfully grasped and placed
 */
bool cw2::graspAndPlaceObjectsOfType(
  const std::vector<PointCPtr> &object_clusters,
  const std::vector<bool> &is_cross_shape,
  const std::vector<ObjectOrientationData> &object_orientations,
  bool grasp_cross_shape,
  const geometry_msgs::Point &basket_center) {
ROS_INFO("\n====== GRASPING AND PLACING LARGEST OBJECT OF TYPE: %s ======",
         grasp_cross_shape ? "CROSS" : "NOUGHT");

if (object_clusters.empty()) {
  ROS_ERROR("No objects to grasp");
  return false;
}

if (object_clusters.size() != 1) {
  ROS_WARN("Expected exactly 1 object to grasp, but got %zu", object_clusters.size());
}

// 只处理第一个（也是唯一一个）物体
size_t i = 0;
if (is_cross_shape[i] != grasp_cross_shape) {
  ROS_ERROR("Object shape does not match expected type");
  return false;
}

Eigen::Vector4f centroid;
pcl::compute3DCentroid(*object_clusters[i], centroid);
geometry_msgs::Point object_center;
object_center.x = centroid[0];
object_center.y = centroid[1];
object_center.z = centroid[2] + t3_grasp_height_offset_;

float grasp_offset = calculateGraspOffset(
    object_clusters[i], centroid, object_orientations[i], is_cross_shape[i]);

bool grasp_success = planAndExecuteGrasp(
    object_center, object_orientations[i], 
    is_cross_shape[i] ? "cross" : "nought", grasp_offset);

if (!grasp_success) {
  ROS_WARN("Failed to grasp the largest object");
  return false;
}

// 放置到篮子中心
geometry_msgs::Point place_point = basket_center;
place_point.z += 0.01; // 稍微抬高一点，避免碰撞

bool place_success = planAndExecutePlace(place_point);
if (!place_success) {
  ROS_WARN("Failed to place the largest object");
  return false;
}

ROS_INFO("Successfully grasped and placed the largest object");
return true;
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

// Filter out green points
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
 * Callback for the point cloud subscription in continuous scanning mode
 */
void cw2::continuousScanCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // Only process clouds when scanning is active
  if (!is_collecting_clouds_) {
    return;
  }
  
  // Process every Nth frame to avoid overwhelming memory
  if (cloud_frame_counter_++ % t3_pointcloud_save_interval_ != 0) {
    return;
  }
  
  ROS_INFO("Processing point cloud frame %d", cloud_frame_counter_);
  
  // Convert from ROS message to PCL point cloud
  PointCPtr cloud(new PointC);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Skip empty clouds
  if (cloud->empty()) {
    ROS_WARN("Received empty point cloud, skipping");
    return;
  }
  
  // Transform cloud to base frame if needed
  if (msg->header.frame_id != base_frame_) {
    try {
      // Look up transform from camera frame to base frame
      geometry_msgs::TransformStamped transformStamped = 
          tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.5));
      
      // Transform point cloud
      PointCPtr transformed_cloud(new PointC);
      pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transformStamped.transform);
      cloud = transformed_cloud;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform point cloud from frame %s to %s: %s", 
               msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
  }
  
  // Filter out green floor and background using the same logic as filterPointCloudByColor
  PointCPtr filtered_cloud = filterPointCloudByColor(cloud);
  
  // Downsample using voxel grid filter to speed up processing and reduce memory usage
  PointCPtr downsampled_cloud(new PointC);
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(filtered_cloud);
  voxel_filter.setLeafSize(t3_continuous_scan_voxel_size_, 
                           t3_continuous_scan_voxel_size_, 
                           t3_continuous_scan_voxel_size_);
  voxel_filter.filter(*downsampled_cloud);
  
  // Save the filtered and downsampled cloud
  collected_clouds_.push_back(downsampled_cloud);
  
  ROS_INFO("Added cloud with %zu points (after filtering and downsampling from %zu points)",
           downsampled_cloud->points.size(), cloud->points.size());
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
  return (h >= 60.0f && h <= 160.0f && s >= 0.05f && v >= 0.1f);
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

  // 添加离群点移除
  PointCPtr cleaned_cloud(new PointC);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(merged_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cleaned_cloud);
  
  ROS_INFO("Merged %zu clouds with total %zu points", clouds.size(), merged_cloud->points.size());

  // Downsample the merged cloud
  PointCPtr final_cloud(new PointC);
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(merged_cloud);
  voxel_filter.setLeafSize(t3_merge_voxel_size_, 
                          t3_merge_voxel_size_, 
                          t3_merge_voxel_size_);
  voxel_filter.filter(*final_cloud);
  ROS_INFO("After final voxel filtering (%f mm): %zu points",
           t3_merge_voxel_size_ * 1000.0, final_cloud->points.size());
  
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
  // Predefined rectangular path, no need to change
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
  float speed_factor = 0.04;
  
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
