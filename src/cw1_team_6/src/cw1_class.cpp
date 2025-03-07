/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  // Initialise the member variables
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  cloud_(new PointC),
  collision_object_vector_(),
  debug_ (false)
{
  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  sub_img_ = nh.subscribe ("/r200/camera/color/image_raw",
    1,
    &cw1::cameraImgCallback,
    this);

  sub_img_info_ = nh.subscribe ("/r200/camera/color/camera_info",
    1,
    &cw1::cameraInfoCallback,
    this);

  sub_depth_ = nh.subscribe("/r200/camera/depth_registered/points",
    1,
    &cw1::depthImgCallback,
    this);

  if (debug_)
  {
    pub_filtered_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
  }

  cw1Config();

  ROS_INFO("cw1 class initialised");
}

void
cw1::cw1Config()
{
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

  return;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  bool sucess = t1_process(request, response);
  
  return sucess;
}

bool
cw1::t1_process(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response)
{
  geometry_msgs::PoseStamped pick_pose = request.object_loc;
  geometry_msgs::PointStamped place_point = request.goal_loc;
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  ROS_INFO("Current pose: x=%f, y=%f, z=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("Current orientation: x=%f, y=%f, z=%f, w=%f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);

  clearCollisionObject();
  addCollitionGround();
  addCollisionBasket(place_point.point);

  pickAndPlace(pick_pose, place_point);

  ROS_INFO("Task1 completed");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  bool success = t2_process(request, response);

  return success;
}

bool
cw1::t2_process(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  std::vector<geometry_msgs::PointStamped> potential_basket_locs = request.basket_locs;
  int num_baskets = potential_basket_locs.size();

  // Move to scan pos
  geometry_msgs::PoseStamped scan_pose;
  scan_pose.header.frame_id = base_frame_;
  scan_pose.pose = scan_pose_;

  ROS_INFO("Moving to scan pose: x=%f, y=%f, z=%f", 
      scan_pose_.position.x, scan_pose_.position.y, scan_pose_.position.z);
  moveArm(scan_pose);

  // Wait for the basket to spawn for 1s
  ros::Duration(1.0).sleep();

  int image_height = camera_info_.height;
  int image_width = camera_info_.width;
  // Iterate through potential baskets
  for (int i = 0; i < num_baskets; ++i) {
    geometry_msgs::PointStamped basket_loc = potential_basket_locs[i];
    std::pair<int, int> image_loc = getLocOfCameraImage(basket_loc);

    ROS_INFO("Basket %d at 2D image coordinate: u=%d, v=%d", i+1, image_loc.first, image_loc.second);
  
    // Check if the basket is in the image
    if (image_loc.first < 0 || image_loc.first > image_width || image_loc.second < 0 || image_loc.second > image_height) {
      ROS_WARN("Basket %d not in image", i+1);
      continue;
    }
    
    // Get the color of the basket using opencv
    cv::Mat image;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(camera_image_.latest_image, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;

    // Get the color of the basket
    cv::Vec3b color = image.at<cv::Vec3b>(image_loc.second, image_loc.first);
    float r = color[2] / 255.0;
    float g = color[1] / 255.0;
    float b = color[0] / 255.0;

    // Print rgb values
    ROS_INFO("Basket %d r,g,b: r=%.3f, g=%.3f, b=%.3f", i+1, r, g, b);

    std::string basket_colour = colorMapping(r, g, b);
    response.basket_colours.push_back(basket_colour);

    // Print out the colours strings
    ROS_INFO("Basket %d/%d: %s", i+1, num_baskets ,basket_colour.c_str());
  }
  ROS_INFO("Task2 completed");

  ROS_INFO("Response message:");
  std::string response_msg = "";
  // Show the response msg
  for (int i = 0; i < num_baskets; ++i) {
    response_msg += response.basket_colours[i];
    response_msg += " ";
  }
  ROS_INFO("Response message: %s", response_msg.c_str());

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */
  ROS_INFO("The coursework solving callback for task 3 has been triggered");
  bool success = t3_process(request, response);

  return success;
}

bool
cw1::t3_process(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */
  bool success = false;

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  // 1. Move to scan pos
  geometry_msgs::PoseStamped scan_pose;
  scan_pose.header.frame_id = base_frame_;
  scan_pose.pose = scan_pose_;

  ROS_INFO("Moving to scan pose: x=%f, y=%f, z=%f", 
      scan_pose_.position.x, scan_pose_.position.y, scan_pose_.position.z);
  moveArm(scan_pose);

  // Wait 1s for the basket to spawn
  ros::Duration(1.0).sleep();

  // 2. Get PointCloud under base frame
  PointCPtr cloud_base = transformCloudToBaseFrame(cloud_);

  // 3. Clustering
  // Use regionGrowing to segment the point cloud
  // regionGrowing enables both distance and color based segmentation
  std::vector<PointCPtr> clusters = regionGrowing(cloud_base);
  // Merge clusters that are close to each other
  clusters = mergeClusters(clusters);

  // Log out the number of clusters
  ROS_INFO("Number of clusters: %lu", clusters.size());

  std::map<std::string, geometry_msgs::PointStamped> basket_map;
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> box_map;
  for (auto cluster : clusters)
  {
    // Compute the centroid of the cluster to pick or place
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    geometry_msgs::Point centroid_position;

    // Round to 3 decimal places
    centroid_position.x = std::round(centroid[0] * position_precision_) / position_precision_;
    centroid_position.y = std::round(centroid[1] * position_precision_) / position_precision_;
    centroid_position.z = std::round(centroid[2] * position_precision_) / position_precision_;

    // Pick a random point for color
    int random_point = rand() % cluster->size();;
    PointT point = cluster->points[random_point];
    float r = static_cast<float>(point.r) / 255.0f;
    float g = static_cast<float>(point.g) / 255.0f;
    float b = static_cast<float>(point.b) / 255.0f;

    ROS_INFO("==== Cluster info ====");
    ROS_INFO("Cluster center: x=%.3f, y=%.3f, z=%.3f", centroid_position.x, centroid_position.y, centroid_position.z);
    ROS_INFO("Cluster color: r=%.3f, g=%.3f, b=%.3f", r, g, b);
    std::string color_str = colorMapping(r, g, b);
    ROS_INFO("Cluster color: %s", color_str.c_str());
    ROS_INFO("Cluster size: %ld", cluster->size());
    
    if (cluster->size() < box_basket_size_thresh_)
    {
      geometry_msgs::PoseStamped pick_pose;
      pick_pose.header.frame_id = base_frame_;
      pick_pose.pose.position.x = centroid_position.x;
      pick_pose.pose.position.y = centroid_position.y;
      pick_pose.pose.position.z = centroid_position.z - box_size_ / 2.0;
      pick_pose.pose.orientation = arm_group_.getCurrentPose().pose.orientation;
      box_map[color_str].push_back(pick_pose);
    }
    else
    {
      geometry_msgs::PointStamped basket_loc;
      basket_loc.header.frame_id = base_frame_;
      basket_loc.point = centroid_position;
      basket_map[color_str] = basket_loc;
    }
  }

  // Log out num of box and basket
  ROS_INFO("Number of boxes: %lu", box_map.size());
  ROS_INFO("Number of baskets: %lu", basket_map.size());

  // 4. Add all baskets and boxes to the collision scene
  clearCollisionObject();
  addCollitionGround();
  for (auto &entry : basket_map)
  {
    // Add all basket to the collision scene
    geometry_msgs::Point basket_loc_point = entry.second.point;
    addCollisionBasket(basket_loc_point);
  }

  // 5. Iterate through all boxes and pick and place them to the corresponding basket
  for (auto &entry : box_map)
  {
    std::string color = entry.first;
    const auto &box_poses = entry.second;
    if (basket_map.find(color) == basket_map.end()) {
      ROS_WARN("No basket found for color %s; skipping boxes of that color.", color.c_str());
      continue;
    }
    geometry_msgs::PointStamped basket_loc = basket_map[color];
    // Record the num of boxes placed in the basket, since the height will increase
    int count = 0; 
    for (const auto &pick_pose : box_poses)
    {
      geometry_msgs::PointStamped place_point;
      place_point.header.frame_id = base_frame_;
      place_point.point.x = basket_loc.point.x;
      place_point.point.y = basket_loc.point.y;
      place_point.point.z = basket_loc.point.z + count * box_size_;
      ROS_INFO("For color %s: picking box at (%.3f, %.3f, %.3f), placing at (%.3f, %.3f, %.3f)",
              color.c_str(),
              pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z,
              place_point.point.x, place_point.point.y, place_point.point.z);
      pickAndPlace(pick_pose, place_point);
      ++count;
    }
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool
cw1::moveArm(geometry_msgs::PoseStamped target_pose)
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
cw1::moveGripper(float width, float wait_time)
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
cw1::pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point)
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

void
cw1::addCollitionGround()
{
  moveit_msgs::CollisionObject ground;

  float thickness = 9e-3;
  float length = ground_length_;
  float width = ground_width_;
  float height = 0.0;

  ground.id = "ground";
  ground.header.frame_id = base_frame_;

  ground.primitives.resize(1);
  ground.primitives[0].type = ground.primitives[0].BOX;
  ground.primitives[0].dimensions.resize(3);
  ground.primitives[0].dimensions[0] = length;
  ground.primitives[0].dimensions[1] = width;
  ground.primitives[0].dimensions[2] = thickness;

  ground.operation = ground.ADD;

  collision_object_vector_.push_back(ground);

  planning_scene_interface_.applyCollisionObjects(collision_object_vector_);

  return;
}

void 
cw1::addCollisionBasket(geometry_msgs::Point centre)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject basket_bottom;

  ROS_INFO("Adding collision basket at x=%f, y=%f, z=%f", centre.x, centre.y, centre.z);

  float thickness = 9e-3;
  float length = basket_size_;
  float width = basket_size_;
  float height = basket_size_;
  
  // input header information
  basket_bottom.id = "basket_bottm";
  basket_bottom.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  basket_bottom.primitives.resize(1);
  basket_bottom.primitives[0].type = basket_bottom.primitives[0].BOX;
  basket_bottom.primitives[0].dimensions.resize(3);
  basket_bottom.primitives[0].dimensions[0] = length;
  basket_bottom.primitives[0].dimensions[1] = width;
  basket_bottom.primitives[0].dimensions[2] = thickness;

  // define the pose of the collision object
  basket_bottom.primitive_poses.resize(1);
  basket_bottom.primitive_poses[0].position.x = centre.x;
  basket_bottom.primitive_poses[0].position.y = centre.y;
  basket_bottom.primitive_poses[0].position.z = centre.z - height/2.0 + thickness/2.0;
  basket_bottom.primitive_poses[0].orientation.w = 1;

  basket_bottom.operation = basket_bottom.ADD;

  // add the collision object to the vector, then apply to planning scene
  collision_object_vector_.push_back(basket_bottom);

  std::vector<geometry_msgs::Pose> wall_poses;
  geometry_msgs::Pose pose;
  std::vector<std::string> wall_names = {"left_wall", "right_wall", "front_wall", "back_wall"};
  // left
  pose.position.x = centre.x - length/2.0 + thickness/2.0;
  pose.position.y = centre.y;
  pose.position.z = centre.z;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  wall_poses.push_back(pose);

  // right
  pose.position.x = centre.x + length/2.0 - thickness/2.0;
  wall_poses.push_back(pose);

  // front
  pose.position.x = centre.x;
  pose.position.y = centre.y + width/2.0 - thickness/2.0;
  wall_poses.push_back(pose);

  // back
  pose.position.y = centre.y - width/2.0 + thickness/2.0;
  wall_poses.push_back(pose);

  for (size_t i = 0; i < wall_names.size(); ++i) {
      moveit_msgs::CollisionObject wall;
      wall.id = wall_names[i];
      wall.header.frame_id = base_frame_;

      shape_msgs::SolidPrimitive wall_primitive;
      wall_primitive.type = shape_msgs::SolidPrimitive::BOX;
      
      if (i < 2) {  
          wall_primitive.dimensions = {thickness, width, height};
      } else {  
          wall_primitive.dimensions = {length, thickness, height};
      }
      
      wall.primitives.push_back(wall_primitive);
      wall.primitive_poses.push_back(wall_poses[i]);
      wall.operation = wall.ADD;

      collision_object_vector_.push_back(wall);
  }

  planning_scene_interface_.applyCollisionObjects(collision_object_vector_);

  return;
}

void
cw1::clearCollisionObject()
{
  collision_object_vector_.clear();
  planning_scene_interface_.clear();
  return;
}

void
cw1::cameraImgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  camera_image_.latest_image = *msg;
  camera_image_.image_updated = true;
  camera_image_.last_image_time = msg->header.stamp;

  return;
}

void 
cw1::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) 
{
  camera_info_.height = msg->height;
  camera_info_.width = msg->width;
  camera_info_.fx = msg->K[0];
  camera_info_.fy = msg->K[4];
  camera_info_.cx = msg->K[2];
  camera_info_.cy = msg->K[5];
  camera_info_.info_received = true;

  return;
}

void
cw1::depthImgCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, *cloud_);
  cloud_frame_id_ = msg->header.frame_id;

  return;
}

Color 
cw1::stringToColor(std::string color) 
{
  if (color == "red") {
      return red;
  } else if (color == "purple") {
      return purple;
  } else if (color == "blue") {
      return blue;
  } else {
      return none;
  }
}

std::string 
cw1::colorToString(Color color) 
{
  switch (color) {
      case none:
          return "none";
      case red:
          return "red";
      case purple:
          return "purple";
      case blue:
          return "blue";
      default:
          return "unknown";
  }
}

std::vector<PointCPtr>
cw1::regionGrowing(PointCPtr cloud)
{
  PointCPtr filtered_cloud = filterCloudWithColor(cloud);

  // Note: VX Abondoned since it makes the distance thresh hard to tune
  // Apply vx to the cloud
  // pcl::VoxelGrid<PointT> vx;
  // vx.setInputCloud(filtered_cloud);
  // vx.setLeafSize(0.015f, 0.015f, 0.015f);
  // vx.filter(*filtered_cloud);

  // Apply region growing to the cloud, to segment the point cloud
  std::vector<PointCPtr> clusters;
  std::vector<PointCPtr> merged_clusters;
  pcl::RegionGrowingRGB<PointT> reg;
  reg.setInputCloud(filtered_cloud); 
  reg.setDistanceThreshold(cluster_dist_thresh_);
  reg.setPointColorThreshold(cluster_color_thresh_);
  reg.setRegionColorThreshold(cluster_color_thresh_ + 0.1);
  reg.setMinClusterSize(min_cluster_thresh_);
  
  std::vector<pcl::PointIndices> clusters_indices;
  reg.extract(clusters_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin (); it != clusters_indices.end (); ++it)
  {
    PointCPtr cloud_cluster (new PointC);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (filtered_cloud->points[*pit]); 
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }

  return clusters;
}


/* Get the location of a base_frame point in the image coordinate */
std::pair<int, int>
cw1::getLocOfCameraImage(geometry_msgs::PointStamped basket_loc)
{
  
  geometry_msgs::PointStamped point_in_camera;
  std::string target_frame = "color";
  basket_loc.header.stamp = ros::Time(0);
  
  tf_buffer_.transform(basket_loc, point_in_camera, target_frame);

  //ROS_INFO("Point in camera frame: x=%f, y=%f, z=%f", point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z);
  
  int u = static_cast<int>((camera_info_.fx * point_in_camera.point.x / point_in_camera.point.z) + camera_info_.cx);
  int v = static_cast<int>((camera_info_.fy * point_in_camera.point.y / point_in_camera.point.z) + camera_info_.cy);
  
  return std::make_pair(u, v);
}

std::string
cw1::colorMapping(float r, float g, float b)
{
  // 0.1 0.1 0.8 = blue
  // 0.8 0.1 0.8 = purple
  // 0.8 0.1 0.1 = red
  // Use a margin with 0.1 value to detect colors

  if ((r > 0.7 && r < 0.9)
        && (g > 0.0 && g < 0.2)
        && (b > 0.7 && b < 0.9)) {
    return "purple";
  } else if ((r > 0.7 && r < 0.9)
        && (g > 0.0 && g < 0.2)
        && (b > 0.0 && b < 0.2)) {
    return "red";
  } else if ((r > 0.0 && r < 0.2)
        && (g > 0.0 && g < 0.2)
        && (b > 0.7 && b < 0.9)) {
    return "blue";
  } else {
    return "none";
  }

  return "none";
}


/* Remove all items that is not in red/purple/blue */
PointCPtr
cw1::filterCloudWithColor(PointCPtr cloud)
{
  PointCPtr filtered_cloud(new PointC);

  for (const auto& pt : cloud->points)
  {
      float r = static_cast<float>(pt.r) / 255.0f;
      float g = static_cast<float>(pt.g) / 255.0f;
      float b = static_cast<float>(pt.b) / 255.0f;

      bool isPurple = (r > 0.7f && r < 0.9f) &&
                      (g > 0.0f && g < 0.2f) &&
                      (b > 0.7f && b < 0.9f);

      bool isRed    = (r > 0.7f && r < 0.9f) &&
                      (g > 0.0f && g < 0.2f) &&
                      (b > 0.0f && b < 0.2f);

      bool isBlue   = (r > 0.0f && r < 0.2f) &&
                      (g > 0.0f && g < 0.2f) &&
                      (b > 0.7f && b < 0.9f);

      if (isPurple || isRed || isBlue)
      {
          filtered_cloud->points.push_back(pt);
      }
  }

  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;

  return filtered_cloud;
}

/* Transform PCloud from camera frame to base frame */
PointCPtr
cw1::transformCloudToBaseFrame(PointCPtr cloud_in)
{
  PointCPtr cloud_out(new PointC);
  pcl_ros::transformPointCloud(base_frame_, *cloud_, *cloud_out, tf_buffer_);

  return cloud_out;
}

std::vector<PointCPtr> 
cw1::mergeClusters(const std::vector<PointCPtr>& clusters)
{
  std::vector<PointCPtr> mergedClusters;
  std::map<std::string, size_t> colorMap;

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    PointCPtr cluster = clusters[i];

    // Only merge basket, since there will not be multiple basket in the same color
    if (cluster->size() > box_basket_size_thresh_)
    {
      int random_index = rand() % cluster->size();
      PointT point = cluster->points[random_index];
      float r = static_cast<float>(point.r) / 255.0f;
      float g = static_cast<float>(point.g) / 255.0f;
      float b = static_cast<float>(point.b) / 255.0f;
      std::string colStr = colorMapping(r, g, b);

      // Add the color to the map
      if (colorMap.find(colStr) == colorMap.end())
      {
        PointCPtr mergedCluster(new PointC);
        *mergedCluster += *cluster;
        mergedClusters.push_back(mergedCluster);
        colorMap[colStr] = mergedClusters.size() - 1;
      }
      else
      {
        // Merge the cluster
        size_t idx = colorMap[colStr];
        *mergedClusters[idx] += *cluster;
      }
    }
    else
    {
      // Do not merge boxes
      mergedClusters.push_back(cluster);
    }
  }
  
  return mergedClusters;
}