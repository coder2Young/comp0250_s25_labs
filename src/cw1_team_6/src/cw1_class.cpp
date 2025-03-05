/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

bool debug = false;

///////////////////////////////////////////////////////////////////////////////

cw1::cw1(ros::NodeHandle nh):
  tf_buffer_(),
  tf_listener_(tf_buffer_)
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

  ROS_INFO("cw1 class initialised");

  // Scan position should be in (0.5, 0, MAX_HEIGHT can be achieved)
  // to make the camera cover the whole table
  // Init position x=0.557381, y=-0.000013, z=0.612747
  // Orientation x=0.923953, y=-0.382500, z=-0.001784, w=0.000723
  scan_pose_.position.x = 0.35;
  scan_pose_.position.y = 0.0;
  scan_pose_.position.z = 0.84;
  scan_pose_.orientation.x = 0.923953;
  scan_pose_.orientation.y = -0.382500;
  scan_pose_.orientation.z = -0.001784;
  scan_pose_.orientation.w = 0.000723;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  geometry_msgs::PoseStamped pick_pose = request.object_loc;
  geometry_msgs::PointStamped place_point = request.goal_loc;

  ROS_INFO("The coursework solving callback for task 1 has been triggered");
  
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  ROS_INFO("Current pose: x=%f, y=%f, z=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("Current orientation: x=%f, y=%f, z=%f, w=%f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);


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

  // Wait for the basket to spawn for 3s
  ros::Duration(3.0).sleep();

  // Iterate through potential baskets
  for (int i = 0; i < num_baskets; ++i) {
    geometry_msgs::PointStamped basket_loc = potential_basket_locs[i];
    std::pair<int, int> image_loc = getLocOfCameraImage(basket_loc);

    ROS_INFO("Basket %d at image location: u=%d, v=%d", i+1, image_loc.first, image_loc.second);
  
    // Check if the basket is in the image
    if (image_loc.first < 0 || image_loc.first > 640 || image_loc.second < 0 || image_loc.second > 480) {
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
    ROS_INFO("Basket %d at image location: r=%.3f, g=%.3f, b=%.3f", i+1, r, g, b);

    std::string basket_colour = colorMapping(r, g, b);
    response.basket_colours.push_back(basket_colour);

    // Print out the colours strings
    ROS_INFO("Basket %d/%d: %s", i+1, num_baskets ,basket_colour.c_str());
  }
  ROS_INFO("Task2 completed");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

bool
cw1::moveArm(geometry_msgs::PoseStamped target_pose)
{
  /* function to move the arm to a target pose */

  ROS_INFO("Moving the arm to the target pose");

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

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
  ROS_INFO("Attempting to plan the path");
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
        hand_group_.getMoveGroupClient().waitForResult(ros::Duration(wait_time)); // 等待最多 5 秒
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

void
cw1::pickAndPlace(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point)
{
  float hand_length = 0.1;
  float gripper_closed = 0.01;
  float gripper_open = 0.045;
  geometry_msgs::PoseStamped place_point_pose;

  ROS_INFO("Picking and placing object");

  // always consider the griper length
  pick_pose.pose.position.z += hand_length;
  place_point.point.z += hand_length;
  place_point_pose.pose.position = place_point.point;
  place_point_pose.pose.orientation = arm_group_.getCurrentPose().pose.orientation;
  place_point_pose.header.frame_id = base_frame_;

  // move the arm to top of the grasp point 
  pick_pose.pose.orientation = arm_group_.getCurrentPose().pose.orientation;
  pick_pose.pose.position.z += 0.1;
  moveArm(pick_pose);

  moveGripper(gripper_open);

  pick_pose.pose.position.z -= 0.095;
  moveArm(pick_pose);

  // move the gripper to the closed width
  moveGripper(gripper_closed, 4.0);

  ROS_INFO("Object picked up");

  // move to top of place point
  place_point_pose.pose.position.z += 0.2;
  // move the arm to the place point
  moveArm(place_point_pose);

  place_point_pose.pose.position.z -= 0.1;
  // move the arm to the place point
  moveArm(place_point_pose);

  // move the gripper to the closed width
  moveGripper(gripper_open);

  ROS_INFO("Object placed");

  return;
}

void
cw1::task1(geometry_msgs::PoseStamped pick_pose, geometry_msgs::PointStamped place_point)
{
  /* function to solve task 1 */

  ROS_INFO("Solving task 1");

  addCollisionBasket(place_point.point);

  pickAndPlace(pick_pose, place_point);

  return;
}

void 
cw1::addCollisionBasket(geometry_msgs::Point centre)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject basket_bottom;

  std::vector<moveit_msgs::CollisionObject> object_vector;

  ROS_INFO("Adding collision basket at x=%f, y=%f, z=%f", centre.x, centre.y, centre.z);

  float thickness = 9e-3;
  float length = 0.1;
  float width = 0.1;
  float height = 0.1;
  
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
  object_vector.push_back(basket_bottom);


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

      object_vector.push_back(wall);
  }

  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}


//////// For task2 //////
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