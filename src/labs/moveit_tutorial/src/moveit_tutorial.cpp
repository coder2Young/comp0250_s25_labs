/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */



#include <moveit_tutorial/moveit_tutorial.h>

////////////////////////////////////////////////////////////////////////////////

SrvClass::SrvClass(ros::NodeHandle &nh)
{
  // save the input nodehandle as an internal class variable
  nh_ = nh;

  // namespace for our ROS services, they will appear as "/namespace/srv_name"
  std::string service_ns = "/moveit_tutorial";

  // advertise the services available from this node
  set_arm_srv_ = nh_.advertiseService(service_ns + "/set_arm",
    &SrvClass::setArmCallback, this);
  set_gripper_srv_ = nh_.advertiseService(service_ns + "/set_gripper",
    &SrvClass::setGripperCallback, this);
  add_collision_srv_ = nh_.advertiseService(service_ns + "/add_collision",
    &SrvClass::addCollisionCallback, this);
  remove_collision_srv_ = nh_.advertiseService(service_ns + "/remove_collision",
    &SrvClass::removeCollisionCallback, this);
  pick_srv_ = nh_.advertiseService(service_ns + "/pick",
    &SrvClass::pickCallback, this);

  // print to the terminal
  ROS_INFO("MoveIt! services initialisation finished, ready to go");
}


////////////////////////////////////////////////////////////////////////////////
bool 
SrvClass::setArmCallback(moveit_tutorial::set_arm::Request &request,
  moveit_tutorial::set_arm::Response &response)
{
  // set arm position, true if sucessful 
  bool success = moveArm(request.pose);

  response.success = success;

  return success;
}

////////////////////////////////////////////////////////////////////////////////
bool
SrvClass::setGripperCallback(moveit_tutorial::set_gripper::Request &request,
  moveit_tutorial::set_gripper::Response &response)
{
  // set gripper fingers to a specific width
  bool success = moveGripper(request.finger_distance);

  response.success = success;

  return success;  
}

////////////////////////////////////////////////////////////////////////////////
bool
SrvClass::addCollisionCallback(moveit_tutorial::add_collision::Request &request,
  moveit_tutorial::add_collision::Response &response)
{
  // TODO: use the addCollisionObject function
  addCollisionObject(request.object_name, request.centre, request.dimensions,
    request.orientation);
  response.success = true;

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool 
SrvClass::moveArm(geometry_msgs::Pose target_pose)
{
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

////////////////////////////////////////////////////////////////////////////////
bool 
SrvClass::moveGripper(float width)
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

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////
void 
SrvClass::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // Hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////
void
SrvClass::removeCollisionObject(std::string object_name)
{
  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = base_frame_;

  // define that we will be removing this collision object 
  collision_object.operation = collision_object.REMOVE;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

////////////////////////////////////////////////////////////////////////////////
bool
SrvClass::removeCollisionCallback(moveit_tutorial::remove_collision::Request &request,
  moveit_tutorial::remove_collision::Response &response)
{
  removeCollisionObject(request.object_name);
  response.success = true;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
void
SrvClass::pickObject(geometry_msgs::Point grasp_point)
{
  // movearm to the grasp point
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = grasp_point;
  // get the current orientation of the end effector
  grasp_pose.orientation = arm_group_.getCurrentPose().pose.orientation;

  // adjust the orientation by roll=0, pitch=0, yaw=pi/4
  // grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI/4);

  // open the gripper
  moveGripper(gripper_open_);

  moveArm(grasp_pose);

  // close the gripper
  moveGripper(gripper_closed_);

  // move the object to a higher position
  grasp_pose.position.z += 0.1;
  moveArm(grasp_pose);
}

////////////////////////////////////////////////////////////////////////////////
bool
SrvClass::pickCallback(moveit_tutorial::pick::Request &request,
  moveit_tutorial::pick::Response &response)
{
  pickObject(request.grasp_point);
  response.success = true;

  return true;
}
