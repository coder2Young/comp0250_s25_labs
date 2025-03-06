/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>

int main(int argc, char **argv){
  
  ros::init(argc,argv, "cw1_solution_node");
  ros::NodeHandle nh;

  // MoveIt! requirement for non-blocking group.move()
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // create an instance of the cw1 class
  cw1 cw_class(nh);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_img =
                  nh.subscribe ("/r200/camera/color/image_rect",
                  1,
                  &cw1::cameraImgCallback,
                  &cw_class);

  ros::Subscriber sub_info = 
                nh.subscribe ("/r200/camera/color/camera_info",
                1,
                &cw1::cameraInfoCallback,
                &cw_class);

  ros::Subscriber sub_depth = 
                nh.subscribe("/r200/camera/depth_registered/points",
                1,
                &cw1::depthImgCallback,
                &cw_class);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}