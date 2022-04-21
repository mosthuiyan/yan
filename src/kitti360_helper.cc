// ROS
#include <ros/ros.h>
// for kitti360 data publish
#include "kitti360_publisher.hpp"

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "kitti360_helper");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  ros::Rate loop_rate(10);

  string line;
  size_t frame_idx = 0;
  std::string dataset_root, sequence;
  pnh.getParam("dataset_root", dataset_root);
  pnh.getParam("sequence", sequence);

  yan::Kitti360StereoImagePub kitti360_img_pub(nh, pnh);
  while(ros::ok){ // getline(timestamp_file, line) && 
    
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/kitti360_image";
    
    if(!kitti360_img_pub.image_publish(header, frame_idx)){
      ROS_INFO("system error : image publish fail.");
    }

    frame_idx++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}