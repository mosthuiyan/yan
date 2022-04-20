// for convert string path to string stream and to str() at end
#include <sstream>

// for file io
// for ROS image message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
// for image of cv::Mat to ROS sensor_msgs::Image 
#include <cv_bridge/cv_bridge.h>
// for read image
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

int main(int argc, char **argv){
  ros::init(argc, argv, "kitti360_helper");
  ros::NodeHandle nh("~");
  
  ros::Rate loop_rate(10);

  string dataset_root, sequence;
  nh.getParam("dataset_root", dataset_root);
  nh.getParam("sequence", sequence);
  ROS_INFO("%s", ("Use dataset : " + dataset_root + sequence).c_str());
  // use left image timestamp as sync
  // string timestamp_path = sequence + "image_00/timestamps.txt";
  // ifstream timestamp_file(dataset_root + timestamp_path, ifstream::in);

  // image Publisher
  image_transport::ImageTransport img_tp(nh);
  image_transport::Publisher left_img_pub = img_tp.advertise("left_image", 2);
  image_transport::Publisher right_img_pub = img_tp.advertise("right_image", 2);

  string line;
  size_t img_idx = 0;
  while(ros::ok){ // getline(timestamp_file, line) && 
    
    stringstream left_img_root_ss, right_img_root_ss, suffix;
    suffix << std::setfill('0') << std::setw(10) << img_idx++ << ".png";
    left_img_root_ss << dataset_root << "data_2d_raw/" << sequence << "image_00/data_rect/" << suffix.str();
    right_img_root_ss << dataset_root << "data_2d_raw/" << sequence << "image_01/data_rect/"  << suffix.str();
    cv::Mat left_img = cv::imread(left_img_root_ss.str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat right_img = cv::imread(left_img_root_ss.str(), CV_LOAD_IMAGE_GRAYSCALE);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/kitti360_image";
    sensor_msgs::ImagePtr left_img_msg = cv_bridge::CvImage(header, "mono8", left_img).toImageMsg();
    sensor_msgs::ImagePtr right_img_msg = cv_bridge::CvImage(header, "mono8", right_img).toImageMsg();

    left_img_pub.publish(left_img_msg);
    right_img_pub.publish(right_img_msg);

    loop_rate.sleep();
  }
  return 0;
}