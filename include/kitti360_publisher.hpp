// c++
#include <string>
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

namespace yan{
  /**
   * @brief kitti360 stereo image data loader and publish it to ros
   *        kitti360 双目图像数据加载, 同时提供发布到ROS的接口
   * 
   */
  class Kitti360StereoImagePub{
  public:
    /**
     * @brief Construct a new Kitti360 Stereo Image Pub object
     * 
     * @param nh  继承的 ros::NodeHandle, 命名空间从node名称开始
     * @param pnh 私有的 ros::NodeHandle, 命名空间从给定的名字开始
     */
    Kitti360StereoImagePub(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
                : nh_(nh), pnh_(pnh) {
      // read param
      // 读取参数
      pnh.getParam("dataset_root", dataset_root_);
      pnh.getParam("sequence", sequence_);
      ROS_INFO("%s", ("Use dataset : " + dataset_root_ + sequence_).c_str());
      
      // initrialize ros publisher of left/righr image
      // 初始化两个 ros publisher, 对应左右目的图像
      image_transport::ImageTransport img_tp(pnh);
      left_pub_ = img_tp.advertise("left_image", 2);
      right_pub_ = img_tp.advertise("right_image", 2);
      
    }

    ~Kitti360StereoImagePub(){}

    /**
     * @brief 发布消息
     * 
     * @param header 
     * @param frame_index 
     * @return true 
     * @return false 
     */
    bool image_publish(std_msgs::Header &header, size_t &frame_index){
      // read data, you can see KITTI360 dataset format in http://www.cvlibs.net/datasets/kitti-360/documentation.php
      // 读取数据, 可以从下面的链接了解 KITTI360 数据集的格式 http://www.cvlibs.net/datasets/kitti-360/documentation.php
      std::stringstream left_img_root_ss, right_img_root_ss, suffix;
      suffix << std::setfill('0') << std::setw(10) << frame_index << ".png";
      left_img_root_ss << dataset_root_ << "data_2d_raw/" << sequence_ << "image_00/data_rect/" << suffix.str();
      right_img_root_ss << dataset_root_ << "data_2d_raw/" << sequence_ << "image_01/data_rect/"  << suffix.str();

      
      cv::Mat left_img = cv::imread(left_img_root_ss.str(), CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat right_img = cv::imread(right_img_root_ss.str(), CV_LOAD_IMAGE_GRAYSCALE);

      sensor_msgs::ImageConstPtr rosmsg_left_img_cptr_ 
                = cv_bridge::CvImage(header, "mono8", left_img).toImageMsg();
      sensor_msgs::ImageConstPtr rosmsg_right_img_cptr_
                = cv_bridge::CvImage(header, "mono8", right_img).toImageMsg();

      left_pub_.publish(rosmsg_left_img_cptr_);
      right_pub_.publish(rosmsg_right_img_cptr_);

      return true;
    }

  private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    std::string dataset_root_;
    std::string sequence_;
  };
}