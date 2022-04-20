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
    Kitti360StereoImagePub(ros::NodeHandle &nh, ros::NodeHandle &pnh, size_t frame_index) 
                : nh_(nh), pnh_(pnh), id_(frame_index) {
      // read param
      // 读取参数
      std::string dataset_root, sequence;
      pnh.getParam("dataset_root", dataset_root);
      pnh.getParam("sequence", sequence);
      ROS_INFO("%s", ("Use dataset : " + dataset_root + sequence).c_str());
      
      // initrialize ros publisher of left/righr image
      // 初始化两个 ros publisher, 对应左右目的图像
      image_transport::ImageTransport img_tp(pnh);
      left_pub_ = img_tp.advertise("left_image", 2);
      right_pub_ = img_tp.advertise("right_image", 2);

      // read data, you can see KITTI360 dataset format in http://www.cvlibs.net/datasets/kitti-360/documentation.php
      // 读取数据, 可以从下面的链接了解 KITTI360 数据集的格式 http://www.cvlibs.net/datasets/kitti-360/documentation.php
      std::stringstream left_img_root_ss, right_img_root_ss, suffix;
      suffix << std::setfill('0') << std::setw(10) << frame_index << ".png";
      left_img_root_ss << dataset_root << "data_2d_raw/" << sequence << "image_00/data_rect/" << suffix.str();
      right_img_root_ss << dataset_root << "data_2d_raw/" << sequence << "image_01/data_rect/"  << suffix.str();

      left_img_ = left_img_root_ss.str();
      right_img_ = right_img_root_ss.str();

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
      
      
      cv::Mat left_img = cv::imread(left_img_, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat right_img = cv::imread(right_img_, CV_LOAD_IMAGE_GRAYSCALE);

      sensor_msgs::ImageConstPtr rosmsg_left_img_cptr_ 
                = cv_bridge::CvImage(header, "mono8", left_img).toImageMsg();
      sensor_msgs::ImageConstPtr rosmsg_right_img_cptr_
                = cv_bridge::CvImage(header, "mono8", right_img).toImageMsg();

      left_pub_.publish(rosmsg_left_img_cptr_);
      right_pub_.publish(rosmsg_right_img_cptr_);

      return true;
    }

  private:
    std::size_t id_;
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    image_transport::Publisher left_pub_;
    image_transport::Publisher right_pub_;
    std::string left_img_;
    std::string right_img_;
  };
}