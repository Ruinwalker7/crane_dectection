#include <cstdio>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(const rclcpp::NodeOptions &options) : Node("image_publisher", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Camera Driver!");
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_img", 1);
    node_clock = this->get_clock();

    // Check if camera is alive every 100ms.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ImagePublisher::timerCallback, this));
  }

  void timerCallback()
  {
    while (!is_open_ && rclcpp::ok())
    {
      open();
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    cv::Mat camera_img;
    capture.read(camera_img);
    cv::Mat mat;
    cv::resize(camera_img,mat,cv::Size(1280,720));
    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", camera_img).toImageMsg();
    image_msg_->header.stamp = node_clock->now();
    publisher_->publish(*image_msg_.get());

    cv::imshow("camera", camera_img);
    cv::waitKey(1);
  }

  void open()
  {
    //open default camera
    capture.open(-1);

    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FPS, 60);
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // 亮度 1

    int height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    int weight = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    RCLCPP_INFO(this->get_logger(), "%d %d", height, weight);

    if (!capture.isOpened())
    {
      is_open_ = false;
      RCLCPP_INFO(this->get_logger(), "Camera Error!");
    }
    else
    {
      is_open_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera opened");
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  sensor_msgs::msg::Image::SharedPtr image_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr node_clock;
  bool is_open_;
  cv::VideoCapture capture;
};


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ImagePublisher)
