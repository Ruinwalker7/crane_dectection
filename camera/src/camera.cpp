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

    // watch dog
    // double dt = (this->now().seconds() - rclcpp::Time(camera_info_.header.stamp).seconds());
    // if (dt > 2.0)
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Frame has been lost for %f seconds, restart!", dt);
    //   this->close();
    //   return;
    // }
    // set camera exposure
    // int temp_exposure_time = this->get_parameter("exposure_time").as_int();
    // double temp_gain = this->get_parameter("gain").as_double();
    // if (abs(temp_exposure_time - exposure_time_) > 0.5 || abs(temp_gain - gain_) > 0.05)
    // {
    //   exposure_time_ = temp_exposure_time;
    //   gain_ = temp_gain;
    //   RCLCPP_INFO(this->get_logger(), "Set exposure_time: %d, gain: %f", exposure_time_, gain_);
    // }

    cv::Mat camera_img;
    capture1.read(camera_img);
    // std::cout << camera_img.size() << std::endl;
    cv::Mat mat;
    cv::resize(camera_img,mat,cv::Size(1280,720));
    // sensor_msgs::msg::Image::SharedPtr msg =
    // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",mat).toImageMsg();
    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", camera_img).toImageMsg();
    image_msg_->header.stamp = node_clock->now();
    publisher_->publish(*image_msg_.get());

    cv::imshow("camera", camera_img);
    cv::waitKey(1);
  }

  void open()
  {
    capture1.open(-1);

    capture1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture1.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture1.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture1.set(cv::CAP_PROP_FPS, 60);
    capture1.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25); // 亮度 1

    int height = capture1.get(cv::CAP_PROP_FRAME_HEIGHT);
    int weight = capture1.get(cv::CAP_PROP_FRAME_WIDTH);
    RCLCPP_INFO(this->get_logger(), "%d %d", height, weight);

    if (!capture1.isOpened())
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

  cv::VideoCapture capture1;
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ImagePublisher)