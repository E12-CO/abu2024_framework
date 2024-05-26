#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>

// for Size
#include <opencv2/core/types.hpp>
// for CV_8UC3
#include <opencv2/core/hal/interface.h>
// for compressing the image
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
  public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MinimalPublisher::timer_callback, this)); // Reduced timer interval to 100 milliseconds
    }

  private:
    void timer_callback() {
        cv::Mat img;
        cv::VideoCapture cap(0);
        cap >> img;

        // Resize the image to dimensions 360x240
        cv::Size new_size(360, 240);
        cv::Mat resized_img;
        cv::resize(img, resized_img, new_size);

        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_img)
                .toImageMsg();

        publisher_->publish(*msg.get());
        std::cout << "Published!" << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    size_t count_;
};

int main(int argc, char *argv[]) {
    printf("Starting...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
