#ifndef VIDEO_PROCESSOR_HPP
#define VIDEO_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <prob_rob_msgs/msg/point2_d.hpp>
#include <prob_rob_msgs/msg/point2_d_array_stamped.hpp>

class VideoProcessor
{
public:
    VideoProcessor(rclcpp::Node::SharedPtr node);
    void spin();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    image_transport::Subscriber img_subscription_;
    image_transport::Publisher img_goodfeature_pub_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<prob_rob_msgs::msg::Point2DArrayStamped>::SharedPtr goodfeature_pub_;
    int max_corners_;
    bool run_color_filter_;
};

#endif // VIDEO_PROCESSOR_HPP
