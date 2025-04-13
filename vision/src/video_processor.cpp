#include "video_processor.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

VideoProcessor::VideoProcessor(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    image_transport::ImageTransport it(node_);
    node_->declare_parameter<std::string>("image_topic", "camera/image_raw");
    node_->declare_parameter<std::string>("goodfeature_image_topic", "goodfeature/image_raw");
    node_->declare_parameter<std::string>("goodfeature_corners_topic", "goodfeature/corners");
    node_->declare_parameter<int>("max_features", 50);
    node_->declare_parameter<bool>("run_color_filter", false);
    node_->declare_parameter<int>("min_h", 0);
    node_->declare_parameter<int>("max_h", 360);
    node_->declare_parameter<int>("min_s", 0);
    node_->declare_parameter<int>("max_s", 255);
    node_->declare_parameter<int>("min_v", 0);
    node_->declare_parameter<int>("max_v", 255);

    std::string image_topic = node_->get_parameter("image_topic").as_string();
    std::string goodfeature_image_topic = node_->get_parameter("goodfeature_image_topic").as_string();
    std::string goodfeature_corners_topic = node_->get_parameter("goodfeature_corners_topic").as_string();
    max_corners_ = node_->get_parameter("max_features").as_int();
    run_color_filter_ = node_->get_parameter("run_color_filter").as_bool();
    min_h_ = node_->get_parameter("min_h").as_int();
    max_h_ = node_->get_parameter("max_h").as_int();
    min_s_ = node_->get_parameter("min_s").as_int();
    max_s_ = node_->get_parameter("max_s").as_int();
    min_v_ = node_->get_parameter("min_v").as_int();
    max_v_ = node_->get_parameter("max_v").as_int();
    img_subscription_ = it.subscribe(image_topic, 1,
                                     &VideoProcessor::imageCallback, this);
    img_goodfeature_pub_ = it.advertise(goodfeature_image_topic, 5);
    goodfeature_pub_ = node_->create_publisher<prob_rob_msgs::msg::Point2DArrayStamped>(goodfeature_corners_topic, 1);
    RCLCPP_INFO(node_->get_logger(), "max corners is %d", max_corners_);
    if (run_color_filter_) {
        RCLCPP_INFO(node->get_logger(), "using color filter");
        RCLCPP_INFO(node->get_logger(), "h=[%d, %d], s=[%d, %d], v=[%d, %d]",
                    min_h_, max_h_, min_s_, max_s_, min_v_, max_v_);
    } else
        RCLCPP_INFO(node->get_logger(), "not using color filter");
}

void VideoProcessor::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        // Process the frame (e.g., convert to grayscale)
        cv::Mat gray_frame;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

        /// Parameters for Shi-Tomasi algorithm
        std::vector<cv::Point2f> corners;
        double quality_level = 0.01;
        double min_distance = 10;
        int block_size = 3;
        bool use_harris_detector = false;
        double k = 0.04;
        prob_rob_msgs::msg::Point2DArrayStamped corners_msg;

        cv::RNG rng(12345);

        /// Apply corner detection
        cv::goodFeaturesToTrack(gray_frame, corners, max_corners_,
                                quality_level, min_distance, cv::Mat(),
                                block_size, use_harris_detector, k);
        /// Draw corners detected
        int r = 4;
        for (const cv::Point2f& corner : corners) {
            cv::circle(frame, corner, r, cv::Scalar(rng.uniform(0, 255),
                                                    rng.uniform(0, 255),
                                                    rng.uniform(0, 255)),
                       -1, 8, 0);
        }
        RCLCPP_INFO(node_->get_logger(), "number of corners detected: %ld",
                    corners.size());

        sensor_msgs::msg::Image::SharedPtr goodfeature_msg =
            cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();

        for (const cv::Point2f& i : corners) {
            prob_rob_msgs::msg::Point2D corner;
            corner.x = i.x;
            corner.y = i.y;
            corners_msg.points.push_back(corner);
        }
        corners_msg.header = msg->header;

        img_goodfeature_pub_.publish(goodfeature_msg);
        goodfeature_pub_->publish(corners_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void VideoProcessor::spin(void)
{
    rclcpp::spin(node_);
    rclcpp::shutdown();
}
