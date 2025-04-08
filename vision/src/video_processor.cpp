#include "video_processor.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

VideoProcessor::VideoProcessor(rclcpp::Node::SharedPtr node) : max_corners_(200)
{
    node_ = node;
    image_transport::ImageTransport it(node_);
    img_subscription_ = it.subscribe("/camera/image_raw", 1,
                                     &VideoProcessor::imageCallback, this);
    img_goodfeature_pub_ = it.advertise("goodfeature/image_raw", 5);
    RCLCPP_INFO(node_->get_logger(), "max corners is %d", max_corners_);
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
        img_goodfeature_pub_.publish(goodfeature_msg);
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
