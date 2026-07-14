#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>

using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

class StereoDepthSGBM : public rclcpp::Node
{
public:
    StereoDepthSGBM() : Node{"stereo_depth_SGBM"}
    {
        left_sub_.subscribe(this, "/kitti/left/image_raw");
        right_sub_.subscribe(this, "/kitti/right/image_raw");
        disparity_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", 10);

        constexpr size_t QUEUE_SIZE = 10;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(QUEUE_SIZE), left_sub_, right_sub_);
        sync_->registerCallback(&StereoDepthSGBM::stereo_callback, this);

        sgbm_ = cv::StereoSGBM::create(0, 96, 9);
    }

private:
    void stereo_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Matched stereo pair left (%s) and right (%s)", left_msg->header.frame_id.c_str(), right_msg->header.frame_id.c_str());

        cv::Mat left = cv_bridge::toCvShare(left_msg, left_msg->encoding)->image;
        cv::Mat right = cv_bridge::toCvShare(right_msg, right_msg->encoding)->image;
        cv::cvtColor(left, left, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right, right, cv::COLOR_BGR2GRAY);

        // SGBM
        cv::Mat disparity;
        sgbm_->compute(left, right, disparity);
        // disparity is CV_16S and it's scaled by 16
        disparity.convertTo(disparity, CV_32F, 1.0f / 16.0f);

        // Prepare output
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = left_msg->header.frame_id;
        auto disparity_msg = cv_bridge::CvImage(header, "32FC1", disparity).toImageMsg();

        disparity_pub_->publish(*disparity_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    cv::Ptr<cv::StereoSGBM> sgbm_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoDepthSGBM>());
    rclcpp::shutdown();

    return 0;
}