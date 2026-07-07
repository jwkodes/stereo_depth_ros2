#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.hpp>
#include <vector>
#include <string>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;
class KittiLoader : public rclcpp::Node
{
public:
    KittiLoader() : Node{"kitti_loader"}, frame_idx_{0}
    {
        declare_parameter("left_image_folder_path", "data/left_images");
        declare_parameter("right_image_folder_path", "data/right_images");
        declare_parameter("publish_rate_hz", 10.0);

        const std::string left_image_folder_path = get_parameter("left_image_folder_path").as_string();
        const std::string right_image_folder_path = get_parameter("right_image_folder_path").as_string();
        const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();

        // Create publishers
        left_pub_ = create_publisher<sensor_msgs::msg::Image>("/kitti/left/image_raw", 10);
        right_pub_ = create_publisher<sensor_msgs::msg::Image>("/kitti/right/image_raw", 10);

        // Create timer
        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz)), std::bind(&KittiLoader::timer_callback, this));

        // Get all image paths
        init_image_paths(left_image_folder_path, true);
        init_image_paths(right_image_folder_path, false);
        RCLCPP_INFO(get_logger(), "Found %zu left images in %s", left_paths_.size(), left_image_folder_path.c_str());
        RCLCPP_INFO(get_logger(), "Found %zu right images in %s", right_paths_.size(), right_image_folder_path.c_str());
    }

private:
    void init_image_paths(const std::string &image_folder_path, const bool left = true)
    {
        auto &paths = left ? left_paths_ : right_paths_;
        paths.clear();
        for (const auto &entry : fs::directory_iterator(image_folder_path))
        {
            paths.push_back(entry.path().string());
        }
        std::sort(paths.begin(), paths.end());
    }

    bool next_stereo_pair(cv::Mat &left_image, cv::Mat &right_image)
    {
        bool status = false;
        if (frame_idx_ < left_paths_.size() && frame_idx_ < right_paths_.size())
        {
            left_image = cv::imread(left_paths_[frame_idx_], cv::IMREAD_COLOR);
            right_image = cv::imread(right_paths_[frame_idx_], cv::IMREAD_COLOR);

            frame_idx_++;
            status = true;
        }

        return status;
    }
    void timer_callback()
    {
        cv::Mat left_image, right_image;
        if (next_stereo_pair(left_image, right_image))
        {
            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = std::to_string(frame_idx_);
            auto left_msg = cv_bridge::CvImage(header, "bgr8", left_image).toImageMsg();
            auto right_msg = cv_bridge::CvImage(header, "bgr8", right_image).toImageMsg();

            left_pub_->publish(*left_msg);
            right_pub_->publish(*right_msg);
            RCLCPP_INFO(this->get_logger(), "Publishing frame %zu / %zu", frame_idx_, left_paths_.size());
        }
        else
        {
            // End of data
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> left_paths_;
    std::vector<std::string> right_paths_;
    size_t frame_idx_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KittiLoader>());
    rclcpp::shutdown();

    return 0;
}