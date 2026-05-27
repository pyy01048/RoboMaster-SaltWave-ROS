#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/detected_object.hpp>
#include <sentinel_msgs/msg/tracked_target.hpp>
#include <deque>
#include <mutex>

class ByteTrackNode : public rclcpp::Node {
public:
    ByteTrackNode() : Node("bytetrack_node") {
        this->declare_parameter("detections_topic", "/detections");
        this->declare_parameter("tracked_targets_topic", "/tracked_targets");
        this->declare_parameter("track_buffer", 30);
        this->declare_parameter("match_thresh", 0.6);
        this->declare_parameter("track_thresh", 0.5);
        this->declare_parameter("confirm_thresh", 0.7);

        detections_topic_ = this->get_parameter("detections_topic").as_string();
        tracked_targets_topic_ = this->get_parameter("tracked_targets_topic").as_string();
        track_buffer_ = this->get_parameter("track_buffer").as_int();
        match_thresh_ = this->get_parameter("match_thresh").as_double();
        track_thresh_ = this->get_parameter("track_thresh").as_double();
        confirm_thresh_ = this->get_parameter("confirm_thresh").as_double();

        detections_sub_ = this->create_subscription<sentinel_msgs::msg::DetectedObject>(
            detections_topic_, 10,
            std::bind(&ByteTrackNode::detections_callback, this, std::placeholders::_1));

        tracked_targets_pub_ = this->create_publisher<sentinel_msgs::msg::TrackedTarget>(tracked_targets_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "ByteTrack Node started");
    }

private:
    void detections_callback(const sentinel_msgs::msg::DetectedObject::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        update_tracks(msg);
        publish_tracks(msg->header);
    }

    void update_tracks(const sentinel_msgs::msg::DetectedObject::SharedPtr msg) {
    }

    void publish_tracks(const std_msgs::msg::Header& header) {
    }

    rclcpp::Subscription<sentinel_msgs::msg::DetectedObject>::SharedPtr detections_sub_;
    rclcpp::Publisher<sentinel_msgs::msg::TrackedTarget>::SharedPtr tracked_targets_pub_;
    std::mutex mutex_;

    std::string detections_topic_;
    std::string tracked_targets_topic_;
    int track_buffer_;
    double match_thresh_;
    double track_thresh_;
    double confirm_thresh_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ByteTrackNode>());
    rclcpp::shutdown();
    return 0;
}
