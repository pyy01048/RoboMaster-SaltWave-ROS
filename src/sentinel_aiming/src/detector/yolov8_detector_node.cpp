#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sentinel_msgs/msg/detected_object.hpp>
#include <opencv2/opencv.hpp>

class YOLOv8DetectorNode : public rclcpp::Node {
public:
    YOLOv8DetectorNode() : Node("yolov8_detector_node") {
        this->declare_parameter("camera_topic", "/camera/image_raw");
        this->declare_parameter("camera_info_topic", "/camera/camera_info");
        this->declare_parameter("detection_topic", "/detections");
        this->declare_parameter("model_path", "/home/pyy/Documents/sentiel/models/yolov8n.engine");
        this->declare_parameter("conf_threshold", 0.25);
        this->declare_parameter("nms_threshold", 0.65);

        camera_topic_ = this->get_parameter("camera_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
        detection_topic_ = this->get_parameter("detection_topic").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        conf_threshold_ = this->get_parameter("conf_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 10,
            std::bind(&YOLOv8DetectorNode::image_callback, this, std::placeholders::_1));

        detection_pub_ = this->create_publisher<sentinel_msgs::msg::DetectedObject>(detection_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "YOLOv8 Detector Node started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);

        auto start = std::chrono::high_resolution_clock::now();
        auto detections = run_inference(cv_image);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "Inference time: %ld ms", duration);

        publish_detections(detections, msg->header);
    }

    std::vector<cv::Rect> run_inference(cv::Mat& image) {
        return {};
    }

    void publish_detections(const std::vector<cv::Rect>& detections, const std_msgs::msg::Header& header) {
        sentinel_msgs::msg::DetectedObject detection;
        detection.header = header;

        for (const auto& det : detections) {
            detection.x_min = det.x;
            detection.y_min = det.y;
            detection.x_max = det.x + det.width;
            detection.y_max = det.y + det.height;
        }

        detection_pub_->publish(detection);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sentinel_msgs::msg::DetectedObject>::SharedPtr detection_pub_;

    std::string camera_topic_;
    std::string camera_info_topic_;
    std::string detection_topic_;
    std::string model_path_;
    double conf_threshold_;
    double nms_threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YOLOv8DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
