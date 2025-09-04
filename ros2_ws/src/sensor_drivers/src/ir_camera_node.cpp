#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_drivers/ir_driver.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

class IRCameraNode : public rclcpp::Node {
public:
    IRCameraNode() : Node("ir_camera_node"), use_simulation_(true) {
        // Declare parameters
        this->declare_parameter("use_simulation", true);
        this->declare_parameter("device_path", "/dev/ir_sensor");
        this->declare_parameter("frame_rate", 10.0);
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);

        // Get parameters
        use_simulation_ = this->get_parameter("use_simulation").as_bool();
        std::string device_path = this->get_parameter("device_path").as_string();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("sensors/ir_image", 10);

        if (!use_simulation_) {
            driver_ = std::make_unique<sensor_drivers::IRDriver>();
            driver_->setResolution(width, height);
            driver_->setFrameRate(frame_rate);
            if (!driver_->initialize(device_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize IR driver, falling back to simulation");
                use_simulation_ = true;
            }
        }

        int period_ms = static_cast<int>(1000.0 / frame_rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&IRCameraNode::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "IR Camera Node started. Publishing to sensors/ir_image. Simulation: %s",
                   use_simulation_ ? "true" : "false");
    }

private:
    void publish_image() {
        cv::Mat frame;

        if (use_simulation_) {
            // Fallback to simulation
            frame = cv::Mat(480, 640, CV_8UC1, cv::Scalar(50));
            // Add hot spots for heat signatures
            cv::circle(frame, cv::Point(320, 240), 10, cv::Scalar(255), -1);
            cv::circle(frame, cv::Point(100, 100), 5, cv::Scalar(220), -1);
        } else {
            // Use real driver
            if (!driver_->captureFrame(frame)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from IR driver");
                return;
            }
        }

        // Apply preprocessing
        preprocess_frame(frame);

        // Convert to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "ir_camera_link";

        publisher_->publish(*msg);
    }

    void preprocess_frame(cv::Mat& frame) {
        // Noise filtering
        cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

        // Enhance heat signatures (contrast enhancement)
        cv::equalizeHist(frame, frame);

        // Additional heat signature enhancement
        cv::Mat enhanced;
        cv::convertScaleAbs(frame, enhanced, 1.2, 10); // Increase contrast and brightness
        frame = enhanced;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<sensor_drivers::IRDriver> driver_;
    bool use_simulation_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRCameraNode>());
    rclcpp::shutdown();
    return 0;
}
