#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_drivers/eo_driver.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

class EOCameraNode : public rclcpp::Node {
public:
    EOCameraNode() : Node("eo_camera_node"), use_simulation_(true) {
        // Declare parameters
        this->declare_parameter("use_simulation", true);
        this->declare_parameter("device_path", "/dev/eo_camera");
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1080);
        this->declare_parameter("exposure", 10.0);
        this->declare_parameter("gain", 1.0);

        // Get parameters
        use_simulation_ = this->get_parameter("use_simulation").as_bool();
        std::string device_path = this->get_parameter("device_path").as_string();
        double frame_rate = this->get_parameter("frame_rate").as_double();
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        double exposure = this->get_parameter("exposure").as_double();
        double gain = this->get_parameter("gain").as_double();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("sensors/eo_image", 10);

        if (!use_simulation_) {
            driver_ = std::make_unique<sensor_drivers::EOOpticalDriver>();
            driver_->setResolution(width, height);
            driver_->setFrameRate(frame_rate);
            driver_->setExposure(exposure);
            driver_->setGain(gain);
            if (!driver_->initialize(device_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize EO driver, falling back to simulation");
                use_simulation_ = true;
            }
        }

        int period_ms = static_cast<int>(1000.0 / frame_rate);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&EOCameraNode::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "EO Camera Node started. Publishing to sensors/eo_image. Simulation: %s",
                   use_simulation_ ? "true" : "false");
    }

private:
    void publish_image() {
        cv::Mat frame;

        if (use_simulation_) {
            // Fallback to simulation
            frame = cv::Mat(1080, 1920, CV_8UC3);
            // Create a simple gradient
            for (int y = 0; y < frame.rows; ++y) {
                for (int x = 0; x < frame.cols; ++x) {
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(x % 256, y % 256, (x + y) % 256);
                }
            }
        } else {
            // Use real driver
            if (!driver_->captureFrame(frame)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to capture frame from EO driver");
                return;
            }
        }

        // Apply preprocessing
        preprocess_frame(frame);

        // Convert to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "eo_camera_link";

        publisher_->publish(*msg);
    }

    void preprocess_frame(cv::Mat& frame) {
        // Noise filtering
        cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

        // Color correction (simple white balance)
        cv::Mat ycrcb;
        cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> channels;
        cv::split(ycrcb, channels);
        cv::equalizeHist(channels[0], channels[0]); // Equalize luminance
        cv::merge(channels, ycrcb);
        cv::cvtColor(ycrcb, frame, cv::COLOR_YCrCb2BGR);

        // Lens distortion correction (mock - in real implementation use camera calibration)
        // For now, just apply a slight sharpening
        cv::Mat sharpened;
        cv::GaussianBlur(frame, sharpened, cv::Size(0, 0), 3);
        cv::addWeighted(frame, 1.5, sharpened, -0.5, 0, frame);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<sensor_drivers::EOOpticalDriver> driver_;
    bool use_simulation_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EOCameraNode>());
    rclcpp::shutdown();
    return 0;
}
