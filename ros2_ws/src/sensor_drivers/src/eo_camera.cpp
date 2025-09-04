#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class EOCameraNode : public rclcpp::Node {
public:
    EOCameraNode() : Node("eo_camera_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("sensors/eo_image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30 Hz
            std::bind(&EOCameraNode::publish_image, this));

        RCLCPP_INFO(this->get_logger(), "EO Camera Node started. Publishing to sensors/eo_image.");
    }

private:
    void publish_image() {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "eo_camera_link";
        msg->height = 1080;
        msg->width = 1920;
        msg->encoding = "rgb8";
        msg->is_bigendian = false;
        msg->step = msg->width * 3;
        msg->data.resize(msg->height * msg->step);

        // Simulate a simple gradient image
        for (size_t i = 0; i < msg->data.size(); ++i) {
            msg->data[i] = (i % 256);
        }

        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EOCameraNode>());
    rclcpp::shutdown();
    return 0;
}
