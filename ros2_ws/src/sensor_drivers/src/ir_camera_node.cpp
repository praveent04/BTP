#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class IRCameraNode : public rclcpp::Node {
public:
    IRCameraNode() : Node("ir_camera_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("sensors/ir_image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&IRCameraNode::publish_image, this));
        
        RCLCPP_INFO(this->get_logger(), "IR Camera Node started. Publishing to sensors/ir_image.");
    }

private:
    void publish_image() {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "ir_camera_link";
        msg->height = 480;
        msg->width = 640;
        msg->encoding = "mono8";
        msg->is_bigendian = false;
        msg->step = msg->width;
        msg->data.resize(msg->height * msg->width);

        // Simulate a plain background with a hot spot
        std::fill(msg->data.begin(), msg->data.end(), 50); // Cool background
        
        // Add a bright "hot spot" to simulate a launch signature
        int center_x = 320;
        int center_y = 240;
        int radius = 10;
        for (int y = center_y - radius; y < center_y + radius; ++y) {
            for (int x = center_x - radius; x < center_x + radius; ++x) {
                if (y >= 0 && y < (int)msg->height && x >= 0 && x < (int)msg->width) {
                    msg->data[y * msg->width + x] = 255; // Hot spot
                }
            }
        }

        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRCameraNode>());
    rclcpp::shutdown();
    return 0;
}
