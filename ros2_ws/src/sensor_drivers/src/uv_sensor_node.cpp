#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/illuminance.hpp"

class UVSensorNode : public rclcpp::Node {
public:
    UVSensorNode() : Node("uv_sensor_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Illuminance>("sensors/uv_intensity", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), // 5 Hz
            std::bind(&UVSensorNode::publish_intensity, this));
        
        RCLCPP_INFO(this->get_logger(), "UV Sensor Node started. Publishing to sensors/uv_intensity.");
    }

private:
    void publish_intensity() {
        auto msg = std::make_unique<sensor_msgs::msg::Illuminance>();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "uv_sensor_link";
        
        // Simulate high UV intensity from a plume
        msg->illuminance = 1500.0; // Arbitrary high value for UV intensity
        msg->variance = 100.0;

        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UVSensorNode>());
    rclcpp::shutdown();
    return 0;
}
