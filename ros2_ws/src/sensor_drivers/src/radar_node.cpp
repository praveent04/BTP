#include "rclcpp/rclcpp.hpp"
#include "threat_detection_interfaces/msg/radar_scan.hpp"

class RadarNode : public rclcpp::Node {
public:
    RadarNode() : Node("radar_node") {
        publisher_ = this->create_publisher<threat_detection_interfaces::msg::RadarScan>("sensors/radar_scan", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&RadarNode::publish_scan, this));
        
        RCLCPP_INFO(this->get_logger(), "Radar Node started. Publishing to sensors/radar_scan.");
    }

private:
    void publish_scan() {
        auto msg = std::make_unique<threat_detection_interfaces::msg::RadarScan>();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "radar_link";
        
        // Simulate a high-speed target
        msg->target_id = 1;
        msg->range = 5000.0; // 5 km
        msg->azimuth = 0.1;  // radians
        msg->elevation = 0.5; // radians
        msg->doppler_velocity = 800.0; // 800 m/s

        publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<threat_detection_interfaces::msg::RadarScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarNode>());
    rclcpp::shutdown();
    return 0;
}
