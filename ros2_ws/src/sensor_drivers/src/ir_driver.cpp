#include "sensor_drivers/ir_driver.hpp"
#include <iostream>
#include <random>
#include <chrono>

namespace sensor_drivers {

IRDriver::IRDriver()
    : initialized_(false), width_(640), height_(480), fps_(10.0), device_path_("/dev/ir_sensor") {
}

IRDriver::~IRDriver() {
    if (initialized_) {
        disconnectFromHardware();
    }
}

bool IRDriver::initialize(const std::string& device_path) {
    device_path_ = device_path;
    if (connectToHardware()) {
        initialized_ = true;
        std::cout << "IR Driver initialized successfully on " << device_path_ << std::endl;
        return true;
    }
    std::cout << "Failed to initialize IR Driver on " << device_path_ << std::endl;
    return false;
}

bool IRDriver::isInitialized() const {
    return initialized_;
}

bool IRDriver::captureFrame(cv::Mat& frame) {
    if (!initialized_) {
        return false;
    }

    std::vector<uint8_t> raw_data;
    if (!readRawData(raw_data)) {
        return false;
    }

    frame = processRawData(raw_data);
    return true;
}

void IRDriver::setResolution(int width, int height) {
    width_ = width;
    height_ = height;
}

void IRDriver::setFrameRate(double fps) {
    fps_ = fps;
}

bool IRDriver::connectToHardware() {
    // Mock hardware connection
    // In real implementation, this would open device file or establish connection
    std::cout << "Connecting to IR hardware..." << std::endl;
    return true; // Assume success for now
}

void IRDriver::disconnectFromHardware() {
    // Mock disconnection
    std::cout << "Disconnecting from IR hardware..." << std::endl;
}

bool IRDriver::readRawData(std::vector<uint8_t>& data) {
    // Mock raw data reading
    // In real implementation, this would read from hardware buffer
    data.resize(width_ * height_);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (auto& val : data) {
        val = static_cast<uint8_t>(dis(gen));
    }

    // Simulate heat signatures by adding hot spots
    int num_hotspots = 3;
    for (int i = 0; i < num_hotspots; ++i) {
        int x = dis(gen) % width_;
        int y = dis(gen) % height_;
        int radius = 5 + dis(gen) % 10;

        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                    double distance = std::sqrt(dx*dx + dy*dy);
                    if (distance <= radius) {
                        int intensity = 200 + dis(gen) % 55; // Hot temperature
                        data[ny * width_ + nx] = static_cast<uint8_t>(intensity);
                    }
                }
            }
        }
    }

    return true;
}

cv::Mat IRDriver::processRawData(const std::vector<uint8_t>& data) {
    cv::Mat frame(height_, width_, CV_8UC1);
    std::memcpy(frame.data, data.data(), data.size());
    return frame;
}

} // namespace sensor_drivers