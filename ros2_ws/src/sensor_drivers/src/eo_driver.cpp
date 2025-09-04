#include "sensor_drivers/eo_driver.hpp"
#include <iostream>
#include <random>
#include <chrono>

namespace sensor_drivers {

EOOpticalDriver::EOOpticalDriver()
    : initialized_(false), width_(1920), height_(1080), fps_(30.0),
      exposure_(10.0), gain_(1.0), device_path_("/dev/eo_camera") {
}

EOOpticalDriver::~EOOpticalDriver() {
    if (initialized_) {
        disconnectFromHardware();
    }
}

bool EOOpticalDriver::initialize(const std::string& device_path) {
    device_path_ = device_path;
    if (connectToHardware()) {
        initialized_ = true;
        std::cout << "EO Optical Driver initialized successfully on " << device_path_ << std::endl;
        return true;
    }
    std::cout << "Failed to initialize EO Optical Driver on " << device_path_ << std::endl;
    return false;
}

bool EOOpticalDriver::isInitialized() const {
    return initialized_;
}

bool EOOpticalDriver::captureFrame(cv::Mat& frame) {
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

void EOOpticalDriver::setResolution(int width, int height) {
    width_ = width;
    height_ = height;
}

void EOOpticalDriver::setFrameRate(double fps) {
    fps_ = fps;
}

void EOOpticalDriver::setExposure(double exposure_ms) {
    exposure_ = exposure_ms;
}

void EOOpticalDriver::setGain(double gain) {
    gain_ = gain;
}

bool EOOpticalDriver::connectToHardware() {
    // Mock hardware connection
    std::cout << "Connecting to EO optical hardware..." << std::endl;
    return true;
}

void EOOpticalDriver::disconnectFromHardware() {
    // Mock disconnection
    std::cout << "Disconnecting from EO optical hardware..." << std::endl;
}

bool EOOpticalDriver::readRawData(std::vector<uint8_t>& data) {
    // Mock raw data reading for RGB image
    int channels = 3;
    data.resize(width_ * height_ * channels);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (auto& val : data) {
        val = static_cast<uint8_t>(dis(gen));
    }

    // Simulate some objects in the scene
    int num_objects = 5;
    for (int i = 0; i < num_objects; ++i) {
        int x = dis(gen) % width_;
        int y = dis(gen) % height_;
        int obj_width = 50 + dis(gen) % 100;
        int obj_height = 50 + dis(gen) % 100;

        for (int dy = 0; dy < obj_height; ++dy) {
            for (int dx = 0; dx < obj_width; ++dx) {
                int nx = x + dx;
                int ny = y + dy;
                if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                    int idx = (ny * width_ + nx) * channels;
                    data[idx] = 100 + dis(gen) % 100;     // R
                    data[idx + 1] = 100 + dis(gen) % 100; // G
                    data[idx + 2] = 100 + dis(gen) % 100; // B
                }
            }
        }
    }

    return true;
}

cv::Mat EOOpticalDriver::processRawData(const std::vector<uint8_t>& data) {
    cv::Mat frame(height_, width_, CV_8UC3);
    std::memcpy(frame.data, data.data(), data.size());
    return frame;
}

} // namespace sensor_drivers