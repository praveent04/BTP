#ifndef IR_DRIVER_HPP
#define IR_DRIVER_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace sensor_drivers {

class IRDriver {
public:
    IRDriver();
    ~IRDriver();

    bool initialize(const std::string& device_path = "/dev/ir_sensor");
    bool isInitialized() const;
    bool captureFrame(cv::Mat& frame);
    void setResolution(int width, int height);
    void setFrameRate(double fps);

private:
    bool initialized_;
    int width_;
    int height_;
    double fps_;
    std::string device_path_;

    // Hardware-specific methods (mocked for now)
    bool connectToHardware();
    void disconnectFromHardware();
    bool readRawData(std::vector<uint8_t>& data);
    cv::Mat processRawData(const std::vector<uint8_t>& data);
};

} // namespace sensor_drivers

#endif // IR_DRIVER_HPP