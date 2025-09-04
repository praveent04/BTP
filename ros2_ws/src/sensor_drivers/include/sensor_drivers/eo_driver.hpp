#ifndef EO_DRIVER_HPP
#define EO_DRIVER_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

namespace sensor_drivers {

class EOOpticalDriver {
public:
    EOOpticalDriver();
    ~EOOpticalDriver();

    bool initialize(const std::string& device_path = "/dev/eo_camera");
    bool isInitialized() const;
    bool captureFrame(cv::Mat& frame);
    void setResolution(int width, int height);
    void setFrameRate(double fps);
    void setExposure(double exposure_ms);
    void setGain(double gain);

private:
    bool initialized_;
    int width_;
    int height_;
    double fps_;
    double exposure_;
    double gain_;
    std::string device_path_;

    // Hardware-specific methods (mocked for now)
    bool connectToHardware();
    void disconnectFromHardware();
    bool readRawData(std::vector<uint8_t>& data);
    cv::Mat processRawData(const std::vector<uint8_t>& data);
};

} // namespace sensor_drivers

#endif // EO_DRIVER_HPP