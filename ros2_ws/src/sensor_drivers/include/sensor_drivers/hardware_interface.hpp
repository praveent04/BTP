#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

namespace sensor_drivers {

enum class HardwareType {
    USB_CAMERA,
    SERIAL_SENSOR,
    GPIO_DEVICE,
    NETWORK_CAMERA
};

enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR,
    RECONNECTING
};

struct HardwareCapabilities {
    bool supports_resolution_change = false;
    bool supports_frame_rate_change = false;
    bool supports_exposure_control = false;
    bool supports_gain_control = false;
    bool supports_focus_control = false;
    bool supports_zoom_control = false;
    std::vector<std::string> supported_formats;
    cv::Size max_resolution;
    double max_frame_rate = 30.0;
};

struct HardwareDiagnostics {
    double temperature = 0.0;
    double voltage = 0.0;
    double current = 0.0;
    int error_count = 0;
    int frame_count = 0;
    double fps_measured = 0.0;
    std::string last_error;
    ConnectionStatus status = ConnectionStatus::DISCONNECTED;
};

class HardwareInterface {
public:
    HardwareInterface(HardwareType type);
    virtual ~HardwareInterface();

    // Connection management
    virtual bool connect(const std::string& device_path = "") = 0;
    virtual bool disconnect() = 0;
    virtual bool reconnect() = 0;
    virtual ConnectionStatus getConnectionStatus() const = 0;

    // Hardware control
    virtual bool setResolution(int width, int height) = 0;
    virtual bool setFrameRate(double fps) = 0;
    virtual bool setExposure(double exposure_ms) = 0;
    virtual bool setGain(double gain) = 0;

    // Data acquisition
    virtual bool captureFrame(cv::Mat& frame) = 0;
    virtual bool isFrameAvailable() = 0;

    // Diagnostics and monitoring
    virtual HardwareDiagnostics getDiagnostics() = 0;
    virtual HardwareCapabilities getCapabilities() = 0;
    virtual bool performSelfTest() = 0;

    // Configuration
    virtual bool loadConfiguration(const std::string& config_file) = 0;
    virtual bool saveConfiguration(const std::string& config_file) = 0;

    // Error handling
    virtual std::string getLastError() const = 0;
    virtual void clearError() = 0;

protected:
    HardwareType hardware_type_;
    ConnectionStatus connection_status_;
    HardwareCapabilities capabilities_;
    HardwareDiagnostics diagnostics_;
    std::string device_path_;
    std::string last_error_;

    void setLastError(const std::string& error);
    void updateDiagnostics();
};

// USB Camera Interface for real hardware testing
class USBCameraInterface : public HardwareInterface {
public:
    USBCameraInterface();
    ~USBCameraInterface() override;

    bool connect(const std::string& device_path = "") override;
    bool disconnect() override;
    bool reconnect() override;
    ConnectionStatus getConnectionStatus() const override;

    bool setResolution(int width, int height) override;
    bool setFrameRate(double fps) override;
    bool setExposure(double exposure_ms) override;
    bool setGain(double gain) override;

    bool captureFrame(cv::Mat& frame) override;
    bool isFrameAvailable() override;

    HardwareDiagnostics getDiagnostics() override;
    HardwareCapabilities getCapabilities() override;
    bool performSelfTest() override;

    bool loadConfiguration(const std::string& config_file) override;
    bool saveConfiguration(const std::string& config_file) override;

    std::string getLastError() const override;
    void clearError() override;

private:
    cv::VideoCapture capture_;
    int camera_index_;
    bool initialized_;

    bool initializeCamera();
    bool configureCamera();
    void detectCapabilities();
};

// Serial Sensor Interface for sensor control
class SerialSensorInterface : public HardwareInterface {
public:
    SerialSensorInterface();
    ~SerialSensorInterface() override;

    bool connect(const std::string& device_path = "") override;
    bool disconnect() override;
    bool reconnect() override;
    ConnectionStatus getConnectionStatus() const override;

    bool setResolution(int width, int height) override;
    bool setFrameRate(double fps) override;
    bool setExposure(double exposure_ms) override;
    bool setGain(double gain) override;

    bool captureFrame(cv::Mat& frame) override;
    bool isFrameAvailable() override;

    HardwareDiagnostics getDiagnostics() override;
    HardwareCapabilities getCapabilities() override;
    bool performSelfTest() override;

    bool loadConfiguration(const std::string& config_file) override;
    bool saveConfiguration(const std::string& config_file) override;

    std::string getLastError() const override;
    void clearError() override;

    // Serial-specific methods
    bool sendCommand(const std::string& command);
    bool readResponse(std::string& response, int timeout_ms = 1000);
    bool configureSensor(const std::string& config);

private:
    int serial_fd_;
    std::string port_name_;
    int baud_rate_;
    bool connected_;

    bool openSerialPort();
    bool closeSerialPort();
    bool configurePort();
};

// GPIO Interface for Raspberry Pi hardware control
class GPIOInterface : public HardwareInterface {
public:
    GPIOInterface();
    ~GPIOInterface() override;

    bool connect(const std::string& device_path = "") override;
    bool disconnect() override;
    bool reconnect() override;
    ConnectionStatus getConnectionStatus() const override;

    bool setResolution(int width, int height) override;
    bool setFrameRate(double fps) override;
    bool setExposure(double exposure_ms) override;
    bool setGain(double gain) override;

    bool captureFrame(cv::Mat& frame) override;
    bool isFrameAvailable() override;

    HardwareDiagnostics getDiagnostics() override;
    HardwareCapabilities getCapabilities() override;
    bool performSelfTest() override;

    bool loadConfiguration(const std::string& config_file) override;
    bool saveConfiguration(const std::string& config_file) override;

    std::string getLastError() const override;
    void clearError() override;

    // GPIO-specific methods
    bool setPinMode(int pin, const std::string& mode);
    bool digitalWrite(int pin, bool value);
    bool digitalRead(int pin, bool& value);
    bool analogWrite(int pin, int value);
    bool analogRead(int pin, int& value);
    bool pwmWrite(int pin, double duty_cycle, double frequency);

private:
    std::vector<int> gpio_pins_;
    std::string gpio_chip_;
    bool initialized_;

    bool exportGPIO(int pin);
    bool unexportGPIO(int pin);
    bool initializeGPIOLibrary();
};

// Factory function to create hardware interfaces
std::unique_ptr<HardwareInterface> createHardwareInterface(HardwareType type);

} // namespace sensor_drivers

#endif // HARDWARE_INTERFACE_HPP