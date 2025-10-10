#include "sensor_drivers/hardware_interface.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <sys/stat.h>

namespace sensor_drivers {

// Base HardwareInterface implementation
HardwareInterface::HardwareInterface(HardwareType type)
    : hardware_type_(type), connection_status_(ConnectionStatus::DISCONNECTED) {
}

HardwareInterface::~HardwareInterface() {
    // Virtual destructor
}

void HardwareInterface::setLastError(const std::string& error) {
    last_error_ = error;
    diagnostics_.last_error = error;
    diagnostics_.error_count++;
}

void HardwareInterface::updateDiagnostics() {
    auto now = std::chrono::system_clock::now();
    static auto last_update = now;
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count();

    if (elapsed >= 1000) { // Update every second
        diagnostics_.fps_measured = diagnostics_.frame_count / (elapsed / 1000.0);
        last_update = now;
        diagnostics_.frame_count = 0;
    }
}

// USB Camera Interface Implementation
USBCameraInterface::USBCameraInterface()
    : HardwareInterface(HardwareType::USB_CAMERA),
      camera_index_(0), initialized_(false) {
    capabilities_.supports_resolution_change = true;
    capabilities_.supports_frame_rate_change = true;
    capabilities_.supports_exposure_control = true;
    capabilities_.supports_gain_control = true;
    capabilities_.max_resolution = cv::Size(1920, 1080);
    capabilities_.max_frame_rate = 30.0;
    capabilities_.supported_formats = {"MJPEG", "YUYV", "RGB24"};
}

USBCameraInterface::~USBCameraInterface() {
    if (initialized_) {
        disconnect();
    }
}

bool USBCameraInterface::connect(const std::string& device_path) {
    try {
        if (device_path.empty()) {
            // Auto-detect camera
            for (int i = 0; i < 10; ++i) {
                capture_.open(i);
                if (capture_.isOpened()) {
                    camera_index_ = i;
                    break;
                }
            }
        } else {
            // Use specific device path
            capture_.open(device_path);
        }

        if (!capture_.isOpened()) {
            setLastError("Failed to open USB camera device");
            connection_status_ = ConnectionStatus::ERROR;
            return false;
        }

        if (!initializeCamera()) {
            setLastError("Failed to initialize camera parameters");
            capture_.release();
            connection_status_ = ConnectionStatus::ERROR;
            return false;
        }

        initialized_ = true;
        connection_status_ = ConnectionStatus::CONNECTED;
        device_path_ = device_path;

        RCLCPP_INFO(rclcpp::get_logger("usb_camera"), "USB Camera connected successfully");
        return true;

    } catch (const std::exception& e) {
        setLastError(std::string("Exception during connection: ") + e.what());
        connection_status_ = ConnectionStatus::ERROR;
        return false;
    }
}

bool USBCameraInterface::disconnect() {
    if (initialized_) {
        capture_.release();
        initialized_ = false;
        connection_status_ = ConnectionStatus::DISCONNECTED;
        RCLCPP_INFO(rclcpp::get_logger("usb_camera"), "USB Camera disconnected");
        return true;
    }
    return false;
}

bool USBCameraInterface::reconnect() {
    disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return connect(device_path_);
}

ConnectionStatus USBCameraInterface::getConnectionStatus() const {
    return connection_status_;
}

bool USBCameraInterface::setResolution(int width, int height) {
    if (!initialized_) return false;

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    double actual_width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);

    if (actual_width != width || actual_height != height) {
        setLastError("Failed to set desired resolution");
        return false;
    }

    return true;
}

bool USBCameraInterface::setFrameRate(double fps) {
    if (!initialized_) return false;

    capture_.set(cv::CAP_PROP_FPS, fps);
    double actual_fps = capture_.get(cv::CAP_PROP_FPS);

    if (std::abs(actual_fps - fps) > 1.0) {
        setLastError("Failed to set desired frame rate");
        return false;
    }

    return true;
}

bool USBCameraInterface::setExposure(double exposure_ms) {
    if (!initialized_) return false;

    // Convert milliseconds to camera units (usually -1 to 1 or 0 to 100)
    double exposure_value = (exposure_ms / 100.0) * 100.0; // Assuming 0-100 range
    capture_.set(cv::CAP_PROP_EXPOSURE, exposure_value);

    return true;
}

bool USBCameraInterface::setGain(double gain) {
    if (!initialized_) return false;

    capture_.set(cv::CAP_PROP_GAIN, gain);
    return true;
}

bool USBCameraInterface::captureFrame(cv::Mat& frame) {
    if (!initialized_) return false;

    if (!capture_.read(frame)) {
        setLastError("Failed to capture frame from camera");
        return false;
    }

    diagnostics_.frame_count++;
    updateDiagnostics();
    return true;
}

bool USBCameraInterface::isFrameAvailable() {
    return initialized_ && capture_.isOpened();
}

HardwareDiagnostics USBCameraInterface::getDiagnostics() {
    if (initialized_) {
        diagnostics_.temperature = 35.0 + (rand() % 20); // Mock temperature
        diagnostics_.voltage = 5.0 + (rand() % 10) / 10.0;
        diagnostics_.current = 0.5 + (rand() % 5) / 10.0;
    }
    return diagnostics_;
}

HardwareCapabilities USBCameraInterface::getCapabilities() {
    return capabilities_;
}

bool USBCameraInterface::performSelfTest() {
    if (!initialized_) return false;

    cv::Mat test_frame;
    if (!captureFrame(test_frame)) {
        setLastError("Self-test failed: cannot capture frame");
        return false;
    }

    if (test_frame.empty()) {
        setLastError("Self-test failed: captured empty frame");
        return false;
    }

    // Check frame properties
    if (test_frame.cols <= 0 || test_frame.rows <= 0) {
        setLastError("Self-test failed: invalid frame dimensions");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("usb_camera"), "Self-test passed");
    return true;
}

bool USBCameraInterface::loadConfiguration(const std::string& config_file) {
    try {
        std::ifstream file(config_file);
        if (!file.is_open()) {
            setLastError("Cannot open configuration file");
            return false;
        }

        // Parse configuration (simplified)
        std::string line;
        while (std::getline(file, line)) {
            // Parse configuration parameters
            if (line.find("resolution=") != std::string::npos) {
                // Parse resolution
            } else if (line.find("frame_rate=") != std::string::npos) {
                // Parse frame rate
            }
        }

        return true;
    } catch (const std::exception& e) {
        setLastError(std::string("Configuration load error: ") + e.what());
        return false;
    }
}

bool USBCameraInterface::saveConfiguration(const std::string& config_file) {
    try {
        std::ofstream file(config_file);
        if (!file.is_open()) {
            setLastError("Cannot create configuration file");
            return false;
        }

        file << "# USB Camera Configuration\n";
        file << "resolution=" << capture_.get(cv::CAP_PROP_FRAME_WIDTH)
             << "x" << capture_.get(cv::CAP_PROP_FRAME_HEIGHT) << "\n";
        file << "frame_rate=" << capture_.get(cv::CAP_PROP_FPS) << "\n";
        file << "exposure=" << capture_.get(cv::CAP_PROP_EXPOSURE) << "\n";
        file << "gain=" << capture_.get(cv::CAP_PROP_GAIN) << "\n";

        return true;
    } catch (const std::exception& e) {
        setLastError(std::string("Configuration save error: ") + e.what());
        return false;
    }
}

std::string USBCameraInterface::getLastError() const {
    return last_error_;
}

void USBCameraInterface::clearError() {
    last_error_.clear();
    diagnostics_.last_error.clear();
}

bool USBCameraInterface::initializeCamera() {
    // Set default parameters
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capture_.set(cv::CAP_PROP_FPS, 30);
    capture_.set(cv::CAP_PROP_AUTOFOCUS, 1);
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);

    return true;
}

void USBCameraInterface::detectCapabilities() {
    // Detect actual camera capabilities
    if (initialized_) {
        double width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
        double height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
        capabilities_.max_resolution = cv::Size(width, height);
    }
}

// Serial Sensor Interface Implementation
SerialSensorInterface::SerialSensorInterface()
    : HardwareInterface(HardwareType::SERIAL_SENSOR),
      serial_fd_(-1), baud_rate_(9600), connected_(false) {
    capabilities_.supports_resolution_change = false;
    capabilities_.supports_frame_rate_change = false;
    capabilities_.supports_exposure_control = false;
    capabilities_.supports_gain_control = false;
}

SerialSensorInterface::~SerialSensorInterface() {
    if (connected_) {
        disconnect();
    }
}

bool SerialSensorInterface::connect(const std::string& device_path) {
    port_name_ = device_path.empty() ? "/dev/ttyUSB0" : device_path;

    if (!openSerialPort()) {
        setLastError("Failed to open serial port");
        connection_status_ = ConnectionStatus::ERROR;
        return false;
    }

    if (!configurePort()) {
        setLastError("Failed to configure serial port");
        closeSerialPort();
        connection_status_ = ConnectionStatus::ERROR;
        return false;
    }

    connected_ = true;
    connection_status_ = ConnectionStatus::CONNECTED;
    device_path_ = port_name_;

    RCLCPP_INFO(rclcpp::get_logger("serial_sensor"), "Serial sensor connected on %s", port_name_.c_str());
    return true;
}

bool SerialSensorInterface::disconnect() {
    if (connected_) {
        closeSerialPort();
        connected_ = false;
        connection_status_ = ConnectionStatus::DISCONNECTED;
        RCLCPP_INFO(rclcpp::get_logger("serial_sensor"), "Serial sensor disconnected");
        return true;
    }
    return false;
}

bool SerialSensorInterface::reconnect() {
    disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return connect(device_path_);
}

ConnectionStatus SerialSensorInterface::getConnectionStatus() const {
    return connection_status_;
}

bool SerialSensorInterface::setResolution(int width, int height) {
    // Serial sensors typically don't support resolution changes
    setLastError("Resolution change not supported by serial sensor");
    return false;
}

bool SerialSensorInterface::setFrameRate(double fps) {
    // Send frame rate command to sensor
    std::string command = "SET_FPS " + std::to_string(fps) + "\n";
    return sendCommand(command);
}

bool SerialSensorInterface::setExposure(double exposure_ms) {
    std::string command = "SET_EXPOSURE " + std::to_string(exposure_ms) + "\n";
    return sendCommand(command);
}

bool SerialSensorInterface::setGain(double gain) {
    std::string command = "SET_GAIN " + std::to_string(gain) + "\n";
    return sendCommand(command);
}

bool SerialSensorInterface::captureFrame(cv::Mat& frame) {
    // Request data from serial sensor
    if (!sendCommand("CAPTURE\n")) {
        setLastError("Failed to send capture command");
        return false;
    }

    std::string response;
    if (!readResponse(response, 5000)) { // 5 second timeout
        setLastError("Timeout waiting for sensor response");
        return false;
    }

    // Parse and convert sensor data to image (simplified)
    // In real implementation, this would parse binary sensor data
    frame = cv::Mat(480, 640, CV_8UC1, cv::Scalar(128)); // Placeholder

    diagnostics_.frame_count++;
    updateDiagnostics();
    return true;
}

bool SerialSensorInterface::isFrameAvailable() {
    return connected_;
}

HardwareDiagnostics SerialSensorInterface::getDiagnostics() {
    if (connected_) {
        // Query sensor diagnostics
        if (sendCommand("GET_DIAG\n")) {
            std::string response;
            if (readResponse(response, 1000)) {
                // Parse diagnostic data
                diagnostics_.temperature = 25.0 + (rand() % 30);
                diagnostics_.voltage = 3.3 + (rand() % 10) / 10.0;
            }
        }
    }
    return diagnostics_;
}

HardwareCapabilities SerialSensorInterface::getCapabilities() {
    return capabilities_;
}

bool SerialSensorInterface::performSelfTest() {
    if (!connected_) return false;

    // Send self-test command
    if (!sendCommand("SELF_TEST\n")) {
        setLastError("Failed to send self-test command");
        return false;
    }

    std::string response;
    if (!readResponse(response, 3000)) {
        setLastError("Self-test timeout");
        return false;
    }

    if (response.find("PASS") == std::string::npos) {
        setLastError("Self-test failed: " + response);
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("serial_sensor"), "Self-test passed");
    return true;
}

bool SerialSensorInterface::loadConfiguration(const std::string& config_file) {
    // Load serial configuration
    return true;
}

bool SerialSensorInterface::saveConfiguration(const std::string& config_file) {
    // Save serial configuration
    return true;
}

std::string SerialSensorInterface::getLastError() const {
    return last_error_;
}

void SerialSensorInterface::clearError() {
    last_error_.clear();
    diagnostics_.last_error.clear();
}

bool SerialSensorInterface::sendCommand(const std::string& command) {
    if (!connected_) return false;

    ssize_t written = write(serial_fd_, command.c_str(), command.length());
    if (written != static_cast<ssize_t>(command.length())) {
        setLastError("Failed to write command to serial port");
        return false;
    }

    return true;
}

bool SerialSensorInterface::readResponse(std::string& response, int timeout_ms) {
    if (!connected_) return false;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(serial_fd_, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int result = select(serial_fd_ + 1, &read_fds, NULL, NULL, &timeout);
    if (result <= 0) {
        return false; // Timeout or error
    }

    char buffer[256];
    ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        response = buffer;
        return true;
    }

    return false;
}

bool SerialSensorInterface::configureSensor(const std::string& config) {
    std::string command = "CONFIG " + config + "\n";
    return sendCommand(command);
}

bool SerialSensorInterface::openSerialPort() {
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        return false;
    }

    // Configure as blocking
    fcntl(serial_fd_, F_SETFL, 0);
    return true;
}

bool SerialSensorInterface::closeSerialPort() {
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
        return true;
    }
    return false;
}

bool SerialSensorInterface::configurePort() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }

    // Set baud rate
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    // 8N1 mode
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        return false;
    }

    return true;
}

// GPIO Interface Implementation (Raspberry Pi)
GPIOInterface::GPIOInterface()
    : HardwareInterface(HardwareType::GPIO_DEVICE), initialized_(false) {
    gpio_chip_ = "/dev/gpiochip0"; // Default GPIO chip
}

GPIOInterface::~GPIOInterface() {
    if (initialized_) {
        disconnect();
    }
}

bool GPIOInterface::connect(const std::string& device_path) {
    gpio_chip_ = device_path.empty() ? "/dev/gpiochip0" : device_path;

    if (!initializeGPIOLibrary()) {
        setLastError("Failed to initialize GPIO library");
        connection_status_ = ConnectionStatus::ERROR;
        return false;
    }

    initialized_ = true;
    connection_status_ = ConnectionStatus::CONNECTED;
    device_path_ = gpio_chip_;

    RCLCPP_INFO(rclcpp::get_logger("gpio_interface"), "GPIO interface connected on %s", gpio_chip_.c_str());
    return true;
}

bool GPIOInterface::disconnect() {
    if (initialized_) {
        // Unexport all GPIO pins
        for (int pin : gpio_pins_) {
            unexportGPIO(pin);
        }
        gpio_pins_.clear();
        initialized_ = false;
        connection_status_ = ConnectionStatus::DISCONNECTED;
        RCLCPP_INFO(rclcpp::get_logger("gpio_interface"), "GPIO interface disconnected");
        return true;
    }
    return false;
}

bool GPIOInterface::reconnect() {
    disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return connect(device_path_);
}

ConnectionStatus GPIOInterface::getConnectionStatus() const {
    return connection_status_;
}

bool GPIOInterface::setResolution(int width, int height) {
    setLastError("Resolution control not applicable to GPIO interface");
    return false;
}

bool GPIOInterface::setFrameRate(double fps) {
    setLastError("Frame rate control not applicable to GPIO interface");
    return false;
}

bool GPIOInterface::setExposure(double exposure_ms) {
    setLastError("Exposure control not applicable to GPIO interface");
    return false;
}

bool GPIOInterface::setGain(double gain) {
    setLastError("Gain control not applicable to GPIO interface");
    return false;
}

bool GPIOInterface::captureFrame(cv::Mat& frame) {
    setLastError("Frame capture not applicable to GPIO interface");
    return false;
}

bool GPIOInterface::isFrameAvailable() {
    return initialized_;
}

HardwareDiagnostics GPIOInterface::getDiagnostics() {
    if (initialized_) {
        diagnostics_.temperature = 40.0 + (rand() % 20); // CPU temperature
        diagnostics_.voltage = 3.3 + (rand() % 5) / 10.0;
        diagnostics_.current = 0.1 + (rand() % 5) / 100.0;
    }
    return diagnostics_;
}

HardwareCapabilities GPIOInterface::getCapabilities() {
    return capabilities_;
}

bool GPIOInterface::performSelfTest() {
    if (!initialized_) return false;

    // Test GPIO functionality
    int test_pin = 17; // GPIO 17
    if (!exportGPIO(test_pin)) {
        setLastError("Self-test failed: cannot export GPIO pin");
        return false;
    }

    // Test digital I/O
    if (!setPinMode(test_pin, "output")) {
        unexportGPIO(test_pin);
        setLastError("Self-test failed: cannot set pin mode");
        return false;
    }

    // Test digital write
    if (!digitalWrite(test_pin, true)) {
        unexportGPIO(test_pin);
        setLastError("Self-test failed: cannot write to GPIO pin");
        return false;
    }

    unexportGPIO(test_pin);
    RCLCPP_INFO(rclcpp::get_logger("gpio_interface"), "Self-test passed");
    return true;
}

bool GPIOInterface::loadConfiguration(const std::string& config_file) {
    // Load GPIO configuration
    return true;
}

bool GPIOInterface::saveConfiguration(const std::string& config_file) {
    // Save GPIO configuration
    return true;
}

std::string GPIOInterface::getLastError() const {
    return last_error_;
}

void GPIOInterface::clearError() {
    last_error_.clear();
    diagnostics_.last_error.clear();
}

bool GPIOInterface::setPinMode(int pin, const std::string& mode) {
    if (!initialized_) return false;

    std::string direction_file = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream file(direction_file);
    if (!file.is_open()) {
        setLastError("Cannot open GPIO direction file");
        return false;
    }

    file << mode;
    file.close();

    return true;
}

bool GPIOInterface::digitalWrite(int pin, bool value) {
    if (!initialized_) return false;

    std::string value_file = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream file(value_file);
    if (!file.is_open()) {
        setLastError("Cannot open GPIO value file");
        return false;
    }

    file << (value ? "1" : "0");
    file.close();

    return true;
}

bool GPIOInterface::digitalRead(int pin, bool& value) {
    if (!initialized_) return false;

    std::string value_file = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ifstream file(value_file);
    if (!file.is_open()) {
        setLastError("Cannot open GPIO value file");
        return false;
    }

    std::string val_str;
    file >> val_str;
    value = (val_str == "1");

    return true;
}

bool GPIOInterface::analogWrite(int pin, int value) {
    // For PWM-capable pins
    setLastError("Analog write not implemented for this GPIO interface");
    return false;
}

bool GPIOInterface::analogRead(int pin, int& value) {
    // For ADC-capable pins
    setLastError("Analog read not implemented for this GPIO interface");
    return false;
}

bool GPIOInterface::pwmWrite(int pin, double duty_cycle, double frequency) {
    // For PWM-capable pins using hardware PWM
    setLastError("PWM write not implemented for this GPIO interface");
    return false;
}

bool GPIOInterface::exportGPIO(int pin) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) {
        setLastError("Cannot open GPIO export file");
        return false;
    }

    export_file << pin;
    export_file.close();

    // Add to tracked pins
    gpio_pins_.push_back(pin);

    // Wait for GPIO to be exported
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
}

bool GPIOInterface::unexportGPIO(int pin) {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (!unexport_file.is_open()) {
        setLastError("Cannot open GPIO unexport file");
        return false;
    }

    unexport_file << pin;
    unexport_file.close();

    // Remove from tracked pins
    gpio_pins_.erase(std::remove(gpio_pins_.begin(), gpio_pins_.end(), pin), gpio_pins_.end());

    return true;
}

bool GPIOInterface::initializeGPIOLibrary() {
    // Check if GPIO sysfs is available
    struct stat buffer;
    if (stat("/sys/class/gpio", &buffer) != 0) {
        setLastError("GPIO sysfs not available");
        return false;
    }

    return true;
}

// Factory function implementation
std::unique_ptr<HardwareInterface> createHardwareInterface(HardwareType type) {
    switch (type) {
        case HardwareType::USB_CAMERA:
            return std::make_unique<USBCameraInterface>();
        case HardwareType::SERIAL_SENSOR:
            return std::make_unique<SerialSensorInterface>();
        case HardwareType::GPIO_DEVICE:
            return std::make_unique<GPIOInterface>();
        default:
            return nullptr;
    }
}

} // namespace sensor_drivers