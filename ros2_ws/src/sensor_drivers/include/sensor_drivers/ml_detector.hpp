#ifndef ML_DETECTOR_HPP
#define ML_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <memory>

namespace sensor_drivers {

struct DetectionResult {
    int class_id;
    std::string class_name;
    float confidence;
    cv::Rect bounding_box;
    cv::Point center;
    double distance_estimate; // Estimated distance in meters
    std::string threat_level; // "LOW", "MEDIUM", "HIGH", "CRITICAL"
};

struct MLModelConfig {
    std::string model_path;
    std::string config_path;
    std::string classes_path;
    std::string framework; // "tensorflow", "caffe", "onnx", "darknet"
    cv::Size input_size;
    cv::Scalar mean;
    double scale;
    bool swap_rb;
    float confidence_threshold;
    float nms_threshold;
};

class MLObjectDetector {
public:
    MLObjectDetector();
    ~MLObjectDetector();

    bool initialize(const MLModelConfig& config);
    bool isInitialized() const;

    std::vector<DetectionResult> detect(const cv::Mat& frame);
    std::vector<DetectionResult> detectWithPreprocessing(const cv::Mat& frame);

    // Model management
    bool loadModel(const std::string& model_path, const std::string& config_path = "");
    bool loadClasses(const std::string& classes_path);
    bool setInputParams(cv::Size size, cv::Scalar mean = cv::Scalar(0, 0, 0),
                       double scale = 1.0, bool swap_rb = false);

    // Detection parameters
    void setConfidenceThreshold(float threshold);
    void setNMSThreshold(float threshold);
    void setMaxDetections(int max_detections);

    // Advanced features
    std::vector<DetectionResult> trackObjects(const cv::Mat& frame,
                                            const std::vector<DetectionResult>& previous_detections);
    std::vector<DetectionResult> filterByClass(const std::vector<DetectionResult>& detections,
                                             const std::vector<std::string>& target_classes);
    std::vector<DetectionResult> filterByConfidence(const std::vector<DetectionResult>& detections,
                                                   float min_confidence);

    // Threat assessment
    std::string assessThreatLevel(const DetectionResult& detection);
    double estimateDistance(const DetectionResult& detection, const cv::Mat& frame);

    // Performance monitoring
    double getInferenceTime() const;
    int getModelInputWidth() const;
    int getModelInputHeight() const;

private:
    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    MLModelConfig config_;

    bool initialized_;
    double last_inference_time_;
    int max_detections_;

    // Preprocessing methods
    cv::Mat preprocessImage(const cv::Mat& frame);
    void applyNMS(std::vector<DetectionResult>& detections);

    // Post-processing methods
    std::vector<DetectionResult> postprocessDetections(const std::vector<cv::Mat>& outputs,
                                                     const cv::Size& frame_size);
    cv::Rect scaleBoundingBox(const cv::Rect& box, const cv::Size& from_size, const cv::Size& to_size);

    // Utility methods
    std::string getClassName(int class_id) const;
    bool isValidDetection(const DetectionResult& detection) const;
};

// Specialized Thermal Object Detector
class ThermalObjectDetector : public MLObjectDetector {
public:
    ThermalObjectDetector();

    std::vector<DetectionResult> detectThermalSignatures(const cv::Mat& thermal_frame);
    std::vector<DetectionResult> detectHeatSources(const cv::Mat& thermal_frame);
    std::vector<DetectionResult> classifyThermalObjects(const cv::Mat& thermal_frame);

    // Thermal-specific methods
    double calculateHeatIntensity(const cv::Mat& thermal_frame, const cv::Rect& region);
    std::string classifyHeatSource(double intensity, double area);
    bool isAnomalousHeat(const cv::Mat& thermal_frame, const cv::Rect& region);

private:
    // Thermal image processing
    cv::Mat enhanceThermalImage(const cv::Mat& thermal_frame);
    std::vector<cv::Rect> findHotRegions(const cv::Mat& thermal_frame);
    double calculateRegionTemperature(const cv::Mat& thermal_frame, const cv::Rect& region);
};

// Multi-Sensor Fusion Detector
class MultiSensorFusionDetector {
public:
    MultiSensorFusionDetector();
    ~MultiSensorFusionDetector();

    bool initialize(const MLModelConfig& visual_config, const MLModelConfig& thermal_config);
    bool isInitialized() const;

    // Fusion detection methods
    std::vector<DetectionResult> detectFused(const cv::Mat& visual_frame, const cv::Mat& thermal_frame);
    std::vector<DetectionResult> detectWithSensorFusion(const cv::Mat& visual_frame,
                                                      const cv::Mat& thermal_frame,
                                                      const std::vector<double>& radar_data = {});

    // Fusion algorithms
    std::vector<DetectionResult> fuseDetections(const std::vector<DetectionResult>& visual_detections,
                                              const std::vector<DetectionResult>& thermal_detections);
    DetectionResult correlateDetection(const DetectionResult& visual_det,
                                     const std::vector<DetectionResult>& thermal_dets);

    // Confidence calculation
    float calculateFusionConfidence(const DetectionResult& visual_det,
                                  const DetectionResult& thermal_det);
    std::string determineFusionThreatLevel(const DetectionResult& fused_detection);

private:
    std::unique_ptr<MLObjectDetector> visual_detector_;
    std::unique_ptr<ThermalObjectDetector> thermal_detector_;

    bool initialized_;

    // Fusion parameters
    double spatial_threshold_; // Maximum distance for correlation (pixels)
    double temporal_threshold_; // Maximum time difference for correlation (seconds)
    float confidence_boost_; // Confidence boost for fused detections

    // Fusion methods
    cv::Point calculateCentroid(const cv::Rect& box);
    double calculateDistance(const cv::Point& p1, const cv::Point& p2);
    cv::Rect mergeBoundingBoxes(const cv::Rect& box1, const cv::Rect& box2);
};

// Factory functions
std::unique_ptr<MLObjectDetector> createYOLOv3Detector(const std::string& model_path,
                                                     const std::string& config_path,
                                                     const std::string& classes_path);

std::unique_ptr<MLObjectDetector> createMobileNetSSDDetector(const std::string& model_path,
                                                          const std::string& config_path,
                                                          const std::string& classes_path);

std::unique_ptr<MLObjectDetector> createTensorFlowDetector(const std::string& model_path,
                                                        const std::string& config_path = "");

// Utility functions
cv::Mat drawDetections(const cv::Mat& frame, const std::vector<DetectionResult>& detections);
cv::Mat drawThermalDetections(const cv::Mat& thermal_frame, const std::vector<DetectionResult>& detections);
void printDetectionInfo(const std::vector<DetectionResult>& detections);

} // namespace sensor_drivers

#endif // ML_DETECTOR_HPP