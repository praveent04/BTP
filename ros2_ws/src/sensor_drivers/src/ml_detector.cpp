#include "sensor_drivers/ml_detector.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <random>

namespace sensor_drivers {

// MLObjectDetector Implementation
MLObjectDetector::MLObjectDetector()
    : initialized_(false), last_inference_time_(0.0), max_detections_(100) {
}

MLObjectDetector::~MLObjectDetector() {
    // Cleanup resources
}

bool MLObjectDetector::initialize(const MLModelConfig& config) {
    config_ = config;

    try {
        // Load the neural network
        if (!loadModel(config.model_path, config.config_path)) {
            std::cerr << "Failed to load model: " << config.model_path << std::endl;
            return false;
        }

        // Load class names if provided
        if (!config.classes_path.empty()) {
            if (!loadClasses(config.classes_path)) {
                std::cerr << "Failed to load classes: " << config.classes_path << std::endl;
                return false;
            }
        }

        // Set input parameters
        setInputParams(config.input_size, config.mean, config.scale, config.swap_rb);

        initialized_ = true;
        std::cout << "ML Object Detector initialized successfully" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception during initialization: " << e.what() << std::endl;
        return false;
    }
}

bool MLObjectDetector::isInitialized() const {
    return initialized_;
}

std::vector<DetectionResult> MLObjectDetector::detect(const cv::Mat& frame) {
    if (!initialized_ || frame.empty()) {
        return {};
    }

    try {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Preprocess the image
        cv::Mat blob = preprocessImage(frame);

        // Set the input to the network
        net_.setInput(blob);

        // Forward pass
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        // Post-process detections
        std::vector<DetectionResult> detections = postprocessDetections(outputs, frame.size());

        // Apply NMS
        applyNMS(detections);

        // Limit detections
        if (detections.size() > max_detections_) {
            detections.resize(max_detections_);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        last_inference_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();

        return detections;

    } catch (const std::exception& e) {
        std::cerr << "Exception during detection: " << e.what() << std::endl;
        return {};
    }
}

std::vector<DetectionResult> MLObjectDetector::detectWithPreprocessing(const cv::Mat& frame) {
    if (frame.empty()) {
        return {};
    }

    // Apply additional preprocessing
    cv::Mat processed_frame = frame.clone();

    // Enhance contrast
    cv::Mat lab_frame;
    cv::cvtColor(processed_frame, lab_frame, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_channels;
    cv::split(lab_frame, lab_channels);

    // Apply CLAHE to L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(lab_channels[0], lab_channels[0]);

    cv::merge(lab_channels, lab_frame);
    cv::cvtColor(lab_frame, processed_frame, cv::COLOR_Lab2BGR);

    // Sharpen the image
    cv::Mat sharpened;
    cv::GaussianBlur(processed_frame, sharpened, cv::Size(0, 0), 3);
    cv::addWeighted(processed_frame, 1.5, sharpened, -0.5, 0, processed_frame);

    return detect(processed_frame);
}

bool MLObjectDetector::loadModel(const std::string& model_path, const std::string& config_path) {
    try {
        if (config_.framework == "darknet") {
            net_ = cv::dnn::readNetFromDarknet(config_path, model_path);
        } else if (config_.framework == "tensorflow") {
            net_ = cv::dnn::readNetFromTensorflow(model_path, config_path);
        } else if (config_.framework == "caffe") {
            net_ = cv::dnn::readNetFromCaffe(config_path, model_path);
        } else if (config_.framework == "onnx") {
            net_ = cv::dnn::readNetFromONNX(model_path);
        } else {
            // Try to auto-detect
            net_ = cv::dnn::readNet(model_path, config_path);
        }

        if (net_.empty()) {
            std::cerr << "Failed to load network from: " << model_path << std::endl;
            return false;
        }

        // Set preferable backend and target
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception loading model: " << e.what() << std::endl;
        return false;
    }
}

bool MLObjectDetector::loadClasses(const std::string& classes_path) {
    try {
        std::ifstream file(classes_path);
        if (!file.is_open()) {
            std::cerr << "Cannot open classes file: " << classes_path << std::endl;
            return false;
        }

        classes_.clear();
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                classes_.push_back(line);
            }
        }

        std::cout << "Loaded " << classes_.size() << " classes" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception loading classes: " << e.what() << std::endl;
        return false;
    }
}

bool MLObjectDetector::setInputParams(cv::Size size, cv::Scalar mean, double scale, bool swap_rb) {
    config_.input_size = size;
    config_.mean = mean;
    config_.scale = scale;
    config_.swap_rb = swap_rb;
    return true;
}

void MLObjectDetector::setConfidenceThreshold(float threshold) {
    config_.confidence_threshold = threshold;
}

void MLObjectDetector::setNMSThreshold(float threshold) {
    config_.nms_threshold = threshold;
}

void MLObjectDetector::setMaxDetections(int max_detections) {
    max_detections_ = max_detections;
}

cv::Mat MLObjectDetector::preprocessImage(const cv::Mat& frame) {
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, config_.scale, config_.input_size,
                          config_.mean, config_.swap_rb, false);
    return blob;
}

std::vector<DetectionResult> MLObjectDetector::postprocessDetections(
    const std::vector<cv::Mat>& outputs, const cv::Size& frame_size) {

    std::vector<DetectionResult> detections;

    for (const auto& output : outputs) {
        const auto* data = (float*)output.data;

        for (int i = 0; i < output.rows; ++i) {
            float confidence = data[i * output.cols + 4];

            if (confidence > config_.confidence_threshold) {
                int class_id = 0;
                float max_class_conf = 0.0f;

                // Find the class with highest confidence
                for (int j = 5; j < output.cols; ++j) {
                    float class_conf = data[i * output.cols + j];
                    if (class_conf > max_class_conf) {
                        max_class_conf = class_conf;
                        class_id = j - 5;
                    }
                }

                // Extract bounding box
                float center_x = data[i * output.cols + 0] * frame_size.width;
                float center_y = data[i * output.cols + 1] * frame_size.height;
                float width = data[i * output.cols + 2] * frame_size.width;
                float height = data[i * output.cols + 3] * frame_size.height;

                float x = center_x - width / 2;
                float y = center_y - height / 2;

                cv::Rect box(x, y, width, height);

                // Create detection result
                DetectionResult result;
                result.class_id = class_id;
                result.class_name = getClassName(class_id);
                result.confidence = confidence;
                result.bounding_box = box;
                result.center = cv::Point(center_x, center_y);
                result.distance_estimate = estimateDistance(result, cv::Mat()); // Placeholder
                result.threat_level = assessThreatLevel(result);

                if (isValidDetection(result)) {
                    detections.push_back(result);
                }
            }
        }
    }

    return detections;
}

void MLObjectDetector::applyNMS(std::vector<DetectionResult>& detections) {
    if (detections.empty()) return;

    // Sort by confidence
    std::sort(detections.begin(), detections.end(),
              [](const DetectionResult& a, const DetectionResult& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<bool> keep(detections.size(), true);

    for (size_t i = 0; i < detections.size(); ++i) {
        if (!keep[i]) continue;

        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (!keep[j]) continue;

            cv::Rect intersection = detections[i].bounding_box & detections[j].bounding_box;
            cv::Rect union_rect = detections[i].bounding_box | detections[j].bounding_box;

            float intersection_area = intersection.area();
            float union_area = union_rect.area();

            if (union_area > 0) {
                float iou = intersection_area / union_area;

                if (iou > config_.nms_threshold) {
                    keep[j] = false;
                }
            }
        }
    }

    // Remove suppressed detections
    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                      [&keep, &detections](const DetectionResult& det) {
                          size_t index = &det - &detections[0];
                          return !keep[index];
                      }),
        detections.end());
}

std::string MLObjectDetector::getClassName(int class_id) const {
    if (class_id >= 0 && class_id < static_cast<int>(classes_.size())) {
        return classes_[class_id];
    }
    return "unknown";
}

bool MLObjectDetector::isValidDetection(const DetectionResult& detection) const {
    return detection.confidence > config_.confidence_threshold &&
           detection.bounding_box.area() > 100 &&
           detection.bounding_box.x >= 0 &&
           detection.bounding_box.y >= 0;
}

double MLObjectDetector::estimateDistance(const DetectionResult& detection, const cv::Mat& frame) {
    // Simple distance estimation based on bounding box size
    // In a real implementation, this would use camera calibration and object size priors
    double box_area = detection.bounding_box.area();
    double focal_length = 1000.0; // pixels (approximate)
    double real_object_height = 1.7; // meters (person height)

    if (box_area > 0) {
        double distance = (focal_length * real_object_height) / std::sqrt(box_area);
        return distance;
    }

    return 0.0;
}

std::string MLObjectDetector::assessThreatLevel(const DetectionResult& detection) {
    // Simple threat assessment based on class and confidence
    if (detection.class_name.find("person") != std::string::npos ||
        detection.class_name.find("vehicle") != std::string::npos) {
        if (detection.confidence > 0.8) {
            return "HIGH";
        } else if (detection.confidence > 0.6) {
            return "MEDIUM";
        }
    }

    if (detection.confidence > 0.9) {
        return "CRITICAL";
    } else if (detection.confidence > 0.7) {
        return "HIGH";
    } else if (detection.confidence > 0.5) {
        return "MEDIUM";
    }

    return "LOW";
}

double MLObjectDetector::getInferenceTime() const {
    return last_inference_time_;
}

int MLObjectDetector::getModelInputWidth() const {
    return config_.input_size.width;
}

int MLObjectDetector::getModelInputHeight() const {
    return config_.input_size.height;
}

// Thermal Object Detector Implementation
ThermalObjectDetector::ThermalObjectDetector() : MLObjectDetector() {
    // Initialize with thermal-specific parameters
}

std::vector<DetectionResult> ThermalObjectDetector::detectThermalSignatures(const cv::Mat& thermal_frame) {
    if (thermal_frame.empty()) {
        return {};
    }

    // Enhance thermal image
    cv::Mat enhanced = enhanceThermalImage(thermal_frame);

    // Find hot regions
    std::vector<cv::Rect> hot_regions = findHotRegions(enhanced);

    std::vector<DetectionResult> detections;

    for (const auto& region : hot_regions) {
        DetectionResult result;
        result.class_id = 0; // Heat source
        result.class_name = "heat_source";
        result.confidence = 0.8f; // High confidence for thermal detections
        result.bounding_box = region;
        result.center = cv::Point(region.x + region.width/2, region.y + region.height/2);
        result.distance_estimate = estimateDistance(result, thermal_frame);
        result.threat_level = classifyHeatSource(
            calculateHeatIntensity(thermal_frame, region),
            region.area()
        );

        detections.push_back(result);
    }

    return detections;
}

cv::Mat ThermalObjectDetector::enhanceThermalImage(const cv::Mat& thermal_frame) {
    cv::Mat enhanced;

    // Apply CLAHE for contrast enhancement
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(thermal_frame, enhanced);

    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(enhanced, enhanced, cv::Size(3, 3), 0);

    return enhanced;
}

std::vector<cv::Rect> ThermalObjectDetector::findHotRegions(const cv::Mat& thermal_frame) {
    std::vector<cv::Rect> regions;

    // Threshold to find hot areas
    cv::Mat thresholded;
    cv::threshold(thermal_frame, thresholded, 200, 255, cv::THRESH_BINARY);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Convert contours to bounding rectangles
    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        if (rect.area() > 50) { // Filter small regions
            regions.push_back(rect);
        }
    }

    return regions;
}

double ThermalObjectDetector::calculateHeatIntensity(const cv::Mat& thermal_frame, const cv::Rect& region) {
    cv::Mat roi = thermal_frame(region);
    cv::Scalar mean_intensity = cv::mean(roi);
    return mean_intensity[0];
}

std::string ThermalObjectDetector::classifyHeatSource(double intensity, double area) {
    // Classify based on intensity and size
    if (intensity > 240 && area > 1000) {
        return "CRITICAL"; // Large, very hot source (e.g., missile launch)
    } else if (intensity > 220 && area > 500) {
        return "HIGH"; // Significant heat source (e.g., vehicle engine)
    } else if (intensity > 200 && area > 100) {
        return "MEDIUM"; // Moderate heat source (e.g., person)
    }

    return "LOW"; // Small heat source
}

// Multi-Sensor Fusion Implementation
MultiSensorFusionDetector::MultiSensorFusionDetector()
    : initialized_(false), spatial_threshold_(50.0), temporal_threshold_(0.1), confidence_boost_(0.2) {
}

MultiSensorFusionDetector::~MultiSensorFusionDetector() {
    // Cleanup
}

bool MultiSensorFusionDetector::initialize(const MLModelConfig& visual_config,
                                         const MLModelConfig& thermal_config) {
    try {
        visual_detector_ = std::make_unique<MLObjectDetector>();
        thermal_detector_ = std::make_unique<ThermalObjectDetector>();

        if (!visual_detector_->initialize(visual_config)) {
            std::cerr << "Failed to initialize visual detector" << std::endl;
            return false;
        }

        if (!thermal_detector_->initialize(thermal_config)) {
            std::cerr << "Failed to initialize thermal detector" << std::endl;
            return false;
        }

        initialized_ = true;
        std::cout << "Multi-sensor fusion detector initialized" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception during fusion detector initialization: " << e.what() << std::endl;
        return false;
    }
}

bool MultiSensorFusionDetector::isInitialized() const {
    return initialized_;
}

std::vector<DetectionResult> MultiSensorFusionDetector::detectFused(const cv::Mat& visual_frame,
                                                                  const cv::Mat& thermal_frame) {
    if (!initialized_ || visual_frame.empty() || thermal_frame.empty()) {
        return {};
    }

    // Get detections from both sensors
    auto visual_detections = visual_detector_->detect(visual_frame);
    auto thermal_detections = thermal_detector_->detectThermalSignatures(thermal_frame);

    // Fuse detections
    return fuseDetections(visual_detections, thermal_detections);
}

std::vector<DetectionResult> MultiSensorFusionDetector::fuseDetections(
    const std::vector<DetectionResult>& visual_detections,
    const std::vector<DetectionResult>& thermal_detections) {

    std::vector<DetectionResult> fused_detections;

    // For each visual detection, try to find corresponding thermal detection
    for (const auto& visual_det : visual_detections) {
        DetectionResult correlated = correlateDetection(visual_det, thermal_detections);

        if (correlated.class_id != -1) {
            // Found correlation, create fused detection
            DetectionResult fused;
            fused.class_id = visual_det.class_id;
            fused.class_name = visual_det.class_name;
            fused.confidence = calculateFusionConfidence(visual_det, correlated);
            fused.bounding_box = mergeBoundingBoxes(visual_det.bounding_box, correlated.bounding_box);
            fused.center = cv::Point(fused.bounding_box.x + fused.bounding_box.width/2,
                                   fused.bounding_box.y + fused.bounding_box.height/2);
            fused.distance_estimate = (visual_det.distance_estimate + correlated.distance_estimate) / 2.0;
            fused.threat_level = determineFusionThreatLevel(fused);

            fused_detections.push_back(fused);
        } else {
            // No thermal correlation, use visual detection with reduced confidence
            DetectionResult reduced_conf = visual_det;
            reduced_conf.confidence *= 0.8f; // Reduce confidence for non-correlated detections
            fused_detections.push_back(reduced_conf);
        }
    }

    // Add thermal-only detections with low confidence
    for (const auto& thermal_det : thermal_detections) {
        bool correlated = false;
        for (const auto& visual_det : visual_detections) {
            if (calculateDistance(thermal_det.center, visual_det.center) < spatial_threshold_) {
                correlated = true;
                break;
            }
        }

        if (!correlated) {
            DetectionResult thermal_only = thermal_det;
            thermal_only.confidence *= 0.6f; // Low confidence for thermal-only detections
            thermal_only.class_name = "thermal_only_" + thermal_det.class_name;
            fused_detections.push_back(thermal_only);
        }
    }

    return fused_detections;
}

DetectionResult MultiSensorFusionDetector::correlateDetection(
    const DetectionResult& visual_det,
    const std::vector<DetectionResult>& thermal_dets) {

    DetectionResult best_match;
    best_match.class_id = -1; // No match
    double min_distance = spatial_threshold_;

    for (const auto& thermal_det : thermal_dets) {
        double distance = calculateDistance(visual_det.center, thermal_det.center);

        if (distance < min_distance) {
            min_distance = distance;
            best_match = thermal_det;
        }
    }

    return best_match;
}

float MultiSensorFusionDetector::calculateFusionConfidence(
    const DetectionResult& visual_det, const DetectionResult& thermal_det) {

    // Boost confidence when both sensors agree
    float base_confidence = (visual_det.confidence + thermal_det.confidence) / 2.0f;
    return std::min(base_confidence + confidence_boost_, 1.0f);
}

std::string MultiSensorFusionDetector::determineFusionThreatLevel(const DetectionResult& fused_detection) {
    // Fusion increases threat assessment reliability
    if (fused_detection.confidence > 0.9) {
        return "CRITICAL";
    } else if (fused_detection.confidence > 0.8) {
        return "HIGH";
    } else if (fused_detection.confidence > 0.7) {
        return "MEDIUM";
    }

    return "LOW";
}

cv::Point MultiSensorFusionDetector::calculateCentroid(const cv::Rect& box) {
    return cv::Point(box.x + box.width/2, box.y + box.height/2);
}

double MultiSensorFusionDetector::calculateDistance(const cv::Point& p1, const cv::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

cv::Rect MultiSensorFusionDetector::mergeBoundingBoxes(const cv::Rect& box1, const cv::Rect& box2) {
    int x = std::min(box1.x, box2.x);
    int y = std::min(box1.y, box2.y);
    int width = std::max(box1.x + box1.width, box2.x + box2.width) - x;
    int height = std::max(box1.y + box1.height, box2.y + box2.height) - y;

    return cv::Rect(x, y, width, height);
}

// Factory Functions
std::unique_ptr<MLObjectDetector> createYOLOv3Detector(const std::string& model_path,
                                                     const std::string& config_path,
                                                     const std::string& classes_path) {
    auto detector = std::make_unique<MLObjectDetector>();

    MLModelConfig config;
    config.model_path = model_path;
    config.config_path = config_path;
    config.classes_path = classes_path;
    config.framework = "darknet";
    config.input_size = cv::Size(416, 416);
    config.mean = cv::Scalar(0, 0, 0);
    config.scale = 1.0/255.0;
    config.swap_rb = true;
    config.confidence_threshold = 0.5f;
    config.nms_threshold = 0.4f;

    if (detector->initialize(config)) {
        return detector;
    }

    return nullptr;
}

std::unique_ptr<MLObjectDetector> createMobileNetSSDDetector(const std::string& model_path,
                                                          const std::string& config_path,
                                                          const std::string& classes_path) {
    auto detector = std::make_unique<MLObjectDetector>();

    MLModelConfig config;
    config.model_path = model_path;
    config.config_path = config_path;
    config.classes_path = classes_path;
    config.framework = "caffe";
    config.input_size = cv::Size(300, 300);
    config.mean = cv::Scalar(127.5, 127.5, 127.5);
    config.scale = 1.0/127.5;
    config.swap_rb = false;
    config.confidence_threshold = 0.5f;
    config.nms_threshold = 0.4f;

    if (detector->initialize(config)) {
        return detector;
    }

    return nullptr;
}

// Utility Functions
cv::Mat drawDetections(const cv::Mat& frame, const std::vector<DetectionResult>& detections) {
    cv::Mat result = frame.clone();

    for (const auto& detection : detections) {
        // Draw bounding box
        cv::Scalar color;
        if (detection.threat_level == "CRITICAL") {
            color = cv::Scalar(0, 0, 255); // Red
        } else if (detection.threat_level == "HIGH") {
            color = cv::Scalar(0, 165, 255); // Orange
        } else if (detection.threat_level == "MEDIUM") {
            color = cv::Scalar(0, 255, 255); // Yellow
        } else {
            color = cv::Scalar(0, 255, 0); // Green
        }

        cv::rectangle(result, detection.bounding_box, color, 2);

        // Draw label
        std::string label = detection.class_name + ": " +
                          std::to_string(static_cast<int>(detection.confidence * 100)) + "%";

        cv::putText(result, label,
                   cv::Point(detection.bounding_box.x, detection.bounding_box.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }

    return result;
}

cv::Mat drawThermalDetections(const cv::Mat& thermal_frame, const std::vector<DetectionResult>& detections) {
    cv::Mat result;
    cv::cvtColor(thermal_frame, result, cv::COLOR_GRAY2BGR);

    for (const auto& detection : detections) {
        cv::Scalar color(0, 0, 255); // Red for thermal detections
        cv::rectangle(result, detection.bounding_box, color, 2);

        std::string label = "Heat: " + detection.threat_level;
        cv::putText(result, label,
                   cv::Point(detection.bounding_box.x, detection.bounding_box.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }

    return result;
}

void printDetectionInfo(const std::vector<DetectionResult>& detections) {
    std::cout << "=== Detection Results ===" << std::endl;
    std::cout << "Total detections: " << detections.size() << std::endl;

    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        std::cout << "Detection " << i + 1 << ":" << std::endl;
        std::cout << "  Class: " << det.class_name << " (ID: " << det.class_id << ")" << std::endl;
        std::cout << "  Confidence: " << det.confidence << std::endl;
        std::cout << "  Bounding Box: [" << det.bounding_box.x << ", " << det.bounding_box.y
                  << ", " << det.bounding_box.width << ", " << det.bounding_box.height << "]" << std::endl;
        std::cout << "  Center: (" << det.center.x << ", " << det.center.y << ")" << std::endl;
        std::cout << "  Distance Estimate: " << det.distance_estimate << "m" << std::endl;
        std::cout << "  Threat Level: " << det.threat_level << std::endl;
        std::cout << std::endl;
    }
}

} // namespace sensor_drivers