# 🔬 **COMPREHENSIVE TESTING GUIDE**
## Advanced Sensor System Testing & Validation

This guide provides step-by-step instructions for testing all advanced functionalities of the enhanced sensor data processing system.

---

## 📋 **TESTING OVERVIEW**

### **What We'll Test:**
1. ✅ **USB Webcam Hardware Integration**
2. ✅ **Real-time Processing Pipeline**
3. ✅ **Performance Metrics (CPU/Memory/Processing Time)**
4. ✅ **System Health Monitoring**
5. ✅ **Intelligent Alert System**
6. ✅ **Data Logging & Recording**

### **Prerequisites:**
- ROS 2 Humble installed
- USB webcam connected
- Python 3.8+ with required packages
- OpenCV installed
- System monitoring tools (htop, psutil)

---

## 🖥️ **1. USB WEBCAM TESTING**

### **Step 1: Hardware Detection**
```bash
# Check available video devices
ls /dev/video*

# Test webcam with OpenCV (without ROS)
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print(f'Webcam working! Resolution: {frame.shape[1]}x{frame.shape[0]}')
        cv2.imwrite('test_frame.jpg', frame)
    cap.release()
else:
    print('Webcam not accessible')
"
```

### **Step 2: ROS 2 Webcam Testing**
```bash
# Terminal 1: Source and launch
cd ros2_ws
source install/setup.bash

# Launch with real hardware
ros2 launch sensor_drivers sensor_drivers_launch.py \
    eo_camera_node:=true \
    eo_camera_node.ros__parameters.use_simulation:=false \
    eo_camera_node.ros__parameters.device_path:='/dev/video0'

# Terminal 2: Monitor topics
ros2 topic list | grep sensors

# Terminal 3: View camera feed
ros2 run image_view image_view image:=/sensors/eo_image
```

### **Step 3: Verify Real-time Processing**
```bash
# Check topic publishing rate
ros2 topic hz /sensors/eo_image

# Check image properties
ros2 topic echo /sensors/eo_image --once | head -20

# Monitor processing performance
ros2 topic echo /system/metrics --once
```

---

## ⚡ **2. REAL-TIME PROCESSING TESTING**

### **Step 1: Launch Enhanced System**
```bash
# Launch complete system with monitoring
cd ros2_ws
source install/setup.bash

ros2 launch sensor_drivers sensor_drivers_launch.py \
    enhanced_preprocessing_node:=true \
    system_monitor:=true \
    eo_camera_node.ros__parameters.use_simulation:=false
```

### **Step 2: Monitor Processing Pipeline**
```bash
# Terminal 1: Monitor raw camera feed
ros2 topic hz /sensors/eo_image

# Terminal 2: Monitor processed feed
ros2 topic hz /sensors/eo_processed

# Terminal 3: Monitor detections
ros2 topic hz /sensors/detections_overlay

# Terminal 4: Monitor system metrics
ros2 topic hz /system/metrics
```

### **Step 3: Performance Analysis**
```bash
# Check processing latency
ros2 topic delay /sensors/eo_image

# Monitor CPU usage
htop

# Check memory usage
free -h
```

---

## 📊 **3. PERFORMANCE METRICS TESTING**

### **Step 1: Launch Performance Monitoring**
```bash
# Start system with performance monitoring enabled
cd ros2_ws
source install/setup.bash

ros2 run sensor_drivers system_monitor.py \
    --ros-args -p enable_alerts:=true \
    -p alert_log_file:=logs/test_alerts.log \
    -p metrics_log_file:=logs/test_metrics.log
```

### **Step 2: Generate Load for Testing**
```bash
# Terminal 1: Run stress test
python3 -c "
import time
import numpy as np

# Generate processing load
for i in range(100):
    # Simulate heavy computation
    data = np.random.rand(1000, 1000)
    result = np.fft.fft2(data)
    time.sleep(0.1)
    print(f'Load test iteration: {i+1}')
"
```

### **Step 3: Monitor Performance Metrics**
```bash
# Terminal 2: Monitor real-time metrics
ros2 topic echo /system/metrics

# Terminal 3: Check system resources
watch -n 1 'ps aux | grep sensor | head -5'

# Terminal 4: Monitor CPU/memory
watch -n 1 'top -p $(pgrep -f sensor) -b -n 1 | head -10'
```

### **Step 4: Analyze Performance Logs**
```bash
# View metrics log
tail -f logs/test_metrics.log

# Analyze performance data
python3 -c "
import json
import matplotlib.pyplot as plt

# Read and analyze metrics
with open('logs/test_metrics.log', 'r') as f:
    lines = f.readlines()

cpu_values = []
memory_values = []

for line in lines[-20:]:  # Last 20 entries
    try:
        # Parse log line and extract metrics
        if 'CPU:' in line and 'MEM:' in line:
            cpu = float(line.split('CPU:')[1].split('%')[0])
            mem = float(line.split('MEM:')[1].split('%')[0])
            cpu_values.append(cpu)
            memory_values.append(mem)
    except:
        continue

print(f'Average CPU: {sum(cpu_values)/len(cpu_values):.1f}%')
print(f'Average Memory: {sum(memory_values)/len(memory_values):.1f}%')
print(f'Peak CPU: {max(cpu_values):.1f}%')
print(f'Peak Memory: {max(memory_values):.1f}%')
"
```

---

## 🏥 **4. SYSTEM HEALTH MONITORING TESTING**

### **Step 1: Launch Health Monitoring**
```bash
# Start health monitoring system
cd ros2_ws
source install/setup.bash

ros2 run sensor_drivers system_monitor.py
```

### **Step 2: Test Sensor Connectivity**
```bash
# Terminal 1: Monitor sensor status
ros2 topic echo /system/health

# Terminal 2: Simulate sensor failure (stop camera node)
ros2 lifecycle set /eo_camera_node shutdown

# Terminal 3: Observe health status change
ros2 topic echo /system/health --once
```

### **Step 3: Test Multi-Sensor Health**
```bash
# Launch all sensors
ros2 launch sensor_drivers sensor_drivers_launch.py

# Monitor all sensor health
ros2 topic echo /system/health

# Check individual sensor status
python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class HealthChecker(Node):
    def __init__(self):
        super().__init__('health_checker')
        self.subscription = self.create_subscription(
            String, '/system/health', self.health_callback, 10)

    def health_callback(self, msg):
        health_data = json.loads(msg.data)
        print('=== SYSTEM HEALTH STATUS ===')
        print(f'System Status: {health_data[\"system_status\"]}')
        print(f'Active Sensors: {health_data[\"sensor_status\"][\"active_sensors\"]}')
        print(f'Offline Sensors: {health_data[\"sensor_status\"][\"offline_sensors\"]}')
        print(f'Uptime: {health_data[\"uptime\"]:.1f} seconds')
        print()

rclpy.init()
checker = HealthChecker()
rclpy.spin_once(checker, timeout_sec=1.0)
rclpy.shutdown()
"
```

---

## 🚨 **5. INTELLIGENT ALERT SYSTEM TESTING**

### **Step 1: Configure Alert Thresholds**
```bash
# Launch with custom alert thresholds
cd ros2_ws
source install/setup.bash

ros2 run sensor_drivers system_monitor.py \
    --ros-args \
    -p enable_alerts:=true \
    -p alert_log_file:=logs/alert_test.log
```

### **Step 2: Generate Alert Conditions**
```bash
# Terminal 1: Monitor alerts
ros2 topic echo /system/alerts

# Terminal 2: Generate high CPU load
python3 -c "
import time
import multiprocessing

def cpu_stress():
    while True:
        pass

# Create multiple CPU stress processes
processes = []
for i in range(multiprocessing.cpu_count()):
    p = multiprocessing.Process(target=cpu_stress)
    p.start()
    processes.append(p)

try:
    time.sleep(30)  # Run for 30 seconds
finally:
    for p in processes:
        p.terminate()
        p.join()
"

# Terminal 3: Generate memory pressure
python3 -c "
import time

# Allocate large amounts of memory
memory_hog = []
for i in range(100):
    memory_hog.append('x' * 10**6)  # 1MB per iteration
    time.sleep(0.1)

print('Memory pressure test completed')
"
```

### **Step 3: Verify Alert Generation**
```bash
# Check alert log
tail -f logs/alert_test.log

# Count alerts by severity
python3 -c "
import json

alert_counts = {'WARNING': 0, 'CRITICAL': 0, 'INFO': 0}

try:
    with open('logs/alert_test.log', 'r') as f:
        for line in f:
            if '[WARNING]' in line:
                alert_counts['WARNING'] += 1
            elif '[CRITICAL]' in line:
                alert_counts['CRITICAL'] += 1
            elif '[INFO]' in line:
                alert_counts['INFO'] += 1

    print('=== ALERT SUMMARY ===')
    for level, count in alert_counts.items():
        print(f'{level}: {count} alerts')
    print(f'Total: {sum(alert_counts.values())} alerts')
except FileNotFoundError:
    print('No alert log found')
"
```

---

## 📝 **6. DATA LOGGING TESTING**

### **Step 1: Configure Logging**
```bash
# Create logs directory
mkdir -p logs

# Launch system with logging enabled
cd ros2_ws
source install/setup.bash

ros2 launch sensor_drivers sensor_drivers_launch.py \
    system_monitor:=true \
    system_monitor.ros__parameters.enable_alerts:=true \
    system_monitor.ros__parameters.alert_log_file:='logs/comprehensive_test.log' \
    system_monitor.ros__parameters.metrics_log_file:='logs/metrics_test.log'
```

### **Step 2: Generate Test Data**
```bash
# Run comprehensive test for 5 minutes
timeout 300 bash -c '
echo "Starting comprehensive logging test..."

# Monitor log file growth
watch -n 10 "ls -lh logs/*.log" &

# Generate various system loads
for i in {1..30}; do
    echo "Test iteration $i/30"

    # CPU stress
    stress --cpu 2 --timeout 10 &

    # Memory stress
    stress --vm 1 --vm-bytes 256M --timeout 10 &

    # I/O stress
    dd if=/dev/zero of=/tmp/test_file bs=1M count=100 &
    rm -f /tmp/test_file

    sleep 10
done

echo "Comprehensive logging test completed"
'
```

### **Step 3: Analyze Logged Data**
```bash
# Check log file sizes
ls -lh logs/*.log

# Analyze metrics log
python3 -c "
import re
from collections import defaultdict
import matplotlib.pyplot as plt

# Parse metrics log
cpu_values = []
memory_values = []
timestamps = []

with open('logs/metrics_test.log', 'r') as f:
    for line in f:
        if 'CPU:' in line and 'MEM:' in line:
            try:
                cpu_match = re.search(r'CPU:([0-9.]+)%', line)
                mem_match = re.search(r'MEM:([0-9.]+)%', line)

                if cpu_match and mem_match:
                    cpu_values.append(float(cpu_match.group(1)))
                    memory_values.append(float(mem_match.group(1)))
                    timestamps.append(len(timestamps))
            except:
                continue

print(f'Logged {len(cpu_values)} metric entries')
print(f'Average CPU: {sum(cpu_values)/len(cpu_values):.1f}%')
print(f'Average Memory: {sum(memory_values)/len(memory_values):.1f}%')
print(f'Peak CPU: {max(cpu_values):.1f}%')
print(f'Peak Memory: {max(memory_values):.1f}%')
"

# Analyze alert log
python3 -c "
import re
from collections import Counter

alerts = []

with open('logs/comprehensive_test.log', 'r') as f:
    for line in f:
        if any(level in line for level in ['[WARNING]', '[CRITICAL]', '[INFO]']):
            alerts.append(line.strip())

print(f'Total alerts logged: {len(alerts)}')

# Count by type
warning_count = sum(1 for alert in alerts if '[WARNING]' in alert)
critical_count = sum(1 for alert in alerts if '[CRITICAL]' in alert)
info_count = sum(1 for alert in alerts if '[INFO]' in alert)

print(f'Warnings: {warning_count}')
print(f'Critical: {critical_count}')
print(f'Info: {info_count}')

print('\nSample alerts:')
for alert in alerts[:5]:
    print(f'  {alert}')
"
```

---

## 🔧 **7. AUTOMATED TESTING SCRIPTS**

### **Create Automated Test Runner**
```bash
# Create automated test script
cat > automated_system_test.sh << 'EOF'
#!/bin/bash

echo "=== AUTOMATED SYSTEM TEST ==="
echo "Testing all advanced functionalities..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run test
run_test() {
    local test_name=$1
    local command=$2

    echo -n "Running $test_name... "

    if eval "$command" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        ((TESTS_PASSED++))
    else
        echo -e "${RED}FAIL${NC}"
        ((TESTS_FAILED++))
    fi
}

# Test 1: System startup
run_test "System Startup" "timeout 10 ros2 launch sensor_drivers sensor_drivers_launch.py"

# Test 2: Topic availability
run_test "Topic Availability" "ros2 topic list | grep -q sensors"

# Test 3: Node responsiveness
run_test "Node Responsiveness" "ros2 node list | grep -q sensor"

# Test 4: Performance monitoring
run_test "Performance Monitoring" "timeout 5 ros2 topic echo /system/metrics --once > /dev/null"

# Test 5: Alert system
run_test "Alert System" "timeout 5 ros2 topic echo /system/alerts --once > /dev/null"

# Test 6: Health monitoring
run_test "Health Monitoring" "timeout 5 ros2 topic echo /system/health --once > /dev/null"

# Test 7: Log file creation
run_test "Log File Creation" "ls logs/*.log > /dev/null 2>&1"

# Summary
echo
echo "=== TEST SUMMARY ==="
echo "Tests Passed: $TESTS_PASSED"
echo "Tests Failed: $TESTS_FAILED"
echo "Success Rate: $(( (TESTS_PASSED * 100) / (TESTS_PASSED + TESTS_FAILED) ))%"

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed! System is fully functional.${NC}"
else
    echo -e "${RED}Some tests failed. Check system configuration.${NC}"
fi
EOF

chmod +x automated_system_test.sh
```

### **Run Automated Tests**
```bash
# Execute automated test suite
./automated_system_test.sh
```

---

## 📈 **8. PERFORMANCE BENCHMARKING**

### **Create Performance Benchmark**
```bash
# Create performance benchmark script
cat > performance_benchmark.py << 'EOF'
#!/usr/bin/env python3

import time
import psutil
import subprocess
import json
from datetime import datetime

class PerformanceBenchmark:
    def __init__(self):
        self.results = {}
        self.baseline_metrics = {}

    def capture_baseline(self):
        """Capture baseline system metrics"""
        print("Capturing baseline metrics...")
        self.baseline_metrics = {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_usage': psutil.disk_usage('/').percent
        }
        print(f"Baseline - CPU: {self.baseline_metrics['cpu_percent']}%, "
              f"Memory: {self.baseline_metrics['memory_percent']}%, "
              f"Disk: {self.baseline_metrics['disk_usage']}%")

    def benchmark_processing(self, duration=60):
        """Benchmark sensor processing performance"""
        print(f"\nBenchmarking for {duration} seconds...")

        start_time = time.time()
        metrics_over_time = []

        while time.time() - start_time < duration:
            # Capture metrics every second
            metrics = {
                'timestamp': time.time(),
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': psutil.virtual_memory().percent,
                'memory_used_mb': psutil.virtual_memory().used / (1024 * 1024)
            }
            metrics_over_time.append(metrics)
            time.sleep(1)

        # Calculate statistics
        cpu_values = [m['cpu_percent'] for m in metrics_over_time]
        memory_values = [m['memory_percent'] for m in metrics_over_time]

        self.results['processing'] = {
            'duration': duration,
            'avg_cpu': sum(cpu_values) / len(cpu_values),
            'max_cpu': max(cpu_values),
            'avg_memory': sum(memory_values) / len(memory_values),
            'max_memory': max(memory_values),
            'cpu_overhead': sum(cpu_values) / len(cpu_values) - self.baseline_metrics['cpu_percent'],
            'memory_overhead': sum(memory_values) / len(memory_values) - self.baseline_metrics['memory_percent']
        }

        print("Processing Benchmark Results:")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")

    def benchmark_topics(self):
        """Benchmark ROS topic performance"""
        print("\nBenchmarking ROS topics...")

        topics_to_test = [
            '/sensors/ir_image',
            '/sensors/eo_image',
            '/sensors/radar_scan',
            '/system/metrics',
            '/system/health'
        ]

        topic_results = {}

        for topic in topics_to_test:
            try:
                # Test topic availability
                result = subprocess.run(
                    ['ros2', 'topic', 'hz', topic],
                    capture_output=True, text=True, timeout=5
                )

                if result.returncode == 0:
                    # Parse hz output (simplified)
                    topic_results[topic] = {'status': 'active'}
                else:
                    topic_results[topic] = {'status': 'inactive'}

            except subprocess.TimeoutExpired:
                topic_results[topic] = {'status': 'timeout'}
            except Exception as e:
                topic_results[topic] = {'status': 'error', 'error': str(e)}

        self.results['topics'] = topic_results

        print("Topic Status:")
        for topic, status in topic_results.items():
            print(f"  {topic}: {status['status']}")

    def generate_report(self):
        """Generate comprehensive benchmark report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'baseline': self.baseline_metrics,
            'results': self.results,
            'summary': {
                'processing_efficiency': 'good' if self.results.get('processing', {}).get('cpu_overhead', 100) < 20 else 'poor',
                'memory_efficiency': 'good' if self.results.get('processing', {}).get('memory_overhead', 100) < 10 else 'poor',
                'system_stability': 'good' if len([t for t in self.results.get('topics', {}).values() if t['status'] == 'active']) >= 3 else 'poor'
            }
        }

        with open('logs/performance_report.json', 'w') as f:
            json.dump(report, f, indent=2)

        print("
=== PERFORMANCE REPORT ===")
        print(f"Processing Efficiency: {report['summary']['processing_efficiency']}")
        print(f"Memory Efficiency: {report['summary']['memory_efficiency']}")
        print(f"System Stability: {report['summary']['system_stability']}")
        print("Detailed report saved to logs/performance_report.json"

if __name__ == "__main__":
    benchmark = PerformanceBenchmark()
    benchmark.capture_baseline()
    benchmark.benchmark_topics()
    benchmark.benchmark_processing(duration=30)  # 30 second benchmark
    benchmark.generate_report()
EOF

python3 performance_benchmark.py
```

---

## 🎯 **9. TROUBLESHOOTING GUIDE**

### **Common Issues & Solutions**

#### **USB Webcam Issues**
```bash
# Check webcam permissions
ls -l /dev/video0

# Fix permissions if needed
sudo chmod 666 /dev/video0

# Test with OpenCV directly
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Webcam OK' if cap.isOpened() else 'Webcam FAIL')"
```

#### **Performance Issues**
```bash
# Check system resources
htop
free -h

# Monitor ROS processes
ps aux | grep ros

# Check topic frequencies
ros2 topic hz /sensors/eo_image
```

#### **Alert System Issues**
```bash
# Check alert logs
tail -f logs/system_alerts.log

# Verify alert thresholds
ros2 param get /system_monitor alert_thresholds

# Test alert generation manually
ros2 topic pub /system/metrics std_msgs/String "data: {\"cpu_percent\": 95.0}"
```

#### **Data Logging Issues**
```bash
# Check log directory permissions
ls -ld logs/

# Verify log file creation
ls -la logs/*.log

# Check disk space
df -h
```

---

## 📊 **10. TEST RESULTS INTERPRETATION**

### **Performance Metrics Interpretation**
- **CPU Usage < 30%**: Excellent performance
- **CPU Usage 30-50%**: Good performance
- **CPU Usage 50-70%**: Acceptable performance
- **CPU Usage > 70%**: Performance issues

### **Memory Usage Guidelines**
- **Memory < 50%**: Excellent
- **Memory 50-70%**: Good
- **Memory 70-85%**: Monitor closely
- **Memory > 85%**: Memory pressure

### **Topic Frequency Standards**
- **Camera topics**: 10-30 Hz expected
- **Processing topics**: 5-15 Hz expected
- **System topics**: 1-5 Hz expected

### **Alert System Validation**
- **No alerts**: System operating normally
- **Warning alerts**: Monitor system
- **Critical alerts**: Immediate attention required

---

## 🎉 **SUCCESS CRITERIA**

### **All Tests Pass When:**
1. ✅ **USB webcam connects and streams video**
2. ✅ **Real-time processing maintains <50ms latency**
3. ✅ **Performance metrics show <30% CPU overhead**
4. ✅ **All sensors report healthy status**
5. ✅ **Alert system responds to threshold violations**
6. ✅ **Data logging captures all system events**
7. ✅ **Automated tests pass with 100% success rate**

### **Ready for Production When:**
- All performance benchmarks meet or exceed standards
- System can handle peak loads without alerts
- All sensors maintain stable connections
- Data logging is comprehensive and reliable
- Alert system provides timely notifications

---

**This comprehensive testing guide ensures your enhanced sensor system is thoroughly validated and ready for production deployment!** 🚀