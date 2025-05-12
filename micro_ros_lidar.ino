#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "lds_all_models.h"

#ifndef ESP32
#error This example runs on ESP32
#endif

// #define DEBUG_GPIO
// #define DEBUG_PACKETS
// #define DEBUG_SERIAL_IN
// #define DEBUG_SERIAL_OUT
//#define INVERT_PWM_PIN

const uint8_t LIDAR_GPIO_EN = 19;
const uint8_t LIDAR_GPIO_RX = 16;
const uint8_t LIDAR_GPIO_TX = 17;
const uint8_t LIDAR_GPIO_PWM = 15;
#define SLAMTEC_RPLIDAR_A1
const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 2;

HardwareSerial LidarSerial(2);
LDS *lidar;

// micro-ROS
rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan laser_scan_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

#define LED_PIN 2
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Forward declaration of timer callback
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

#define MAX_SCAN_POINTS 360
float ranges[MAX_SCAN_POINTS];
float intensities[MAX_SCAN_POINTS];
bool scan_ready_to_publish = false;

const uint16_t PRINT_EVERY_NTH_POINT = 20;
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  static int i = 0;

  if (scan_completed) {
    // When scan is complete, mark it as ready to publish
    i = 0;
    Serial.print("Scan completed; scans-per-second ");
    Serial.println(lidar->getCurrentScanFreqHz());

    // Update message header timestamp
    laser_scan_msg.header.stamp.sec = millis() / 1000;
    laser_scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    // Set the size of the arrays to match the actual filled points
    laser_scan_msg.ranges.size = MAX_SCAN_POINTS;
    laser_scan_msg.intensities.size = MAX_SCAN_POINTS;
    // rcl_publish(&publisher, &laser_scan_msg, NULL);
    // Signal that scan is ready to publish
    scan_ready_to_publish = true;
  }

  // Convert angle to index in the arrays (0-359 degrees to 0-359 index)
  int index = (int)(angle_deg) % MAX_SCAN_POINTS;

  // Store data
  ranges[index] = distance_mm / 1000.0;  // Convert mm to meters
  intensities[index] = quality;

  // // Debug print
  // if (i % PRINT_EVERY_NTH_POINT == 0) {
  //   Serial.print(i);
  //   Serial.print(' ');
  //   Serial.print(distance_mm);
  //   Serial.print(' ');
  //   Serial.println(angle_deg);
  // }
  // i++;
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t *buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = LIDAR_GPIO_PWM;

  Serial.print("GPIO ");
  Serial.print(pin);
  Serial.print(' ');
  Serial.print(lidar->pinIDToString(lidar_pin));
  Serial.print(" mode set to ");
  Serial.println(lidar->pinStateToString((LDS::lds_pin_state_t) int(value)));

  if (value <= (float)LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
#if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
#else
      if (!ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
#endif
    } else {
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }

  if (value < (float)LDS::VALUE_PWM) {
    // Set constant output
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  } else {
    // set PWM duty cycle
#ifdef INVERT_PWM_PIN
    value = 1 - value;
#endif
    int pwm_value = ((1 << LIDAR_PWM_BITS) - 1) * value;

#if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
#else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
#endif
  }
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t *packet, uint16_t length, bool scan_completed) {
  // Serial.println(*packet);
  // Serial.print("Packet callback, length=");
  // Serial.print(length);
  // Serial.print(", scan_completed=");
  // Serial.println(scan_completed);
  return;
}

void setupLidar() {
  lidar = new LDS_RPLIDAR_A1();
  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);
  LidarSerial.begin(256000, SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);
  lidar->init();

  Serial.print("LiDAR model ");
  Serial.print(lidar->getModelName());
  Serial.print(", baud rate ");
  Serial.print(lidar->getSerialBaudRate());
  Serial.print(", TX GPIO ");
  Serial.print(LIDAR_GPIO_TX);
  Serial.print(", RX GPIO ");
  Serial.println(LIDAR_GPIO_RX);

  LDS::result_t result = lidar->start();
  Serial.print("startLidar() result: ");
  Serial.println(lidar->resultCodeToString(result));
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(SERIAL_MONITOR_BAUD);

  // Set up LiDAR
  setupLidar();

  // Set up micro-ROS
  set_microros_wifi_transports("Barelang63", "barelang63", "192.168.50.247", 8888);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("WIFI START");

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "lidar_node", "", &support));

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_profile.depth = 1;  // Only keep the most recent message

  // Initialize publisher with custom QoS
  RCCHECK(rclc_publisher_init_default(&publisher, &node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
                              "scan"));
  // Timer for heartbeat and potential publishing
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize laser scan message
  laser_scan_msg.header.frame_id.data = (char *)"lidar_link";
  laser_scan_msg.header.frame_id.size = strlen("lidar_link");
  laser_scan_msg.header.frame_id.capacity = strlen("lidar_link") + 1;

  laser_scan_msg.angle_min = 0.0;
  laser_scan_msg.angle_max = 2 * PI;
  laser_scan_msg.angle_increment = 2 * PI / MAX_SCAN_POINTS;
  laser_scan_msg.time_increment = 0.0;
  laser_scan_msg.scan_time = 0.1;   // 10Hz scan rate, adjust as needed
  laser_scan_msg.range_min = 0.1;   // 100mm
  laser_scan_msg.range_max = 12.0;  // 12m

  // Initialize arrays
  laser_scan_msg.ranges.data = ranges;
  laser_scan_msg.ranges.size = 0;
  laser_scan_msg.ranges.capacity = MAX_SCAN_POINTS;

  laser_scan_msg.intensities.data = intensities;
  laser_scan_msg.intensities.size = 0;
  laser_scan_msg.intensities.capacity = MAX_SCAN_POINTS;

  // Initialize arrays with default values
  for (int i = 0; i < MAX_SCAN_POINTS; i++) {
    ranges[i] = 0.0;
    intensities[i] = 0.0;
  }

  Serial.println("Setup complete");
}

void loop() {
  // Process LiDAR data
  // rcl_publish(&publisher, &laser_scan_msg, NULL);
  rcl_ret_t publish_ret = rcl_publish(&publisher, &laser_scan_msg, NULL);
  if (publish_ret == RCL_RET_OK) {
    Serial.println("Published LaserScan message successfully");
  } else {
    Serial.print("Error publishing: ");
    Serial.println(publish_ret);
  }

  lidar->loop();

  // Process micro-ROS communications - THIS WAS MISSING IN YOUR CODE
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // // If scan is ready, publish it
  // if (scan_ready_to_publish) {
  //   rcl_ret_t publish_ret = rcl_publish(&publisher, &laser_scan_msg, NULL);
  //   if (publish_ret == RCL_RET_OK) {
  //     Serial.println("Published LaserScan message successfully");
  //   } else {
  //     Serial.print("Error publishing: ");
  //     Serial.println(publish_ret);
  //   }

  //   scan_ready_to_publish = false;

  //   // Reset arrays for next scan if needed
  //   // (We're keeping values between scans to avoid gaps)
  // }

  // Small delay to prevent CPU hogging
  // delay(1);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}