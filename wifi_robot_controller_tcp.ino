#define RMW_UXRCE_MAX_INPUT_BUFFER_SIZE 1024
#define RMW_UXRCE_MAX_TRANSPORT_MTU 512

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// =============================================================================
// NETWORK & HARDWARE CONFIGURATION
// =============================================================================
enum States {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool micro_ros_init_materials();
void micro_ros_destroy_materials();
void setupJointStatePublisher(rcl_node_t *node, rclc_support_t *support);
void publishJointStates(float left_rad_s, float right_rad_s);
void stopMotors();
void arm_callback(const void * msvin);
void gripper_subscription_callback(const void * msvin);
void cmd_vel_callback(const void * msin);
void handleArmTcpServer();

SemaphoreHandle_t serialMutex;

const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
const uint16_t LIDAR_TCP_PORT = 8889;
const uint16_t ARM_TCP_PORT   = 8890; // TCP Server Port for Robotic Arm trajectory

size_t agent_port = 8888;
char* agent_ip = (char*)"192.168.1.160"; // Replace with your PC's actual Wi-Fi IP

// Motor Driver Pin Definitions (L298N)
#define L298N_IN1 14
#define L298N_IN2 15
#define L298N_IN3 12
#define L298N_IN4 13

// L298N PWM Speed Control Pins
#define L298N_ENA 2  // Left Motor PWM
#define L298N_ENB 4  // Right Motor PWM

// One-way LiDAR RX Pin
#define LIDAR_RX_PIN 16
#define LIDAR_MOTOR_PWM_PIN 5

#define ARDUINO_RX_PIN 18
#define ARDUINO_TX_PIN 19

// Hardware Serial Interfaces
HardwareSerial ArduinoSerial(2); // UART2 for Arduino Uno Arm Controller

// TCP Server for LiDAR Bridge & Arm Control
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiClient lidarClient;

WiFiServer armServer(ARM_TCP_PORT);
WiFiClient armClient;

// micro-ROS Core Objects
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t arm_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

std_msgs__msg__Float32MultiArray arm_msg;

rcl_subscription_t gripper_sub;
std_msgs__msg__Float32 gripper_cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// PWM Configuration Parameters
const int PWM_FREQ = 5000;    // 5 kHz PWM frequency
const int PWM_RES = 8;        // 8-bit resolution (0-255)

// Robot Physical Parameters
const float WHEEL_SEPARATION = 0.233; // Distance between wheels in meters
const float MAX_LINEAR_VEL = 0.5;    // Max linear speed (m/s)

// =============================================================================
// QUADRATURE ENCODER CONFIGURATION (JGY-370)
// =============================================================================
#define RIGHT_ENCA_PIN 22
#define RIGHT_ENCB_PIN 23

#define LEFT_ENCA_PIN  25
#define LEFT_ENCB_PIN  26

// Volatile variables modified inside Quadrature ISRs
volatile long left_wheel_ticks = 0;
volatile long right_wheel_ticks = 0;

// Track timing for velocity calculation
unsigned long last_encoder_time = 0;
const int CALC_INTERVAL = 100; // 100 ms (10 Hz)

// JGY-370: 11 PPC * 87 Gear Ratio = 957 Ticks/Rev per channel
const float TICKS_PER_REV = 717.0;  
const float WHEEL_DIAMETER = 0.065;  // 65mm diameter
const float WHEEL_RADIUS = 0.0325;   // 32.5mm radius

// =============================================================================
// PID CONTROL VARIABLES
// =============================================================================
struct PIDState {
  float prev_error = 0.0;
  float integral = 0.0;
};

PIDState left_pid;
PIDState right_pid;

float Kv = 10.9303718369895;
int Bv = 152.718267291324;
float Kp = 0.0;
float Ki = 10.0;  
float Kd = 0.0;   

int MIN_PWM = 150; 
int PWM_MAX = 220;

volatile float target_left_rad_s = 0.0;
volatile float target_right_rad_s = 0.0;

// micro-ROS Publisher Setup
rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

rosidl_runtime_c__String name_elements[2];
double position_elements[2] = {0.0, 0.0};
double velocity_elements[2] = {0.0, 0.0};

unsigned long last_joint_state_pub_time = 0;
const unsigned long PUB_INTERVAL = 50; 
unsigned long last_pid_time = 0;
const unsigned long PID_INTERVAL = 100; 

// Initialize dynamic buffer memory for Float32MultiArray
void init_arm_message() {
  rosidl_runtime_c__float__Sequence__init(&arm_msg.data, 5);
}

bool micro_ros_init_materials() {
  allocator = rcl_get_default_allocator();

  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) return false;

  ret = rclc_node_init_default(&node, "esp32_base_node", "", &support);
  if (ret != RCL_RET_OK) return false;

  init_arm_message();

  rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  // rclc_subscription_init_default(
  //   &arm_subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  //   "/arm_servo_cmds"
  // );

  rclc_subscription_init_default(
    &gripper_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/gripper_cmd"
  );

  setupJointStatePublisher(&node, &support);

  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &arm_subscriber, &arm_msg, &arm_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &gripper_sub, &gripper_cmd_msg, &gripper_subscription_callback, ON_NEW_DATA);

  rmw_uros_sync_session(1000);

  return true;
}

void micro_ros_destroy_materials() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&joint_state_pub, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  // rcl_subscription_fini(&arm_subscriber, &node);
  rcl_subscription_fini(&gripper_sub, &node);
  rclc_executor_fini(&executor);
  
  rosidl_runtime_c__float__Sequence__fini(&arm_msg.data);

  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

int computePID(float target_rad_s, float actual_rad_s, PIDState &pid, float dt_sec, int wheel_no) {
  if (abs(target_rad_s) < 0.05) {
    pid.prev_error = 0.0;
    return 0;
  }

  float feed_forward_term = Kv * target_rad_s;
  float error = target_rad_s - actual_rad_s;
  float p_term = Kp * error;

  pid.integral += (Ki * error * dt_sec);
  pid.integral = constrain(pid.integral, -255.0f, 255.0f); 
  float i_term = pid.integral;

  float d_term = 0.0;
  if (dt_sec > 0.0) {
    d_term = Kd * ((error - pid.prev_error) / dt_sec);
  }
  pid.prev_error = error;

  float total_output = feed_forward_term + p_term + i_term + d_term;

  int target_sign = (target_rad_s >= 0) ? 1 : -1;
  int unified_output = total_output * target_sign;

  unified_output = constrain(Bv + unified_output, MIN_PWM, PWM_MAX);

  return target_sign * unified_output;
}

// =============================================================================
// QUADRATURE ISR ROUTINES
// =============================================================================
void IRAM_ATTR leftEncoderISR() {
  int valB = digitalRead(LEFT_ENCB_PIN);
  if (valB == LOW) {
    left_wheel_ticks++;
  } else {
    left_wheel_ticks--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  int valB = digitalRead(RIGHT_ENCB_PIN);
  if (valB == LOW) {
    right_wheel_ticks++;
  } else {
    right_wheel_ticks--;
  }
}

void setupEncoders() {
  pinMode(LEFT_ENCA_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCB_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCA_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCB_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA_PIN), rightEncoderISR, RISING);
}

void calculateWheelVelocities(float &left_rad_s, float &right_rad_s) {
  unsigned long now = millis();
  unsigned long dt_ms = now - last_encoder_time;

  if (dt_ms >= CALC_INTERVAL) {
    noInterrupts();
    long current_left_ticks = left_wheel_ticks;
    long current_right_ticks = right_wheel_ticks;
    left_wheel_ticks = 0;
    right_wheel_ticks = 0;
    interrupts();

    float dt_sec = dt_ms / 1000.0;

    float left_revs = (float)current_left_ticks / TICKS_PER_REV;
    float right_revs = (float)current_right_ticks / TICKS_PER_REV;

    left_rad_s = (left_revs * 2.0 * PI) / dt_sec;
    right_rad_s = (right_revs * 2.0 * PI) / dt_sec;

    left_rad_s *= -1.0;
    right_rad_s *= -1.0;

    last_encoder_time = now;
  }
}

void setupJointStatePublisher(rcl_node_t *node, rclc_support_t *support) {
  rclc_publisher_init_default(
    &joint_state_pub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states_raw");

  rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

  joint_state_msg.name.data = name_elements;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.capacity = 2;

  rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);

  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

  joint_state_msg.position.data = position_elements;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;

  joint_state_msg.velocity.data = velocity_elements;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.capacity = 2;
}

void publishJointStates(float left_rad_s, float right_rad_s) {
  unsigned long now = millis();
  joint_state_msg.header.stamp.sec = now / 1000;
  joint_state_msg.header.stamp.nanosec = (now % 1000) * 1000000;

  joint_state_msg.velocity.data[0] = left_rad_s;
  joint_state_msg.velocity.data[1] = right_rad_s;

  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}

void setupMotors() {
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);

  ledcAttach(L298N_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENB, PWM_FREQ, PWM_RES);

  stopMotors();
}

void stopMotors() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
  ledcWrite(L298N_ENA, 0);
  ledcWrite(L298N_ENB, 0);
}

void setLeftMotor(int speed) {
  int pwm_val = constrain(abs(speed), 0, 255);
  ledcWrite(L298N_ENA, pwm_val); 

  if (speed > 0) {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
  }
}

void setRightMotor(int speed) {
  int pwm_val = constrain(abs(speed), 0, 255);
  ledcWrite(L298N_ENB, pwm_val); 

  if (speed > 0) {
    digitalWrite(L298N_IN3, HIGH);
    digitalWrite(L298N_IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, HIGH);
  } else {
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, LOW);
  }
}

void cmd_vel_callback(const void * msin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;

  float linear = msg->linear.x;   
  float angular = msg->angular.z; 

  float left_m_s  = linear - (angular * WHEEL_SEPARATION / 2.0);
  float right_m_s = linear + (angular * WHEEL_SEPARATION / 2.0);

  target_left_rad_s  = left_m_s / WHEEL_RADIUS;
  target_right_rad_s = right_m_s / WHEEL_RADIUS;
}

void arm_callback(const void * msvin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msvin;
  if (msg == NULL || msg->data.data == NULL) return;

  static int current_deg[5] = {0, 0, 0, 0, 0};
  const int MAX_STEP_DEG = 5;

  if (msg->data.size >= 5) {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      ArduinoSerial.print("<");
      for (size_t i = 0; i < 5; i++) {
        int target_deg = (int)(msg->data.data[i] * 180.0 / 3.14159);        
        int delta = target_deg - current_deg[i];
        if (abs(delta) > MAX_STEP_DEG) {
          target_deg = current_deg[i] + (delta > 0 ? MAX_STEP_DEG : -MAX_STEP_DEG);
        }
        current_deg[i] = target_deg;

        ArduinoSerial.print(target_deg);
        if (i < 4) ArduinoSerial.print(",");
      }
      ArduinoSerial.println(">");      
      xSemaphoreGive(serialMutex);
    }
  }
}

void gripper_subscription_callback(const void * msvin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msvin;
  if (msg == NULL) return;

  int deg = (int)(msg->data * 180.0 / 3.14159);

  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    ArduinoSerial.print(">");
    ArduinoSerial.print(deg);
    ArduinoSerial.println(">");
    xSemaphoreGive(serialMutex);
  }
}

// =============================================================================
// TCP SERVER HANDLER FOR ARM COMMANDS (PORT 8890)
// =============================================================================
int serial_tmp_counter = 0;

void handleArmTcpServer() {
  // Check for incoming client connection
  if (armServer.hasClient()) {
    if (!armClient || !armClient.connected()) {
      if (armClient) armClient.stop();
      armClient = armServer.available();
      Serial.println("Arm TCP Client Connected!");
    } else {
      WiFiClient extraClient = armServer.available();
      extraClient.stop(); // Reject additional clients
    }
  }

  // Read line-delimited messages from TCP client
  if (armClient && armClient.connected() && armClient.available()) {
    String payload = armClient.readStringUntil('\n');
    payload.trim();

    if (payload.length() > 0) {
      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ArduinoSerial.println(payload); // Forward string directly to Arduino Uno UART
        Serial.println(serial_tmp_counter++);
        xSemaphoreGive(serialMutex);
      }
    }
  }
}

volatile float latest_actual_left_rad_s = 0.0;
volatile float latest_actual_right_rad_s = 0.0;

void microRosTask(void * pvParameters) {
  unsigned long last_pub_time = 0;
  unsigned long last_ping_time = 0;
  const unsigned long PUB_INTERVAL = 50;   // 20 Hz publish rate
  const unsigned long PING_INTERVAL = 2000; // Ping every 2 seconds
  uint8_t failed_pings = 0;

  state = WAITING_AGENT;

  for(;;) {
    switch (state) {
      case WAITING_AGENT:
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          state = AGENT_AVAILABLE;
        } else {
          vTaskDelay(pdMS_TO_TICKS(500)); 
        }
        break;

      case AGENT_AVAILABLE:
        if (micro_ros_init_materials()) {
          state = AGENT_CONNECTED;
          failed_pings = 0;
          Serial.println("micro-ROS Connected & Synced!");
        } else {
          micro_ros_destroy_materials();
          state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
        // Continuous non-blocking spin for ROS sub/pub
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        if (millis() - last_ping_time >= PING_INTERVAL) {
          last_ping_time = millis();
          if (RMW_RET_OK != rmw_uros_ping_agent(20, 1)) {
            failed_pings++;
          } else {
            failed_pings = 0;
          }

          if (failed_pings >= 10) {
            state = AGENT_DISCONNECTED;
            break;
          }
        }

        if (millis() - last_pub_time >= PUB_INTERVAL) {
          last_pub_time = millis();
          publishJointStates(latest_actual_left_rad_s, latest_actual_right_rad_s);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
        break;

      case AGENT_DISCONNECTED:
        stopMotors();
        micro_ros_destroy_materials();
        state = WAITING_AGENT;
        break;
    }
  }
}

void setup() {
  serialMutex = xSemaphoreCreateMutex();

  Serial.begin(115200);
  ArduinoSerial.begin(115200, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);

  setupMotors();
  setupEncoders();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Wi-Fi connected successfully");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start Arm TCP Server on Port 8890
  armServer.begin();
  armServer.setNoDelay(true); // Disable Nagle's algorithm for low-latency transmission
  Serial.println("Arm TCP Server started on port 8890");

  set_microros_wifi_transports(
    (char*)WIFI_SSID, 
    (char*)WIFI_PASS, 
    agent_ip, 
    agent_port
  );

  // Disable Wi-Fi power save mode for maximum UDP throughput
  WiFi.setSleep(false);

  // Pin micro-ROS task to Core 1 (Core 0 handles Wi-Fi stack)
  xTaskCreatePinnedToCore(
    microRosTask,
    "microRosTask",
    10240,
    NULL,
    10,    // Priority 10
    NULL,
    1     // Core 1
  );
  
  Serial.println("micro-ROS Task started on Core 1.");
}

void loop() {
  // Service incoming TCP socket commands for the arm
  handleArmTcpServer();

  unsigned long current_time = millis();

  if (current_time - last_pid_time >= PID_INTERVAL) {
    float dt_sec = (current_time - last_pid_time) / 1000.0;
    last_pid_time = current_time;
    
    float actual_left_rad_s = 0.0;
    float actual_right_rad_s = 0.0;

    calculateWheelVelocities(actual_left_rad_s, actual_right_rad_s);

    int left_pwm  = computePID(target_left_rad_s, actual_left_rad_s, left_pid, dt_sec, 0);
    int right_pwm = computePID(target_right_rad_s, actual_right_rad_s, right_pid, dt_sec, 1);

    setLeftMotor(left_pwm);
    setRightMotor(right_pwm);
    
    latest_actual_left_rad_s = actual_left_rad_s;
    latest_actual_right_rad_s = actual_right_rad_s;
  }
}
