#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h> // Dla M_PI, cos, sin

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/header.h>

// Definicje pinów enkoderów
const uint8_t enkL_A_pin = 2;
const uint8_t enkL_B_pin = 4;  // Zmieniono z 3 na 4, aby uniknąć konfliktu z Serial RX na ESP32
const uint8_t enkP_A_pin = 18;
const uint8_t enkP_B_pin = 19;

// Parametry robota
const float promien_kola = 0.045;
const float rozstaw_kol = 0.37;
const float impulsy_na_obrot = 40000.0;
const float metry_na_impuls = (2.0 * M_PI * promien_kola) / impulsy_na_obrot;

volatile long int impulsy_enkL = 0;
volatile long int impulsy_enkP = 0;

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

long int poprzednie_impulsy_L = 0;
long int poprzednie_impulsy_R = 0;

rcl_publisher_t odometry_publisher;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

const unsigned long timer_period_ms = 100;

char odom_frame_buffer[10];
char base_link_frame_buffer[10];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.print("Failed status on line "); Serial.print(__LINE__); Serial.print(" ("); Serial.print(rcl_get_error_string().str); Serial.println(")"); while(1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.print("Failed status on line "); Serial.print(__LINE__); Serial.print(" ("); Serial.print(rcl_get_error_string().str); Serial.println(")");}}

void enkL_A_ISR() {
  if (digitalRead(enkL_A_pin) == digitalRead(enkL_B_pin)) {
    impulsy_enkL++;
  } else {
    impulsy_enkL--;
  }
}

void enkL_B_ISR() {
  if (digitalRead(enkL_A_pin) != digitalRead(enkL_B_pin)) {
    impulsy_enkL++;
  } else {
    impulsy_enkL--;
  }
}

void enkP_A_ISR() {
  if (digitalRead(enkP_A_pin) == digitalRead(enkP_B_pin)) {
    impulsy_enkP++;
  } else {
    impulsy_enkP--;
  }
}

void enkP_B_ISR() {
  if (digitalRead(enkP_A_pin) != digitalRead(enkP_B_pin)) {
    impulsy_enkP++;
  } else {
    impulsy_enkP--;
  }
}

void yaw_to_quaternion(double yaw, geometry_msgs__msg__Quaternion *q) {
  q->x = 0.0;
  q->y = 0.0;
  q->z = sin(yaw / 2.0);
  q->w = cos(yaw / 2.0);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCL_UNUSED(last_call_time);

  if (timer != NULL) {
    noInterrupts();
    long int aktualne_impulsy_L = impulsy_enkL;
    long int aktualne_impulsy_R = impulsy_enkP;
    interrupts();

    double delta_L = (double)(aktualne_impulsy_L - poprzednie_impulsy_L);
    double delta_R = (double)(aktualne_impulsy_R - poprzednie_impulsy_R);

    poprzednie_impulsy_L = aktualne_impulsy_L;
    poprzednie_impulsy_R = aktualne_impulsy_R;

    double dystans_L = delta_L * metry_na_impuls;
    double dystans_R = delta_R * metry_na_impuls;

    double delta_dystans = (dystans_L + dystans_R) / 2.0;
    double delta_theta = (dystans_R - dystans_L) / rozstaw_kol;

    x_pos += delta_dystans * cos(theta + delta_theta / 2.0);
    y_pos += delta_dystans * sin(theta + delta_theta / 2.0);
    theta += delta_theta;

    while (theta > M_PI) theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;

    double dt_sec = (double)timer_period_ms / 1000.0;
    double v_x = (dt_sec > 0) ? (delta_dystans / dt_sec) : 0.0;
    double v_theta = (dt_sec > 0) ? (delta_theta / dt_sec) : 0.0;

    // --- POPRAWKA TUTAJ ---
    rcl_time_point_value_t current_time_ns;
    RCCHECK(rcl_clock_get_now(&support.clock, &current_time_ns)); // Przekazanie wskaźnika do current_time_ns
    // --- KONIEC POPRAWKI ---

    odom_msg.header.stamp.sec = RCL_NS_TO_S(current_time_ns);
    odom_msg.header.stamp.nanosec = current_time_ns % RCL_S_TO_NS(1);

    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;

    yaw_to_quaternion(theta, &odom_msg.pose.pose.orientation);

    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
     // Przykładowe kowariancje (odkomentuj i dostosuj w razie potrzeby)
    // odom_msg.pose.covariance[0] = 0.01;  // x
    // odom_msg.pose.covariance[7] = 0.01;  // y
    // odom_msg.pose.covariance[35] = 0.01; // yaw (indeks dla yaw w kowariancji orientacji)
    // odom_msg.twist.covariance[0] = 0.01; // vx
    // odom_msg.twist.covariance[35] = 0.01;// vth (indeks dla yaw rate w kowariancji twist)


    odom_msg.twist.twist.linear.x = v_x;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = v_theta;

    RCSOFTCHECK(rcl_publish(&odometry_publisher, &odom_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);

  // --- POPRAWKA TUTAJ ---
  set_microros_transports(); // Użyj bardziej ogólnej funkcji
  // --- KONIEC POPRAWKI ---
  // delay(2000); // Daj czas na połączenie agenta, jeśli potrzebne

  pinMode(enkL_A_pin, INPUT_PULLUP);
  pinMode(enkL_B_pin, INPUT_PULLUP);
  pinMode(enkP_A_pin, INPUT_PULLUP);
  pinMode(enkP_B_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enkL_A_pin), enkL_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enkL_B_pin), enkL_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enkP_A_pin), enkP_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enkP_B_pin), enkP_B_ISR, CHANGE);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "odometry_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &odometry_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  strcpy(odom_frame_buffer, "odom");
  odom_msg.header.frame_id.data = odom_frame_buffer;
  odom_msg.header.frame_id.size = strlen(odom_frame_buffer);
  odom_msg.header.frame_id.capacity = sizeof(odom_frame_buffer);

  strcpy(base_link_frame_buffer, "base_link");
  odom_msg.child_frame_id.data = base_link_frame_buffer;
  odom_msg.child_frame_id.size = strlen(base_link_frame_buffer);
  odom_msg.child_frame_id.capacity = sizeof(base_link_frame_buffer);

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period_ms),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  noInterrupts();
  poprzednie_impulsy_L = impulsy_enkL;
  poprzednie_impulsy_R = impulsy_enkP;
  interrupts();

  Serial.println("Setup complete. micro-ROS Odometry Publisher started.");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10); // Zmniejszone opóźnienie, aby częściej kręcić egzekutorem, jeśli timer_period_ms jest mały
}