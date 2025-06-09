#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Ustaw te dane na swoje!
#define WIFI_SSID     "RC2024"
#define WIFI_PASSWORD "robochallenge"
#define AGENT_IP      "192.168.1.249"
#define AGENT_PORT    8888

#define LED_PIN 14 // Możesz zmienić na dowolny dostępny pin

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void error_loop(const char* msg) {
  Serial.println(msg);
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {Serial.print("Error in line: "); Serial.println(__LINE__); error_loop("RCL error!");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {Serial.print("Warning in line: "); Serial.println(__LINE__);}}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n\n=== ESP32 micro-ROS WiFi Publisher Debug ===");

  // 1. Połączenie z WiFi
  Serial.print("Łączenie z WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED && tryCount < 30) { // 30 prób x 500ms = 15s
    delay(500);
    Serial.print(".");
    tryCount++;
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    error_loop("Nie udało się połączyć z WiFi!");
  } else {
    Serial.print("Połączono! IP: ");
    Serial.println(WiFi.localIP());
  }

  // 2. Połączenie z agentem micro-ROS
  Serial.print("Ustawianie transportu micro-ROS (WiFi) na IP agenta: ");
  Serial.print(AGENT_IP); Serial.print(":"); Serial.println(AGENT_PORT);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  // 3. Inicjalizacja micro-ROS
  allocator = rcl_get_default_allocator();

  Serial.println("Inicjalizacja rclc_support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  Serial.println("Tworzenie node...");
  RCCHECK(rclc_node_init_default(&node, "esp32_publisher_node", "", &support));

  Serial.println("Tworzenie publishera...");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "test_topic"
  ));

  msg.data = 0;

  Serial.println("Publisher gotowy, zaczynamy pętlę główną!");
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  msg.data++;
  Serial.print("Publikuję na /test_topic: ");
  Serial.println(msg.data);

  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.print("Błąd publikacji (kod: ");
    Serial.print(ret);
    Serial.println(")");
  }

  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // miga diodą
  delay(1000); // co sekundę
}