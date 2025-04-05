#include <Arduino.h>
#include <Wire.h>
#include "DHT20.h"
#include <WiFi.h>             // Thư viện WiFi cho ESP32
#include <PubSubClient.h>     // Thư viện MQTT
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;

// DHT20 Sensor
DHT20 DHT;

// Cấu hình WiFi và ThingsBoard (CoreIoT)
constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";
constexpr char TOKEN[] = "su90rvop2rvzxlxixpxg";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Định nghĩa chân I2C
#define I2C_SDA 21
#define I2C_SCL 22

// Hàm kết nối lại MQTT khi mất kết nối
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Kết nối MQTT...");
    if (mqttClient.connect("ArduinoClient", TOKEN, NULL)) {
      Serial.println("Kết nối thành công!");
      mqttClient.subscribe("v1/devices/me/rpc/request/+");  // Đăng ký lắng nghe RPC
    } else {
      Serial.print("Lỗi kết nối, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - Thử lại sau 5 giây");
      delay(5000);
    }
  }
}


void Task1(void *pvParameters) {
  while (1) {
    Serial.println("Hello from Task1");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void Task2(void *pvParameters) {
  while (1) {
    Serial.println("Hello from Task2");
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}


void Task3(void *pvParameters) {
  while (1) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();

    if (millis() - DHT.lastRead() >= 5000) {  // Đọc mỗi 5 giây
      int status = DHT.read();
      float temperature = DHT.getTemperature();
      float humidity = DHT.getHumidity();

      Serial.print("DHT20 Temperature: ");
      Serial.print(temperature, 1);
      Serial.println(" °C");

      Serial.print("DHT20 Humidity: ");
      Serial.print(humidity, 1);
      Serial.println(" %");

    
      char payload[100];
      snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);

      // Gửi dữ liệu lên CoreIoT
      if (mqttClient.publish("v1/devices/me/telemetry", payload)) {
        Serial.println("Gửi dữ liệu thành công.");
      } else {
        Serial.println("Gửi dữ liệu thất bại.");
      }

      Serial.print("Status: ");
      switch (status) {
        case DHT20_OK: Serial.println("OK"); break;
        case DHT20_ERROR_CHECKSUM: Serial.println("Checksum error"); break;
        case DHT20_ERROR_CONNECT: Serial.println("Connect error"); break;
        case DHT20_MISSING_BYTES: Serial.println("Missing bytes"); break;
        case DHT20_ERROR_BYTES_ALL_ZERO: Serial.println("All bytes read zero"); break;
        case DHT20_ERROR_READ_TIMEOUT: Serial.println("Read time out"); break;
        case DHT20_ERROR_LASTREAD: Serial.println("Read too fast"); break;
        default: Serial.println("Unknown error"); break;
      }
      Serial.println();
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void rpcCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("📩 Nhận yêu cầu RPC từ CoreIoT!");

  // Đọc dữ liệu từ cảm biến ngay lập tức
  int status = DHT.read();
  float temperature = DHT.getTemperature();
  float humidity = DHT.getHumidity();


  char response[100];
  snprintf(response, sizeof(response), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);

  // Gửi phản hồi về CoreIoT
  mqttClient.publish("v1/devices/me/rpc/response/1", response);
  Serial.println(" Đã gửi phản hồi RPC!");
}


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Kết nối WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi đã kết nối!");

  mqttClient.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  mqttClient.setCallback(rpcCallback); 
  // Kết nối MQTT
  reconnectMQTT();


  if (!DHT.begin()) {
    Serial.println(" Khởi tạo cảm biến DHT20 thất bại!");
    while (1);
  }
  Serial.println(" Cảm biến DHT20 đã khởi tạo.");

  // Tạo các task FreeRTOS
  xTaskCreate(Task1, "Task1", 1000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Task2", 1000, NULL, 1, &Task2Handle);
  xTaskCreate(Task3, "DHT20Task", 3000, NULL, 1, &Task3Handle);
}


void loop() {
  mqttClient.loop(); 
}
