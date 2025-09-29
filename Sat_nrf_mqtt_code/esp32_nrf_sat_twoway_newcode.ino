#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define CE_PIN 4
#define CSN_PIN 5

// ---------- NRF Pins ----------
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t PIPE_TX = 0xB1B1B1B1B1LL; // ESP32 sends movement here
const uint64_t PIPE_RX = 0xA1A1A1A1A1LL; // ESP32 listens for telemetry
SemaphoreHandle_t radioMutex;

// ---------- WiFi + MQTT Config ----------
const char* WIFI_SSID = "rover";
const char* WIFI_PASS = "123456789";
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* TOPIC_TELEMETRY = "esp32/nrf/telemetry";
const char* TOPIC_MOVEMENT  = "esp32/nrf/movement";
const char* TOPIC_CONTROL   = "esp32/nrf/control"; // subscribed topic

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ---------- Queues ----------
QueueHandle_t telemetryQueue;
QueueHandle_t movementQueue;

// ---------- Data Structs ----------
struct __attribute__((packed)) Telemetry {
  char deviceName[16];
  float temp;
  float humidity;
  uint32_t count;
  uint32_t seq;
};

struct __attribute__((packed)) Movement {
  char device[16];
  bool moving;
  char direction;   // single character
  int16_t speed;
  uint32_t seq;
};

volatile uint32_t txSeq = 0;

// ---------- NRF Send ----------
bool sendNRFMovement(Movement &m) {
  xSemaphoreTake(radioMutex, portMAX_DELAY);
  radio.stopListening();
  bool ok = radio.write(&m, sizeof(m));
  Serial.println("Try to control data....");
  radio.startListening();
  xSemaphoreGive(radioMutex);
  return ok;
}

// ---------- NRF Receive (Telemetry JSON → Queue) ----------
void receiveTelemetry() {
  Telemetry t{};
  while (radio.available()) {
    xSemaphoreTake(radioMutex, portMAX_DELAY);
    radio.read(&t, sizeof(t));
    xSemaphoreGive(radioMutex);

    char jsonPayload[128];
    snprintf(jsonPayload, sizeof(jsonPayload),
             "{\"device\":\"%s\",\"temp\":%.2f,\"humidity\":%.2f,\"count\":%u,\"seq\":%u}",
             t.deviceName, t.temp, t.humidity, t.count, t.seq);

    if (telemetryQueue != NULL) {
      xQueueSend(telemetryQueue, jsonPayload, 0);
    }

    Serial.printf("Telemetry received: %s\n", jsonPayload);
  }
}

// ---------- MQTT Callback ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_CONTROL) != 0) return;

  payload[length] = '\0'; // null-terminate
  Serial.printf("Control message received: %s\n", (char*)payload);

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) {
    Serial.println("Failed to parse JSON");
    return;
  }

  // Build NRF movement struct
  Movement m{};
  strncpy(m.device, "ESP32-B", sizeof(m.device));
  m.moving = doc["moving"] | false;
  m.direction = (doc["direction"] | "S")[0];
  m.speed = doc["speed"] | 0;
  m.seq = ++txSeq;
  xSemaphoreTake(radioMutex, portMAX_DELAY);
  radio.stopListening();
  Serial.println("Try to send control data....");
  bool ok = radio.write(&m, sizeof(m));
  
  radio.startListening();
  xSemaphoreGive(radioMutex);
  if(ok){
    Serial.print("Control data sended...");
  }
  else{
    Serial.print("Control data sending failed...");
  }
  // Send to NRF
  // if (sendNRFMovement(m)) {
  //   Serial.printf("NRF Movement sent: moving=%d dir=%c speed=%d seq=%u\n",
  //                 m.moving, m.direction, m.speed, m.seq);
  // } else {
  //   Serial.println("NRF Movement send FAILED");
  // }

  // Publish same JSON to movement topic
  if (movementQueue != NULL) {
    char jsonPayload[128];
    strncpy(jsonPayload, (char*)payload, length);
    jsonPayload[length] = '\0';
    xQueueSend(movementQueue, jsonPayload, 0);
  }
}

// ---------- Tasks ----------
void recvTask(void *p) {
  for (;;) {
    if (radio.available()) receiveTelemetry();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void mqttTask(void *p) {
  char payload[128];
  for (;;) {
    // Ensure MQTT connection
    if (!mqttClient.connected()) {
      Serial.println("Connecting to MQTT...");
      while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP32-B")) {
          Serial.println("MQTT connected");
          mqttClient.subscribe(TOPIC_CONTROL);
        } else {
          Serial.print("MQTT connect failed, rc=");
          Serial.println(mqttClient.state());
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
    }

    // Publish queued movement (from control topic)
    if (xQueueReceive(movementQueue, &payload, 0)) {
      mqttClient.publish(TOPIC_MOVEMENT, payload);
      Serial.printf("Published to movement topic: %s\n", payload);
    }

    mqttClient.loop();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

  // NRF setup
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (true) delay(1000);
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
  radio.startListening();

  radioMutex = xSemaphoreCreateMutex();
  telemetryQueue = xQueueCreate(10, 128);
  movementQueue  = xQueueCreate(10, 128);

  // WiFi setup
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Tasks
  xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MqttTask", 4096, NULL, 2, NULL, 1);
}

void loop() {}