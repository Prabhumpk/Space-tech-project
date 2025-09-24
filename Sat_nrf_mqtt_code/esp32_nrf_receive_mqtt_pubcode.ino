#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Define CE and CSN pins
RF24 radio(4, 5);               // CE, CSN
//| **nRF24L01 Pin**     | **ESP32 Pin (default VSPI)** |
//| -------------------- | ---------------------------- |
//| **GND**              | GND                          |
//| **VCC (3.3V only!)** | 3.3 V (⚠️ not 5 V)           |
//| **CE**               | GPIO 4 (configurable)        |
//| **CSN (CS)**         | GPIO 5 (configurable)        |
//| **SCK**              | GPIO 18 (VSPI SCK)           |
//| **MOSI**             | GPIO 23 (VSPI MOSI)          |
//| **MISO**             | GPIO 19 (VSPI MISO)          |
//| **IRQ** (optional)   | Not connected                |
//

const byte address[6] = "00001";

// Data structure must match the sender
struct DataPacket {
  char device[6];       // MAC address (6 bytes)
  uint32_t count;    // data count (4 bytes)
  float temp;
  float hum;// analog reading
};

DataPacket packet;
// Wi-Fi credentials
const char* ssid = "rover";
const char* password = "123456789";

// MQTT broker settings
const char* mqtt_server = "broker.hivemq.com"; // Replace with your broker IP
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_Receiver")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println("NRF24 not responding!");
    while (1);
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(5, 15);
  radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);

  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("NRF24 Struct Receiver ready...");
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if (radio.available()) {
    // Read the incoming packet
    radio.read(&packet, sizeof(packet));
    StaticJsonDocument<200> doc;
    
    doc["device"] = packet.device;
    doc["count"] = packet.count;
    doc["Temperature"]=packet.temp;
    doc["Humidity"]=packet.hum;
    char payload[200];                              // 32-byte nRF limit
    size_t n = serializeJson(doc, payload, sizeof(payload) - 1);
    Serial.print("RX Count: ");
    Serial.print(packet.count);
    Serial.print(" | Temperature: ");
    Serial.print(packet.temp);
    Serial.print(" | Humidity: ");
    Serial.print(packet.hum);
    Serial.print(" | device: ");
    Serial.print(packet.device);
    client.publish("esp32/nrf/data", payload);
    Serial.println("");
  }
}
