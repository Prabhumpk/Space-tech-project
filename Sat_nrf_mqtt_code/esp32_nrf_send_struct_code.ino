#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DHT.h"

#define DHTPIN 2     
#define DHTTYPE DHT11
// #include <WiFi.h>  // Not needed since you're using a name instead
DHT dht(DHTPIN, DHTTYPE);
RF24 radio(4, 5); 
//| **nRF24L01 Pin**     | **ESP32 Pin (default VSPI)** |
//| -------------------- | ---------------------------- |
//| **GND**              | GND                          |
//| **VCC (3.3V only!)** | 3.3 V (⚠️ not 5 V)           |
//| **CE**               | GPIO 4 (configurable)        |
//| **CSN (CS)**         | GPIO 5 (configurable)        |
//| **SCK**              | GPIO 18 (VSPI SCK)           |
//| **MOSI**             | GPIO 23 (VSPI MOSI)          |
//| **MISO**             | GPIO 19 (VSPI MISO)          |
//| **IRQ** (optional)   | Not connected 

const byte address[6] = "00001";
uint32_t counter = 0;

       // ADC1 channel, change as needed

// Data structure to send
struct DataPacket {
  char device[6];      // Device name (6 bytes)
  uint32_t count;      // Data count (4 bytes)
  float temp;
  float hum;
};

DataPacket packet;

void setup() {
  Serial.begin(115200);
  dht.begin();
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

  radio.openWritingPipe(address);
  radio.stopListening();



  Serial.println("NRF24 Struct Sender ready...");
}

void loop() {
  counter++;
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Copy "Rover" into packet.device safely
  strncpy(packet.device, "Rover", sizeof(packet.device));
  
  packet.count = counter;
  packet.temp = t;
  packet.hum = h;

  // Send the packet
  bool ok = radio.write(&packet, sizeof(packet));

  Serial.print("TX ");
  Serial.print(ok ? "OK" : "FAIL");
  Serial.print(" | Count: ");
  Serial.print(packet.count);
  Serial.print(" | temperature: ");
  Serial.print(packet.temp);
  Serial.print(" | Hummidity: ");
  Serial.print(packet.hum);
  Serial.print(" | Device: ");
  Serial.println(packet.device);

  delay(2000);
}
