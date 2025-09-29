#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DHT.h"
#include <Wire.h>

#define SLAVE_ADDR 0x08


#define DHTPIN 2     
#define DHTTYPE DHT11
#define CE_PIN 4 
#define CSN_PIN 5

uint32_t counter = 0;
float humidity = 0;
float temperature = 0;

DHT dht(DHTPIN, DHTTYPE);
RF24 radio(CE_PIN, CSN_PIN);

// Unique pipes (must match opposite on Board B)
const uint64_t PIPE_TX = 0xA1A1A1A1A1LL; // where A writes telemetry
const uint64_t PIPE_RX = 0xB1B1B1B1B1LL; // where A listens for movement

SemaphoreHandle_t radioMutex;

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
  char direction;
  int16_t speed;
  uint32_t seq;
};

volatile uint32_t txSeq = 0;

void sendTelemetry() {
  Telemetry t{};
  counter++;

  float h = dht.readHumidity();
  float tC = dht.readTemperature();

  // validate sensor readings
  if (isnan(h) || isnan(tC)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  strncpy(t.deviceName, "ESP32-Rover", sizeof(t.deviceName));
  t.temp = tC;
  t.humidity = h;
  t.seq = ++txSeq;
  t.count = counter;

  xSemaphoreTake(radioMutex, portMAX_DELAY);
  radio.stopListening();
  bool ok = radio.write(&t, sizeof(t));
  Serial.printf("A Telemetry data: %s T=%.2f H=%.2f Count=%u Seq=%u\n",
                  t.deviceName, t.temp, t.humidity, t.count, t.seq);
  radio.startListening();
  xSemaphoreGive(radioMutex);

  if (ok) {
    Serial.print(" |A Telemetry send Success|");
    // Serial.printf("A sent Telemetry: %s T=%.2f H=%.2f Count=%u Seq=%u\n",
    //               t.deviceName, t.temp, t.humidity, t.count, t.seq);
  } else {
    Serial.print("|A Telemetry send FAILED|");
  }
  Serial.println(" ");
}

void receiveMovement() {
  Movement m{};
  while (radio.available()) {
    xSemaphoreTake(radioMutex, portMAX_DELAY);
    radio.read(&m, sizeof(m));
    xSemaphoreGive(radioMutex);

    Serial.printf("A got Movement: %s moving=%d direction=%c speed=%d Seq=%u\n",
                  m.device, m.moving, m.direction, m.speed, m.seq);
    Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(m.direction);       // send "f" string
  Wire.endTransmission();
  }
  Serial.println(" ");
}

void sendTask(void *p) {
  for (;;) {
    sendTelemetry();
    vTaskDelay(pdMS_TO_TICKS(1000)); // send every 1s
   
  }
}

void recvTask(void *p) {
  for (;;) {
    if (radio.available()) receiveMovement();
    vTaskDelay(pdMS_TO_TICKS(5)); // check frequently
  }
}
void setup() {
  Serial.begin(115200);
  dht.begin();
  Wire.begin();        // join I2C bus with address 0x08

  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (true) { delay(1000); }
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
  radio.startListening();

  radioMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(sendTask, "SendTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(recvTask, "RecvTask", 4096, NULL, 2, NULL, 1);
}

void loop() {}

