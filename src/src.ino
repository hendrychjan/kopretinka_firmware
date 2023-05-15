#include <EEPROM.h>
#include <SoftwareSerial.h>

#include "text.cpp"

namespace config {
// ======= Pin configuration ========================================
const uint8_t SS_RX_PIN = 2;                   // Arduino software serial RX pin
const uint8_t SS_TX_PIN = 3;                   // Arduino software serial TX pin
const uint8_t MUL_SEL_PINS[] = {10, 9, 8, 7};  // Multiplexer control pins
const uint8_t EN_MUL_WP_PIN = 11;              // Water pump mux enable pin
const uint8_t EN_MUL_SENS_PIN = 12;            // Sensors multiplexer enable pin
const uint8_t EN_VCC_SENS_PIN = 6;  // Sensors power supply enable pin
const uint8_t VAL_SENS_PIN = A0;    // Analog input pin for sensors
const uint8_t VAL_WP_PIN = 13;      // Global water pump enable pin

// Hardcoded configuration - remove this later
// const char* WIFI_SSID = "";
// const char* WIFI_PASS = "";
// const char* SERVER_IP = "";
// const char* SERVER_PORT = "";
const uint8_t ACTIVE_PORTS[] = {13, 14, 15};
const uint16_t UPDATE_INTERVAL = 10000;
// ==================================================================
}  // namespace config

struct Unit {
  bool enabled;
  bool autoMoisture;
  uint8_t targetMoisture;
};

enum MuxTarget { SENSORS, WATER_PUMPS, DISABLE };
enum OperationMode { CONFIG, RUNNING, ERROR };

void ard_run();
void ard_initialize();
void ard_pinModeSetup();
void ard_logicSetup();
void ard_configure();
void ard_initializePorts();
void ard_handleUnit(uint8_t unitNumber);

template <uint16_t S>
void esp_send(const Text<S>& text);

void mux_selectPin(uint8_t pinNumber);
void mux_enableTarget(MuxTarget target);
uint8_t mux_readSensor(uint8_t pinNumber);
void mux_decode(uint8_t target, uint8_t* channel);

Unit units[16];
OperationMode operationMode = CONFIG;
SoftwareSerial espSerial(config::SS_RX_PIN, config::SS_TX_PIN);

void setup() {
  ard_initialize();

  ard_pinModeSetup();
  ard_logicSetup();

  delay(1000);

  mux_enableTarget(MuxTarget::DISABLE);
  mux_enableTarget(MuxTarget::WATER_PUMPS);
}

void loop() {
  switch (operationMode) {
    case OperationMode::CONFIG:
      ard_configure();
      operationMode = OperationMode::RUNNING;
      break;
    case OperationMode::RUNNING:
      ard_run();
      break;
    case OperationMode::ERROR:
      break;
  }
}

// ==== Arduino ====
void ard_initialize() { Serial.begin(115200); }

void ard_pinModeSetup() {
  // Multiplexer control pins
  pinMode(config::EN_MUL_WP_PIN, OUTPUT);
  pinMode(config::EN_MUL_SENS_PIN, OUTPUT);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(config::MUL_SEL_PINS[i], OUTPUT);
  }
  // Multiplexer data pins
  pinMode(config::VAL_WP_PIN, OUTPUT);
  pinMode(config::VAL_SENS_PIN, INPUT);

  // Sensors power supply enable pin
  pinMode(config::EN_VCC_SENS_PIN, OUTPUT);
}

void ard_logicSetup() {
  // Multiplexer control pins
  digitalWrite(config::EN_MUL_WP_PIN, HIGH);
  digitalWrite(config::EN_MUL_SENS_PIN, HIGH);
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(config::MUL_SEL_PINS[i], LOW);
  }

  // Sensors power supply enable pin
  digitalWrite(config::EN_VCC_SENS_PIN, LOW);
}

void ard_configure() {
  Serial.println("[INFO] Entered the configuration mode.");

  ard_initializePorts();

  Serial.println("[INFO] Module has been configured.");
}

void ard_initializePorts() {
  Serial.println("[INFO] Initializing ports.");

  for (uint8_t i : config::ACTIVE_PORTS) {
    units[i].enabled = true;
    units[i].targetMoisture = mux_readSensor(i);
    units[i].autoMoisture = true;
    Serial.print("> ...port ");
    Serial.print(i);
    Serial.print(" initialized with threshold: ");
    Serial.print(units[i].targetMoisture);
    Serial.println(" %");
  }

  Serial.println("[INFO] Ports have been initialized.");
}

void ard_run() {
  Serial.println("[INFO] Starting the unit check procedure.");
  Serial.println("> ...checking the moisture levels...");
  for (uint8_t i = 0; i < 16; i++) {
    if (!units[i].enabled) continue;

    Serial.print("[INFO] Handling unit ");
    Serial.println(i);
    ard_handleUnit(i);
    delay(config::UPDATE_INTERVAL);
  }
  Serial.println("[INFO] Unit check procedure finished.");
}

void ard_handleUnit(uint8_t unitNumber) {
  const Unit& unit = units[unitNumber];

  // Enable the water pump and the sensor multiplexer and the sensors power
  // supply
  digitalWrite(config::EN_MUL_WP_PIN, LOW);
  digitalWrite(config::EN_MUL_SENS_PIN, LOW);

  // Select the channel
  mux_selectPin(unitNumber);

  // Read the value from the sensor and print it
  uint8_t sensorValue = mux_readSensor(unitNumber);
  Serial.print("> value is ");
  Serial.print(sensorValue);
  Serial.println("%.");

  // Send the value to the server
  esp_send(Text<32>(
      ("[DATA]:" + String(unitNumber) + ":" + String(sensorValue)).c_str()));

  // If moisture is too low, turn on the water pump
  if (unit.autoMoisture && sensorValue < unit.targetMoisture) {
    Serial.println("> ...turning on the water pump...");
    digitalWrite(config::VAL_WP_PIN, HIGH);
    while (sensorValue < unit.targetMoisture) {
      sensorValue = mux_readSensor(unitNumber);
      Serial.print("> now ");
      Serial.print(sensorValue);
      Serial.println("%.");
      delay(100);
    }
    Serial.println("> ...turning off the water pump...");
    digitalWrite(config::VAL_WP_PIN, LOW);
    esp_send(Text<32>(
        ("[DATA]:" + String(unitNumber) + ":" + String(sensorValue)).c_str()));
  }


  // Cleanup, disable the water pump and the sensor multiplexer and the sensors
  // power supply
  digitalWrite(config::EN_MUL_WP_PIN, HIGH);
  digitalWrite(config::EN_MUL_SENS_PIN, HIGH);
  digitalWrite(config::EN_VCC_SENS_PIN, LOW);
}

// ==== ESP ====
template <uint16_t S>
void esp_send(const Text<S>& text) {
  delay(100);
  Serial.println(text.c_str());
  delay(100);
  Serial.flush();
}

// ==== Multiplexor ====
uint8_t mux_readSensor(uint8_t pinNumber) {
  // Enable the sensors power supply
  digitalWrite(config::EN_VCC_SENS_PIN, HIGH);
  digitalWrite(config::EN_MUL_SENS_PIN, LOW);

  // Select the desired pin on the multiplexer
  mux_selectPin(pinNumber);

  // Read the value
  uint16_t value = analogRead(config::VAL_SENS_PIN);
  uint8_t percentage = map(value, 0, 1023, 100, 0);

  // Disable the sensors power supply
  digitalWrite(config::EN_MUL_SENS_PIN, HIGH);
  digitalWrite(config::EN_VCC_SENS_PIN, LOW);

  return percentage;
}

void mux_selectPin(uint8_t pinNumber) {
  uint8_t channel[4];
  mux_decode(pinNumber, channel);

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(config::MUL_SEL_PINS[i], channel[i]);
  }
}

void mux_enableTarget(MuxTarget target) {
  switch (target) {
    case MuxTarget::SENSORS:
      digitalWrite(config::EN_MUL_SENS_PIN, LOW);
      digitalWrite(config::EN_MUL_WP_PIN, HIGH);
      break;
    case MuxTarget::WATER_PUMPS:
      digitalWrite(config::EN_MUL_SENS_PIN, HIGH);
      digitalWrite(config::EN_MUL_WP_PIN, LOW);
      break;
    case MuxTarget::DISABLE:
      digitalWrite(config::EN_MUL_SENS_PIN, HIGH);
      digitalWrite(config::EN_MUL_WP_PIN, HIGH);
      break;
  }
}

void mux_decode(uint8_t target, uint8_t* channel) {
  uint8_t code[16][4] = {
      {0, 0, 0, 0},  // channel 0
      {1, 0, 0, 0},  // channel 1
      {0, 1, 0, 0},  // channel 2
      {1, 1, 0, 0},  // channel 3
      {0, 0, 1, 0},  // channel 4
      {1, 0, 1, 0},  // channel 5
      {0, 1, 1, 0},  // channel 6
      {1, 1, 1, 0},  // channel 7
      {0, 0, 0, 1},  // channel 8
      {1, 0, 0, 1},  // channel 9
      {0, 1, 0, 1},  // channel 10
      {1, 1, 0, 1},  // channel 11
      {0, 0, 1, 1},  // channel 12
      {1, 0, 1, 1},  // channel 13
      {0, 1, 1, 1},  // channel 14
      {1, 1, 1, 1}   // channel 15
  };

  channel[0] = code[target][0];
  channel[1] = code[target][1];
  channel[2] = code[target][2];
  channel[3] = code[target][3];
}
