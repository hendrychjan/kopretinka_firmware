#include "SoftwareSerial.h"

SoftwareSerial espSerial(2, 3);

void setup() {
  Serial.begin(115200);
  espSerial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    // Read data from the serial monitor (PC)
    char data = Serial.read();
    // Send data to the ESP8266
    espSerial.print(data);
  }

  if (espSerial.available()) {
    // Read data from the ESP8266
    char data = espSerial.read();
    // Print the received data
    Serial.print(data);
  }
}

// void esp_connectTCP() {
//   espSerial.print("AT+CIPSTART=0,\"TCP\",");
//   espSerial.print(config::SERVER_IP);
//   espSerial.print(",");
//   espSerial.print(config::SERVER_PORT);
//   espSerial.print("\r\n");
//   delay(1000);
// }