#include <SoftwareSerial.h>

SoftwareSerial SerialPort(7, 8); // RX, TX

float tset = -26.23;
float tgen;

unsigned long lastTime = 0;
const unsigned long interval = 100;

void setup() {
  Serial.begin(9600);
  SerialPort.begin(9600);
}

void loop() {
  // Listen for incoming data on the SoftwareSerial port
  if (SerialPort.available()) {
    tgen = SerialPort.parseFloat();
  }

  // Send data back on the same SoftwareSerial port
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    String tset_string = String(tset);
    SerialPort.write(tset_string.c_str());
    lastTime = currentTime;
  }

  // Print the values of tgen and tset to the Serial Monitor
  Serial.print("tgen: ");
  Serial.println(tgen);
  
  
}
