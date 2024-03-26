#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>



#define ONE_WIRE_BUS 15
HardwareSerial SerialPort(2); // RX, TX
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float tgen;
float tset;

unsigned long lastTime = 0;
const unsigned long interval = 1000;


void update_gentemp() {
  sensors.requestTemperatures();
  float t1 = sensors.getTempCByIndex(0);
  float t2 = sensors.getTempCByIndex(1);
  tgen=(t1+t2)/2;
  
}


void setup() {
  Serial.begin(9600);
  SerialPort.begin(9600);
}

void loop() {
  // Listen for incoming data on the HardwareSerial port
  update_gentemp();
  if (SerialPort.available()) {
    tset = SerialPort.parseFloat();
  }

  // Send data back on the same HardwareSerial port
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    String tgen_string = String(tgen);
    SerialPort.write(tgen_string.c_str());
    lastTime = currentTime;
  }

  // Print the values of tgen and tset to the Serial Monitor
  
  Serial.print("tset:");
  Serial.println(tset);
}
