#include <OneWire.h>  
#include <DallasTemperature.h>
//The required libraries are included.

const int oneWireBus = 4; //Defining the GPIO pin connected for sensor. 

OneWire oneWire(oneWireBus); //Passing OneWire instance to communicate with existing OneWire devices.

DallasTemperature sensors(&oneWire); //Passing OneWire reference to Dallas Temperature Sensor 

float temperatureC, temperatureF; //Declaring variables


void getTemperature(){ 

  sensors.requestTemperatures(); //To read the updated measured temperature this method is called.

  temperatureC = sensors.getTempCByIndex(0); //To read the measured data by Celsius getTempCBy Index(0) is used.
  temperatureF = sensors.getTempFByIndex(0); //To read the measured data by Fahrenheit  getTempFBy Index(0) is used.

  Serial.print(temperatureC); //Measured data is printed on serial monitor.
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");

}

void setup() {
  
  Serial.begin(9600); //Initializing Serial Monitor to display the measured Data at baud rate 9600.
  sensors.begin(); //Initialize the DS18B20+ sensor.
}

void loop() {
  getTemperature(); //Function to measure temperature and update global variables temperatureC and temperatureF
  delay(2500); //Delayed for 2.5 Second
}