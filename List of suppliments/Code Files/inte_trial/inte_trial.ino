#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "Vrekrer_scpi_parser.h"



#define GPIO_PWM0A_OUT 2  //Set GPIO 15 as PWM0A
#define GPIO_PWM1B_OUT 4  //Set GPIO 16 as PWM0B
#define GPIO_FAN_OUT 13
#define ONE_WIRE_BUS 15


HardwareSerial SerialPort(2);  // UART2: RX=16, TX=17
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
SCPI_Parser my_instrument;


float n;
float tgen, tgen2;
float tset, lastTset, tsetPotentiometer;
float error, P, I;
float Pconst = 0.60;//0.45, 0.65,0.85
float Iconst = 0.0040; //0.0045, 0.0050
float Dconst = 1;
bool state = false;
int FANpwm = 255;
bool setTempMode = 0; //0  .. normal regulation, 1.. direct PWM

unsigned long lastTime = 0;
const unsigned long interval = 1000;


TaskHandle_t Task1;

static void mcpwm_example_gpio_initialize(void) {
  printf("initializing mcpwm gpio...\n");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}


static void peltier_set(float duty_cycle) {
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);

  if (duty_cycle > 0) {
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_TIMER0_SYNC, 10);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
  } else {
    mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_TIMER0_SYNC, 990);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, -(duty_cycle));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
  }
}


void maprange() {
  float in_max = 4;
  float in_min = -in_max;
  float out_max = 98;
  float out_min = -out_max;
  
  float error = (tgen - tset);

  if (error <= in_max && error >= in_min) {
    //n = map(error, -15.0, 15.0, -97.0, 97.0);
    float x;
   P = float(tgen - tset) * Pconst;
  I += float(tgen - tset) * Iconst;
  x = P + I;
  
    n = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  if (error > in_max) {
    n = out_max;
    I = 0;  //clear integration part of regulator - it eliminate wind up effect
  }
  if (error < in_min) {
    n = out_min;
    I = 0;  //clear integration part of regulator - it eliminate wind up effect
  }
  if (error == 0) {
    n = 0;
  }
}

void calculate_pid() {
  P = float(tgen - tset) * Pconst;
  I += float(tgen - tset) * Iconst;
  error = P + I;
  // error = (P*error + I*error );
}


void update_gentemp() {

  float sum = 0;
  float t1 = 0;
  float t2 = 0;
  int roundNo = 1;
  for (int i = 0; i < roundNo; i++) {
    sensors.requestTemperatures();
    float t1 = sensors.getTempCByIndex(0);
    float t2 = sensors.getTempCByIndex(1);
    float t3 = sensors.getTempCByIndex(2);
    float t4 = sensors.getTempCByIndex(3);
    if (t1 != -127)
      tgen = t1;
    //float I = analogRead(A0);
    Serial.print("n:");
    Serial.print(n);
    //I += analogRead(A0);
    Serial.print(" t1:");
    Serial.print(t1);
    //I += analogRead(A0);
    Serial.print(" t2:");
    Serial.print(t2);
    //I += analogRead(A0);
    Serial.print(" t3:");
    Serial.print(t3);
    //I += analogRead(A0);
    Serial.print(" t4:");
    Serial.print(t4);
    //I += analogRead(A0);

     Serial.print(" tset:");
    Serial.print(tset, 2);
   // Serial.print(" I:");
    //Serial.println(I / 4095.0 * 3.3 * 10.0 / 5.0, 3);
    Serial.print(" Preg:");
    Serial.print(P, 4);
     Serial.print(" Ireg:");
    Serial.println(I, 5);
 
  }
}

void Task1code(void* parameter) {
  for (;;) {
    if (SerialPort.available()) {
      tset = SerialPort.parseFloat();
    }


    // Send data back on the same UART port
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
      String tgen_string = String(tgen);
      SerialPort.write("S");
      SerialPort.write(tgen_string.c_str());
      lastTime = currentTime;
    }
  }
}



void PWM_init() {
  //1. mcpwm gpio initialization
  mcpwm_example_gpio_initialize();

  //2. initial mcpwm configuration
  //printf("Configuring Initial Parameters of mcpwm...\n");
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;  //frequency = 500Hz,
  pwm_config.cmpr_a = 0;        //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;        //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  //Configure PWM0A & PWM0B with above settings
  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_TIMER0_SYNC, 0);

  mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_TIMER0_SYNC, 30);
  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
  float dutyC = 1.0;
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyC);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dutyC);
}  // setup_mcpwm



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);
  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1",   /* Name of the task */
    10000,     /* Stack size in words */
    NULL,      /* Task input parameter */
    0,         /* Priority of the task */
    &Task1,    /* Task handle. */
    0);
  PWM_init();
  delay(500);

  my_instrument.RegisterCommand(F("*IDN?"), &Identify);  //identifikace
  my_instrument.SetCommandTreeBase(F("FAN"));
  my_instrument.RegisterCommand(F("PWM"), &FANSetPWM);
  my_instrument.RegisterCommand(F("PWM?"), &FANGetPWM);
  my_instrument.SetCommandTreeBase(F("MEAS"));
  my_instrument.RegisterCommand(F("TEMP#?"), &QueryTemp);
  my_instrument.SetCommandTreeBase(F("REGulator"));
  my_instrument.RegisterCommand(F("TEMP"), &REGsetTemp);
  my_instrument.RegisterCommand(F("TEMP?"), &REGgetTemp);
  my_instrument.RegisterCommand(F("PWM"), &REGsetPWM);


  analogWrite(GPIO_FAN_OUT, FANpwm);
  pinMode(12, INPUT_PULLUP);
  //analogReference(INTERNAL1V1);
}

void loop() {

  if (digitalRead(12) == HIGH) {
    // put your main code here, to run repeatedly:
    update_gentemp();
    //calculate_pid();
    if (setTempMode == 0){
      maprange();
    };
   
    peltier_set(n);


    my_instrument.ProcessInput(Serial, "\n");

    //Serial.print("tset:");
    //Serial.println(tset);
  }
}


//SCPI 
void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("TU Liberec,PAVAN Peltier module rev. 1.0,v0.1.1"));
}

void FANSetPWM(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  // For simplicity no bad parameter check is done.
  if (parameters.Size() > 0) {
    FANpwm = constrain(String(parameters[0]).toInt(), 0, 255);
    analogWrite(GPIO_FAN_OUT, FANpwm);
    interface.print(F("FAN set "));
    interface.println(FANpwm);
  }
}

void FANGetPWM(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(String(FANpwm, DEC));
}

void QueryTemp(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //TEMP<index>?
  //Queries the Temp[index]
  //Examples:
  // TEMP0?    (Queries the setpoint)
  // TEMP1?    (Queries the temp sensor #1)

  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(), "%*[TEMP]%u", &suffix);

  //If the suffix is valid, print the pin's logic state to the interface
  switch (suffix) {
    case 0:
      interface.println(tset);
      break;
    case 1:
      interface.println(tgen);
      break;
      /*case 2:
      interface.println(tgen2);
      break;*/
  }
}

void REGsetTemp(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  // For simplicity no bad parameter check is done.
  if (parameters.Size() > 0) {
    tset = constrain(String(parameters[0]).toFloat(), -5.0, 62.0);
    setTempMode = 1;  //switch to USB control - back by turninf potentiometer
    interface.print(F("REG Tepm setpoint "));
    interface.println(tset);
    setTempMode = 0;
  }
}

void REGsetPWM(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  // For simplicity no bad parameter check is done.
  if (parameters.Size() > 0) {
    n = constrain(String(parameters[0]).toFloat(), -97.0, 97.0);
    interface.print(F("PWM setpoint "));
    interface.println(n);
    setTempMode = 1;
  }
}

void REGgetTemp(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(tset);
}
