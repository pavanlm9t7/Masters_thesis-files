#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define GPIO_PWM0A_OUT 2  //Set GPIO 15 as PWM0A
#define GPIO_PWM1B_OUT 4  //Set GPIO 16 as PWM0B


float n;
float tgen, tgen2;
float tset, lastTset;
float error, P, I;
float Pconst = 0.45;
float Iconst = 0.0045;
float Dconst = 1;

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
    
    Serial.print("n:");
    Serial.print(n);
    
    Serial.print(" t1:");
    Serial.print(t1);
    
    Serial.print(" t2:");
    Serial.print(t2);
    
    Serial.print(" t3:");
    Serial.print(t3);
    
    Serial.print(" t4:");
    Serial.print(t4);
    

    Serial.print(" tset:");
    Serial.print(tset, 2);
    Serial.print(" tgen:");
    Serial.print(tgen, 2);
    Serial.print(" Preg:");
    Serial.print(P, 4);
     Serial.print(" Ireg:");
    Serial.println(I, 5);
 
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
  PWM_init();
  delay(500);


}

void loop() {
  // put your main code here, to run repeatedly:
  update_gentemp();
  maprange();
  peltier_set(n);

}
