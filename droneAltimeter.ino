#include "Adafruit_MPL3115A2.h"
//Write
#define PWM_PIN 4
//Read
#define PWM_READ 3
#define PIN_AUX 10  //Change this later
//PID
#define KP 10.0
#define KI 0
#define KD 0
float sensor, prop, integral, deriv, lastE = 0;

const int LOOP_TIME = 20000; // 50 HZ

uint32_t timer;
bool controlActive = false;
float targetAlt = 0;
int pwm_out;

int pwm_in = 1109;
int pwm_in_timer=0;
int pwm_in_state=0;
Adafruit_MPL3115A2 barom;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_READ, INPUT);
  pinMode(PIN_AUX, INPUT);
 

  Serial.begin(9600);
  timer = millis();

}

void loop() {

  //Print Sensor Data
  //Serial.println("Altitude:");
  //Serial.println(barom.getAltitude());
  //Start Control Loop
  if(digitalRead(PIN_AUX)){
    sensor = barom.getAltitude();
    //Set Target
    if(controlActive==0)
      {controlActive=1; targetAlt = sensor;}
    prop = targetAlt - sensor;
    integral += prop;
    deriv = lastE - prop;
    pwm_out = pwm_out+(KP*prop+KI*integral+KD*deriv);
    if (pwm_out>2000) pwm_out = 2000;
    if(pwm_out<1109) pwm_out = 1109;
    lastE = prop;
  }
  //Pass through throttle data
  else{
    pwm_out=pwm_in;
  }
  //PWM Write
  //Serial.println("Power out:");
  //Serial.println(pwm_out);
//PWM OUT
  while(micros()< timer + LOOP_TIME - pwm_out){
    if(digitalRead(PWM_READ)&&pwm_in_state==0) pwm_in_state=1; pwm_in_timer=micros(); 
    if(digitalRead(PWM_READ)==0&&pwm_in_state==1) pwm_in_state=0; pwm_in = micros()-pwm_in_timer;
  }
  digitalWrite(PWM_PIN, HIGH);
  timer=micros();
  while(micros() < timer + pwm_out){
    if(digitalRead(PWM_READ)&&pwm_in_state==0) pwm_in_state=1; pwm_in_timer=micros(); 
    if(digitalRead(PWM_READ)==0&&pwm_in_state==1) pwm_in_state=0; pwm_in = micros()-pwm_in_timer;
  }
  digitalWrite(PWM_PIN, LOW);

}
