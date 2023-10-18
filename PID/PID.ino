#include <Arduino.h>

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int maxIntegral = 500; // Define el límite máximo para el valor integral

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {
  // set target position
  int target = 300; // Cambiado de 1200 a 300 para propósitos de este ejemplo

  // PID constants
  float kp = 5;
  float kd = .5;
  float ki = 3;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / (1e6);
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while we read posi
  pos = posi;
  interrupts(); // enable interrupts again
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e - eprev) / deltaT;

  // integral with anti-windup
  eintegral += e * deltaT;
  if (eintegral > maxIntegral) { // Si el valor integral excede el máximo permitido,
    eintegral = maxIntegral; // lo limitamos al máximo.
  } else if (eintegral < -maxIntegral) { // De manera similar, si está por debajo del mínimo,
    eintegral = -maxIntegral; // lo limitamos al mínimo.
  }

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  //Serial.print(" ");
  //Serial.print(e);
  //Serial.print(" ");
  //Serial.print(eintegral);
  //Serial.print(" ");
  //Serial.print(dedt);
  Serial.println();
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}
