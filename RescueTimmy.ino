#include <SPI.h>         // SPI library
#include <MFRC522.h>     // RFID reader library
#include <QTRSensors.h>  // QTR Sensor Library
#include <SD.h>          //   SD Library

QTRSensors qtr;

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

//Ultrosonic Sensor
const int trigPin = A4;
const int echoPin = A5;
unsigned long delay_time;
bool timmy = false;

float duration, distance;

//PID
float output;
unsigned long p_time;
bool turn = LOW;
unsigned long prev_time;

//P
bool state_ahead = HIGH;
bool state_back = LOW;
bool state_ahead_2 = HIGH;
bool state_back_2 = LOW;
int target_val = 1500;
int actual_val;
float error;
float p_error;
float p;
float Kp_scale = 0.1;
float m1, m2;
//I
unsigned long dT;
float i;
float Ki_scale = 0.025;
//D
float d;
float Kd_scale = 0.25;

//Left Motor
const int In1 = 7;
const int In2 = 8;
const int EnA = 6;

//Right Motor
const int In3 = 2;
const int In4 = 4;
const int EnB = 5;

unsigned long motor_delay;
unsigned long pm_time;

int delay_motor;

//SD Card
File myFile;
const int CSPin = 10;
bool sd_card;
String dance;
char moves[64];

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  SPI.begin();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3 }, SensorCount);

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  //Motor Left
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(EnA, OUTPUT);

  //Motor Right
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(EnB, OUTPUT);

  pinMode(CSPin, OUTPUT);
  myFile = SD.open("dance.txt");

  if (myFile) {
    if (myFile.available()) {
      sd_card = true;
      dance = myFile.read();
      dance.toCharArray(moves, dance.length());
      myFile.close();
    } 
  }
}

void loop() {
  if(sd_card == false){
    distance_timmy();

    if (distance < 30){
      timmy = true;
      if (millis() - pm_time > 10) {
        pm_time = millis();
        analogWrite(EnA, 100);
        analogWrite(EnB, 100);

        digitalWrite(In1, state_ahead);
        digitalWrite(In2, state_back);
        digitalWrite(In3, state_ahead_2);
        digitalWrite(In4, state_back_2);
      }

      if (millis() - pm_time > 20) {
        pm_time = millis();
        digitalWrite(In1, LOW);
        digitalWrite(In2, LOW);
        digitalWrite(In3, LOW);
        digitalWrite(In4, LOW);
      }
    }
    if (timmy == false){
      analogArray();
      motor();
      PID();
    }   
  }
  else {
    for (int i =0; i < dance.length(); i++){
      if(moves[i] == 'L'){
        left();
      }
      if(moves[i] == 'R'){
        right();
      }
      if(moves[i] == 'F'){
        front();
      }
      if(moves[i] == 'B'){
        back();
      }
    }
  }
}

void distance_timmy() {
  if(millis() - delay_time > 100){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);
    delay_time = millis();
  }
}

void analogArray() {
  // line position
  uint16_t position = qtr.readLineBlack(sensorValues);
  actual_val = position;
  Serial.println(position);
}

void PID() {
  p_error = error;

  error = target_val - actual_val;
  error = map(error, -1500, 1500, -75, 75);
  dT = millis() - prev_time;
  p = Kp_scale * error;
  i = Ki_scale * (error * dT) + i;
  i = constrain(i, -15, 15);
  d = Kd_scale * ((error - p_error) / (dT));
  output = p + i + d;

if(error < -4){
    m1 = 110;
    state_ahead = LOW;
    state_back = HIGH;
    m2 = 110;
  }
  else if(error > 4){
    m1 = 110;
    m2 = 110;
    state_ahead_2 = LOW;
    state_back_2 = HIGH;
  }
  else{
    m1 = 110 + output;
    m2 = 130 - output;
    state_ahead = HIGH;
    state_back = LOW;
    state_ahead_2 = HIGH;
    state_back_2 = LOW;
  }
  prev_time = millis();
}

void motor() {
  if (millis() - pm_time > 10) {
    pm_time = millis();
    analogWrite(EnA, m1);
    analogWrite(EnB, m2);

    digitalWrite(In1, state_ahead);
    digitalWrite(In2, state_back);
    digitalWrite(In3, state_ahead_2);
    digitalWrite(In4, state_back_2);
  }

  if (millis() - pm_time > 10) {
    pm_time = millis();
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, LOW);
  }
}

void left() {
  analogWrite(EnA, 150);
  analogWrite(EnB, 150);

  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(200);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(200);
}

void right() {
  analogWrite(EnA, 150);
  analogWrite(EnB, 150);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  delay(290);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(200);
}

void front() {
  analogWrite(EnA, 150);
  analogWrite(EnB, 150);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(200);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(200);
}

void back() {
  analogWrite(EnA, 150);
  analogWrite(EnB, 150);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(200);

  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
  delay(200);
}
