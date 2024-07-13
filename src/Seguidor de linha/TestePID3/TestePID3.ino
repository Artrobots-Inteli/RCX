#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Pinos para os motores
#define AIN1 4
#define AIN2 3
#define PWMA 2

#define STBY 5

#define BIN1 8
#define BIN2 9
#define PWMB 11

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int lastError = 0;
boolean onoff = 0;
int val, cnt = 0, v[3];

const uint16_t threshold = 500;
const int maxspeeda = 255;
const int maxspeedb = 255;
const int basespeeda = 165;
const int basespeedb = 165;
const int minspeeda = 150;
const int minspeedb = 150;

float Kp = 0.6; // Ganho proporcional
float Ki = 0.0; // Ganho integral
float Kd = 0.3; // Ganho derivativo
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;

int P;
int I;
int D;
float Pvalue;
float Ivalue;
float Dvalue;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  BTSerial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(7);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  digitalWrite(STBY, HIGH);
  calibration();

  forward_brake(0, 0);
}

void calibration() {
  digitalWrite(13, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(13, LOW);
}

void loop() {
  if (BTSerial.available()) {
    String bluetoothData = BTSerial.readString();
    Serial.println(bluetoothData);

    if (bluetoothData.indexOf("On") >= 0) {
      onoff = 1;
      digitalWrite(13, HIGH);
    } else if (bluetoothData.indexOf("Off") >= 0) {
      onoff = 0;
      digitalWrite(13, LOW);
      stop();
    }
  }

  if (onoff == 1) {
    robot_control();
  } else {
    stop();
  }

  delay(50); // Pequeno atraso para estabilidade
}

void robot_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  PID(error);
}

void PID(int error) {
  int P = error;
  int I = I + error;
  int D = error - lastError;
  lastError = error;

  Pvalue = Kp * P;
  Ivalue = Ki * I;
  Dvalue = Kd * D;

  float motorspeed = Pvalue + Ivalue + Dvalue;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  motorspeeda = constrain(motorspeeda, minspeeda, maxspeeda);
  motorspeedb = constrain(motorspeedb, minspeedb, maxspeedb);

  speedcontrol(motorspeeda, motorspeedb);
}

void speedcontrol(int mota, int motb) {
  if (mota >= 0 && motb >= 0) {
    forward_brake(mota, motb);
  } else if (mota < 0 && motb >= 0) {
    mota = -mota;
    right_brake(mota, motb);
  } else if (mota >= 0 && motb < 0) {
    motb = -motb;
    left_brake(mota, motb);
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, posa);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, posb);
}

void left_brake(int posa, int posb) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, posa);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, posb);
}

void right_brake(int posa, int posb) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, posa);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, posb);
}

void stop() {
  forward_brake(0, 0);
}
