#include <QTRSensors.h>
#include <SoftwareSerial.h>  // Biblioteca para comunicação serial via Bluetooth

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Propriedades do sensor de linha
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             QTR_NO_EMITTER_PIN

// Ordem dos sensores (ajuste conforme necessário)
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Propriedades do driver de motor
#define AIN1 4
#define AIN2 3
#define PWMA 2
#define BIN1 8
#define BIN2 9
#define PWMB 11
#define STBY 5

// Propriedades do PID
const float Kp = 1.0;
const float Kd = 0.0;
const float Ki = 0.0;
int lastError = 0;
int integral = 0;
const int GOAL = 3500;
const int BASE_SPEED = 200;
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;

// Tipo de linha: false para linha preta, true para linha branca
const bool whiteLine = false;

void setup() {
  Serial.begin(9600);       // Inicializa a comunicação serial via USB
  BTSerial.begin(9600);     // Inicializa a comunicação serial via Bluetooth

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  calibrateLineSensor();
}

void loop() {
  unsigned int position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, whiteLine);

  int error = position - GOAL;

  integral += error;
  int derivative = error - lastError;
  int correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  int motorEsq = BASE_SPEED + correction;
  int motorDir = BASE_SPEED - correction;

  motorEsq = constrain(motorEsq, MIN_SPEED, MAX_SPEED);
  motorDir = constrain(motorDir, MIN_SPEED, MAX_SPEED);

  runMotor(0, motorEsq, 0);
  runMotor(1, motorDir, 0);

  // Mensagens de depuração via Serial (USB)
  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Correction: ");
  Serial.println(correction);
  Serial.print("Motor Left Speed: ");
  Serial.println(motorEsq);
  Serial.print("Motor Right Speed: ");
  Serial.println(motorDir);
  Serial.println("-----------------------------");

  // Envia dados via Bluetooth
  BTSerial.print("Position: ");
  BTSerial.println(position);
  BTSerial.print("Error: ");
  BTSerial.println(error);
  BTSerial.print("Correction: ");
  BTSerial.println(correction);
  BTSerial.print("Motor Left Speed: ");
  BTSerial.println(motorEsq);
  BTSerial.print("Motor Right Speed: ");
  BTSerial.println(motorDir);
  BTSerial.println("-----------------------------");

  delay(50); // Pequeno atraso para estabilidade
}

void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 400; i++) {
    qtra.calibrate();
  }
  digitalWrite(13, LOW);
}

/*
 * Funções de Movimentação
 * *****************************************************
 */

void runMotor(int motor, int spd, int dir) {
  digitalWrite(STBY, HIGH);

  bool dirPin1 = LOW;
  bool dirPin2 = HIGH;

  if (dir == 1) {
    dirPin1 = HIGH;
    dirPin2 = LOW;
  }

  if (motor == 0) { // Motor A (esquerdo)
    digitalWrite(AIN1, dirPin1);
    digitalWrite(AIN2, dirPin2);
    analogWrite(PWMA, spd);
  } else { // Motor B (direito)
    digitalWrite(BIN1, dirPin1);
    digitalWrite(BIN2, dirPin2);
    analogWrite(PWMB, spd);
  }
}

void turnLeft(int spd) {
  runMotor(0, spd, 0);
  runMotor(1, spd, 1);
}

void turnRight(int spd) {
  runMotor(0, spd, 1);
  runMotor(1, spd, 0);
}

void forward(int spd) {
  runMotor(0, spd, 0);
  runMotor(1, spd, 0);
}

void reverse(int spd) {
  runMotor(0, spd, 1);
  runMotor(1, spd, 1);
}

void stopMotors() {
  digitalWrite(STBY, LOW);
}
