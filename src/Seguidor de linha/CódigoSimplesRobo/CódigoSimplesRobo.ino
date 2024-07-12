#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(3, 2); // RX, TX

// Definições dos pinos dos sensores
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6

// Definições dos pinos do driver de motor
// Motor A
#define AIN1 4
#define AIN2 3
#define PWMA 2

#define STBY 5

// Motor B
#define BIN1 8
#define BIN2 9
#define PWMB 111

// Limite para considerar a detecção da linha
#define THRESHOLD 100

// Velocidade dos motores (0-255)
#define MOTOR_SPEED 100

void setup() {
  Serial.begin(9600);

  BTSerial.begin(9600);

  // Setup Pins as OUTPUT
  pinMode(standBy, OUTPUT);

  // Configuração dos pinos do driver de motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  // Ativa o driver de motor
  digitalWrite(STBY, HIGH);
}

void loop() {

  // Lê os valores dos sensores
  int sensor1 = analogRead(S1);
  int sensor2 = analogRead(S2);
  int sensor3 = analogRead(S3);
  int sensor4 = analogRead(S4);
  int sensor5 = analogRead(S5);
  int sensor6 = analogRead(S6);
  int sensor7 = analogRead(S7);

  // Serial.print(sensor1); Serial.print(" ");
  // Serial.print(sensor2); Serial.print(" ");
  // Serial.print(sensor3); Serial.print(" ");
  // Serial.print(sensor4); Serial.print(" ");
  // Serial.print(sensor5); Serial.print(" ");
  // Serial.print(sensor6); Serial.print(" ");
  // Serial.println(sensor7);

  if (BTSerial.available()){

    BTSerial.print(sensor1); Serial.print(" ");
    BTSerial.print(sensor2); Serial.print(" ");
    BTSerial.print(sensor3); Serial.print(" ");
    BTSerial.print(sensor4); Serial.print(" ");
    BTSerial.print(sensor5); Serial.print(" ");
    BTSerial.print(sensor6); Serial.print(" ");
    BTSerial.println(sensor7);

    String bluetoothData = BTSerial.readString();

    Serial.prinln(bluetoothData);

    if (bluetoothData.indexOf("Off") >= 0) {
      digitalWrite(13, HIGH);
    } else {
      // Determina a direção com base nos sensores
      if (sensor4 > THRESHOLD) {
        // Linha está no meio
        moveForward();
      } else if (sensor3 > THRESHOLD || sensor2 > THRESHOLD) {
        // Linha está à esquerda
        turnLeft();
      } else if (sensor5 > THRESHOLD || sensor6 > THRESHOLD) {
        // Linha está à direita
        turnRight();
      } else {
        // Linha não detectada, parar
        stopMotors();
      }
    }

    delay(100); // Pequeno atraso para estabilidade

  }
  else{
    stop();
  }
}

/*
 * Moviment Functions
 * *****************************************************
 */

void turnLeft(int spd)
{
  runMotor(0, spd, 0);
  runMotor(1, spd, 1);
}

void turnRight(int spd)
{
  runMotor(0, spd, 1);
  runMotor(1, spd, 0);
}

void forward(int spd) 
{
  runMotor(0, spd, 0);
  runMotor(1, spd, 0);
}

void reverse(int spd)
{
  runMotor(0, spd, 1);
  runMotor(1, spd, 1);
}

void runMotor(int motor, int spd, int dir)
{
  digitalWrite(standBy, HIGH); // Turn on Motor

  boolean dirPin1 = LOW;
  boolean dirPin2 = HIGH;

  if(dir == 1) {
    dirPin1 = HIGH;
    dirPin2 = LOW;
  }

  if(motor == 0) { // Motor A
    digitalWrite(AIN1, dirPin1);
    digitalWrite(AIN2, dirPin2);
    analogWrite(PWMA, spd); // Use the speed directly (0-255)
  } else { // Motor B
    digitalWrite(BIN1, dirPin1);
    digitalWrite(BIN2, dirPin2);
    analogWrite(PWMB, spd); // Use the speed directly (0-255)
  }
}

void stop() {
  digitalWrite(standBy, LOW);
}