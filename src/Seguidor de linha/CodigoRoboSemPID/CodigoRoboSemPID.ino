#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Setup pins
int standBy = 5;

// Motor A
int PWMA = 2;   // PWM Speed Control
int AIN1 = 4;   // Direction pin 1
int AIN2 = 3;   // Direction pin 2

// Motor B
int PWMB = 11;   // PWM Speed Control
int BIN1 = 8;  // Direction pin 1
int BIN2 = 9;  // Direction pin 2

// Sensores
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6

// Limite para considerar a detecção da linha 
// TODO: Calibrar o valor do limiar
#define THRESHOLD 100

// Velocidade dos motores (0-255)
#define MOTOR_SPEED 100

void setup() {
  // Inicializa a comunicação serial para o monitor serial
  Serial.begin(9600);
  
  // Inicializa a comunicação serial para o módulo Bluetooth
  BTSerial.begin(9600);
  
  Serial.println("Módulo Bluetooth HC-05 Inicializado");

  // Setup Pins as OUTPUT
  pinMode(standBy, OUTPUT);

  // Motor A
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Motor B
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Sensores TCRT5000
  pinMode(13, INPUT); // Direita
  pinMode(12, INPUT); // Esquerda
}

void loop() {
  int sensorValues[8];

  // Leitura dos valores dos sensores
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(i);
  }

  // Leitura dos sensores TCRT5000
  int sensorDireita = digitalRead(13);
  int sensorEsquerda = digitalRead(12);

  // Envia dados para o monitor serial
  // Serial.print("Valores dos sensores: ");
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(sensorValues[i]);
  //   Serial.print(" ");
  // }
  // Serial.print(sensorDireita);
  // Serial.print(" ");
  // Serial.print(sensorEsquerda);
  // Serial.print(" ");
  // Serial.println();

  // Envia dados para o celular via Bluetooth
  BTSerial.print("Valores dos sensores: ");
  for (int i = 0; i < 8; i++) {
    BTSerial.print(sensorValues[i]);
    BTSerial.print(" ");
  }
  BTSerial.print(sensorDireita);
  BTSerial.print(" ");
  BTSerial.print(sensorEsquerda);
  BTSerial.print(" ");
  BTSerial.println();

  // Disposição dos sensores: [1 2 3 4 5 6 7 8]
  // Controle do robô baseado nos sensores
  if (sensorValues[4] > THRESHOLD || sensorValues[5] > THRESHOLD) {
    // Linha está no meio
    forward(MOTOR_SPEED);
  } else if (sensorValues[2] > THRESHOLD || sensorValues[3] > THRESHOLD) {
    // Linha está um pouco à direita
    turnRight(70);
  } else if (sensorValues[6] > THRESHOLD || sensorValues[7] > THRESHOLD) {
    // Linha está um pouco à esquerda
    turnLeft(70);
  } else if {sensorValues[1] > THRESHOLD}{
    // Linha está muito à direita
    turnRight(150);
  } else if {sensorValues[8] > THRESHOLD}{
    // Linha está muito à esquerda
    turnLeft(150);
  }
  else {
    // Linha não detectada, parar
    stop();
  }

  delay(200); // Pequeno atraso para estabilidade
}

/*
 * Funções para controle do motor
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

  if(motor == 1) {
    digitalWrite(AIN1, dirPin1);
    digitalWrite(AIN2, dirPin2);
    analogWrite(PWMA, spd);
  } else {
    digitalWrite(BIN1, dirPin1);
    digitalWrite(BIN2, dirPin2);
    analogWrite(PWMB, spd);
  }
}

void stop() {
  digitalWrite(standBy, LOW);
}
