#include <QTRSensors.h>
#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Definições dos pinos do driver de motor
// Motor A
#define AIN1 4
#define AIN2 3
#define PWMA 2

#define STBY 5

// Motor B
#define BIN1 8
#define BIN2 9
#define PWMB 11

// Limite para considerar a detecção da linha
#define THRESHOLD 250

// Sensores
QTRSensors qtr;

const int SensorCount = 8;
int sensorValues[SensorCount];

bool isRunning = false; // Variável para armazenar o estado do robô

void setup() {
  Serial.begin(9600);

  BTSerial.begin(9600);

  // Ligando IR dos sensores
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Ligar para indicar que estamos no modo de calibração

  for (int i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // Printar os valores de calibração minimos e máximos
  for (int i = 0; i < SensorCount; i++)
  {
    BTSerial.print(qtr.calibrationOn.minimum[i]);
    BTSerial.print(' ');
  }
  BTSerial.println();

  for (int i = 0; i < SensorCount; i++)
  {
    BTSerial.print(qtr.calibrationOn.maximum[i]);
    BTSerial.print(' ');
  }
  BTSerial.println();
  BTSerial.println();
  delay(1000);

  // Setup Pins as OUTPUT
  pinMode(STBY, OUTPUT);

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
  // Lê os dados recebidos do módulo Bluetooth
  if (BTSerial.available()) {
    String bluetoothData = BTSerial.readString();
    Serial.println(bluetoothData);

    if (bluetoothData.indexOf("On") >= 0) {
      isRunning = true;
      digitalWrite(13, HIGH);
    } else if (bluetoothData.indexOf("Off") >= 0) {
      isRunning = false;
      digitalWrite(13, LOW);
      stop();
    }
  }

  if (isRunning) {
    int position = qtr.readLineWhite(sensorValues);

    for (int i = 0; i < SensorCount; i++) {
      BTSerial.print(sensorValues[i]);
      BTSerial.print('\t');
    }
    BTSerial.println(position);

    // Disposição dos sensores: [1 2 3 4 5 6 7 8]
    // Determina a direção com base nos sensores
    if (sensorValues[4] < THRESHOLD || sensorValues[5] < THRESHOLD) {
      // Linha está no meio
      forward(128);
    } else if (sensorValues[3] < THRESHOLD || sensorValues[2] < THRESHOLD) {
      // Linha está à esquerda
      turnRight(128);
    } else if (sensorValues[6] < THRESHOLD || sensorValues[7] < THRESHOLD) {
      // Linha está à direita
      turnLeft(128);
    } else if (sensorValues[1] < THRESHOLD){
      // Linha está muito à direita
      turnRight(150);
    } else if (sensorValues[8] < THRESHOLD){
      // Linha está muito à esquerda
      turnLeft(150);
    } else {
      // Linha não detectada, parar
      stop();
    }
  }
  else{
    stop();
  }

  delay(100); // Pequeno atraso para estabilidade
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
  digitalWrite(STBY, HIGH); // Turn on Motor

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
  digitalWrite(STBY, LOW);
}
