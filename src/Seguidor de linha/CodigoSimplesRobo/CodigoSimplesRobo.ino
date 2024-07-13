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

  // Configurar os sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Ligar para indicar que estamos no modo de calibração

  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // Apagar o LED para indicar que terminamos a calibração

  // Printar os valores de calibração mínimos e máximos
  for (int i = 0; i < SensorCount; i++) {
    BTSerial.print(qtr.calibrationOn.minimum[i]);
    BTSerial.print(' ');
  }
  BTSerial.println();

  for (int i = 0; i < SensorCount; i++) {
    BTSerial.print(qtr.calibrationOn.maximum[i]);
    BTSerial.print(' ');
  }
  BTSerial.println();
  BTSerial.println();
  delay(1000);

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

    // Disposição dos sensores: [0 1 2 3 4 5 6 7]
    // Determina a direção com base nos sensores
    if (sensorValues[3] < THRESHOLD || sensorValues[4] < THRESHOLD) {
      // Linha está no meio
      forward(128);
    } else if (sensorValues[0] < THRESHOLD || sensorValues[1] < THRESHOLD || sensorValues[2] < THRESHOLD) {
      // Linha está à esquerda
      turnLeft(128);
    } else if (sensorValues[5] < THRESHOLD || sensorValues[6] < THRESHOLD || sensorValues[7] < THRESHOLD) {
      // Linha está à direita
      turnRight(128);
    } else {
      // Linha não detectada, parar
      stop();
    }
  }

  delay(100); // Pequeno atraso para estabilidade
}

/*
 * Funções de Movimentação
 * *****************************************************
 */

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

void runMotor(int motor, int spd, int dir) {
  digitalWrite(STBY, HIGH); // Ligar motor

  boolean dirPin1 = LOW;
  boolean dirPin2 = HIGH;

  if (dir == 1) {
    dirPin1 = HIGH;
    dirPin2 = LOW;
  }

  if (motor == 0) { // Motor A
    digitalWrite(AIN1, dirPin1);
    digitalWrite(AIN2, dirPin2);
    analogWrite(PWMA, spd); // Usar a velocidade diretamente (0-255)
  } else { // Motor B
    digitalWrite(BIN1, dirPin1);
    digitalWrite(BIN2, dirPin2);
    analogWrite(PWMB, spd); // Usar a velocidade diretamente (0-255)
  }
}

void stop() {
  digitalWrite(STBY, LOW);
}
