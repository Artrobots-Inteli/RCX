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
bool isRunning = false; // Variável para armazenar o estado do robô
int baseSpeed = 150; // Velocidade base para os motores

float Kp = 1.7;
float Kd = 7.5;
float Ki = 0.0;
int integral = 0;
int setpoint = 3500; // Ponto central ideal para 8 sensores

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  BTSerial.begin(9600);

  // Configuração dos pinos do driver de motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Ativa o driver de motor

  // Configurar os sensores QTR
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(7);

  // Iniciar calibração
  digitalWrite(13, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(13, LOW);
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
    uint16_t position = qtr.readLineWhite(sensorValues);
    int error = setpoint - position;
    integral += error;
    int derivative = error - lastError;
    int correction = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    int motorEsq = baseSpeed + correction;
    int motorDir = baseSpeed - correction;

    runMotor(0, constrain(motorEsq, 128, 255), motorEsq > 0 ? 0 : 1);
    runMotor(1, constrain(motorDir, 128, 255), motorDir > 0 ? 0 : 1);

    Serial.print("Posição: ");
    Serial.print(position);
    Serial.print(" | Correção: ");
    Serial.println(correction);
  } else {
    stop();
  }

  delay(50); // Pequeno atraso para estabilidade
}

void runMotor(int motor, int spd, int dir) {
  digitalWrite(STBY, HIGH); // Ligar motor

  bool dirPin1 = LOW;
  bool dirPin2 = HIGH;

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
