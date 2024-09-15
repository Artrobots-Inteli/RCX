#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Pinos para os sensores
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6
#define S8 A7

// Pinos para os motores
#define AIN1 4
#define AIN2 3
#define PWMA 2

#define STBY 5

#define BIN1 8
#define BIN2 9
#define PWMB 11

int sensores[8];
int digital[8];
int leituraFundo[8];
int leituraLinha[8];
int linha = 1; // 1 = linha branca; 0 = linha preta
int limiar[8]; // utilizado para calibração
long int somap, soma, pos, posicao, lastPos; // cálculo da posição do robô

bool isRunning = false; // Variável para armazenar o estado do robô
int baseSpeed = 60; // Velocidade base para os motores

float Kp = 1.8;
float Kd = 8.2;
float Ki = 0.0;
int last_error = 0;
int integral = 0;
int setpoint = 350; // Ponto central ideal para 8 sensores

void setup() {
  pinMode(13, OUTPUT);
  
  // Define todas as portas dos sensores como INPUT
  for (int i = 0; i < 8; i++) {
    pinMode(A0 + i, INPUT);
  }
  Serial.begin(9600);
  BTSerial.begin(9600);

  // Inicia calibração
  delay(1000);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 150; i++) {
    calibraFundo();
  }
  digitalWrite(13, LOW);
  delay(2000);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 150; i++) {
    calibraLinha();
  }
  digitalWrite(13, LOW);
  valorMedioCalibracao();

  // Configuração dos pinos do driver de motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Ativa o driver de motor
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
    posicao = leitura();
    int error = setpoint - posicao;
    integral += error;
    int derivative = error - last_error;
    int correction = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;

    int motorEsq = baseSpeed + correction;
    int motorDir = baseSpeed - correction;

    motorEsq = constrain(motorEsq, 128, 255);
    motorDir = constrain(motorDir, 128, 255);

    runMotor(0, motorEsq, motorEsq > 0 ? 0 : 1);
    runMotor(1, motorDir, motorDir > 0 ? 0 : 1);

    BTSerial.print("Posição: ");
    BTSerial.print(posicao);
    BTSerial.print(" | Correção: ");
    BTSerial.println(correction);
  } else {
    stop();
  }

  delay(50); // Pequeno atraso para estabilidade
}

// Funções de calibração
void calibraFundo() {
  for (int i = 7; i >= 0; i--) {
    leituraFundo[i] = analogRead(A0 + i);
    Serial.print(leituraFundo[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void calibraLinha() {
  for (int i = 7; i >= 0; i--) {
    leituraLinha[i] = analogRead(A0 + i);
    Serial.print(leituraLinha[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void valorMedioCalibracao() {
  for (int i = 7; i >= 0; i--) {
    limiar[i] = (leituraLinha[i] + leituraFundo[i]) / 2;
    Serial.print(limiar[i]);
    Serial.print("\t");
  }
  Serial.println();
}

int leitura() {
  somap = 0;
  soma = 0;

  // Lê cada sensor e armazena no array
  for (int i = 7; i >= 0; i--) {
    sensores[i] = analogRead(A0 + i);
    if (linha == 0) {
      if (sensores[i] <= limiar[i]) {
        digital[i] = 0;
      } else {
        digital[i] = 1;
      }
    }
    if (linha == 1) {
      if (sensores[i] <= limiar[i]) {
        digital[i] = 1;
      } else {
        digital[i] = 0;
      }
    }
  }

  // Calcula a posição do robô
  somap = (700 * digital[0]) + (600 * digital[1]) + (500 * digital[2]) + (400 * digital[3]) + (300 * digital[4]) + (200 * digital[5]) + (100 * digital[6]) + (0 * digital[7]);
  soma = digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7];

  if (soma > 0) {
    pos = somap / soma;
  } else {
    // Se a linha não for detectada, retorna a última posição válida
    if (lastPos <= 100) {
      pos = 0;
    } else if (lastPos >= 600) {
      pos = 700;
    } else {
      pos = lastPos;
    }
  }

  lastPos = pos;
  return pos;
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
