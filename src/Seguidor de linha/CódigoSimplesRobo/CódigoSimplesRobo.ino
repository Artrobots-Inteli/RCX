// Definições dos pinos dos sensores
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6

// Definições dos pinos do driver de motor
#define AIN1 10
#define AIN2 11
#define STBY 7
#define BIN1 5
#define BIN2 6

// Limite para considerar a detecção da linha
#define THRESHOLD 100

// Velocidade dos motores (0-255)
#define MOTOR_SPEED 100

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(9600);

  // Configuração dos pinos do driver de motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
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

  // Exibe os valores dos sensores na Serial Monitor
  Serial.print(sensor1); Serial.print(" ");
  Serial.print(sensor2); Serial.print(" ");
  Serial.print(sensor3); Serial.print(" ");
  Serial.print(sensor4); Serial.print(" ");
  Serial.print(sensor5); Serial.print(" ");
  Serial.print(sensor6); Serial.print(" ");
  Serial.println(sensor7);

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

  delay(100); // Pequeno atraso para estabilidade
}

void moveForward() {
  analogWrite(AIN1, MOTOR_SPEED);
  digitalWrite(AIN2, LOW);
  analogWrite(BIN1, MOTOR_SPEED);
  digitalWrite(BIN2, LOW);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  analogWrite(AIN2, MOTOR_SPEED);
  analogWrite(BIN1, MOTOR_SPEED);
  digitalWrite(BIN2, LOW);
}

void turnRight() {
  analogWrite(AIN1, MOTOR_SPEED);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  analogWrite(BIN2, MOTOR_SPEED);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
