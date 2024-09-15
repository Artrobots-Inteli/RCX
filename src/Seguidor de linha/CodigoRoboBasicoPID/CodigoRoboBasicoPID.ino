#include <QTRSensors.h>

// Propriedades do sensor de linha
#define NUM_SENSORS             8  // número de sensores usados
#define NUM_SAMPLES_PER_SENSOR  4  // média de 4 amostras por leitura de sensor
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // pino do emissor (não usado neste caso)

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
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 3500;  // Meta central do sensor de linha
const unsigned char BASE_SPEED = 80;  // Velocidade base
const unsigned char MAX_SPEED = 255;  // Velocidade máxima do PWM (0-255)

void setup() {
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // Ativa o driver de motor

  // Configura os pinos do driver de motor como saídas
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  // Inicializa o array de sensores de linha
  calibrateLineSensor();
}

void loop() {
  // Obtém a posição da linha
  unsigned int position = qtra.readLine(sensorValues);

  // Calcula o erro em relação ao centro da linha
  int error = GOAL - position;

  // Calcula o ajuste dos motores com base no erro e no PID
  int adjustment = KP * error + KD * (error - lastError);

  // Armazena o erro para o próximo cálculo
  lastError = error;

  // Calcula as velocidades dos motores
  int motorASpeed = BASE_SPEED - adjustment;
  int motorBSpeed = BASE_SPEED + adjustment;

  // Limita as velocidades dos motores
  motorASpeed = constrain(motorASpeed, -MAX_SPEED, MAX_SPEED);
  motorBSpeed = constrain(motorBSpeed, -MAX_SPEED, MAX_SPEED);

  // Ajusta a potência dos motores usando as funções de movimentação
  runMotor(0, abs(motorASpeed), motorASpeed >= 0 ? 0 : 1);
  runMotor(1, abs(motorBSpeed), motorBSpeed >= 0 ? 0 : 1);
}

// Função para calibrar os sensores de linha
void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  // Liga o LED do Arduino para indicar que está em modo de calibração
  for (int i = 0; i < 400; i++) {  // A calibração dura cerca de 10 segundos
    qtra.calibrate();  // Lê todos os sensores várias vezes para calibrar
  }
  digitalWrite(13, LOW);  // Desliga o LED do Arduino para indicar o fim da calibração
}

/*
 * Funções de Movimentação
 * *****************************************************
 */

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
