#include <SparkFun_TB6612.h>

// Definição dos pinos para o controlador de motor
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// Constantes para a direção dos motores
const int offsetA = 1;
const int offsetB = 1;

// Inicialização dos motores
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
  // Inicialização serial para debug
  Serial.begin(9600);
}

void loop()
{
  // Teste do motor 1
  Serial.println("Motor 1 forward at full speed");
  motor1.drive(255, 2000); // Motor 1 para frente em velocidade máxima por 2 segundos
  Serial.println("Motor 1 backward at full speed");
  motor1.drive(-255, 2000); // Motor 1 para trás em velocidade máxima por 2 segundos
  motor1.brake(); // Freia o motor 1
  delay(1000);

  // Teste do motor 2
  Serial.println("Motor 2 forward at full speed");
  motor2.drive(255, 2000); // Motor 2 para frente em velocidade máxima por 2 segundos
  Serial.println("Motor 2 backward at full speed");
  motor2.drive(-255, 2000); // Motor 2 para trás em velocidade máxima por 2 segundos
  motor2.brake(); // Freia o motor 2
  delay(1000);

  // Movimentação dos dois motores para frente
  Serial.println("Both motors forward");
  forward(motor1, motor2, 150);
  delay(2000);

  // Movimentação dos dois motores para trás
  Serial.println("Both motors backward");
  back(motor1, motor2, 150);
  delay(2000);

  // Freia os dois motores
  Serial.println("Both motors brake");
  brake(motor1, motor2);
  delay(1000);

  // Giro à esquerda
  Serial.println("Turning left");
  left(motor1, motor2, 100);
  delay(2000);

  // Giro à direita
  Serial.println("Turning right");
  right(motor1, motor2, 100);
  delay(2000);

  // Freia os dois motores novamente
  Serial.println("Both motors brake");
  brake(motor1, motor2);
  delay(1000);
}
