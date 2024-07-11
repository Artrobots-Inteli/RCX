#include <SparkFun_TB6612.h>

// Definição dos pinos para o controlador de motor
#define AIN1 4
#define BIN1 8
#define AIN2 3
#define BIN2 9
#define PWMA 2
#define PWMB 11
#define STBY 5

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
  // Movimentação dos dois motores para frente
  Serial.println("Both motors forward");
  forward(motor1, motor2, 150);
  delay(1000);

  // Freia os dois motores novamente
  Serial.println("Both motors brake");
  brake(motor1, motor2);
  delay(10000);
}
