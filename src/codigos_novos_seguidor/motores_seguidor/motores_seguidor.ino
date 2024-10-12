#define STDBY 5

// MOTOR ESQUERDO //
#define PWMA 10
#define AI1  4
#define AI2  3

// MOTOR DIREITO //
#define PWMB 11
#define BI1  8  
#define BI2  9


void setup() {
  // Pinos de controle de direção dos motores //
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);

  pinMode(STDBY, OUTPUT);

  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  digitalWrite(STDBY, HIGH);
}

void loop() {
  motores(180, 180);
  delay(50);
}


void motores(int velMotorEsquerdo, int velMotorDireito) {
  if (velMotorEsquerdo >= 0) {
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
  } else {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    velMotorEsquerdo = velMotorEsquerdo * (-1);
  }

  analogWrite(PWMA, velMotorEsquerdo);

  if (velMotorDireito >= 0) {
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
  } else {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    velMotorDireito = velMotorDireito * (-1);
  }

  analogWrite(PWMB, velMotorDireito);

  
}
