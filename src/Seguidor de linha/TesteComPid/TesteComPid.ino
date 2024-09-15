#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

// Pinos para os motores
#define AIN1 4    // pin 1 de direção do Motor Esquerdo
#define AIN2 3    // pin 2 de direção do Motor Esquerdo
#define PWMA 2    // pin PWM do Motor Esquerdo

#define BIN1 8    // pin 1 de direção do Motor Direito
#define BIN2 9    // pin 2 de direção do Motor Direito
#define PWMB 11   // pin PWM do Motor Direito

// Limite para considerar a detecção da linha
#define THRESHOLD 250

const int SensorCount = 8;
int sensorValues[SensorCount];

bool isRunning = false; // Variável para armazenar o estado do robô
int base = 0;
float Kprop = 0.3;
float Kderiv = 0.6;
float Kinte = 0.0;
int setpoint = 0;
int last_error = 0;
int pot_limite = 250;
int l_pos;
int pos; // Variável global para armazenar a posição

int v_s_min[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int v_s_max[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int s_p[8];
boolean online;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  // Ligando IR dos sensores
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Ligar para indicar que estamos no modo de calibração

  calibracion();

  digitalWrite(LED_BUILTIN, LOW); // Apagar o LED para indicar que terminamos a calibração

  // Configuração dos pinos do driver de motor
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

void loop() {
  // Lê os dados recebidos do módulo Bluetooth
  if (BTSerial.available()) {
    String bluetoothData = BTSerial.readString();
    Serial.println(bluetoothData);

    if (bluetoothData.indexOf("On") >= 0) {
      base = 175; // Definir uma velocidade base
      isRunning = true;
      digitalWrite(13, HIGH);
    } else if (bluetoothData.indexOf("Off") >= 0) {
      base = 0;
      isRunning = false;
      digitalWrite(13, LOW);
      stop();
    }
  }

  if (isRunning) {
    int line_position = GetPos();
    int Correction_power = PIDLambo(line_position, Kprop, Kderiv, Kinte);
    Motores(base + Correction_power, base - Correction_power);
    Serial.print(line_position);
    Serial.print("\t");
    Serial.println(Correction_power);
  } else {
    stop();
  }

  delay(100); // Pequeno atraso para estabilidade
}

void calibracion() {
  int v_s[8];

  for (int j = 0; j < 100; j++) {
    delay(10);
    for (int i = 0; i < SensorCount; i++) {
      v_s[i] = analogRead(i + A0);
    }

    for (int i = 0; i < SensorCount; i++) {
      Serial.print(v_s[i]);
      Serial.print("\t");
    }
    Serial.println();

    for (int i = 0; i < SensorCount; i++) {
      if (v_s[i] < v_s_min[i]) {
        v_s_min[i] = v_s[i];
      }
    }

    for (int i = 0; i < SensorCount; i++) {
      if (v_s[i] > v_s_max[i]) {
        v_s_max[i] = v_s[i];
      }
    }
  }

  Serial.println();
  Serial.print("Mínimos ");
  Serial.print("\t");

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(v_s_min[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Máximos ");
  Serial.print("\t");

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(v_s_max[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();
  Serial.println();
}

void readSensors() {
  volatile int s[8];

  for (int i = 0; i < SensorCount; i++) {
    s[i] = analogRead(i + A0);
  }

  for (int i = 0; i < SensorCount; i++) {
    if (s[i] < v_s_min[i]) {
      s[i] = v_s_min[i];
    }

    if (s[i] > v_s_max[i]) {
      s[i] = v_s_max[i];
    }
    s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
  }

  volatile int sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    sum += s_p[i];
  }

  if (sum > 100) {
    online = 1;
  } else {
    online = 0;
    sum = 100;
  }

  if (online) {
    for (int i = 0; i < SensorCount; i++) {
      Serial.print(s_p[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

int GetPos() {
  readSensors();
  int prom = -3.5 * s_p[0] - 2.5 * s_p[1] - 1.5 * s_p[2] - 0.5 * s_p[3] + 0.5 * s_p[4] + 1.5 * s_p[5] + 2.5 * s_p[6] + 3.5 * s_p[7];
  int sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    sum += s_p[i];
  }

  if (online) {
    pos = int(100.0 * prom / sum);
  } else {
    if (l_pos < 0) {
      pos = -255;
    }
    if (l_pos >= 0) {
      pos = 255;
    }
  }
  l_pos = pos;
  return pos;
}

int PIDLambo(int POS, float Kp, float Kd, float Ki) {
  int error = pos - setpoint;
  int derivative = error - last_error;
  last_error = error;
  int pot_giro = (error * Kp + derivative * Kd);

  if (pot_giro > pot_limite) {
    pot_giro = pot_limite;
  } else if (pot_giro < -pot_limite) {
    pot_giro = -pot_limite;
  }
  return pot_giro;
}

void MotorIz(int value) {
  if (value >= 0) {
    // Se o valor for positivo, o motor vai para frente
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    // Se o valor for negativo, o motor vai para trás
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }

  // Define a velocidade
  analogWrite(PWMA, value);
}

void MotorDe(int value) {
  if (value >= 0) {
    // Se o valor for positivo, o motor vai para frente
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    // Se o valor for negativo, o motor vai para trás
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }

  // Define a velocidade
  analogWrite(PWMB, value);
}

void Motores(int left, int right) {
  MotorIz(left);
  MotorDe(right);
}

void stop() {
  Motores(0, 0);
}
