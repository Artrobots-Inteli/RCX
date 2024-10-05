#include <SoftwareSerial.h>

#define STDBY 5

// MOTOR ESQUERDO //
#define PWMA 10
#define AI1  4
#define AI2  3

// MOTOR DIREITO //
#define PWMB 11
#define BI1  8  
#define BI2  9

SoftwareSerial bluetoothSerial(7, 6); // RX, TX (Bluetooth)

// Constantes PID mockadas
float KP = 1.0;
float KI = 0.5;
float KD = 0.1;

String dataReceived = "";
bool running = false;  
int velMotorEsquerdo = 0;
int velMotorDireito = 0;

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(STDBY, OUTPUT);

  digitalWrite(STDBY, HIGH); 

  Serial.println("Aguardando comandos via Bluetooth...");
}

void loop() {
  while (bluetoothSerial.available()) {
    char receivedChar = bluetoothSerial.read();
    dataReceived += receivedChar;

    if (receivedChar == '\n') {
      processBluetoothCommand(dataReceived);  
      dataReceived = "";  
    }
  }

  if (running) {
    velMotorEsquerdo = 255;  
    velMotorDireito = 255;
  } else {
    velMotorEsquerdo = 0;  
    velMotorDireito = 0;
  }
  motores(velMotorEsquerdo, velMotorDireito);
}

void processBluetoothCommand(String command) {
  command.trim();

  char constantType = command.charAt(0);  // O primeiro caractere é o tipo de constante (P, I, D)
  
  if (command.length() > 1) {
    if (constantType == 'P' || constantType == 'I' || constantType == 'D') {
      String valueString = command.substring(1);  // O restante da string é o valor

      float value = valueString.toFloat();

      switch (constantType) {
        case 'P':
          KP = value;
          Serial.print("KP atualizado para: ");
          Serial.println(KP);
          break;
        case 'I':
          KI = value;
          Serial.print("KI atualizado para: ");
          Serial.println(KI);
          break;
        case 'D':
          KD = value;
          Serial.print("KD atualizado para: ");
          Serial.println(KD);
          break;
        default:
          Serial.println("");
          break;
      }
    } else {
      if (command == "start") {
        Serial.println("Iniciou a seguir a linha!!");
        running = true;  // Ativa os motores
      } else if (command == "stop") {
        Serial.println("Parou o robô");
        running = false;  // Para os motores
      } else {
        Serial.println("Comando não reconhecido");
      }
    }
  }
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
