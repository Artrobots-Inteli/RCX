#include <SoftwareSerial.h>

// Definir os pinos para a comunicação serial com o módulo Bluetooth
SoftwareSerial bluetoothSerial(7, 6); // RX, TX (Bluetooth)

// Constantes PID mockadas
float KP = 1.0;
float KI = 0.5;
float KD = 0.1;


String dataReceived = "";

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);

  Serial.println("Aguardando comandos via Bluetooth...");
}

void loop() {
  while (bluetoothSerial.available()) {
    char receivedChar = bluetoothSerial.read();
    dataReceived += receivedChar;

    if (receivedChar == '\n') {
      processBluetoothCommand(dataReceived);  // Função para processar o comando recebido
      dataReceived = "";  
    }
  }
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
      Serial.println(command);
      if (command == "start") {
        Serial.println("Iniciou a seguir a linha!!");
      } else if (command == "stop") {
        Serial.println("Parou o robô");
      } else {
        Serial.println("Comando não reconhecido");
      }
    }
  }
}
