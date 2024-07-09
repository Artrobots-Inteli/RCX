#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

void setup() {
  // Inicializa a comunicação serial para o monitor serial
  Serial.begin(9600);
  
  // Inicializa a comunicação serial para o módulo Bluetooth
  BTSerial.begin(9600);
  
  Serial.println("Módulo Bluetooth HC-05 Inicializado");

  // Sensores TCRT5000 da direita e esquerda
  pinMode(13, INPUT); // Direita
  pinMode(12, INPUT); // Esquerda
}

void loop() {
  int sensorValues[8];

  // Leitura dos valores dos sensores
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(i);
  }

  // Leitura dos sensores TCRT5000
  int sensorDireita = digitalRead(13);
  int sensorEsquerda = digitalRead(12);

  // Envia dados para o monitor serial
  Serial.print("Valores dos sensores: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.print(sensorDireita);
  Serial.print(" ");
  Serial.print(sensorEsquerda);
  Serial.print(" ");
  Serial.println();

  // Envia dados para o celular via Bluetooth
  BTSerial.print("Valores dos sensores: ");
  for (int i = 0; i < 8; i++) {
    BTSerial.print(sensorValues[i]);
    BTSerial.print(" ");
  }
  BTSerial.print(sensorDireita);
  BTSerial.print(" ");
  BTSerial.print(sensorEsquerda);
  BTSerial.print(" ");
  BTSerial.println();
  
  // Verifica se há dados recebidos do celular
  if (BTSerial.available()) {
    String receivedData = BTSerial.readString();
    Serial.print("Dados recebidos do celular: ");
    Serial.println(receivedData);
  }
  
  delay(200);
}
