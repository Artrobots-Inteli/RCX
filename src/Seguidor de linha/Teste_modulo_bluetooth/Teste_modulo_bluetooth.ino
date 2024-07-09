#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(2, 3); // RX, TX

void setup() {
  // Inicializa a comunicação serial para o monitor serial
  Serial.begin(9600);
  
  // Inicializa a comunicação serial para o módulo Bluetooth
  BTSerial.begin(9600);
  
  Serial.println("Módulo Bluetooth HC-05 Inicializado");
}

void loop() {
  // Envia dados para o celular via Bluetooth
  BTSerial.println("Hello from Arduino Nano via Bluetooth!");
  
  // Verifica se há dados recebidos do celular
  if (BTSerial.available()) {
    // Lê o dado recebido
    char data = BTSerial.read();
    // Envia o dado para o monitor serial
    Serial.print("Recebido: ");
    Serial.println(data);
  }
  
  // Pequeno atraso para evitar sobrecarregar a comunicação
  delay(1000);
}
