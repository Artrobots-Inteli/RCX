#include <SoftwareSerial.h>

// Pinos para o módulo Bluetooth
SoftwareSerial BTSerial(6, 7); // RX, TX

void setup() {
  Serial.begin(9600);
  
  BTSerial.begin(9600);
  
  Serial.println("Módulo Bluetooth HC-05 Inicializado");

  // LED in-built 
  pinMode(13, OUTPUT);
}

void loop() {
  // Envia dados para o celular via Bluetooth
  BTSerial.println("Hello from Arduino Nano via Bluetooth!");
  
  // Verifica se há dados recebidos do celular
  if (BTSerial.available()) {
    // Lê o dado recebido
    String data = BTSerial.readString();
    // Envia o dado para o monitor serial
    Serial.print("Recebido: ");
    Serial.println(data);

    if (data.indexOf("LigarLed") >= 0) {
      digitalWrite(13, HIGH);
    } else if (data.indexOf("DesligarLed") >= 0) {
      digitalWrite(13, LOW);
    }
  }
  
  delay(1000);
}
