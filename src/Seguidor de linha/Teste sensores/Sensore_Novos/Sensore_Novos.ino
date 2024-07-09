  void setup() {
    Serial.begin(9600);
    
    // Configura o pino de controle IR como saída (opcional)
    // pinMode(2, OUTPUT); // IR LED control (opcional)
    
    // // Ativa os LEDs IR
    // digitalWrite(2, HIGH);
  }

  void loop() {
    int sensorValues[8];

    // Lê os valores dos sensores analógicos
    sensorValues[0] = analogRead(A0);
    sensorValues[1] = analogRead(A1);
    sensorValues[2] = analogRead(A2);
    sensorValues[3] = analogRead(A3);
    sensorValues[4] = analogRead(A4);
    sensorValues[5] = analogRead(A5);
    sensorValues[6] = analogRead(A6);
    sensorValues[7] = analogRead(A7);

    // Imprime os valores no monitor serial
    for (int i = 0; i < 8; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.println();

    delay(100); // Pequeno atraso para leitura estável
  }
