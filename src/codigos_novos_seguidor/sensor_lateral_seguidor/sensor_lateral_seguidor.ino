#define SENSOR_IR 12  // Pino digital onde o sensor infravermelho está conectado

void setup() {
  // Configura o pino do sensor como entrada
  pinMode(SENSOR_IR, INPUT);

  // Inicializa a comunicação serial para monitorar os dados no Serial Monitor
  Serial.begin(9600);
}

void loop() {
  // Lê o valor do sensor infravermelho (HIGH ou LOW)
  int leituraSensor = digitalRead(SENSOR_IR);

  // Exibe o valor lido no Serial Monitor
  if (leituraSensor == HIGH) {
    Serial.println("0");
  } else {
    Serial.println("1");
  }

  // Pequeno atraso para evitar muitas leituras por segundo
  delay(20);
}
