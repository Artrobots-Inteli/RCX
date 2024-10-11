#define SENSOR_IR 12  // Pino digital onde o sensor infravermelho está conectado

void setup() {
  // Configura o pino do sensor como entrada
  pinMode(SENSOR_IR, INPUT);

  // Inicializa a comunicação serial para monitorar os dados no Serial Monitor
  Serial.begin(9600);
}

void loop() {
  
}
