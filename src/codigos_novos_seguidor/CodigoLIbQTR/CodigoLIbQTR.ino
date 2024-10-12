#include <SoftwareSerial.h>
#include <QTRSensors.h>  // Inclui a biblioteca dos sensores
SoftwareSerial bluetoothSerial(7, 6); // RX, TX (Bluetooth)
String dataReceived = "";
bool running = false;

#define STDBY 5

// MOTOR ESQUERDO //
#define PWMA 10
#define AI1  4
#define AI2  3

// MOTOR DIREITO //
#define PWMB 11
#define BI1  8  
#define BI2  9

// Sensores
#define NUM_SENSORS 8
unsigned char sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
// QTRSensors qtra(sensorPins, NUM_SENSORS)
QTRSensors qtra;


unsigned int sensorValues[NUM_SENSORS];

// VALORES PID //
double KP = 1;
double KD = 1.6;
double KI = 0.0;
int erro;
double ajuste;
double ultimoErro = 0;
double erroAcumulado = 0;
const int objetivo = 3500;  // Atualizado para refletir a posição central
unsigned char VELOCIDADE_MAXIMA = 120;

int QRE1113_Pin = 12; // Conectado ao pino digital 12
int contadorLinhasBrancas = 0;
bool linhaBrancaAnterior = false;  // Estado inicial, não está sobre uma linha branca
unsigned long tempoInicio = 0;  // Variável para armazenar o tempo inicial
unsigned long tempoDecorrido = 0;

void setup() {

  qtra.setTypeRC();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);

  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  Serial.println("Aguardando comandos via Bluetooth...");
  pinMode(13, OUTPUT);
  pinMode(12, INPUT);
  
  // Pinos de controle de direção dos motores //
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);

  pinMode(STDBY, OUTPUT);

  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  digitalWrite(STDBY, HIGH);
  
  // Calibração dos sensores
  digitalWrite(13, HIGH);  // Indica que a calibração está acontecendo
  bluetoothSerial.println("Iniciando calibração dos sensores...");
  for (int i = 0; i < 400; i++) {  // Calibra por aproximadamente 4 segundos
    qtra.calibrate();
    delay(10);
  }
  digitalWrite(13, LOW);  // Apaga o LED após a calibração
  bluetoothSerial.println("Calibração dos sensores concluída.");

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
   for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibrationOn.minimum[i]);
     Serial.print(' ');
  }
  Serial.println();

  // print the calibration   maximum values measured when emitters were on
  for (uint8_t i = 0; i < NUM_SENSORS;   i++)
  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print('   ');
  }
  Serial.println();
  Serial.println();

  bluetoothSerial.println("Calibração completa! O robô está pronto.");
}

void loop() {
  // Verifica se há dados recebidos pelo Bluetooth
  while (bluetoothSerial.available()) {
    char receivedChar = bluetoothSerial.read();
    dataReceived += receivedChar;

    if (receivedChar == '\n') {
      processBluetoothCommand(dataReceived);  
      dataReceived = "";  
    }
  }

  if (running) {
    if (tempoInicio == 0) {
      tempoInicio = millis();  // Armazena o tempo em que `running` começou
    }
    
    digitalWrite(STDBY, HIGH);

    // Leitura dos sensores e cálculo da posição
    unsigned int posicao = qtra.readLineWhite(sensorValues);
    Serial.println(posicao);
    
    erro = objetivo - posicao;

    ajuste = KP * erro + KD * (erro - ultimoErro) + KI * erroAcumulado;
    ultimoErro = erro;
    erroAcumulado += erro;

    // Controle dos motores
    int velocidadeEsquerda = VELOCIDADE_MAXIMA + ajuste;
    int velocidadeDireita = VELOCIDADE_MAXIMA - ajuste;
    motores(constrain(velocidadeEsquerda, 0, 255), constrain(velocidadeDireita, 0, 255));

    tempoDecorrido = millis() - tempoInicio;

    if (tempoDecorrido >= 42500) {
      running = false;
    }
    
  } else {
    motores(-50, -50);
    delay(30);
    digitalWrite(STDBY, LOW);
  }
}

void motores(int velMotorEsquerdo, int velMotorDireito) {
  if (velMotorEsquerdo >= 0) {
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
  } else {
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    velMotorEsquerdo = -velMotorEsquerdo;
  }

  analogWrite(PWMA, velMotorEsquerdo);

  if (velMotorDireito >= 0) {
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
  } else {
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    velMotorDireito = -velMotorDireito;
  }

  analogWrite(PWMB, velMotorDireito);
}

void processBluetoothCommand(String command) {
  command.trim();

  char constantType = command.charAt(0);  // O primeiro caractere é o tipo de constante (P, I, D)
  
  if (command.length() > 1) {
    if (constantType == 'P' || constantType == 'I' || constantType == 'D' || constantType == 'V') {
      String valueString = command.substring(1);  // O restante da string é o valor
      String message = "KP = " + String(KP) + ", KD = " + String(KD) + ", KI = " + String(KI);
      float value = valueString.toFloat();
      switch (constantType) {
        case 'P':
          KP = value;
          Serial.print("KP atualizado para: ");
          message = "KP = " + String(KP) + ", KD = " + String(KD) + ", KI = " + String(KI);
          bluetoothSerial.println(message);
          Serial.println(KP);
          break;
        case 'I':
          KI = value;
          Serial.print("KI atualizado para: ");
          message = "KP = " + String(KP) + ", KD = " + String(KD) + ", KI = " + String(KI);
          bluetoothSerial.println(message);
          Serial.println(KI);
          break;
        case 'D':
          KD = value;
          Serial.print("KD atualizado para: ");
          message = "KP = " + String(KP) + ", KD = " + String(KD) + ", KI = " + String(KI);
          bluetoothSerial.println(message);
          Serial.println(KD);
          break;
        case 'V':
          VELOCIDADE_MAXIMA = value;
          break;
        default:
          Serial.println("");
          break;
      }
    } else {
      if (command == "start") {
        Serial.println("Iniciou a seguir a linha!!");
        contadorLinhasBrancas = 0;
        tempoInicio = millis();
        running = true;  // Ativa os motores
      } else if (command == "stop") {
        Serial.println("Parou o robô");
        running = false;  // Para os motores
        bluetoothSerial.println(tempoDecorrido);
      } else {
        Serial.println("Comando não reconhecido");
      }
    }
  }
}

void freios() {
  if (erro <= -3500) {
    motores(250, -250);
  }
  if (erro >= 3500) {
    motores(-250, 250);
  }
}
