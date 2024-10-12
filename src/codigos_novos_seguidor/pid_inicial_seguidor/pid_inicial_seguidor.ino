#include <SoftwareSerial.h>
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

int sensores[8];
int digital[8];
int treshold[8];
int leituraPista[8];
int leituraLinha[8];

// 0: pista branca, linha preta
// 1: pista preta, linha branca 
int linha = 1; 

// Para sensores QTR-8A, que estamos utilizando ://
// Valor baixo: sensor lendo branco //
// Valor alto: sensor lendo preto //

long int somaPonderada, soma, posicao, posicaoLida, ultimaPosicao;

// VALORES PID //
double KP = 1;
double KD = 1.6;
double KI = 0.0;
int erro, ajuste;
double ultimoErro = 0;
const int objetivo = 350;
unsigned char VELOCIDADE_MAXIMA = 120;

int QRE1113_Pin = 12; // Conectado ao pino digital 12
int contadorLinhasBrancas = 0;
bool linhaBrancaAnterior = false;  // Estado inicial, não está sobre uma linha branca
unsigned long tempoInicio = 0;  // Variável para armazenar o tempo inicial
unsigned long tempoDecorrido = 0;

void setup() {
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
  
  bool calibracaoPista = false;
  while (!calibracaoPista) {
    if (bluetoothSerial.available()) {
      String comando = bluetoothSerial.readStringUntil('\n');
      comando.trim();
      if (comando == "pista") {
        digitalWrite(13, HIGH);  // Indica que a calibração da pista está acontecendo
        bluetoothSerial.println("Comecou a calibrar a pista");
        Serial.println("Iniciando calibração da pista...");
        for (int i = 0; i <= 50; i++) {
          calibraPista();
          delay(20);
        }
        calibracaoPista = true;
        digitalWrite(13, LOW);  // Apaga o LED após a calibração
        bluetoothSerial.println("Finalizou a calibracao da pista");
        Serial.println("Calibração da pista concluída.");
      } else {
        Serial.println("Comando não reconhecido, envie 'pista' para calibrar a pista.");
      }
    }
  }

  bool calibracaoLinha = false;
  while (!calibracaoLinha) {
    if (bluetoothSerial.available()) {
      String comando = bluetoothSerial.readStringUntil('\n');
      comando.trim();
      if (comando == "linha") {
        digitalWrite(13, HIGH);  // Indica que a calibração da linha está acontecendo
        bluetoothSerial.println("Comecou a calibrar a linha");
        Serial.println("Iniciando calibração da linha...");
        for (int i = 0; i <= 50; i++) {
          calibraLinha();
          delay(20);
        }
        calibracaoLinha = true;
        digitalWrite(13, LOW);  // Apaga o LED após a calibração
        bluetoothSerial.println("Finalizou a calibracao da linha");
        Serial.println("Calibração da linha concluída.");
      } else {
        Serial.println("Comando não reconhecido, envie 'linha' para calibrar a linha.");
      }
    }
  }

  mediaCalibracao();
  bluetoothSerial.println("Calibração completa! O robô está pronto.");
}

void loop() {// Verifica se há dados recebidos pelo Bluetooth
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
    posicaoLida = leitura();
    Serial.println(posicaoLida);
    
    erro = objetivo - posicao;

    if (abs(erro) >= 200) {
      VELOCIDADE_MAXIMA = 80;
    } else {
      VELOCIDADE_MAXIMA = 120;
    }
    
    ajuste = KP * erro + KD * (erro - ultimoErro);
    ultimoErro = erro;

    if (erro == -350 || erro == 350) {
      freios();
    }else {
      motores(constrain(VELOCIDADE_MAXIMA - ajuste, 0, VELOCIDADE_MAXIMA), constrain(VELOCIDADE_MAXIMA + ajuste, 0, VELOCIDADE_MAXIMA));
    }
    // Leitura do sensor infravermelho
    //int QRE_Value = readQD();
    //Serial.println(QRE_Value);

    //if (QRE_Value < 100) {  // Linha branca detectada (valor baixo)
     // if (!linhaBrancaAnterior) {
        // Se o sensor estava fora da linha e agora detecta uma linha branca, contar uma linha
       // contadorLinhasBrancas++;
        //Serial.print("Contador de Linhas Brancas: ");
        //Serial.println(contadorLinhasBrancas);
        //bluetoothSerial.println(contadorLinhasBrancas);
      //}
      //linhaBrancaAnterior = true;  // Atualizar o estado
    //} else {
     // linhaBrancaAnterior = false;  // Não está detectando linha branca
   // }

    tempoDecorrido = millis() - tempoInicio;
   // if (contadorLinhasBrancas >= 8 && tempoDecorrido >= 39000) {  // 1 segundo = 1000 milissegundos // 6.53s
    //    contadorLinhasBrancas = 0;
     //}

     if (tempoDecorrido >= 42500) {
      running = false;
     }
    
  } else {
    motores(-50, -50);
    delay(30);
    digitalWrite(STDBY, LOW);
  }
}

int leitura(void) {
  for (int i = 0; i < 8; i++) {
    sensores[i] = analogRead(i);
    if (linha == 1) {
      if (sensores[i] < treshold[i]) {
      digital[i] = 1;
      } else {
      digital[i] = 0;
      }  
    } else {
      if (sensores[i] < treshold[i]) {
      digital[i] = 0;
      } else {
      digital[i] = 1;
      }
    }
    
    //Serial.print(digital[i]); 
    //Serial.print(' ');
  }
  //Serial.println(); 

  somaPonderada = ((700 * digital[0]) + (600 * digital[1]) + (500 * digital[2]) + (400 * digital[3]) + (300 * digital[4]) + (200 * digital[5]) + (100 * digital[6]) + (0 + digital[7]));
  soma = (digital [0] + digital [1] + digital [2] + digital [3] + digital [4] + digital [5] + digital [6] + digital [7]);
  posicao = (somaPonderada / soma);
  if (ultimaPosicao <= 100 && posicao == -1) {
    posicao = 0;
  }
  if (ultimaPosicao >= 600 && posicao == -1) {
    posicao = 700;
  }
  ultimaPosicao = posicao;
  return posicao;
}

void calibraPista() {
  for (int i = 0; i < 8; i++) {
    leituraPista[i] = analogRead(i);  
    Serial.print(leituraPista[i]); 
    Serial.print(' ');
  }
  Serial.println(); 
}

void calibraLinha() {
  for (int i = 0; i < 8; i++) {
    leituraLinha[i] = analogRead(i);  
    Serial.print(leituraLinha[i]); 
    Serial.print(' ');
  }
  Serial.println(); 
}

void mediaCalibracao() {
  String linhaCalibracao = "";  // String para armazenar todos os valores

  for (int i = 0; i < 8; i++) {
    treshold[i] = (leituraPista[i] + leituraLinha[i]) / 2;
    
    // Adiciona o valor médio à string
    linhaCalibracao += String(treshold[i]);

    // Adiciona um espaço após cada valor, exceto no último
    if (i < 7) {
      linhaCalibracao += " ";
    }
  }
  
  // Envia a string completa via Serial (para o monitor serial)
  Serial.println(linhaCalibracao);

  // Envia a string completa via Bluetooth
  bluetoothSerial.println(linhaCalibracao);
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
  if (erro == -350) {
      motores(250, -250);
    }
  if (erro == 350) {
    motores(-250, 250);
  }
}

int readQD(){
  // Retorna o valor do QRE1113 
  // Números menores significam mais reflexividade
  // Mais de 3000 significa que nada foi refletido.
  pinMode(QRE1113_Pin, OUTPUT);
  digitalWrite(QRE1113_Pin, HIGH);  
  delayMicroseconds(10);
  pinMode(QRE1113_Pin, INPUT);

  long time = micros();

  // Cronometra quanto tempo o pino fica em HIGH, mas sai após 3ms se nada acontecer
  while (digitalRead(QRE1113_Pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}
