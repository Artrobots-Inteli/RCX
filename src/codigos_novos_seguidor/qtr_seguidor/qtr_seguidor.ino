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

void setup() {
  Serial.begin(9600);
  delay(2000);
  for (int i = 0; i <= 50; i++) {
    calibraPista();
    delay(20);
  }
  delay(2000);
  for (int i = 0; i <= 50; i++) {
    calibraLinha();
    delay(20);
  }
  delay(2000);

  mediaCalibracao();
}

void loop() {
  posicaoLida = leitura();
  Serial.println(posicaoLida);
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
  for (int i = 0; i < 8; i++) {
    treshold[i] = (leituraPista[i] + leituraLinha[i]) / 2;  
    Serial.print(treshold[i]); 
    Serial.print(' ');
  }
  Serial.println(); 
}
