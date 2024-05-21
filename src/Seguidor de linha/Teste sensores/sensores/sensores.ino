#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensor1 = analogRead(S1);
  int sensor2 = analogRead(S2);
  int sensor3 = analogRead(S3);
  int sensor4 = analogRead(S4);
  int sensor5 = analogRead(S5);
  int sensor6 = analogRead(S6);
  int sensor7 = analogRead(S7);

  Serial.print("S1: "); Serial.print(sensor1);
  Serial.print(" S2: "); Serial.print(sensor2);
  Serial.print(" S3: "); Serial.print(sensor3);
  Serial.print(" S4: "); Serial.print(sensor4);
  Serial.print(" S5: "); Serial.print(sensor5);
  Serial.print(" S6: "); Serial.print(sensor6);
  Serial.print(" S7: "); Serial.println(sensor7);
  
  delay(1000);
}
