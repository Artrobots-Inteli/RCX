void setup() {

  Serial.begin(9600);
  pinMode(12, INPUT);


}

void loop() {
  Serial.println(digitalRead(12));

  delay(1000);

}
