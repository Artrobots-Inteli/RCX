// Setup pins
int standBy = 5;

// Motor A
int PWMA = 2;   // PWM Speed Control
int AIN1 = 4;   // Direction pin 1
int AIN2 = 3;   // Direction pin 2

// Motor B
int PWMB = 11;   // PWM Speed Control
int BIN1 = 8;  // Direction pin 1
int BIN2 = 9;  // Direction pin 2

void setup() {
  // Setup Pins as OUTPUT
  pinMode(standBy, OUTPUT);

  // Motor A
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Motor B
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.begin(19200);
  Serial.println("Makers Digest: Ready");
}

void loop() {
  forward(128);   // Move forward with medium speed (128 out of 255)
  delay(1000);    // ... for 2 seconds
  stop();         // ... Stop the motors
  delay(2000);     // Delay between motor runs.
  
  reverse(128);   // Move in reverse with medium speed
  delay(1000);    // ... for 2 seconds
  stop();         // ... Stop the Motors
  delay(2000);     // Delay between motor runs.

  turnLeft(128);   // Turn Left with medium speed
  delay(128);    // ... for 2 seconds
  stop();         // ... stop the motors
  delay(2000);     // Delay between motor runs.

  turnRight(128); // Turn Right with medium speed
  delay(1000);    // ... for 2 seconds
  stop();         // ... stop the motors
  delay(2000);    // Delay between motor runs.
}

/*
 * Functions
 * *****************************************************
 */

void turnLeft(int spd)
{
  runMotor(0, spd, 0);
  runMotor(1, spd, 1);
}

void turnRight(int spd)
{
  runMotor(0, spd, 1);
  runMotor(1, spd, 0);
}

void forward(int spd) 
{
  runMotor(0, spd, 0);
  runMotor(1, spd, 0);
}

void reverse(int spd)
{
  runMotor(0, spd, 1);
  runMotor(1, spd, 1);
}

void runMotor(int motor, int spd, int dir)
{
  digitalWrite(standBy, HIGH); // Turn on Motor

  boolean dirPin1 = LOW;
  boolean dirPin2 = HIGH;

  if(dir == 1) {
    dirPin1 = HIGH;
    dirPin2 = LOW;
  }

  if(motor == 0) { // Motor A
    digitalWrite(AIN1, dirPin1);
    digitalWrite(AIN2, dirPin2);
    analogWrite(PWMA, spd); // Use the speed directly (0-255)
  } else { // Motor B
    digitalWrite(BIN1, dirPin1);
    digitalWrite(BIN2, dirPin2);
    analogWrite(PWMB, spd); // Use the speed directly (0-255)
  }
}

void stop() {
  digitalWrite(standBy, LOW);
}
