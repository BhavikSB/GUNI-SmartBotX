#include <Servo.h>

// ==== Servo Setup ====
Servo headServo;
Servo chestServo;

const int headPin = 6;
const int chestPin = 7;

const int headNeutral = 90;
const int chestNeutral = 90;

// ==== IR Sensor Pins ====
const int irSensor1Pin = 28;
const int irSensor2Pin = 30;
const int irSensor3Pin = 32;
const int irSensor4Pin = 34;
const int irRightPin = 2;
const int irLeftPin = 3;

// ==== Ultrasonic Sensor Pins ====
const int trigPin = 4;
const int echoPin = 5;
long duration;
int distance;

// ==== Motor Pins ====
const int motorA1 = 13;
const int motorA2 = 12;
const int motorB1 = 9;
const int motorB2 = 8;
const int enableA = 11;
const int enableB = 10;

// ==== Setup ====
void setup() {
  Serial.begin(9600);

  // Servo
  headServo.attach(headPin);
  chestServo.attach(chestPin);
  headForward();
  chestForward();

  // IR Sensors
  pinMode(irSensor1Pin, INPUT);
  pinMode(irSensor2Pin, INPUT);
  pinMode(irSensor3Pin, INPUT);
  pinMode(irSensor4Pin, INPUT);
  pinMode(irRightPin, INPUT);
  pinMode(irLeftPin, INPUT);

  // Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor Pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);

  stop();  // Ensure motors start stopped
}

// ==== Movement Functions ====
void forward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(enableA, 80);
  analogWrite(enableB, 80);
}

void stop() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(enableA, 0);
  analogWrite(enableB, 0);
}

// ==== Ultrasonic Distance ====
int getUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  return duration / 58.0;
}

// ==== Servo Movements ====
void headRight()    { headServo.write(0);   delay(1000); Serial.println("Head → Right"); }
void headLeft()     { headServo.write(180); delay(1000); Serial.println("Head → Left"); }
void headForward()  { headServo.write(90);  delay(1000); Serial.println("Head → Center"); }
void chestRight()   { chestServo.write(0);  delay(1000); Serial.println("Chest → Right"); }
void chestLeft()    { chestServo.write(180);delay(1000); Serial.println("Chest → Left"); }
void chestForward() { chestServo.write(90); delay(1000); Serial.println("Chest → Center"); }

void smoothMove(Servo &servo, int target, int stepDelay = 15) {
  int current = servo.read();
  if (current < target)
    for (int i = current; i <= target; i++) { servo.write(i); delay(stepDelay); }
  else
    for (int i = current; i >= target; i--) { servo.write(i); delay(stepDelay); }
}

// ==== Talking Animation ====
void talking() {
  Serial.println("Robot talking...");
  int moves = random(3, 7);
  for (int i = 0; i < moves; i++) {
    int headMove = random(0, 3);
    if (headMove == 0) {
      smoothMove(headServo, headNeutral - 10);
      smoothMove(headServo, headNeutral + 10);
    } else if (headMove == 1) {
      smoothMove(headServo, headNeutral - 20);
    } else {
      smoothMove(headServo, headNeutral + 20);
    }
    smoothMove(headServo, headNeutral);

    int chestMove = random(0, 2);
    if (chestMove == 0) {
      smoothMove(chestServo, chestNeutral - 20);
    } else {
      smoothMove(chestServo, chestNeutral + 20);
    }
    smoothMove(chestServo, chestNeutral);
    delay(random(200, 500));
  }
  Serial.println("Done talking.");
}

// ==== Main Loop ====
void loop() {
  // Read sensors
  int ir1 = digitalRead(irSensor1Pin);
  int ir2 = digitalRead(irSensor2Pin);
  int ir3 = digitalRead(irSensor3Pin);
  int ir4 = digitalRead(irSensor4Pin);
  int irRight = digitalRead(irRightPin);
  int irLeft = digitalRead(irLeftPin);
  int distance = getUltrasonicDistance();

  // Print sensor status
  Serial.print("IR1: "); Serial.print(ir1 == LOW ? "Obj" : "Clear");
  Serial.print(" | IR2: "); Serial.print(ir2 == LOW ? "Obj" : "Clear");
  Serial.print(" | IR3: "); Serial.print(ir3 == LOW ? "Obj" : "Clear");
  Serial.print(" | IR4: "); Serial.print(ir4 == LOW ? "Obj" : "Clear");
  Serial.print(" | IR_R: "); Serial.print(irRight == LOW ? "Obj" : "Clear");
  Serial.print(" | IR_L: "); Serial.print(irLeft == LOW ? "Obj" : "Clear");
  Serial.print(" | US: "); Serial.print(distance); Serial.println(" cm");

  // Obstacle check
  if (ir1 == LOW || ir2 == LOW || ir3 == LOW || ir4 == LOW || irRight == HIGH || irLeft == HIGH || distance < 20) {
    stop();
    Serial.println("Obstacle detected!");
    talking();
  } else {
    forward();
    Serial.println("Path clear. Moving...");
  }

  delay(500);
}
