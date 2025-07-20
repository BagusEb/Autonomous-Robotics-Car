#include <PID_v1.h>
#include <Servo.h>
#include <dht.h>

// Motor driver pins
#define ENA 9  // Speed control for Left motors
#define ENB 10 // Speed control for Right motors
#define IN1 7  // Left motor 1
#define IN2 6  // Left motor 2
#define IN3 5  // Right motor 1
#define IN4 4  // Right motor 2

// Ultrasonic sensor pins
#define TRIG_PIN 8
#define ECHO_PIN 13

#define DHT_PIN 2


// Servo pin
#define SERVO_PIN A0

// Flame sensor pin
#define FLAME_SENSOR 3

// L9110 Fan Module pin
#define INA 11
#define INB 12

Servo scanServo;
dht DHT;

int tempVal;
// PID variables
volatile int leftSpeedFeedback = 0;
volatile int rightSpeedFeedback = 0;
double leftSetpoint = 150;
double rightSetpoint = 150;
double leftInput, leftOutput;
double rightInput, rightOutput;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// PID controllers
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

long getDistanceFromMicro(long microsec) {
   return microsec / 29 / 2;
}

long getPingDuration() {
   pinMode(TRIG_PIN, OUTPUT);
   pinMode(ECHO_PIN, INPUT);
   digitalWrite(TRIG_PIN, LOW);
   delayMicroseconds(2);
   digitalWrite(TRIG_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG_PIN, LOW);
   return pulseIn(ECHO_PIN, HIGH);
}

int checkDistance(int angle) {
  scanServo.write(angle);
  delay(1000);
  return getDistanceFromMicro(getPingDuration());
}

void moveForward() {
  analogWrite(ENA, leftOutput);
  analogWrite(ENB, rightOutput);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, leftOutput);
  analogWrite(ENB, rightOutput);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, leftOutput);
  analogWrite(ENB, rightOutput);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, leftOutput);
  analogWrite(ENB, rightOutput);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void activateFan() {
  digitalWrite(INA, LOW); 
  digitalWrite(INB, HIGH); 
}

void deactivateFan() {
  digitalWrite(INA, LOW); 
  digitalWrite(INB, LOW); 
}

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(FLAME_SENSOR, INPUT);
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(0, 255);
  rightPID.SetOutputLimits(0, 255);
  
  Serial.begin(9600);
}

void loop() {
  DHT.read11(DHT_PIN);
  tempVal = DHT.temperature; 
  int flameValue = analogRead(FLAME_SENSOR);
  long frontDistance = getDistanceFromMicro(getPingDuration());

  if (flameValue > 300 || tempVal > 27) {  
    stopMotors();
    activateFan();
    Serial.println("Fire detected! Extinguishing...");
    delay(3000);
  } else if (frontDistance > 0 && frontDistance < 20) {  
    stopMotors();
    delay(500);

    int leftDistance = checkDistance(150);
    int rightDistance = checkDistance(30);
    scanServo.write(90);

    if (rightDistance < leftDistance) {
      moveBackward();
      delay(200);
      turnLeft();
      delay(200);
    } else {
      moveBackward();
      delay(200);
      turnRight();
      delay(200);
    }
  } else {
    moveForward();
    deactivateFan();
  }
  
  leftInput = leftSpeedFeedback;
  rightInput = rightSpeedFeedback;
  leftPID.Compute();
  rightPID.Compute();
  analogWrite(ENA, leftOutput);
  analogWrite(ENB, rightOutput);
  
  Serial.print("Flame Value: "); 
  Serial.print(flameValue);
  Serial.print(" Temperature Value: "); 
  Serial.print(tempVal);
  Serial.print(" Front Distance: "); 
  Serial.println(frontDistance);
  delay(100);
}