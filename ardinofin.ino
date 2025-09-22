#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default PCA9685 address

// Servo channels
#define LEFT_SERVO     0
#define RIGHT_SERVO    1
#define TOP_SERVO      2
#define BOTTOM_SERVO   3

// Servo pulse limits
#define SERVO_MIN 150
#define SERVO_MAX 600

// Ultrasonic pins
const int trigPin = 18;   // GPIO18
const int echoPin = 19;   // GPIO19

String inputString = "";

uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

int measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout ~5m
  if (duration > 0) {
    int distance = duration * 0.034 / 2; // cm, speed of sound at 20°C
    if (distance >= 2 && distance <= 500) {
      return distance;
    }
  }
  return -1; // Invalid measurement
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500); // Allow serial to stabilize
  Wire.begin(21, 22); // SDA, SCL
  Wire.setClock(50000); // 50kHz for I2C stability
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for servos
  delay(500); // Allow PCA9685 to initialize
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  centerAll();
  inputString.reserve(20); // Reserve buffer for serial input
}

void loop() {
  if (Serial.available()) {
    inputString = "";
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') break;
      inputString += c;
      delay(2); // Prevent buffer overflow
    }
    inputString.trim();
    Serial.flush(); // Clear residual data
    if (inputString == "GET") {
      int distance = -1;
      int attempts = 0;
      const int maxAttempts = 5;
      while (attempts < maxAttempts) {
        distance = measureDistance();
        if (distance >= 0) break;
        delay(50); // Reduced delay for faster retries
        attempts++;
      }
      if (distance >= 0) {
        Serial.println(distance); // Output distance only
      } else {
        Serial.println("E"); // Error indicator
      }
      delay(20); // Ensure serial timing stability
    } else if (inputString == "STOP") {
      centerAll();
    } else if (inputString == "C") {
      centerAll();
    } else {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex != -1 && inputString.length() > commaIndex + 1 && inputString.length() >= 3) {
        String actionDir = inputString.substring(0, commaIndex);
        int seconds = inputString.substring(commaIndex + 1).toInt();
        if (actionDir.length() == 2) {
          char action = actionDir.charAt(0); // F or A
          char direction = actionDir.charAt(1); // L, R, T, B
          if (seconds == 0) {
            firstTilt(action, direction);
          } else {
            performTilt(action, direction, seconds);
          }
        }
      }
    }
  }
}

void centerAll() {
  bool success = true;
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    success = false; // Suppress I2C error output
  }
  if (success) {
    pwm.setPWM(LEFT_SERVO, 0, angleToPulse(90));
    delay(100);
    pwm.setPWM(RIGHT_SERVO, 0, angleToPulse(90));
    delay(100);
    pwm.setPWM(TOP_SERVO, 0, angleToPulse(90));
    delay(100);
    pwm.setPWM(BOTTOM_SERVO, 0, angleToPulse(90));
    delay(200);
  }
}

void tiltDirection(char cmd) {
  bool success = true;
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    success = false; // Suppress I2C error output
  }
  if (success) {
    switch (cmd) {
      case 'L':
        pwm.setPWM(LEFT_SERVO, 0, angleToPulse(60));
        delay(100);
        pwm.setPWM(RIGHT_SERVO, 0, angleToPulse(120));
        break;
      case 'R':
        pwm.setPWM(LEFT_SERVO, 0, angleToPulse(120));
        delay(100);
        pwm.setPWM(RIGHT_SERVO, 0, angleToPulse(60));
        break;
      case 'T':
        pwm.setPWM(TOP_SERVO, 0, angleToPulse(60));
        delay(100);
        pwm.setPWM(BOTTOM_SERVO, 0, angleToPulse(120));
        break;
      case 'B':
        pwm.setPWM(TOP_SERVO, 0, angleToPulse(120));
        delay(100);
        pwm.setPWM(BOTTOM_SERVO, 0, angleToPulse(60));
        break;
    }
    delay(200);
  }
}

void tiltOpposite(char cmd) {
  bool success = true;
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() != 0) {
    success = false; // Suppress I2C error output
  }
  if (success) {
    switch (cmd) {
      case 'L':
        pwm.setPWM(LEFT_SERVO, 0, angleToPulse(120));
        delay(100);
        pwm.setPWM(RIGHT_SERVO, 0, angleToPulse(60));
        break;
      case 'R':
        pwm.setPWM(LEFT_SERVO, 0, angleToPulse(60));
        delay(100);
        pwm.setPWM(RIGHT_SERVO, 0, angleToPulse(120));
        break;
      case 'T':
        pwm.setPWM(TOP_SERVO, 0, angleToPulse(120));
        delay(100);
        pwm.setPWM(BOTTOM_SERVO, 0, angleToPulse(60));
        break;
      case 'B':
        pwm.setPWM(TOP_SERVO, 0, angleToPulse(60));
        delay(100);
        pwm.setPWM(BOTTOM_SERVO, 0, angleToPulse(120));
        break;
    }
    delay(200);
  }
}

void firstTilt(char action, char direction) {
  if (action == 'F') {
    tiltDirection(direction);
  } else {
    tiltOpposite(direction);
  }
  delay(1000);
  centerAll();
}

void performTilt(char action, char direction, int objDuration) {
  if (action == 'F') {
    tiltDirection(direction);
  } else {
    tiltOpposite(direction);
  }
  delay(1000);
  centerAll();
  delay(4000);
  if (action == 'F') {
    tiltDirection(direction);
  } else {
    tiltOpposite(direction);
  }
  delay(1000);
  centerAll();
  delay(objDuration * 1000);
  if (action == 'F') {
    tiltOpposite(direction);
  } else {
    tiltDirection(direction);
  }
  delay(1000);
  centerAll();
}