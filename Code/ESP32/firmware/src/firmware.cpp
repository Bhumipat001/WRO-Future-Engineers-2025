#include <Arduino.h>
#include <ESP32_Servo.h>
void processCommand(String input);

const int MOTOR1_IN_A = 17;
const int MOTOR1_IN_B = 21;
const int MOTOR1_PWM  = 25;
const int MOTOR1_ENCODER_A = 34;
const int MOTOR1_ENCODER_B = 35;

const int MOTOR2_IN_A = 22;
const int MOTOR2_IN_B = 23;
const int MOTOR2_PWM  = 26;
const int MOTOR2_ENCODER_A = 16;
const int MOTOR2_ENCODER_B = 27;

Servo pwmServo;
#define PSERVO_PIN 4
int servoPos = 81;

const int SWITCH_PIN = 5;
int switchState;
int lastReading = HIGH;
int stableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

volatile long countA = 0;
volatile long countB = 0;
const int MAX_PWM = 150;

const long EncoderMotorA = 4399;
const long EncoderMotorB = 4155;
const float MOTOR_RATIO_B = (float)EncoderMotorA / (float)EncoderMotorB;

void IRAM_ATTR isrA() { countA++; }
void IRAM_ATTR isrB() { countB++; }

void runMotor(int IN_A, int IN_B, int PWM, int pwmValue, bool invert = false) {
  if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;
  if (pwmValue < 0) pwmValue = 0;

  if (!invert) {
    digitalWrite(IN_A, LOW);
    digitalWrite(IN_B, HIGH);
  } else {
    digitalWrite(IN_A, HIGH);
    digitalWrite(IN_B, LOW);
  }

  analogWrite(PWM, pwmValue);
}

void stopMotor(int IN_A, int IN_B, int PWM) {
  digitalWrite(IN_A, LOW);
  digitalWrite(IN_B, LOW);
  analogWrite(PWM, 0);
}

void servoInit() {
  pwmServo.attach(PSERVO_PIN);
  pwmServo.write(servoPos);
}

String inputBuffer = "";

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR1_IN_A, OUTPUT);
  pinMode(MOTOR1_IN_B, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_A), isrA, RISING);

  pinMode(MOTOR2_IN_A, OUTPUT);
  pinMode(MOTOR2_IN_B, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR2_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_A), isrB, RISING);

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  servoInit();
  Serial.println("Format: MA:speed,MB:speed,S:servo  OR  ENC:RESET  OR  ENC:READ");
}

void loop() {
  int reading = digitalRead(SWITCH_PIN);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {
        Serial.println("Start");
      }
    }
  }
  lastReading = reading;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

void processCommand(String input) {
  input.trim();

  if (input.equalsIgnoreCase("ENC:RESET")) {
    noInterrupts();
    countA = 0;
    countB = 0;
    interrupts();
    Serial.println("ENC:OK");
    return;
  }

  if (input.equalsIgnoreCase("ENC:READ")) {
    long a, b;
    noInterrupts();
    a = countA;
    b = countB;
    interrupts();
    Serial.print("ENC:A:"); Serial.print(a);
    Serial.print(",B:"); Serial.println(b);
    return;
  }

  int maIdx = input.indexOf("MA:");
  int mbIdx = input.indexOf("MB:");
  int sIdx  = input.indexOf("S:");

  if (maIdx >= 0 && mbIdx >= 0 && sIdx >= 0) {
    int maEnd = input.indexOf(',', maIdx);
    int mbEnd = input.indexOf(',', mbIdx);

    int speedA = input.substring(maIdx + 3, maEnd).toInt();
    int speedB = input.substring(mbIdx + 3, sIdx).toInt();
    int servoTarget = input.substring(sIdx + 2).toInt();

    speedB = (int)(speedB * MOTOR_RATIO_B);

    runMotor(MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_PWM, speedA, true);
    runMotor(MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_PWM, speedB);

    if (servoTarget < 0) servoTarget = 0;
    if (servoTarget > 180) servoTarget = 180;
    pwmServo.write(servoTarget);

    Serial.print("Applied -> MA:"); Serial.print(speedA);
    Serial.print(", MB:"); Serial.print(speedB);
    Serial.print(", Servo:"); Serial.println(servoTarget);
    return;
  }

  Serial.println("ERR:UNKNOWN_CMD");
}