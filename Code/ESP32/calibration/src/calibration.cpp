#include <Arduino.h>
#include <ESP32_Servo.h>

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
int servoPos = 90;
const int SERVO_MIN = 40;
const int SERVO_MAX = 140;
const int SERVO_STEP = 20;

volatile long countA = 0;
volatile long countB = 0;

const int TEST_TIME_MS = 2000;
const int PWM_START = 50;
const int PWM_END = 150;
const int PWM_STEP = 25;

void IRAM_ATTR isrA() { countA++; }
void IRAM_ATTR isrB() { countB++; }

void servoInit() {
  pwmServo.attach(PSERVO_PIN); 
  pwmServo.write(servoPos);
}

void runMotor(int IN_A, int IN_B, int PWM, int pwmValue, bool invert = false) {
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

  servoInit();

  Serial.println("Type 'Start' and press Enter to run.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("Start")) {
      Serial.println("Starting calibration...");

      servoPos = SERVO_MIN;
      pwmServo.write(servoPos);

      long finalCountA = 0;
      long finalCountB = 0;

      for (int pwm = PWM_START; pwm <= PWM_END; pwm += PWM_STEP) {
        countA = 0;
        countB = 0;

        runMotor(MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_PWM, pwm, true);
        runMotor(MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_PWM, pwm);
        delay(TEST_TIME_MS);
        stopMotor(MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_PWM);
        stopMotor(MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_PWM);

        finalCountA = countA;
        finalCountB = countB;

        Serial.print("Motor A PWM: "); Serial.print(pwm);
        Serial.print(" -> Encoder: "); Serial.println(finalCountA);
        Serial.print("Motor B PWM: "); Serial.print(pwm);
        Serial.print(" -> Encoder: "); Serial.println(finalCountB);

        servoPos += SERVO_STEP;
        if (servoPos > SERVO_MAX) servoPos = SERVO_MAX;
        pwmServo.write(servoPos);
        Serial.print("Servo Position: "); Serial.println(servoPos);
        delay(500);
      }

      pwmServo.write(90);
      Serial.println("Servo Position: 90°");

      Serial.println("________________________________________");
      Serial.print("Encoder -> Motor A: ");
      Serial.print(finalCountA);
      Serial.print(", Motor B: ");
      Serial.println(finalCountB);
    }
  }
}