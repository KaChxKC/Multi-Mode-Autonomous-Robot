/*
 * 4-in-1 Autonomous Robot Car - Arduino Uno (controller node)
 * ----------------------------------------------------------
 * Runs the robot: reads the sensors, drives the motors and the servo, and runs one of
 * four behaviours chosen at run time:
 *   1 = obstacle avoiding, 2 = line follower,
 *   3 = line follower + obstacle avoiding, 4 = object following.
 * Power ON/OFF and the active mode are received from the ESP32 gateway (Blynk dashboard)
 * over a serial link; the front distance is sent back for the dashboard gauge.
 *
 * Pins
 *   Servo signal ............ D3          HC-SR04 TRIG / ECHO ..... D11 / D12
 *   L298N ENA / ENB (PWM) ... D5 / D6      L298N IN1..IN4 ........... D7, D8, D9, D10
 *   Bottom IR right / left .. A0 / A1      Top IR left / right ...... A2 / A3
 *
 * Serial link to the ESP32 (SoftwareSerial, 9600 baud, newline-terminated text)
 *   Uno D2 (RX)  <-  ESP32 GPIO17 (TX2)
 *   Uno D4 (TX)  ->  ESP32 GPIO16 (RX2)   through a 1k / 2k voltage divider (5 V -> 3.3 V)
 *   GND <-> GND (common ground is required)
 *
 * Protocol
 *   ESP32 -> Uno : "E1" / "E0" (robot on / off), "M1".."M4" (select mode)
 *   Uno -> ESP32 : "D<cm>"     (front distance, about once per second)
 *
 * Libraries: NewPing, Servo (bundled), SoftwareSerial (bundled).  Board: Arduino Uno.
 */

#include <Servo.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define OBSTACLE_AVOIDING_MODE                    1
#define LINE_FOLLOWER_MODE                        2
#define LINE_FOLLOWER_WITH_OBSTACLE_AVOIDING_MODE 3
#define OBJECT_FOLLOWING_MODE                     4

#define SPEED_PERCENT 85   // 100 = original speed, lower = slower (ENA/ENB now control PWM)

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12

#define BOTTOM_IR_SENSOR_RIGHT A0
#define BOTTOM_IR_SENSOR_LEFT A1
#define TOP_IR_SENSOR_RIGHT A3
#define TOP_IR_SENSOR_LEFT A2

// ---- Runtime control (set over Blynk, no re-upload needed) ----
int  robotControlMode = OBSTACLE_AVOIDING_MODE;   // default mode on boot
bool robotEnabled     = false;                // starts OFF; turn on from Blynk

// SoftwareSerial to the ESP32: RX = D2, TX = D4
SoftwareSerial espSerial(2, 4);  // (RX, TX)
String rxLine = "";

// Telemetry timing
unsigned long lastTelemetry = 0;

//Right motor
int enableRightMotor=5;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=9;
int leftMotorPin2=10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;

int servoPos = 90;                 // tracks current servo angle
#define SERVO_STEP_DELAY 15        // bigger = slower servo (try 20-30 for even slower)

// Move servo gradually to target angle (smooth + slow)
void moveServoSlow(int target)
{
  if (target > servoPos)
    for (; servoPos < target; servoPos++) { myServo.write(servoPos); delay(SERVO_STEP_DELAY); }
  else
    for (; servoPos > target; servoPos--) { myServo.write(servoPos); delay(SERVO_STEP_DELAY); }
}

void setup()
{
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);

  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);

  // Servo self-test on boot (confirms servo hardware works)
  delay(300);
  moveServoSlow(0);   delay(300);
  moveServoSlow(180); delay(300);
  moveServoSlow(90);  delay(300);

  pinMode(BOTTOM_IR_SENSOR_RIGHT, INPUT);
  pinMode(BOTTOM_IR_SENSOR_LEFT, INPUT);
  pinMode(TOP_IR_SENSOR_RIGHT, INPUT);
  pinMode(TOP_IR_SENSOR_LEFT, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);   // debug: shows robotEnabled state

  Serial.begin(9600);       // optional USB debug
  espSerial.begin(9600);    // link to ESP32

  rotateMotor(0,0);
}

// ---- Read commands from the ESP32 ----
void handleEspCommands()
{
  while (espSerial.available())
  {
    char c = espSerial.read();
    if (c == '\n')
    {
      if (rxLine == "E1")      robotEnabled = true;
      else if (rxLine == "E0") robotEnabled = false;
      else if (rxLine.length() == 2 && rxLine[0] == 'M')
      {
        int m = rxLine[1] - '0';
        if (m >= 1 && m <= 4) robotControlMode = m;
      }
      rxLine = "";
    }
    else if (c != '\r')
    {
      rxLine += c;
      if (rxLine.length() > 6) rxLine = "";  // guard against junk
    }
  }
}

// ---- Send distance to the ESP32 every ~1 s for the Blynk gauge ----
void sendTelemetry()
{
  if (millis() - lastTelemetry >= 1000)
  {
    lastTelemetry = millis();
    int d = mySensor.ping_cm();   // 0 = nothing in range
    espSerial.print('D');
    espSerial.print(d);
    espSerial.print('\n');
  }
}

// Pause for `ms` while still watching for an OFF command.
// Returns true if the robot was turned OFF during the wait (caller should abort).
bool abortableDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    handleEspCommands();
    if (!robotEnabled) { rotateMotor(0, 0); return true; }
    delay(5);
  }
  return false;
}

void obstacleAvoidingControl()
{
  static int FORWARD_SPEED      = 150;  // normal driving speed
  static int BACK_SPEED         = 150;  // step-back speed
  static int TURN_SPEED         = 200;  // turning speed
  static int DISTANCE_TO_CHECK  = 40;   // obstacle threshold (cm)
  static int BACK_TIME          = 400;  // how long to step back (ms)
  static int TURN_TIME          = 450;  // how long to turn (ms)

  // Stable reading (median of 3) to avoid false 0 / spikes
  int distance = mySensor.convert_cm(mySensor.ping_median(3));

  // Obstacle ahead? (0 = nothing in range, so ignore it)
  if (distance > 0 && distance < DISTANCE_TO_CHECK)
  {
    // 1. Stop
    rotateMotor(0, 0);
    if (abortableDelay(300)) return;

    // 2. Step back
    rotateMotor(-BACK_SPEED, -BACK_SPEED);
    if (abortableDelay(BACK_TIME)) return;
    rotateMotor(0, 0);
    if (abortableDelay(300)) return;

    // 3. Look left, then right (motors stopped while scanning)
    moveServoSlow(180);
    if (abortableDelay(300)) return;
    int distanceLeft = mySensor.convert_cm(mySensor.ping_median(3));

    moveServoSlow(0);
    if (abortableDelay(300)) return;
    int distanceRight = mySensor.convert_cm(mySensor.ping_median(3));

    moveServoSlow(90);             // re-centre the sensor
    if (abortableDelay(300)) return;

    // 4. Turn toward the more open side (0 = clear/far on that side)
    bool turnLeft;
    if (distanceLeft == 0 && distanceRight == 0)      turnLeft = true;
    else if (distanceLeft == 0)                        turnLeft = true;
    else if (distanceRight == 0)                       turnLeft = false;
    else                                               turnLeft = (distanceLeft >= distanceRight);

    if (turnLeft) rotateMotor(TURN_SPEED, -TURN_SPEED);
    else          rotateMotor(-TURN_SPEED, TURN_SPEED);
    if (abortableDelay(TURN_TIME)) return;

    // 5. Brief stop, then the next loop drives forward again
    rotateMotor(0, 0);
    if (abortableDelay(150)) return;
  }
  else
  {
    // Path clear -> keep moving forward
    rotateMotor(FORWARD_SPEED, FORWARD_SPEED);
  }
}

void lineFollowerControl()
{
  static int MOTOR_SPEED = 100;
  static int TURNING_MOTOR_SPEED = 200;

  int rightIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_LEFT);

  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-TURNING_MOTOR_SPEED, TURNING_MOTOR_SPEED);
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(TURNING_MOTOR_SPEED, -TURNING_MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0);
  }
}

void lineFollowerWithObstacleAvoidingControl()
{
  static int MOTOR_SPEED = 100;
  static int TURNING_MOTOR_SPEED = 200;
  static int DISTANCE_TO_CHECK  = 20;

  int rightIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_LEFT);

  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    int distance = mySensor.ping_cm();
    if (distance > 0 && distance < DISTANCE_TO_CHECK)
    {
      adjustRobotToAvoidObjectWhileFollowingLine();
    }
    else
    {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-TURNING_MOTOR_SPEED, TURNING_MOTOR_SPEED);
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(TURNING_MOTOR_SPEED, -TURNING_MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0);
  }
}

void objectFollowingControl()
{
  static int MAX_FORWARD_MOTOR_SPEED  = 150;
  static int MAX_MOTOR_TURN_SPEED_ADJUSTMENT  = 100;
  static int MIN_DISTANCE  = 10;
  static int MAX_DISTANCE  = 30;

  int distance = mySensor.ping_cm();
  int rightIRSensorValue = digitalRead(TOP_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(TOP_IR_SENSOR_LEFT);

  if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT );
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(MAX_FORWARD_MOTOR_SPEED + MAX_MOTOR_TURN_SPEED_ADJUSTMENT, MAX_FORWARD_MOTOR_SPEED - MAX_MOTOR_TURN_SPEED_ADJUSTMENT);
  }
  else if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE)
  {
    rotateMotor(MAX_FORWARD_MOTOR_SPEED, MAX_FORWARD_MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0);
  }
}

void adjustRobotToAvoidObjectWhileFollowingLine()
{
  static int MAX_MOTOR_TURN_SPEED  = 200;

  rotateMotor(0, 0);
  if (abortableDelay(500)) return;
  moveServoSlow(180);
  if (abortableDelay(200)) return;
  int distanceLeft = mySensor.ping_cm();
  moveServoSlow(0);
  if (abortableDelay(200)) return;
  int distanceRight = mySensor.ping_cm();
  moveServoSlow(90);
  if (abortableDelay(200)) return;

  bool robotMovedToLeftDirection = false;
  if (distanceLeft == 0 )
  {
    robotMovedToLeftDirection = true;
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(500)) return;
  }
  else if (distanceRight == 0 )
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(500)) return;
  }
  else if (distanceLeft >= distanceRight)
  {
    robotMovedToLeftDirection = true;
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(500)) return;
  }
  else
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(500)) return;
  }

  rotateMotor(MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
  if (abortableDelay(500)) return;

  if (robotMovedToLeftDirection)
  {
    rotateMotor(-MAX_MOTOR_TURN_SPEED, MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(800)) return;
  }
  else
  {
    rotateMotor(MAX_MOTOR_TURN_SPEED, -MAX_MOTOR_TURN_SPEED);
    if (abortableDelay(800)) return;
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);
  }
  else if (rightMotorSpeed >= 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);
  }
  else if (leftMotorSpeed >= 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed) * SPEED_PERCENT / 100);
  analogWrite(enableLeftMotor, abs(leftMotorSpeed) * SPEED_PERCENT / 100);
}

void loop()
{
  handleEspCommands();   // check for ON/OFF + mode updates
  sendTelemetry();       // push distance to Blynk gauge

  digitalWrite(LED_BUILTIN, robotEnabled);   // debug LED mirrors ON/OFF

  if (!robotEnabled)
  {
    rotateMotor(0, 0);   // robot is OFF
    return;
  }

  switch (robotControlMode)
  {
    case OBSTACLE_AVOIDING_MODE:
      obstacleAvoidingControl();
      break;
    case LINE_FOLLOWER_MODE:
      lineFollowerControl();
      break;
    case LINE_FOLLOWER_WITH_OBSTACLE_AVOIDING_MODE:
      lineFollowerWithObstacleAvoidingControl();
      break;
    case OBJECT_FOLLOWING_MODE:
      objectFollowingControl();
      break;
    default:
      break;
  }
}
