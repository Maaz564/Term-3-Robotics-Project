#include <Servo.h>

// Define pin connections for servos
const int servoPinTop = 9;
const int servoPinBottom = 10;

// Define pin connections for DC motors
const int E1 = 5;  // PWM pin for motor 1
const int M1 = 4;  // Direction pin for motor 1
const int E2 = 6;  // PWM pin for motor 2 (and motor 3)
const int M2 = 7;  // Direction pin for motor 2 (and motor 3)

// Define pin connections for ultrasonic sensor
const int ultrasonicTrigPin = 11;
const int ultrasonicEchoPin = 12;

// Initialize servo objects
Servo servoTop;
Servo servoBottom;

// Timing variables
unsigned long climbMotorTime = 185000; // 10 seconds in milliseconds
unsigned long waitDuration = 5000;    // 5 seconds waiting time
unsigned long lastSwitchTime = 0;
unsigned long lastMotorStartTime = 0;
unsigned long lastPrintTime = 0;      // Timer for printing distance
unsigned long lastMotorPrintTime = 0; // Timer for printing motor status
const unsigned long printInterval = 2000; // 2 seconds interval for printing

// Define variables for distance measurement
long duration, distance;

// State variables to track motor direction
enum RobotState { INITIALIZING, CLIMBING_UP, LIFTING_BODY_UP, WAITING_FOR_RESET, CLIMBING_DOWN, LOWERING_BODY_DOWN };
RobotState state = INITIALIZING;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach servos
  servoTop.attach(servoPinTop);
  servoBottom.attach(servoPinBottom);

  // Initialize servos to correct starting positions
  servoTop.write(180);      // Start with the top servo at minimum (closed)
  servoBottom.write(180); // Start with the bottom servo at maximum (closed)

  // Print initial servo state to the serial monitor
  Serial.println("Initial state: Top gripper closed (0°), Bottom gripper closed (0°)");

  // Set motor pins as outputs
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);

  // Set initial timing
  lastSwitchTime = millis();
  lastMotorStartTime = millis();
  lastPrintTime = millis(); // Initialize the last print time
  lastMotorPrintTime = millis(); // Initialize the last motor print time
}

void loop() {
  // Measure distance
  distance = measureDistance();

  // Print the distance to the Serial Monitor every 2 seconds
  if (millis() - lastPrintTime >= printInterval) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    lastPrintTime = millis(); // Update the last print time
  }

  // Check if distance is below 20 cm to switch from climbing up to climbing down
  if (distance <= 20 && (state == CLIMBING_UP || state == LIFTING_BODY_UP)) {
    servoTop.write(180);  // Ensure both grippers are closed initially
    servoBottom.write(180);
    Serial.println("Close both grippers for safety, preparing to descend.");
    state = CLIMBING_DOWN;
    servoTop.write(100); // Open top gripper
    delay(waitDuration);
    servoBottom.write(180); // Keep bottom gripper closed
    lastMotorStartTime = millis(); // Reset motor timing for climbing down
  }else{
    delay(waitDuration);
//    lastMotorStartTime = lastMotorStartTime + waitDuration;
  }

  switch (state) {
    case INITIALIZING:
      Serial.print("millis() - lastMotorStartTime:");
      Serial.print(millis() - lastMotorStartTime);
      Serial.print("climbMotorTime:");
      Serial.println(climbMotorTime);
      if (millis() - lastSwitchTime > waitDuration) {
        state = CLIMBING_UP;
        servoBottom.write(100);   // Close bottom gripper
        delay(waitDuration);
        servoTop.write(180);    // Open top gripper
        lastMotorStartTime = millis();
      }
      break;
    case CLIMBING_UP:
      Serial.print("millis() - lastMotorStartTime:");
      Serial.print(millis() - lastMotorStartTime);
      Serial.print("climbMotorTime:");
      Serial.println(climbMotorTime);
      if (millis() - lastMotorStartTime < climbMotorTime) {
        Serial.println("Begin runMotorsUP");
        runMotorsUp();
      } else {
        Serial.println("Switching from climbing up to lifting body up.");
        stopAllDCMotors();
        servoBottom.write(180);   // Close bottom gripper
        delay(waitDuration);
        servoTop.write(100);    // Open top gripper
        Serial.println("Bottom gripper closed (0°), Top gripper open (180°)");
        state = LIFTING_BODY_UP;
        lastMotorStartTime = millis();
      }
      break;
    case LIFTING_BODY_UP:
      Serial.print("millis() - lastMotorStartTime:");
      Serial.print(millis() - lastMotorStartTime);
      Serial.print("climbMotorTime:");
      Serial.println(climbMotorTime);
      if (millis() - lastMotorStartTime < climbMotorTime) {
        Serial.println("Begin runMotorsUPReverse");
        runMotorsUpReverse();
      } else {
        Serial.println("Switching from lifting body up to climbing up.");
        stopAllDCMotors();
        servoTop.write(180);      // Close top gripper
        delay(waitDuration);
        servoBottom.write(100); // Open bottom gripper
        Serial.println("Top gripper closed (0°), Bottom gripper open (180°)");
        state = CLIMBING_UP;  // Transition directly to descending after body lift
        lastMotorStartTime = millis();
      }
      break;
    case CLIMBING_DOWN:
      Serial.print("millis() - lastMotorStartTime:");
      Serial.print(millis() - lastMotorStartTime);
      Serial.print("climbMotorTime:");
      Serial.println(climbMotorTime);
      if (millis() - lastMotorStartTime < climbMotorTime) {
        Serial.println("Begin runMotorsDOWN");
        runMotorsDown();
      } else {
        Serial.println("Switching from climbing down to lowering body down.");
        stopAllDCMotors();
        servoTop.write(180); // Close top gripper
        delay(waitDuration);
        servoBottom.write(100); // Open bottom gripper
        Serial.println("Top gripper closed (0°), Bottom gripper open (180°)");
        state = LOWERING_BODY_DOWN;
        lastMotorStartTime = millis();
      }
      break;
    case LOWERING_BODY_DOWN:
      Serial.print("millis() - lastMotorStartTime:");
      Serial.print(millis() - lastMotorStartTime);
      Serial.print("climbMotorTime:");
      Serial.println(climbMotorTime);
      if (millis() - lastMotorStartTime < climbMotorTime) {
        Serial.println("Begin runMotorsDownReverse");
        runMotorsDownReverse();
      } else {
        Serial.println("Switching from lowering body down to climbing down");
        stopAllDCMotors();
        servoBottom.write(180);   // Close bottom gripper
        delay(waitDuration);
        servoTop.write(100);    // Open top gripper
        Serial.println("Bottom gripper closed (0°), Top gripper open (180°)");
        state = CLIMBING_DOWN; // Prepare to start the cycle again or stop
        lastSwitchTime = millis();
      }
      break;
    case WAITING_FOR_RESET:
      if (millis() - lastSwitchTime > waitDuration) {
        state = INITIALIZING; // Restart the cycle
        lastMotorStartTime = millis();
      }
      break;
  }
}

long measureDistance() {
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);

  duration = pulseIn(ultrasonicEchoPin, HIGH);
  return (duration / 2) * 0.0343; // Calculate distance in cm
}

void runMotorsUp() {
  analogWrite(E1, 255); // Full speed
  digitalWrite(M1, LOW); // Direction
  analogWrite(E2, 255); // Full speed
  digitalWrite(M2, LOW); // Direction

  if (millis() - lastMotorPrintTime >= 1000) {
    Serial.println("Motors running up (anticlockwise)");
    lastMotorPrintTime = millis();
  }
}

void runMotorsUpReverse() {
  analogWrite(E1, 255); // Full speed
  digitalWrite(M1, HIGH); // Reverse direction
  analogWrite(E2, 255); // Full speed
  digitalWrite(M2, HIGH); // Reverse direction

  if (millis() - lastMotorPrintTime >= 1000) {
    Serial.println("Motors running up reverse (clockwise)");
    lastMotorPrintTime = millis();
  }
}

void runMotorsDown() {
  analogWrite(E1, 255); // Full speed
  digitalWrite(M1, HIGH); // Clockwise direction
  analogWrite(E2, 255); // Full speed
  digitalWrite(M2, HIGH); // Clockwise direction

  if (millis() - lastMotorPrintTime >= 1000) {
    Serial.println("Motors running down (clockwise)");
    lastMotorPrintTime = millis();
  }
}

void runMotorsDownReverse() {
  analogWrite(E1, 255); // Full speed
  digitalWrite(M1, LOW); // Anticlockwise direction
  analogWrite(E2, 255); // Full speed
  digitalWrite(M2, LOW); // Anticlockwise direction

  if (millis() - lastMotorPrintTime >= 1000) {
    Serial.println("Motors running down reverse (anticlockwise)");
    lastMotorPrintTime = millis();
  }
}

void stopAllDCMotors() {
  analogWrite(E1, 0); // Stop motor 1
  analogWrite(E2, 0); // Stop motor 2 and 3
  Serial.println("Motors stopped");
}
