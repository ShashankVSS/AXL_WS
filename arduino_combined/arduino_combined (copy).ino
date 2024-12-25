#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

// Pin definitions for motor control
const int motorPWM = 9;   // PWM pin for motor speed control
const int motorIN1 = 7;   // Motor forward pin
const int motorIN2 = 6;   // Motor reverse pin

// Encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Encoder variables
volatile long encoderPosition = 0;
long targetPosition = 0;  // This will be updated only when the "hold" command is received
bool isHolding = false;  // Flag to indicate if we should be holding the position

// Control parameters
const long maxEncoderPosition = 40000; // Maximum encoder counts for extension
const long minEncoderPosition = -40000;    // Minimum encoder counts for retraction
const int motorSpeed = 200;           // Motor speed for extend/retract
const int holdSpeed = 150;             // Motor speed for holding position
const float holdThreshold = 500;      // Tolerance in encoder counts for holding

// State machine states
enum MotorState { IDLE, EXTENDING, RETRACTING, HOLDING };
MotorState currentState = IDLE;

// ROS node handle
ros::NodeHandle nh;

// Publisher for encoder position
std_msgs::Float32 encoder_msg;
ros::Publisher encoder_pub("encoder_position", &encoder_msg);

// Subscriber callbacks
void extensionCallback(const std_msgs::String& msg);

// Subscribers
ros::Subscriber<std_msgs::String> sub_extension("extension_setting", &extensionCallback);

// ISR for encoder signals
void encoderISR() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  if (stateA == stateB) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

// Variable to track consecutive stop commands
int stopCommandCount = 0;

// Callback for 'extension_setting' topic
void extensionCallback(const std_msgs::String& msg) {
  String command = msg.data;
  Serial.print("Received command: ");
  Serial.println(command);
  nh.loginfo(("Received command: " + command).c_str());

  if (command == "extend") {
    if (encoderPosition <= minEncoderPosition) {
      Serial.println("Already fully extended.");
      nh.loginfo("Already fully extended.");
      return;
    }
    currentState = EXTENDING;
    Serial.println("State set to EXTENDING.");
    nh.loginfo("State set to EXTENDING.");
  }
  else if (command == "retract") {
    if (encoderPosition >= maxEncoderPosition) {
      Serial.println("Already fully retracted.");
      nh.loginfo("Already fully retracted.");
      return;
    }
    currentState = RETRACTING;
    Serial.println("State set to RETRACTING.");
    nh.loginfo("State set to RETRACTING.");
  }
  else if (command == "stop") {
    // Check if stop is already sent once
    if (stopCommandCount == 1) {
      // Second consecutive stop command: motor stops completely
      digitalWrite(motorIN1, LOW);
      digitalWrite(motorIN2, LOW);
      analogWrite(motorPWM, 0);
      Serial.println("Motor completely stopped due to consecutive stop commands.");
      nh.loginfo("Motor completely stopped due to consecutive stop commands.");
      stopCommandCount = 0;  // Reset the stop count
      currentState = IDLE;
    } else {
      // First stop command: save the current position as the target and enter holding state
      targetPosition = encoderPosition;
      isHolding = true;  // Flag to indicate holding mode
      currentState = HOLDING;
      stopCommandCount = 1;  // Set count to 1
      Serial.println("State set to HOLDING.");
      nh.loginfo("State set to HOLDING.");
    }
  }
  else {
    Serial.println("Unknown command received.");
    nh.loginfo("Unknown command received.");
  }
}

// Motor control function
void controlMotor() {
  switch (currentState) {
    case IDLE:
      // Ensure motor is stopped
      digitalWrite(motorIN1, LOW);
      digitalWrite(motorIN2, LOW);
      analogWrite(motorPWM, 0);
      break;

    case EXTENDING:
      if (encoderPosition > minEncoderPosition) {
        // Extend: Move motor to decrease encoder position (negative direction)
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, HIGH);  // Reverse direction for extension
        analogWrite(motorPWM, motorSpeed);
        Serial.println("Extending...");
        nh.loginfo("Extending...");
      }
      else {
        // Reached min position
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, LOW);
        analogWrite(motorPWM, 0);
        Serial.println("Reached minimum extension.");
        nh.loginfo("Reached minimum extension.");
        currentState = IDLE;
      }
      break;

    case RETRACTING:
      if (encoderPosition < maxEncoderPosition) {
        // Retract: Move motor to increase encoder position (positive direction)
        digitalWrite(motorIN1, HIGH);
        digitalWrite(motorIN2, LOW);  // Forward direction for retraction
        analogWrite(motorPWM, motorSpeed);
        Serial.println("Retracting...");
        nh.loginfo("Retracting...");
      }
      else {
        // Reached max position
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, LOW);
        analogWrite(motorPWM, 0);
        Serial.println("Reached maximum retraction.");
        nh.loginfo("Reached maximum retraction.");
        currentState = IDLE;
      }
      break;

    case HOLDING:
      if (isHolding) {
        // Holding position with ±100 encoder units tolerance
        long lowerLimit = targetPosition - 500;
        long upperLimit = targetPosition + 500;

        // If encoder position is less than the lower limit, apply motor to move it up
        if (encoderPosition < lowerLimit) {
          // Apply power to move it back up
          digitalWrite(motorIN1, HIGH);  // Move forward
          digitalWrite(motorIN2, LOW);
          analogWrite(motorPWM, holdSpeed);  // Full speed
          Serial.println("Holding: Moving up...");
          nh.loginfo("Holding: Moving up...");
        }
        // If encoder position is more than the upper limit, apply motor to move it down
        else if (encoderPosition > upperLimit) {
          // Apply power to move it back down
          digitalWrite(motorIN1, LOW);  // Move backward
          digitalWrite(motorIN2, HIGH);
          analogWrite(motorPWM, holdSpeed);  // Full speed
          Serial.println("Holding: Moving down...");
          nh.loginfo("Holding: Moving down...");
        }
        else {
          // Encoder is within range, no need to move
          digitalWrite(motorIN1, LOW);
          digitalWrite(motorIN2, LOW);
          analogWrite(motorPWM, 0);
          Serial.println("Holding: Within range, no movement.");
          nh.loginfo("Holding: Within range, no movement.");
        }
      }
      break;
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(57600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB
  }
  Serial.println("Arduino Initialized.");
  nh.loginfo("Arduino Initialized.");

  // Initialize motor control pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(encoder_pub);
  nh.subscribe(sub_extension);

  // Ensure motor is stopped
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, LOW);
  analogWrite(motorPWM, 0);

  Serial.println("Setup complete.");
  nh.loginfo("Setup complete.");
}

void loop() {
  // Control the motor based on current command
  controlMotor();

  // Publish encoder position
  encoder_msg.data = (float)encoderPosition;
  encoder_pub.publish(&encoder_msg);

  // Log encoder position
  String encoderMessage = "Encoder Position: " + String(encoderPosition);
  nh.loginfo(encoderMessage.c_str());
  Serial.println(encoderMessage);

  nh.spinOnce();
  delay(100); // 10 Hz loop
}
