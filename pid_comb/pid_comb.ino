#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h> // Changed from Float32 to Int32
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h> // Include math library for isnan and isinf functions

// ---------------------------- Pin Definitions ----------------------------
const int motorPWM = 9;   // PWM pin for motor speed control
const int motorIN1 = 7;   // Motor forward pin
const int motorIN2 = 6;   // Motor reverse pin

// Encoder pins
const int encoderPinA = 2;  // Encoder pin A
const int encoderPinB = 3;  // Encoder pin B

// -------------------------- Encoder Variables ---------------------------
volatile long encoderPosition = 0;  // Track encoder position

// -------------------------- Control Parameters --------------------------
const long maxEncoderPositionExtend = 80000;    // Maximum encoder counts for extension
const long minEncoderPositionRetract = -80000;  // Minimum encoder counts for retraction
const int motorSpeed = 255;                      // Motor speed for extend/retract
const int holdSpeed = 150;                       // Motor speed for holding position via encoder
const float holdThreshold = 500;                 // Tolerance in encoder counts for holding

// PID Control Parameters
double Setpoint = 0.30;          // Target distance from the ground in meters (30 cm)
double Kp = 800.0;               // Proportional gain
double Ki = 0.0;                 // Integral gain
double Kd = 10.0;                // Derivative gain
double previousError = 0.0;      // Previous error for derivative term
double integral = 0.0;            // Integral accumulator
unsigned long lastPIDTime = 0;    // Last PID computation time
const unsigned long PID_INTERVAL = 50; // PID computation interval in milliseconds
const double maxMotorSpeed = 255;      // Max PWM value for motor speed

// Sensor Data
double sensor_distances[4];      // Array to store distance from 4 Teraranger sensors
bool has_received_data = false;  // Flag to indicate if valid sensor data has been received

// -------------------------- State Machine -------------------------------
enum MotorState { 
    IDLE, 
    EXTENDING, 
    RETRACTING, 
    HOLDING, 
    POSITION_HOLD, 
    ERROR_RETRACTING 
};
MotorState currentState = IDLE;

// Time tracking for ERROR_RETRACTING
unsigned long error_retract_start_time = 0;
const unsigned long ERROR_RETRACT_DURATION = 1000; // 1 second

// -------------------------- ROS Communication ---------------------------
ros::NodeHandle nh;

// Publisher for encoder position
std_msgs::Int32 encoder_msg; // Changed to Int32
ros::Publisher encoder_pub("encoder_position", &encoder_msg);

// Subscriber for extension settings
void extensionCallback(const std_msgs::String& msg);
ros::Subscriber<std_msgs::String> sub_extension("extension_setting", &extensionCallback);

// Subscriber for processed ranges (sensor data)
void distanceCallback(const std_msgs::Float32MultiArray& msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub_processed_ranges("processed_ranges", &distanceCallback);

// -------------------------- ISR for Encoder ----------------------------
void encoderISR() {
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);
  if (stateA == stateB) {
    encoderPosition++;
  } else {
    encoderPosition--;
  }
}

// -------------------------- Callback Functions --------------------------

// Callback for 'extension_setting' topic
void extensionCallback(const std_msgs::String& msg) {
  String command = msg.data;
  Serial.print("Received command: ");
  Serial.println(command);
  nh.loginfo(("Received command: " + command).c_str());

  if (command == "extend") {
    if (encoderPosition >= maxEncoderPositionExtend) {
      Serial.println("Already fully extended.");
      nh.loginfo("Already fully extended.");
      return;
    }
    currentState = EXTENDING;
    Serial.println("State set to EXTENDING.");
    nh.loginfo("State set to EXTENDING.");
  }
  else if (command == "retract") {
    if (encoderPosition <= minEncoderPositionRetract) {
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
    static int stopCommandCount = 0;
    stopCommandCount++;
    if (stopCommandCount >= 2) {
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
      currentState = HOLDING;
      Serial.println("State set to HOLDING.");
      nh.loginfo("State set to HOLDING.");
    }
  }
  else if (command == "position_hold") {
    if (!has_received_data) {
      Serial.println("No valid sensor data received yet. Cannot enter POSITION_HOLD.");
      nh.loginfo("No valid sensor data received yet. Cannot enter POSITION_HOLD.");
      return;
    }
    currentState = POSITION_HOLD;
    Serial.println("State set to POSITION_HOLD.");
    nh.loginfo("State set to POSITION_HOLD.");
  }
  else {
    Serial.println("Unknown command received.");
    nh.loginfo("Unknown command received.");
  }
}

// Callback for 'processed_ranges' topic
void distanceCallback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length < 4) {
    Serial.println("Received insufficient sensor data.");
    nh.loginfo("Received insufficient sensor data.");
    return;
  }

  bool invalid_sensor_detected = false;
  bool positive_inf_detected = false;

  has_received_data = true;
  double minDistance = INFINITY;
  
  for (int i = 0; i < 4; i++) {  // NUMBER OF SENSORS
    double distance = msg.data[i];
    
    // Check if distance is NaN or infinite, or negative
    if (isnan(distance) || isinf(distance) || distance < 0) {
      Serial.print("Sensor ");
      Serial.print(i+1);
      Serial.print(" has invalid data: ");
      Serial.println(distance);
      
      if (distance == -INFINITY) {
        invalid_sensor_detected = true;
      }
      else if (distance == INFINITY) {
        positive_inf_detected = true;
      }
      // For other invalid cases (NaN, other infinities, negative), just skip
      continue;
    }
    
    sensor_distances[i] = distance;
    if (sensor_distances[i] < minDistance) {
      minDistance = sensor_distances[i];
    }
  }

  if (invalid_sensor_detected) {
    // Only trigger ERROR_RETRACTING if not already retracting or holding
    if (currentState != RETRACTING && currentState != HOLDING && currentState != IDLE) {
      // Trigger ERROR_RETRACTING state
      Serial.println("Detected -inf sensor reading. Initiating emergency retraction.");
      nh.loginfo("Detected -inf sensor reading. Initiating emergency retraction.");
      currentState = ERROR_RETRACTING;
      error_retract_start_time = millis();
    } else {
      // If already retracting or holding, do not initiate emergency retraction
      Serial.println("Detected -inf sensor reading, but currently retracting or holding. Ignoring.");
      nh.loginfo("Detected -inf sensor reading, but currently retracting or holding. Ignoring.");
    }
    // Note: Do not return here; allow further processing if needed
  }

  if (positive_inf_detected) {
    // Log the detection but do not take any action
    Serial.println("Detected +inf sensor reading. No action taken.");
    nh.loginfo("Detected +inf sensor reading. No action taken.");
    // Continue processing
  }

  if (isinf(minDistance)) {
    // No valid sensor readings; handle appropriately
    Serial.println("No valid sensor readings. Stopping motor.");
    nh.loginfo("No valid sensor readings. Stopping motor.");
    // For safety, stop the motor
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorPWM, 0);
    // Optionally, set state to IDLE
    currentState = IDLE;
  }
}

// -------------------------- PID Control Functions -----------------------

// Compute PID output based on current distance
double computePID(double currentDistance) {
  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - lastPIDTime) / 1000.0; // in seconds

  if (elapsedTime <= 0.0) {
    return 0.0;
  }

  // Calculate error
  double error = Setpoint - currentDistance;

  // Proportional term
  double Pout = Kp * error;

  // Integral term with anti-windup
  integral += error * elapsedTime;
  double Iout = Ki * integral;

  // Derivative term
  double derivative = (error - previousError) / elapsedTime;
  double Dout = Kd * derivative;

  // Compute total output
  double output = Pout + Iout + Dout;

  // Save error and time for next iteration
  previousError = error;
  lastPIDTime = currentTime;

  // Constrain output to maxMotorSpeed
  output = constrain(output, -maxMotorSpeed, maxMotorSpeed);

  return output;
}

// Control motor based on PID output
void controlMotorPID(double pidOutput) {
  if (pidOutput > 0) {
    // Move motor down
    digitalWrite(motorIN1, HIGH);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorPWM, constrain(pidOutput, 0, maxMotorSpeed));
    Serial.println("PID Control: Moving down...");
    nh.loginfo("PID Control: Moving down...");
  }
  else if (pidOutput < 0) {
    // Move motor up
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, HIGH);
    analogWrite(motorPWM, constrain(-pidOutput, 0, maxMotorSpeed));
    Serial.println("PID Control: Moving up...");
    nh.loginfo("PID Control: Moving up...");
  }
  else {
    // Stop motor
    digitalWrite(motorIN1, LOW);
    digitalWrite(motorIN2, LOW);
    analogWrite(motorPWM, 0);
    Serial.println("PID Control: Stopping motor.");
    nh.loginfo("PID Control: Stopping motor.");
  }
}

// -------------------------- Motor Control Function -----------------------
void controlMotor() {
  switch (currentState) {
    case IDLE:
      // Ensure motor is stopped
      digitalWrite(motorIN1, LOW);
      digitalWrite(motorIN2, LOW);
      analogWrite(motorPWM, 0);
      break;

    case EXTENDING:
      if (encoderPosition < maxEncoderPositionExtend) {
        // Extend: Move motor to increase encoder position (forward direction)
        digitalWrite(motorIN1, HIGH);
        digitalWrite(motorIN2, LOW);  // Forward direction for extension
        analogWrite(motorPWM, motorSpeed);
        Serial.println("Extending...");
        nh.loginfo("Extending...");
      }
      else {
        // Reached max extension
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, LOW);
        analogWrite(motorPWM, 0);
        Serial.println("Reached maximum extension.");
        nh.loginfo("Reached maximum extension.");
        currentState = IDLE;
      }
      break;

    case RETRACTING:
      if (encoderPosition > minEncoderPositionRetract) {
        // Retract: Move motor to decrease encoder position (reverse direction)
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, HIGH);  // Reverse direction for retraction
        analogWrite(motorPWM, motorSpeed);
        Serial.println("Retracting...");
        nh.loginfo("Retracting...");
      }
      else {
        // Reached min retraction
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, LOW);
        analogWrite(motorPWM, 0);
        Serial.println("Reached minimum retraction.");
        nh.loginfo("Reached minimum retraction.");
        currentState = IDLE;
      }
      break;

    case HOLDING:
      // Holding position with ±holdThreshold encoder units tolerance
      {
        long lowerLimit = encoderPosition - holdThreshold;
        long upperLimit = encoderPosition + holdThreshold;

        if (encoderPosition < lowerLimit) {
          // Apply power to move it back up
          digitalWrite(motorIN1, HIGH);  // Move forward
          digitalWrite(motorIN2, LOW);
          analogWrite(motorPWM, holdSpeed);
          Serial.println("Holding: Moving up...");
          nh.loginfo("Holding: Moving up...");
        }
        else if (encoderPosition > upperLimit) {
          // Apply power to move it back down
          digitalWrite(motorIN1, LOW);  // Move backward
          digitalWrite(motorIN2, HIGH);
          analogWrite(motorPWM, holdSpeed);
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

    case POSITION_HOLD:
      // Position hold using PID control based on sensor data
      {
        unsigned long currentTime = millis();
        if (currentTime - lastPIDTime >= PID_INTERVAL) {
          // Find the minimum valid sensor distance
          double currentDistance = INFINITY;
          for (int i = 0; i < 4; i++) {  // NUMBER OF SENSORS
            if (!isinf(sensor_distances[i]) && !isnan(sensor_distances[i])) {
              if (sensor_distances[i] < currentDistance) {
                currentDistance = sensor_distances[i];
              }
            }
          }

          if (!isinf(currentDistance) && !isnan(currentDistance)) {
            // Compute PID output
            double pidOutput = computePID(currentDistance);

            // Control the motor based on PID output
            controlMotorPID(pidOutput);
          } else {
            // Invalid current distance; stop the motor
            digitalWrite(motorIN1, LOW);
            digitalWrite(motorIN2, LOW);
            analogWrite(motorPWM, 0);
            Serial.println("Invalid current distance; stopping motor.");
            nh.loginfo("Invalid current distance; stopping motor.");
            // Optionally, set state to IDLE or take other actions
            currentState = IDLE;
          }
        }
      }
      break;

    case ERROR_RETRACTING:
      {
        unsigned long currentTime = millis();
        if (currentTime - error_retract_start_time < ERROR_RETRACT_DURATION) {
          // Continue retracting at full speed
          digitalWrite(motorIN1, LOW);   // Reverse direction for retraction
          digitalWrite(motorIN2, HIGH);
          analogWrite(motorPWM, motorSpeed);
          Serial.println("Error Retracting: Retracting at full speed...");
          nh.loginfo("Error Retracting: Retracting at full speed...");
        }
        else {
          // Stop motor after retraction duration
          digitalWrite(motorIN1, LOW);
          digitalWrite(motorIN2, LOW);
          analogWrite(motorPWM, 0);
          Serial.println("Error Retracting: Retraction complete. Motor stopped.");
          nh.loginfo("Error Retracting: Retraction complete. Motor stopped.");
          currentState = IDLE;
        }
      }
      break;
  }
}

// ------------------------------ Setup -----------------------------------
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
  nh.subscribe(sub_processed_ranges);

  // Ensure motor is stopped
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, LOW);
  analogWrite(motorPWM, 0);

  // Initialize PID timing
  lastPIDTime = millis();

  Serial.println("Setup complete.");
  nh.loginfo("Setup complete.");
}

// ------------------------------ Loop ------------------------------------
void loop() {
  // Control the motor based on current state
  controlMotor();

  // Publish encoder position
  encoder_msg.data = (int32_t)encoderPosition; // Changed to Int32
  encoder_pub.publish(&encoder_msg);

  // Log encoder position
  String encoderMessage = "Encoder Position: " + String(encoderPosition);
  nh.loginfo(encoderMessage.c_str());
  Serial.println(encoderMessage);

  // Process incoming ROS messages
  nh.spinOnce();

  // Small delay to maintain loop rate (~10 Hz)
  delay(100);
}
