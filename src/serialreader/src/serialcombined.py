#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
import serial
import re
import sys

class CombinedSerialROSNode:
    def __init__(self):
        rospy.init_node('combined_serial_ros_node', anonymous=True)

        # Serial connection parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 57600)
        self.timeout = rospy.get_param('~timeout', 1)

        # Publishers
        self.extension_pub = rospy.Publisher('extension_setting', String, queue_size=10)
        self.altitude_pub = rospy.Publisher('altitude_set', Float32, queue_size=10)

        # Subscriber to send altitude over serial if needed (can be omitted if not used)
        # self.altitude_sub = rospy.Subscriber('current_altitude', Float32, self.send_altitude)

        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            rospy.loginfo(f"Connected to {self.port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to {self.port}: {e}")
            sys.exit(1)

        # Define retract position (encoder counts)
        self.original_retracted_position = 0  # Assuming original retracted position is encoder count 0
        self.retract_position = 50  # 5% of original (if original is 0, set to 50)

        # Regular expressions for command parsing
        self.extend_pattern = re.compile(r'^\s*Extend\s*$', re.IGNORECASE)
        self.retract_pattern = re.compile(r'^\s*Retract\s*$', re.IGNORECASE)
        self.stop_pattern = re.compile(r'^\s*Stop\s*$', re.IGNORECASE)
        self.position_hold_pattern = re.compile(r'^\s*Position_Hold\s*$', re.IGNORECASE)  # New pattern
        # Altitude commands can be retained for flexibility
        self.altitude_pattern = re.compile(r'^\s*Altitude_(\d+(\.\d+)?)\s*$', re.IGNORECASE)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                incoming_bytes = self.ser.readline()
                if not incoming_bytes:
                    rate.sleep()
                    continue  # No data received

                incoming_message = incoming_bytes.decode('utf-8').strip()
                if incoming_message:
                    rospy.loginfo(f"Received: {incoming_message}")
                    self.process_message(incoming_message)
            except serial.SerialException as e:
                rospy.logerr(f"Serial exception: {e}")
                break
            except UnicodeDecodeError as e:
                rospy.logwarn(f"Unicode decode error: {e}")
                continue
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
                break
            rate.sleep()

        # Clean up
        self.ser.close()
        rospy.loginfo("Serial connection closed.")

    def process_message(self, message):
        if self.extend_pattern.match(message):
            self.publish_extension('extend')
            self.publish_altitude(1.0)  # Set target altitude to 1m
            rospy.loginfo("Processed 'Extend' command: Setting altitude to 1.0m")
            return

        if self.retract_pattern.match(message):
            self.publish_extension('retract')
            self.publish_altitude(float(self.retract_position))
            rospy.loginfo(f"Processed 'Retract' command: Setting altitude to {self.retract_position} encoder counts")
            return

        if self.stop_pattern.match(message):
            self.publish_extension('stop')
            rospy.loginfo("Processed 'Stop' command: Holding current position")
            return

        if self.position_hold_pattern.match(message):
            self.publish_extension('position_hold')
            rospy.loginfo("Processed 'Position_Hold' command: Engaging position hold mode")
            return

        # Handle Altitude_X commands if needed
        altitude_match = self.altitude_pattern.match(message)
        if altitude_match:
            altitude_value = float(altitude_match.group(1))
            self.publish_altitude(altitude_value)
            rospy.loginfo(f"Processed 'Altitude' command: Setting altitude to {altitude_value}m")
            return

        rospy.logwarn(f"Unrecognized command: {message}")

    def publish_extension(self, state):
        msg = String()
        msg.data = state
        self.extension_pub.publish(msg)
        rospy.loginfo(f"Published to 'extension_setting': {state}")

    def publish_altitude(self, altitude):
        msg = Float32()
        msg.data = altitude
        self.altitude_pub.publish(msg)
        rospy.loginfo(f"Published to 'altitude_set': {altitude}")

    def send_altitude(self, altitude_msg):
        try:
            altitude_value = altitude_msg.data
            self.ser.write(f"Altitude_{altitude_value}\n".encode('utf-8'))
            rospy.loginfo(f"Sent altitude over serial: Altitude_{altitude_value}")
        except Exception as e:
            rospy.logerr(f"Error sending altitude over serial: {e}")

if __name__ == '__main__':
    try:
        node = CombinedSerialROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

