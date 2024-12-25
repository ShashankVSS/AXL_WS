#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
import serial
import re
import sys

class SerialROSNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('serial_listener', anonymous=True)

        # Parameters (can be modified via ROS parameters or launch files)
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud_rate', 57600)
        self.timeout = rospy.get_param('~timeout', 1)

        # Publishers
        self.extension_pub = rospy.Publisher('extension_request', String, queue_size=10)
        self.altitude_pub = rospy.Publisher('altitude_setting', Float32, queue_size=10)

        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            rospy.loginfo(f"Connected to {self.port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to {self.port}: {e}")
            sys.exit(1)

        # Regular expressions for parsing commands
        self.extend_pattern = re.compile(r'^\s*Extend\s*$', re.IGNORECASE)
        self.retract_pattern = re.compile(r'^\s*Retract\s*$', re.IGNORECASE)
        self.altitude_pattern = re.compile(r'^\s*Altitude_(\d+)\s*$', re.IGNORECASE)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            try:
                # Read a line from the serial port
                incoming_bytes = self.ser.readline()
                if not incoming_bytes:
                    rate.sleep()
                    continue  # Timeout occurred, no data received

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

        # Clean up serial connection
        self.ser.close()
        rospy.loginfo("Serial connection closed.")

    def process_message(self, message):
        # Check for Extend command
        if self.extend_pattern.match(message):
            self.publish_extension('extend')
            return

        # Check for Retract command
        if self.retract_pattern.match(message):
            self.publish_extension('retract')
            self.publish_altitude(0.0)  # Set altitude to 0 when retracting
            return

        # Check for Altitude command
        altitude_match = self.altitude_pattern.match(message)
        if altitude_match:
            altitude_value = int(altitude_match.group(1))
            if 1 <= altitude_value <= 10:
                self.publish_altitude(float(altitude_value))
            else:
                rospy.logwarn(f"Received altitude value out of range: {altitude_value}")
            return

        rospy.logwarn(f"Unrecognized command: {message}")

    def publish_extension(self, state):
        msg = String()
        msg.data = state
        self.extension_pub.publish(msg)
        rospy.loginfo(f"Published to extension_request: {state}")

    def publish_altitude(self, altitude):
        msg = Float32()
        if altitude == 0.0:
            msg.data = 0.0  # You can choose to set to float('nan') if preferred
        else:
            msg.data = altitude
        self.altitude_pub.publish(msg)
        rospy.loginfo(f"Published to altitude_setting: {msg.data}")

if __name__ == '__main__':
    try:
        node = SerialROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

