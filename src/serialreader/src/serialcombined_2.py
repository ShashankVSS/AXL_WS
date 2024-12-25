#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32, Int32, Float32MultiArray
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

        # Subscribers for sending data over serial
        self.extension_sub = rospy.Subscriber('/extension_setting', String, self.extension_data_callback)
        self.processed_ranges_sub = rospy.Subscriber('/processed_ranges', Float32MultiArray, self.processed_ranges_callback)
        self.encoder_position_sub = rospy.Subscriber('/encoder_position', Int32, self.encoder_position_callback)
        # self.detection_sub = rospy.Subscriber('/detection', String, self.detection_callback)  # Uncomment when needed

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
        self.auto_adjust_on_pattern = re.compile(r'^\s*AUTO_ADJUST_ON\s*$', re.IGNORECASE)  # New pattern
        self.auto_adjust_off_pattern = re.compile(r'^\s*AUTO_ADJUST_OFF\s*$', re.IGNORECASE)  # New pattern
        # Altitude commands can be retained for flexibility
        self.altitude_pattern = re.compile(r'^\s*Altitude_(\d+(\.\d+)?)\s*$', re.IGNORECASE)

        # Define ping and pong messages
        self.PING_MESSAGE = "PING"
        self.PONG_MESSAGE = "PONG"

        # Prefix code for data messages
        self.DATA_PREFIX = "DATA|"

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

        if self.auto_adjust_on_pattern.match(message):
            self.publish_extension('position_hold')
            rospy.loginfo("Processed 'AUTO_ADJUST_ON' command: Setting extension to 'position_hold'")
            return

        if self.auto_adjust_off_pattern.match(message):
            self.publish_extension('stop')
            rospy.loginfo("Processed 'AUTO_ADJUST_OFF' command: Setting extension to 'stop'")
            return

        if message == self.PING_MESSAGE:
            self.send_pong()
            rospy.loginfo("Received 'PING'. Sent 'PONG'")
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
        rospy.loginfo(f"Published to '/extension_setting': {state}")

    def publish_altitude(self, altitude):
        msg = Float32()
        msg.data = altitude
        self.altitude_pub.publish(msg)
        rospy.loginfo(f"Published to '/altitude_set': {altitude}")

    def send_pong(self):
        try:
            pong_message = f"{self.PONG_MESSAGE}\n"
            self.ser.write(pong_message.encode('utf-8'))
            rospy.loginfo(f"Sent 'PONG' message")
        except serial.SerialException as e:
            rospy.logerr(f"Error sending 'PONG': {e}")

    def send_serial_data(self, topic, data):
        """
        Formats and sends data over serial with a prefix code.
        Format: DATA|topic_name:data\n
        """
        try:
            # Convert data to string
            if isinstance(data, list):
                data_str = "[" + ", ".join([str(d) for d in data]) + "]"
            else:
                data_str = str(data)

            # Create the serial message
            serial_message = f"{self.DATA_PREFIX}{topic}:{data_str}\n"
            self.ser.write(serial_message.encode('utf-8'))
            rospy.loginfo(f"Sent data over serial: {serial_message.strip()}")
        except serial.SerialException as e:
            rospy.logerr(f"Error sending data over serial: {e}")

    # -------------------------- Subscriber Callbacks --------------------------

    def extension_data_callback(self, msg):
        """
        Callback for '/extension_setting' topic.
        Sends the extension setting over serial.
        """
        state = msg.data
        self.send_serial_data("extension_setting", state)

    def processed_ranges_callback(self, msg):
        """
        Callback for '/processed_ranges' topic.
        Sends the processed ranges over serial.
        """
        ranges = msg.data
        self.send_serial_data("processed_ranges", ranges)

    def encoder_position_callback(self, msg):
        """
        Callback for '/encoder_position' topic.
        Sends the encoder position over serial.
        """
        position = msg.data
        self.send_serial_data("encoder_position", position)

    # Uncomment and implement when /detection topic is available
    # def detection_callback(self, msg):
    #     """
    #     Callback for '/detection' topic.
    #     Sends detection data over serial.
    #     """
    #     detection = msg.data
    #     self.send_serial_data("detection", detection)

if __name__ == '__main__':
    try:
        node = CombinedSerialROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass