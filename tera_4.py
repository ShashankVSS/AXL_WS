#!/usr/bin/env python
import rospy
from teraranger_array.msg import RangeArray
from std_msgs.msg import Float32MultiArray

def callback(data):
    if len(data.ranges) >= 5:  # Ensure the array has at least 5 elements
        range_values = []  # Initialize an empty list to store the values

        # Instead of looping through the first 4 sensors, we select specific sensors
        selected_indices = [0, 1, 3, 4]  # Skip the 3rd sensor and use the 1st, 2nd, 4th, 5th sensors
        
        for i in selected_indices:
            range_value = data.ranges[i].range  # Get the range value for each selected sensor
            if not range_value != range_value:  # Check if the range value is not nan (nan != nan is True for NaN check)
                range_values.append(range_value)
                rospy.loginfo(f"Sensor {i+1} range value: {range_value}")
            else:
                rospy.logwarn(f"Sensor {i+1} range value is nan")
                range_values.append(float('nan'))  # Append nan if the value is invalid

        # Create a Float32MultiArray message to publish the selected sensor values
        range_msg = Float32MultiArray(data=range_values)
        range_pub.publish(range_msg)
    else:
        rospy.logwarn("The range array does not have enough elements!")

if __name__ == '__main__':
    rospy.init_node('range_processor')
    
    # Create a publisher that sends an array of float32 values
    range_pub = rospy.Publisher('processed_ranges', Float32MultiArray, queue_size=10)
    
    # Subscribe to the ranges topic
    rospy.Subscriber('ranges', RangeArray, callback)
    
    rospy.spin()
