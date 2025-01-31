#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import time
from collections import deque

# Initialize the ROS node
rospy.init_node('metal_detect_node', anonymous=True)

# Create a publisher for the /detection topic
detection_pub = rospy.Publisher('/detection', String, queue_size=10)

# Import threading to handle sleep without blocking callbacks
import threading

# Print available audio devices
print(sd.query_devices())  
device = 0  # Select the audio device

# Settings
duration = 0.01  # Duration in seconds for each sample window
sample_rate = 44100  # Sample rate of microphone
target_frequency = 292  # Frequency to monitor (e.g., 292 Hz)
frequency_threshold = 0.02  # Threshold for detecting target frequency amplitude (adjust as needed)

# Smoothing settings
window_size = 30  # Number of samples to average over for smoothing
amplitude_window = deque(maxlen=window_size)  # Circular buffer to store recent amplitude values

# Cooldown settings
cooldown_period = 0.5  # 0.5 seconds cooldown
last_detection_time = 0

# Lock for thread-safe updates
lock = threading.Lock()

# Function to process audio input and detect frequency spikes
def surfPI(indata, frames, time_info, status):
    global last_detection_time
    # Print status message if available
    if status:
        rospy.logwarn(f"Stream Status: {status}")

    # Apply FFT to the audio data
    # Convert the raw audio data from time domain to frequency domain
    fft_data = np.fft.fft(indata[:, 0])  # Only consider one channel (mono)
    fft_freqs = np.fft.fftfreq(len(fft_data), d=1/sample_rate)
    
    # Get the magnitudes of the FFT result (complex numbers)
    fft_magnitude = np.abs(fft_data)
    
    # Find the closest frequency index to the target frequency
    target_index = np.argmin(np.abs(fft_freqs - target_frequency))
    
    # Check the magnitude of the frequency component at the target frequency
    frequency_amplitude = fft_magnitude[target_index]

    # Add the current frequency amplitude to the window buffer
    amplitude_window.append(frequency_amplitude)
    
    # Calculate the moving average of the amplitudes in the window
    if len(amplitude_window) > 0:
        smoothed_amplitude = np.mean(amplitude_window)
    else:
        smoothed_amplitude = 0

    current_time = time.time()
    with lock:
        time_since_last_detection = current_time - last_detection_time

    # Check if the smoothed frequency amplitude exceeds the threshold and cooldown has passed
    if smoothed_amplitude > frequency_threshold and time_since_last_detection >= cooldown_period:
        detection_message = f"Frequency spike detected at {target_frequency} Hz! Smoothed Amplitude: {smoothed_amplitude}"
        print(detection_message)
        rospy.loginfo(detection_message)
        # Publish the detection message to the /detection topic
        detection_pub.publish(detection_message)
        with lock:
            last_detection_time = current_time
    else:
        no_detection_message = f"No target frequency detected. Smoothed Amplitude: {smoothed_amplitude}"
        print(no_detection_message)
        rospy.loginfo(no_detection_message)

# Start monitoring the audio stream
with sd.InputStream(device=device, callback=surfPI, channels=1, samplerate=sample_rate):
    rospy.loginfo("Listening for frequency spikes...")
    try:
        while not rospy.is_shutdown():
            sd.sleep(int(duration * 1000))  # Continuously monitor in a loop
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down metal_detect_node.")