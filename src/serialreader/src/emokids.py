import sounddevice as sd
import numpy as np
import time

print(sd.query_devices())  # Print available audio devices
device = int(input("Enter the device ID: "))  # Select the audio device

# Settings
duration = 0.01  # Duration in seconds for each sample window
sample_rate = 44100  # Sample rate of microphone
threshold = 0.001  # Volume threshold for detection (adjust as needed)

# Function to process audio input and detect target
def surfPI(indata, frames, time, status): 
    # Print status message if available
    if status:
        print(status)
    # Calculate RMS volume level
    volume_norm = np.linalg.norm(indata) / np.sqrt(len(indata))
    
    # if volume_norm > 0.:
    #     print(volume_norm)

    print(indata)
    
    # Trigger detection if volume exceeds the threshold
    if volume_norm > threshold:
        print("Metal target detected!")
    else:
        print("No target detected...")

# Start monitoring the audio stream
with sd.InputStream(device=device, callback=surfPI, channels=1, samplerate=sample_rate):
    print("Listening for metal detector output...")
    while True:
        sd.sleep(int(duration * 1000))  # Continuously monitor in a loop
