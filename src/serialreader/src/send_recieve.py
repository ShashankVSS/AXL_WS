import serial
import time
import threading

# Define the serial port and baud rate
PORT = "/dev/ttyUSB0"  # Adjust this based on your actual Linux port
BAUD_RATE = 57600  # Ensure this matches the device's baud rate

# Initialize the serial connection
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Failed to connect to {PORT}: {e}")
    exit()

def send_data():
    """Function to continuously send data over serial."""
    num = 1
    increment = 1  # Determines whether to count up or down

    try:
        while True:
            # Send the current number over serial with a newline character
            ser.write(f"{num}\n".encode('utf-8'))
            print(f"Sent: {num}")
            
            # Update the number for the next iteration
            num += increment
            
            # Reverse the counting direction at the bounds
            if num == 100:
                increment = -1  # Start counting down
            elif num == 1:
                increment = 1   # Start counting up

            # Wait for 0.5 seconds before sending the next number
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

def listen_data():
    """Function to continuously listen to incoming serial data."""
    try:
        while True:
            # Read a line from the serial port
            incoming_message = ser.readline().decode('utf-8').strip()

            if incoming_message:
                print(f"Received: {incoming_message}")
                # Optional: Send an acknowledgment back over serial
                outgoing_message = f"Acknowledged: {incoming_message}"
                ser.write(outgoing_message.encode('utf-8'))

    except KeyboardInterrupt:
        print("\nListening stopped by user.")
    finally:
        ser.close()
        print("Serial connection closed.")

# Use threads to run sending and receiving simultaneously
send_thread = threading.Thread(target=send_data)
listen_thread = threading.Thread(target=listen_data)

# Start both threads
send_thread.start()
listen_thread.start()

# Join the threads to ensure the program doesn't exit prematurely
send_thread.join()
listen_thread.join()
