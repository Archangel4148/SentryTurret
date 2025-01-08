import serial
import time

# Set up the serial connection (adjust COM port as needed)
ser = serial.Serial('COM7', 115200, timeout=0)

# Wait for Arduino reset (if needed)
time.sleep(2)

print("Enter hex color code to send to Arduino (type 'exit' to quit):")

while True:
    # Get user input (hex color)
    command = input("Send: ")

    if command == "":
        break

    # Ensure correct length (6 digits)
    ser.write((command + '\n').encode())  # Send the hex color code

# Close the serial connection
ser.close()
