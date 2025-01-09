import time

import serial

# Set up the serial connection
ser = serial.Serial('COM7', 115200, timeout=0)

# Wait for Arduino to reset
time.sleep(2)

print("Enter serial command to send to Arduino (send an empty string to quit):")

while True:
    command = input("Send: ")
    if command == "":
        break

    # Ensure correct length (6 digits)
    ser.write((command + '\n').encode())  # Send the hex color code

# Close the serial connection
ser.close()
