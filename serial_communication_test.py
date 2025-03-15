import serial
import time

# Open the serial port
arduino = serial.Serial(
    "COM7", 9600, timeout=1
)  # Replace 'COM4' with your Arduino's COM port


def send_command(command):
    arduino.write(command.encode("utf-8"))
    time.sleep(0.1)  # Wait a bit for the command to be processed


# Example usage
while True:
    command = input("Enter command (0/1/2/3): ")
    send_command(command)
    if command == "exit":
        break

# Close the serial port
arduino.close()
