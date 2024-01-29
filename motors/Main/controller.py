from pydualsense import *
import serial
import time

COM = "COM5"
BAUD_RATE = 115200
def send_command_to_arduino(command, arduino_serial):
    arduino_serial.write(command.encode())
    time.sleep(0.1)  # Adjust as needed

def main():
    arduino_serial = None
    try:
        # Initialize DualSense dualsense
        dualsense = pydualsense()
        dualsense.init()

        arduino_serial = serial.Serial(COM, BAUD_RATE)

        print("DualSense initialized. Press R2 to send command to Arduino.")

        while not dualsense.state.R1:
            # Check R2 button press

            speed_to_send = int(map(dualsense.state.R2, 0, 255, 0, 100))  # Map R2 pressure to speed in the range 0-100
            if dualsense.state.R2 > 20:
                steps_to_send = 100  # Replace with the desired number of steps
                command_to_send = f"up_{steps_to_send}_{speed_to_send}"
                # Send "up" command to Arduino
                send_command_to_arduino(command_to_send, arduino_serial)

                print(f"Command sent to Arduino: {command_to_send}")

            # Check L2 button press
            speed_to_send = int(map(dualsense.state.L2, 0, 255, 0, 100))  # Map R2 pressure to speed in the range 0-100

            if dualsense.state.L2 > 20:
                steps_to_send = 100  # Replace with the desired number of steps
                command_to_send = f"up_{steps_to_send}_{speed_to_send}"
                # Send "down" command to Arduino
                send_command_to_arduino(command_to_send, arduino_serial)

                print(f"Command sent to Arduino: {command_to_send}")
            if arduino_serial.in_waiting > 0:
                arduino_data = arduino_serial.readline().decode('utf-8').rstrip()

                # Check if the received data is "Maximum"
                if arduino_data == "Maximum":
                    dualsense.setLeftMotor(255)
                    dualsense.setRightMotor(255)
            time.sleep(0.1)  # Adjust the delay as needed

    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        if arduino_serial is not None and hasattr(arduino_serial, 'close'):
            # Close the serial connection
            arduino_serial.close()  

if __name__ == "__main__":
    main()
