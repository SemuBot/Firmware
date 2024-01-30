from pydualsense import *
import serial
import time

COM = "COM6"
BAUD_RATE = 115200
speed = 50
steps_to_send = 0
arduino = serial.Serial(port=COM,baudrate=BAUD_RATE)


def write_read(x):
    arduino.write(bytes(x,   'utf-8'))
    time.sleep(0.5)
    #data = arduino.readline()
    #return   data

def linear_map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def main():
    #arduino_serial = None
    try:
        # Initialize DualSense dualsense
        dualsense = pydualsense()
        dualsense.init()
        dualsense.light.setColorI(255, 165, 0)
        #dualsense.triggerL.setMode(TriggerModes.Rigid  )
        #dualsense.triggerL.setForce(1, 255)

        #dualsense.triggerR.setMode(TriggerModes.Rigid  )
        #dualsense.triggerR.setForce(0, 200)
        #dualsense.triggerR.setForce(1, 255)
        #dualsense.triggerR.setForce(2, 175)

        print("DualSense initialized. Press R2 to send command to Arduino.")
        while True:
            # Check R1 button press
            if dualsense.state.R1:
                raise KeyboardInterrupt  # Raise KeyboardInterrupt to exit the loop

            # Check R2 button press
            r2_state = dualsense.state.R2

            if r2_state > 20:
                steps_to_send = linear_map(r2_state, 20, 255, 10, 1000)
                up(steps_to_send,speed)
            elif dualsense.state.L2 > 20:
                steps_to_send = linear_map(dualsense.state.L2, 20, 255, 10, 1000)
                down(steps_to_send,speed)
            elif dualsense.state.square:
                up(500,speed)
            elif dualsense.state.circle:
                down(500,speed)


    except KeyboardInterrupt:
        print("Exiting.")
        dualsense.triggerL.setForce(0, 0)
        dualsense.triggerR.setForce(0, 0)
        dualsense.close()

def down(steps,speedtosend):
    command_to_send = f"down_{steps}_{speedtosend}"
    write_read(command_to_send)
    time.sleep(0.01)
    write_read("")
    time.sleep(0.01)
    print(f"Command sent to Arduino: {command_to_send}")

def up(steps,speedtosend):
    command_to_send = f"up_{steps}_{speedtosend}"
    write_read(command_to_send)
    time.sleep(0.01)
    write_read("")
    time.sleep(0.01)
    print(f"Command sent to Arduino: {command_to_send}")



if __name__ == "__main__":
    main()
