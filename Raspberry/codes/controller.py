import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum
import keyboard
import ser_nucleo
import time
from pydualsense import *
global speed,steps, down_pressed, up_pressed
steps = 4
speed = 8
speed_limit = 8
up_pressed = False
down_pressed = False

def main(ser: serial.Serial):
  global speed, steps, down_pressed, up_pressed
  vals = ser_nucleo.MotorValues()
  if dualsense.state.R1:
    raise KeyboardInterrupt  # Raise KeyboardInterrupt to exit the loop
  if dualsense.state.LX < -40:
    vals.m3_speed = speed
    vals.m3_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.LX > 40:
    vals.m3_speed = -speed
    vals.m3_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)

  if dualsense.state.LY < -40:
    vals.m2_speed = speed
    vals.m2_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.LY > 40:
    vals.m2_speed = -speed
    vals.m2_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RX < -40:
    vals.m1_speed = speed
    vals.m1_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RX > 40:
    vals.m1_speed = -speed
    vals.m1_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RY < -40:
    vals.m4_speed = -speed
    vals.m4_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RY > 40:
    vals.m4_speed = speed
    vals.m4_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1) 
  if dualsense.state.circle:
    vals.m3_speed = speed
    vals.m3_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)   
  if dualsense.state.triangle:
    vals.m3_speed = -speed
    vals.m3_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)  
  if dualsense.state.L1:
    vals.m1_steps = 0
    vals.m2_steps = 0
    vals.m3_steps = 0
    vals.m4_steps = 0
    vals.m5_steps = 0

    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)  
  if dualsense.state.DpadUp:
      if speed < speed_limit:
          if not up_pressed:  # Check if D-pad up has been pressed previously
              speed += 1
              up_pressed = True  
              down_pressed = False  
              print(speed)
  elif dualsense.state.DpadDown:
      if speed > 1:
          if not down_pressed:  # Check if D-pad down has been pressed previously
              speed -= 1
              down_pressed = True  # Set flag to indicate D-pad down has been pressed
              up_pressed = False  # Reset flag for D-pad up
              print(speed)
  else:
      down_pressed = False  # Reset the flag 
      up_pressed = False 
  
if __name__ == "__main__":
  ser = ser_nucleo.ser_init(9600, 5)
  try:
    dualsense = pydualsense()
    dualsense.init()
    dualsense.light.setColorI(255,165,0)
    while True:
      main(ser)
  except KeyboardInterrupt:
    ser_nucleo.ser_close(ser)
    print("Exiting")
    dualsense.close()




