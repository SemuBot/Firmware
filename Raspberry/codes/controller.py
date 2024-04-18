import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum
import keyboard
import ser_nucleo
import time
from pydualsense import *
steps = 4
speed = 8

def main(ser: serial.Serial):
  vals = ser_nucleo.MotorValues()
  if dualsense.state.R1:
    raise KeyboardInterrupt  # Raise KeyboardInterrupt to exit the loop
  if dualsense.state.LX < -40:
    vals.m1_speed = -speed
    vals.m1_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.LX > 40:
    vals.m1_speed = speed
    vals.m1_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)

  if dualsense.state.LY < -40:
    vals.m2_speed = -speed
    vals.m2_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.LY > 40:
    vals.m2_speed = speed
    vals.m2_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RX < -40:
    vals.m3_speed = -speed
    vals.m3_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if dualsense.state.RX > 40:
    vals.m3_speed = speed
    vals.m3_steps = steps
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
    vals.m5_speed = speed
    vals.m5_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)   
  if dualsense.state.triangle:
    vals.m4_speed = speed
    vals.m4_steps = steps
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)  
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




