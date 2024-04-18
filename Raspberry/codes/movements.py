import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum
import keyboard
import ser_nucleo
import time

steps = 16
speed = 8


def wave(ser: serial.Serial):
    vals = ser_nucleo.MotorValues()
    vals.m2_speed = -speed
    vals.m2_steps = 17
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))# Otse üles
    time.sleep(5)
    
    
    vals = ser_nucleo.MotorValues()
    vals.m3_speed = speed
    vals.m3_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals)) #Maapinna suhtes keerab üles
    time.sleep(2)
    
    
    vals = ser_nucleo.MotorValues()
    vals.m4_speed = speed
    vals.m4_steps = 12
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals)) #Küünarnukk
    time.sleep(2)
    
    vals = ser_nucleo.MotorValues() #Lehvitamine
    vals.m1_speed = speed
    vals.m1_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)

    vals = ser_nucleo.MotorValues() 
    vals.m1_speed = -speed
    vals.m1_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)
    
    vals = ser_nucleo.MotorValues()
    vals.m1_speed = speed
    vals.m1_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)
    
    vals = ser_nucleo.MotorValues()
    vals.m1_speed = -speed
    vals.m1_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)
    
    
    vals = ser_nucleo.MotorValues() #Küünarnukk alla
    vals.m4_speed = -speed
    vals.m4_steps = 12
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)
    

    vals = ser_nucleo.MotorValues()
    vals.m3_speed = -speed
    vals.m3_steps = 8
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(2)
    vals = ser_nucleo.MotorValues()
    vals.m2_speed = speed
    vals.m2_steps = 20
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(4)
def main(ser: serial.Serial):
  if keyboard.is_pressed(hotkey="W"):
    print("W pressed")
    wave(ser)
    time.sleep(0.1)

if __name__ == "__main__":
  ser = ser_nucleo.ser_init(9600, 5)
  try:
    while True:
      main(ser)
  except KeyboardInterrupt:
    ser_nucleo.ser_close(ser)
