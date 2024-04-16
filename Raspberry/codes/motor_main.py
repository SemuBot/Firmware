import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum
import keyboard
import ser_nucleo
import time


def main(ser: serial.Serial):
  #print("Press any key to continue...")
  vals = ser_nucleo.MotorValues()
  if keyboard.is_pressed(hotkey="W"):
    print("W pressed")
    vals.m2_speed = -8
    vals.m2_steps = 5
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  if keyboard.is_pressed(hotkey="A"):
    print("A pressed")
    vals.m2_speed = 8
    vals.m2_steps = 5
    ser_nucleo.ser_write(ser, packet=ser_nucleo.ser_make_motor_packet(vals))
    time.sleep(0.1)
  
  
if __name__ == "__main__":
  ser = ser_nucleo.ser_init(9600, 5)
  try:
    while True:
      main(ser)
  except KeyboardInterrupt:
    ser_nucleo.ser_close(ser)



  '''com_ports = serial.tools.list_ports.comports()
  ser = None
  for portinfo in com_ports:
    #print(portinfo.serial_number)
    if portinfo.serial_number in nucleo_serial:
      with serial.Serial(port=portinfo.device ,baudrate=9600, timeout=timeout) as ser:
        while True:
          main(ser)
'''

'''uint16_t crc16(const uint8_t *data_p, uint8_t length) {
	uint8_t x;
	uint16_t crc = 0xFFFF;

	while (length--) {
		x = crc >> 8 ^ *data_p++;
		x ^= x >> 4;
		crc = (crc << 8) ^ ((uint16_t) (x << 12)) ^ ((uint16_t) (x << 5))
				^ ((uint16_t) x);
	}
	return crc;
}'''

'''if keyboard.is_pressed(hotkey="A"):
    print("A pressed")
    vals.m1 = -100
  if keyboard.is_pressed(hotkey="E"):
    print("E pressed")
    vals.m2 = 100
  if keyboard.is_pressed(hotkey="S"):
    print("S pressed")
    vals.m2 = -100
  if keyboard.is_pressed(hotkey="R"):
    print("R pressed")
    vals.m3 = 100
  if keyboard.is_pressed(hotkey="D"):
    print("D pressed")
    vals.m3 = -100
  if keyboard.is_pressed(hotkey="T"):
    print("T pressed")
    vals.m4 = 100
  if keyboard.is_pressed(hotkey="F"):
    print("F pressed")
    vals.m4 = -100'''


