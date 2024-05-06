import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum

#nucleo_serial = "066CFF505057717867095232"
nucleo_serial = "0672FF484971754867122211"

# Not necessary currently
class CMD(IntEnum):
  MOVE_MOTOR = 1
  READ_ENC = 2  

class MotorValues():
  def __init__(self):
    self.m1_speed = 0
    self.m1_steps = 0
    self.m2_speed = 0
    self.m2_steps = 0
    self.m3_speed = 0
    self.m3_steps = 0
    self.m4_speed = 0
    self.m4_steps = 0
    self.m5_speed = 0
    self.m5_steps = 0

packet_header_fmt = "<H"
packet_cmd_fmt = "B"
packet_crc_fmt = "H"

packet_header = 0x4994
formats = {
  CMD.MOVE_MOTOR: "bBbBbBbBbB",
  CMD.READ_ENC: "HHHH"
}

def ser_init(baud_rate: int, time_out: int)->serial.Serial:
  com_ports = serial.tools.list_ports.comports()
  for port in com_ports:
    if port.serial_number == nucleo_serial:
      return serial.Serial(port.device, baudrate=baud_rate, timeout=time_out)

def ser_close(ser: serial.Serial):
  ser.close()

def ser_make_cmd_format(cmd: CMD):
  return packet_header_fmt + packet_cmd_fmt + formats[cmd]

def ser_packet_size(cmd):
  return struct.calcsize(packet_header_fmt) + struct.calcsize(packet_cmd_fmt) + struct.calcsize(formats[cmd])

def ser_make_motor_packet(values: MotorValues):
  cmd = CMD.MOVE_MOTOR
  return struct.pack(ser_make_cmd_format(cmd), packet_header, cmd, values.m1_speed, values.m1_steps, values.m2_speed, 
                     values.m2_steps, values.m3_speed, values.m3_steps, values.m4_speed, values.m4_steps, values.m5_speed, values.m5_steps)


def ser_write(ser: serial.Serial, packet: bytes):
  print(list(packet))
  ser.write(packet)
  print(packet)
  ser.flush()

def ser_read(ser: serial.Serial, cmd: CMD):
  read_bytes = ser.read(ser_packet_size(cmd))
  print(read_bytes)


if __name__ == "__main__":
  com_ports = serial.tools.list_ports.comports() 
  for port in com_ports:
    print(port.device, " ", port.serial_number)

'''
def make_packet(cmd: CMD):
  packet = None
  match cmd:
    case CMD.MOVE_MOTOR:
      packet = struct.pack(formats[cmd], packet_header, 1, 2, 3, 4)
    case CMD.READ_ENC:
      packet = struct.pack(formats[cmd], packet_header, 1, 2)
  
  return packet'''