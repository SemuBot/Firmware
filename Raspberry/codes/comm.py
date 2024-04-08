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
  m1 = 0
  m2 = 0
  m3 = 0
  m4 = 0

packet_header_fmt = "<H"
packet_cmd_fmt = "B"
packet_header = 0x4994
formats = {
  CMD.MOVE_MOTOR: "bbbb",
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
  return struct.pack(ser_make_cmd_format(cmd), packet_header, cmd, values.m1, values.m2, values.m3, values.m4)


def ser_write(ser: serial.Serial, packet: bytes):
  ser.write(packet)
  ser.flush()

def ser_read(ser: serial.Serial, cmd: CMD):
  read_bytes = ser.read(ser_packet_size(cmd))

'''
def make_packet(cmd: CMD):
  packet = None
  match cmd:
    case CMD.MOVE_MOTOR:
      packet = struct.pack(formats[cmd], packet_header, 1, 2, 3, 4)
    case CMD.READ_ENC:
      packet = struct.pack(formats[cmd], packet_header, 1, 2)
  
  return packet'''