import serial
import serial.tools.list_ports
import struct 
from enum import IntEnum
import keyboard


#nucleo_serial = ["066EFF554852707267073305", "AAAAAAAAA"] # Priit's nucleo
nucleo_serial = ["0672FF484971754867122211", "AAAAAAAAA"] # The Nucleo-F303RE

class CMD(IntEnum):
  MOVE_MOTOR = 1
  READ_ENC = 2  


formats = {
  CMD.MOVE_MOTOR:"<HHHH",
  CMD.READ_ENC:"<II"
}

packet_header_fmt = "<H"
packet_header = 0x2AA2

def make_packet(cmd: CMD):
  packet = None
  match cmd:
    case CMD.MOVE_MOTOR:
      cmd_packet = struct.pack(packet_header_fmt, packet_header, 1, 2, 3, 4)
    case CMD.READ_ENC:
      cmd_packet = struct.pack(packet_header_fmt, packet_header, 1, 2)
  return cmd_packet

def ser_write(ser: serial.Serial, cmd: CMD):
  packet = make_packet(cmd)
  ser.write()
  ser.flush()

def main(ser: serial.Serial):
  print("Press any key to continue...")
  while not keyboard.is_pressed(hotkey="A"):
    print("Tere")
  print(f"You pressed ")
  cmd = CMD.MOVE_MOTOR
  ser_write(ser, cmd)
  ser.read(struct.calcsize(formats[CMD.MOVE_MOTOR])) # Amount of bytes

if __name__ == "__main__":
  timeout = 1
  baud_rate = 9600
  com_ports = serial.tools.list_ports.comports()
  ser = None
  for portinfo in com_ports:
    #print(portinfo.serial_number)
    if portinfo.serial_number in nucleo_serial:
      with serial.Serial(port=portinfo.device ,baudrate=9600, timeout=timeout) as ser:
        while True:
          main(ser)


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





