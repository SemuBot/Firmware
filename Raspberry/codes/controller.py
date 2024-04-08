import serial
import serial.tools.list_ports

from pydualsense import *
import comm
import time


def main(ser: serial.Serial):
  if dualsense.state.R1:
      raise KeyboardInterrupt  # Raise KeyboardInterrupt to exit the loop
  vals = comm.MotorValues()

  #print("Press any key to continue...")
  print(dualsense.state.LX)
  if dualsense.state.LX < -20:
    vals.m1 = 100
  if dualsense.state.LX > 20:
    vals.m1 = 100


  if dualsense.state.LY < -20:
    vals.m2 = 100
  if dualsense.state.LY > 20:
    vals.m2 = 100
    
  if dualsense.state.RX < -20:
    vals.m3 = 100
  if dualsense.state.RX > 20:
    vals.m3 = 100
    
    
  if dualsense.state.RY < -20:
    vals.m4 = 100
  if dualsense.state.RY > 20:
    vals.m4 = 100
    
  comm.ser_write(ser, packet=comm.ser_make_motor_packet(vals))
  time.sleep(0.05)
  
if __name__ == "__main__":
  ser = comm.ser_init(9600, 5)
  try:
    dualsense = pydualsense()
    dualsense.init()
    dualsense.light.setColorI(255,165,0)
    while True:
      
      main(ser)
  except KeyboardInterrupt:
    comm.ser_close(ser)
    print("Exiting")
    dualsense.close()




