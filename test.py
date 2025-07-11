import serial
from serial.tools import list_ports

ports = [p.device for p in list_ports.comports()]
print("Available ports:", ports)