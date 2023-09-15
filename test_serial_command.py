# install pyserial with: pip install pyserial
import serial
import time

# use pico serial port, for example COM4 in windows
ser = serial.Serial("/dev/ttyACM0", 115200)

def adelante(speed):
    ser.write(f"{speed},{speed}/".encode())

def atras(speed):
    ser.write(f"-{speed},-{speed}/".encode())

def izquierda(speed):
    ser.write(f"-{speed},{speed}/".encode())

def derecha(speed):
    ser.write(f"{speed},-{speed}/".encode())


adelante(6.28)
time.sleep(1)

izquierda(3.1)
time.sleep(0.5)

adelante(6.28)
time.sleep(1)

izquierda(3.1)
time.sleep(0.5)

adelante(6.28)
time.sleep(1)

izquierda(3.1)
time.sleep(0.5)
