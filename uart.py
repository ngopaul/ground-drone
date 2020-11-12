import serial
from time import sleep

ser = serial.Serial("/dev/ttyS0", 115200)
received_data = b"short message\0"

while True:
    user_input = str(input("Send: "))
    serial_data = user_input.encode('utf-8') + b"\0"
    ser.write(serial_data)

while True:
    sleep(1)
    print("Sending data:", received_data)
    ser.write(received_data)
    print("Waiting for data.")
    sleep(1)
    received_data = ser.read()
    sleep(0.05) # necessary between read and inwaiting
    data_left = ser.inWaiting()
    received_data += ser.read(data_left)
    print("Received data:", received_data)

