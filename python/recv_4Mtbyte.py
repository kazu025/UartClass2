#!/usr/bin/env python3
'''
python.recv_4Mtbyte の Docstring
4Mバイトデータ送信プログラム(Pico側 recv_4Mtbyte.cpp と対になる)
Pico側で受信してデータをそのまま返すことを想定したデータ送信プログラム
'''
import serial, time, os

port = "/dev/ttyUSB0"
#port = "COM4"
baud = 921600
#baud = 115200

ser = serial.Serial(port, baud, timeout=0.2)
time.sleep(0.5)

payload = (b"0123456789ABCDEF" * 256)  # 4096 bytes
for i in range(60000):
    ser.write(payload)
    ser.flush()
    r = ser.read(len(payload))
    if len(r) != len(payload):
        print("short read", i, len(r))
        break
else:
    print("done")
