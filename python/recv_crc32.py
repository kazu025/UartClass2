#!/usr/bin/env python3
'''
python.recv_crc32 の Docstring
CRC32付きデータ送信プログラム(Pico側 recv_crc32.cpp と対になる)
Pico側で受信してCRC32チェックを行うことを想定したデータ送信プログラム	
'''
import serial, struct, zlib, os

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
#ser = serial.Serial("COM4", 115200, timeout=1)

BLOCK = 4096
#N = 100
N = 60000

for i in range(N):
    payload = os.urandom(BLOCK)  # random data
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    frame = payload + struct.pack("<I", crc)  # add little-endian CRC32
    ser.write(frame)

ser.flush()
ser.close()
