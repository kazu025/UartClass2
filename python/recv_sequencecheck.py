#!/usr/bin/env python3
'''
python.recv_sequencecheck の Docstring
シーケンスチェック付きデータ送信プログラム(Pico側 recv_sequencecheck.cpp と対になる)
Pico側で受信してシーケンスチェックを行うことを想定したデータ送信プログラム
'''

import serial
import struct
import zlib
import os

PORT = "/dev/ttyUSB0"	# "COM4" for Windows
#BAUD = 115200
BAUD = 921600
BLOCK = 4096
N = 480000
#N = 2000

MAGIC = 0xA55A5AA5

def make_frame(seq: int) -> bytes:
    payload = os.urandom(BLOCK)
    header = struct.pack("<II", MAGIC, seq)  # little-endian: MAGIC, SEQ
    crc = zlib.crc32(header + payload) & 0xFFFFFFFF
    return header + payload + struct.pack("<I", crc)

def write_all(ser: serial.Serial, data: bytes) -> None:
    # 念のため：全バイト書き切るまで回す
    view = memoryview(data)
    sent = 0
    while sent < len(data):
        n = ser.write(view[sent:])
        sent += n

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    try:
        for seq in range(N):
            frame = make_frame(seq)
            write_all(ser, frame)

            if (seq + 1) % 1000 == 0:
                print(f"sent {seq+1}/{N}")
        ser.flush()
        print("DONE")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
