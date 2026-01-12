#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
TIMEOUT = 0.2
ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

def read_distance():

    if not hasattr(read_distance, "buf"):
        read_distance.buf = bytearray()
        read_distance.distance = 0.0

    global ser
    # đọc tất cả byte đang có
    n = ser.in_waiting
    if n <= 0:
        return read_distance.distance

    read_distance.buf += ser.read(n)
    buf = read_distance.buf

    while len(buf) >= 4:
        # tìm header
        if buf[0] != 0x0A:
            buf.pop(0)
            continue
        frame = buf[:4]

        # checksum XOR
        cs = 0x0A
        for i in range(1, 3):
            cs ^= frame[i]

        if cs != frame[3]:
            buf.pop(0)
            continue

        read_distance.distance = 1.0*((frame[1] << 8) | frame[2])/1000 #m
        print(f"{time.time()}, {read_distance.distance}")

        # bỏ frame đã xử lý
        del buf[:4]
        return read_distance.distance
    
    return read_distance.distance

if __name__ == "__main__":
    while True:
        
        d = read_distance()