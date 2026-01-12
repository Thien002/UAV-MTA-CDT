#!/usr/bin/env python3
import serial
import time
import threading

class UltrasonicReader:
    def __init__(self, port, baudrate=9600, timeout=0.05,
                 min_valid_m=0.05, max_valid_m=4.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.buf = bytearray()
        self.min_valid_m = min_valid_m
        self.max_valid_m = max_valid_m
        self._lock = threading.Lock()
        self._distance = 0.0
        self._timestamp = 0.0
        self._run = False
        self._thread = None

    def start(self):
        if self._run:
            return
        self.ser.reset_input_buffer()
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=0.5)
        try:
            self.ser.close()
        except:
            pass

    def latest(self):
        now = time.time()
        with self._lock:
            d, t = self._distance, self._timestamp
        if t == 0:
            return None, None
        return d, now - t

    def _loop(self):
        while self._run:
            try:
                n = self.ser.in_waiting
                if n <= 0:
                    time.sleep(0.01)
                    continue
                self.buf += self.ser.read(n)
                if len(self.buf) > 1024:
                    del self.buf[:-16]
                while len(self.buf) >= 4:
                    if self.buf[0] != 0x0A:
                        del self.buf[0]
                        continue
                    hi, lo, cs = self.buf[1], self.buf[2], self.buf[3]
                    if (0x0A ^ hi ^ lo) != cs:
                        del self.buf[0]
                        continue
                    dist = ((hi << 8) | lo) / 1000.0
                    del self.buf[:4]
                    # if dist < self.min_valid_m or dist > self.max_valid_m:
                    #     continue
                    now = time.time()
                    with self._lock:
                        self._distance = dist
                        self._timestamp = now
            except Exception:
                time.sleep(0.01)