#!/usr/bin/env python3
from pymavlink import mavutil
import os
from ultralytics import YOLO
import time
import detect
import log
import flight_control as fc

def connect(port="/dev/ttyUSB0", baud=57600):
    f = log.log_file()
    f.write(f"Connecting to {port} @ {baud}...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}")
    return m,f

def wait_to_takeoff(m,f):
    log.request_message(m,65)
    log.request_message(m,33)
    while True:
        ch6, _ = log.stream_data(m,f)
        time.sleep(0.05)
        if ch6 == 2000:
            break

def takeoff(m,f):
    fc.set_mode_guided(m,f)
    fc.arm(m,f)
    time.sleep(1.5)
    fc.takeoff(m, f, 2.5) #cất cánh lên 2.5m

if __name__ =="__main__":
    try:
        m,f = connect()
        wait_to_takeoff(m,f)
        takeoff(m,f)
        t0 = time.time()
        while time.time() - t0 < 20:
            _, alt_m = log.stream_data(m,f)
            if time.time() - t0 > 5:
                fc.send_body_velocity(m,0,0,0)
            time.sleep(0.05)

        fc.land(m,f)
        
    except:
        pass
    finally:
        fc.land(m,f)
        log.stop_stream(m,f)