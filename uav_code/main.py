#!/usr/bin/env python3
from pymavlink import mavutil
import time
import socket
import json
from state import state
from flight_controller import FlightController

# ================= Socket config =================
UDP_IP = "127.0.0.1"
MAIN_PORT = 9000
DETECT_PORT = 10000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
detect_addr = (UDP_IP, DETECT_PORT)
sock.bind(("0.0.0.0", MAIN_PORT))
sock.setblocking(False)

def send_udp(repeat=5, interval_s=0.1):
    pkt = {"type": "cmd", "cmd": "stop"}
    data = json.dumps(pkt).encode("utf-8")

    for _ in range(repeat):
        sock.sendto(data, detect_addr)
        time.sleep(interval_s)

def recv_udp():
    if not hasattr(recv_udp, "last_msg"):
        recv_udp.last_msg = None
    try:
        data, _ = sock.recvfrom(256)
    except BlockingIOError:
        return recv_udp.last_msg
    except OSError:
        return recv_udp.last_msg

    try:
        recv_udp.last_msg = json.loads(data.decode("utf-8"))
    except Exception:
        return recv_udp.last_msg

    return recv_udp.last_msg


def connect(port="/dev/ttyUSB0", baud=57600):
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    f.write(f"Connecting to {port} @ {baud}...\n")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}")
    return m, f

def wait_to_takeoff():
    st.start_stream()
    while True:
        snap = st.poll()
        if snap is not None:
            t_ms, ch6, alt, roll, pitch, yaw = snap
            if ch6 == 2000:
                break
        time.sleep(0.05)

def takeoff():
    fc.arm()
    time.sleep(1.5)
    fc.takeoff(4) #cất cánh lên 4m

if __name__ == "__main__":
    try:
        m, f = connect()
        st = state(m, f)
        fc = FlightController(m, f)
        fc.set_mode_guided()
        wait_to_takeoff()
        takeoff()
        reached_alt = False
        t0 = time.time()
        while time.time() - t0 < 20:
            snap = st.poll()
            pkt = recv_udp()
            if snap is not None:
                t_ms, ch6, alt, roll, pitch, yaw = snap
                if not reached_alt and alt >= 4:
                    reached_alt = True

            if reached_alt:
                fc.send_body_velocity(0,0,0)
            time.sleep(0.01)

        fc.land()
        
    except Exception as e:
        f.write(f"Lỗi xảy ra: {e}")
    finally:
        fc.land()
        st.stop_stream()
        m.close()
        send_udp()