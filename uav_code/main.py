#!/usr/bin/env python3
from pymavlink import mavutil
import time
import socket
import json
from state import state
from flight_controller import FlightController
from HCSR04_thread import UltrasonicReader

# ================= Socket config =================
UDP_IP = "127.0.0.1"
MAIN_PORT = 9000
DETECT_PORT = 10000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
detect_addr = (UDP_IP, DETECT_PORT)
sock.bind(("0.0.0.0", MAIN_PORT))
sock.setblocking(False)

def send_mode(mode: str, repeat=3, interval_s=0.1):
    pkt = {
        "type": "mode",
        "mode": mode
    }
    data = json.dumps(pkt).encode("utf-8")
    for _ in range(repeat):
        sock.sendto(data, detect_addr)
        time.sleep(interval_s)

def send_udp(dz, vx, vy, vz, period_s=0.1):
    if not hasattr(send_udp, "last_t"):
        send_udp.last_t = 0.0

    t_now = time.time()
    if t_now - send_udp.last_t < period_s:
        return

    send_udp.last_t = t_now

    pkt = {
        "type": "controlRC",
        "dz": float(dz),
        "vx": float(vx),
        "vy": float(vy),
        "vz": float(vz),
        "kpx": fc.pid_x.kp,
        "kix": fc.pid_x.ki,
        "kdx": fc.pid_x.kd,
        "kpy": fc.pid_y.kp,
        "kiy": fc.pid_y.ki,
        "kdy": fc.pid_y.kd
    }

    sock.sendto(json.dumps(pkt).encode("utf-8"), detect_addr)


def recv_udp():
    if not hasattr(recv_udp, "last_msg"):
        recv_udp.last_msg = None
        recv_udp.last_t = 0.0
        recv_udp.dt = 0.0

    try:
        data, _ = sock.recvfrom(256)
    except (BlockingIOError, OSError):
        return recv_udp.last_msg, recv_udp.dt, False

    try:
        recv_udp.last_msg = json.loads(data.decode("utf-8"))
    except Exception:
        return recv_udp.last_msg, recv_udp.dt, False

    t_now = time.time()
    recv_udp.dt = t_now - recv_udp.last_t
    recv_udp.last_t = t_now
    return recv_udp.last_msg, recv_udp.dt, True #Có dữ liệu mới cập nhật


def connect(port="/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_49002D001551333034383239-if00", baud=57600):
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    f.write(f"Connecting to {port} @ {baud}...\n")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}\n")
    return m, f

def wait_to_takeoff():
    st.start_stream()
    is_start = True
    while True:
        snap = st.poll()
        pkt, _, _ = recv_udp()
        if pkt is not None and pkt.get("seq", 0) == -1 and is_start:
            fc.set_mode_guided()
            is_start = False
        if snap is not None:
            ch6 = snap[1]
            if ch6 == 2000:
                send_mode("Take off")
                break
        time.sleep(0.02)

def takeoff(des_alt):
    fc.arm()
    fc.takeoff(des_alt)
    while True:
        snap = st.poll()
        if snap is not None:
            alt = snap[2]
        if alt >= 0.95*des_alt:
            break
        time.sleep(0.02)
    print("HOVER")
    f.write("MODE: HOVER\n")
    while True:
        ch6 = st.poll()[1]
        print(ch6)
        if ch6 == 1000:
            break
        else:
            time.sleep(0.02)
    send_mode("Following")
    print("FOLLOWING")
    f.write("MODE: FOLLOWING\n")
    

def landing():
    send_mode("Landing")
    fc.land()

def close():
        st.stop_stream()
        fc.close()
        m.close()
        send_mode("Stop")

if __name__ == "__main__":
    try:
        m, f = connect()
        st = state(m, f)
        fc = FlightController(m, f)

        wait_to_takeoff()
        takeoff(2.5) #cất cánh lên 2.5m
        vx = vy = vz = 0.0
        t0 = time.time()
        while time.time() - t0 < 60: #Thời gian detect
            pkt, dt, d_flag = recv_udp()
            dz = st.poll()[2]
            fc.update_pid_from_keyboard()
            if dz < 0.9:
                if (abs(pkt["dx"]) < 50.0) and (abs(pkt["dy"]) < 40.0) and (pkt["found"] == True):
                    send_mode("Landing 1.0")
                    t0 = time.time()
                    while time.time() - t0 < 3:
                        dz = st.poll()[2]
                        if dz < 0.2:
                            break
                        fc.send_body_velocity(0,0,0.4)
                        time.sleep(0.02)
                    break

                else:
                    if d_flag:
                        vx, vy, vz = fc.pid_compute(pkt["dx"], pkt["dy"], dz, dt, pkt["found"])
                        print(f"time:{time.time():.1f}, dx:{pkt['dx']}, vx:{vx:.2f}\tdy:{pkt['dy']}, vy:{vy:.2f}\tdz:{dz:.2f}, vz0")
                        fc.send_body_velocity(vx, vy, 0)
            else:
                if d_flag:
                    vx, vy, vz = fc.pid_compute(pkt["dx"], pkt["dy"], dz, dt, pkt["found"])
                    print(f"time:{time.time():.1f}, dx:{pkt['dx']}, vx:{vx:.2f}\tdy:{pkt['dy']}, vy:{vy:.2f}\tdz:{dz:.2f}, vz:{vz:.2f}")
                    fc.send_body_velocity(vx, vy, vz)
            send_udp(dz,vx,vy,vz)
            time.sleep(0.02)
        
    except Exception as e:
        f.write(f"Lỗi xảy ra: {e}\n")
    finally:
        landing()        
        close()

