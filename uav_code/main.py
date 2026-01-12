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

def send_mode(mode: str, repeat=5, interval_s=0.1):
    pkt = {
        "type": "mode",
        "mode": mode
    }
    data = json.dumps(pkt).encode("utf-8")
    for _ in range(repeat):
        sock.sendto(data, detect_addr)
        time.sleep(interval_s)

def send_altitude(altitude: float, period_s=0.1):
    if not hasattr(send_altitude, "last_t"):
        send_altitude.last_t = 0.0

    t_now = time.time()
    if t_now - send_altitude.last_t < period_s:
        return

    send_altitude.last_t = t_now
    pkt = {
        "type": "altitude",
        "altitude": float(altitude)
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


def connect(port="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0", baud=57600):
    filename = time.strftime("logs/%Y_%m_%d_%H_%M.csv")
    f = open(filename, "w")
    f.write(f"Connecting to {port} @ {baud}...\n")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    f.write(f"HEARTBEAT received: {m.target_system}, {m.target_component}\n")
    return m, f

def fusion_alt():
    snap = st.poll()
    baro = None
    if snap is not None:
        t_ms, ch6, baro, roll, pitch, yaw = snap
    
    ultra, age = ultra_reader.latest()
    t = time.time()
    # print(f"{t}, ultra={ultra:.3f}, age={age:.3f}, baro={baro}")

    if ultra == 0:
        return baro
    if ultra <= 1.5:
        return ultra
    elif ultra < 2.0:
        return ultra * 0.6 + baro * 0.4
    return baro

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
            t_ms, ch6, baro, roll, pitch, yaw = snap
            if ch6 == 2000:
                send_mode("Take off")
                break
        time.sleep(0.02)

def takeoff(alt):
    fc.arm()
    time.sleep(1.5)
    fc.takeoff(alt)
    while True:
        snap = st.poll()
        baro = None
        if snap is not None:
            t_ms, ch6, baro, roll, pitch, yaw = snap
        if baro >= 0.95*alt:
            time.sleep(3)
            send_mode("Following")
            break
        time.sleep(0.02)

def landing():
    send_mode("Landing")
    fc.land()

if __name__ == "__main__":
    ultra_reader = None
    try:
        m, f = connect()
        st = state(m, f)
        fc = FlightController(m, f)

        # ==== Start ultrasonic thread ====
        ultra_reader = UltrasonicReader(
            port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            baudrate=9600,
            timeout=0.05,
            min_valid_m=0.05,   # lọc 0.0 / nhiễu nhỏ
            max_valid_m=4.0,    # dưới 2m luôn dùng siêu âm
        )
        ultra_reader.start()

        wait_to_takeoff()
        takeoff(2.5) #cất cánh lên 4m
        vx = vy = vz = 0.0
        t0 = time.time()
        while time.time() - t0 < 60: #Thời gian detect
            pkt, dt, d_flag = recv_udp()
            dz = fusion_alt()
            
            if dz < 1.0:
                print(f"abs(dx): {abs(pkt['dx'])}")
                print(f"abs(dy): {abs(pkt['dy'])}")
                if (abs(pkt["dx"]) < 64.0) and (abs(pkt["dy"]) < 48.0) and (pkt["found"] == True):
                    print("ok 0.7")
                    send_mode("Landing 0.7")
                    t0 = time.time()
                    while time.time() - t0 < 3:
                        fc.send_body_velocity(0.01,0.01,0.4)
                        time.sleep(0.02)
                    break

                else:
                    print("else")
                    if d_flag:
                        vx, vy, vz = fc.pid_commute(pkt["dx"], pkt["dy"], dz, dt, pkt["found"])
                        fc.send_body_velocity(vx, vy, 0.0)
            else:
                if d_flag:
                    vx, vy, vz = fc.pid_commute(pkt["dx"], pkt["dy"], dz, dt, pkt["found"])
                    fc.send_body_velocity(vx, vy, vz)
            time.sleep(0.01)
        
    except Exception as e:
        f.write(f"Lỗi xảy ra: {e}\n")
    finally:
        landing()
        
        # stop ultrasonic thread
        if ultra_reader is not None:
            try:
                ultra_reader.stop()
            except Exception:
                pass

        st.stop_stream()
        m.close()
        send_mode("Stop")