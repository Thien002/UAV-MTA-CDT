#!/usr/bin/env python3
import time
from pymavlink import mavutil

class PID:
    def __init__(self, kp, ki, kd, out_min=None, out_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev = False

    def commute(self, error, dt):
        if dt <= 0:
            d = 0.0
        else:
            d = (error - self.prev_error) / dt if self.has_prev else 0.0

        self.integral += error * dt
        u = self.kp * error + self.ki * self.integral + self.kd * d

        # giới hạn output
        if self.out_min is not None:
            u = max(self.out_min, u)
        if self.out_max is not None:
            u = min(self.out_max, u)

        self.prev_error = error
        self.has_prev = True
        return u

pid_x = PID(kp=0.005, ki=0.001, kd=0.001, out_min=-0.5, out_max=0.5)
pid_y = PID(kp=0.002, ki=0.0005, kd=0.005, out_min=-0.5, out_max=0.5)
pid_z = PID(kp=0.1, ki=0.0, kd=0.02, out_min=-0.4, out_max=0.4)

def set_mode_guided(m,f):
    modes = m.mode_mapping()
    if "GUIDED" not in modes:
        f.write("ERROR: GUIDED mode not supported.\n")
        return False

    guided_id = modes["GUIDED"]

    f.write("Setting mode GUIDED...\n")
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        guided_id
    )

    # Chờ đến khi FCU báo vào GUIDED
    t0 = time.time()
    while time.time() - t0 < 8:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == guided_id:
            f.write("Mode = GUIDED\n")
            return True

    f.write("Timeout switching to GUIDED\n")
    return False

def arm(m,f):
    f.write("Arming motors...\n")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0,0,0,0,0,0
    )

    t0 = time.time()
    while time.time() - t0 < 8:
        if m.motors_armed():
            f.write("Armed\n")
            return True
        msg = m.recv_match(type=["HEARTBEAT","STATUSTEXT"], blocking=True, timeout=1)
        if msg and msg.get_type()=="STATUSTEXT":
            f.write(f"[FCU] {getattr(msg, 'text')}\n")
    f.write("Arm timeout\n")
    return False

def takeoff(m, f, alt):
    f.write(f"Takeoff to {alt} m...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,0,0,0,
        0,0,
        alt
    )

def land(m,f):
    f.write("Landing...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,0,0,0,
        0,0,0
    ) 

def send_body_velocity(m, vx, vy, vz):

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_ACCELERATION_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    ts = int(time.time() * 1000) & 0xFFFFFFFF
    m.mav.set_position_target_local_ned_send(
        ts,
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0,0,0,
        vx, vy, vz,
        0,0,0,
        0,0
    )

def tracking(m, dx, dy, dz, dt):
    vx = pid_x.commute(dx, dt)
    vy = pid_y.commute(dy, dt)
    alpha = abs((90 - dx)/180) + abs((160 - dy)/320) #Hệ số điều chỉnh
    vz = alpha * pid_z.commute(dz, dt)
    send_body_velocity(m,vx,vy,vz)
