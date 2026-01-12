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

    def commute(self, error, dt, i = 1):
        if dt <= 0:
            d = 0.0
        else:
            d = (error - self.prev_error) / dt if self.has_prev else 0.0

        self.integral += error * dt
        if self.integral > 0.15:
            self.integral = 0.15
        elif self.integral < -0.15:
            self.integral = -0.15

        u = self.kp/i * error + self.ki * self.integral + self.kd * d

        if self.out_min is not None:
            u = max(self.out_min, u)
        if self.out_max is not None:
            u = min(self.out_max, u)

        self.prev_error = error
        self.has_prev = True
        return u


class FlightController:
    def __init__(self, m, f):
        self.m = m
        self.log_file = f
        self.dxm_old = 0.0
        self.dym_old = 0.0
        # PID cho tracking
        self.pid_x = PID(kp=0.4, ki=0.0, kd=0.0, #0.5
                         out_min=-1.5, out_max=1.5)
        self.pid_y = PID(kp=0.2, ki=0.0, kd=0.0, #0.3
                         out_min=-1.0, out_max=1.0)
    def _log(self, msg):
        self.log_file.write(msg + "\n")

    def set_mode_guided(self):
        modes = self.m.mode_mapping()
        if "GUIDED" not in modes:
            self._log("ERROR: GUIDED mode not supported.")
            return False

        guided_id = modes["GUIDED"]

        self._log("Setting mode GUIDED...")
        self.m.mav.set_mode_send(
            self.m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_id
        )

        t0 = time.time()
        while time.time() - t0 < 8:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.custom_mode == guided_id:
                self._log("Mode = GUIDED")
                return True

        self._log("Timeout switching to GUIDED")
        return False

    def set_mode_loiter(self):
        modes = self.m.mode_mapping()
        if "LOITER" not in modes:
            self._log("ERROR: LOITER mode not supported.")
            return False

        loiter_id = modes["LOITER"]

        self._log("Setting mode LOITER...")
        self.m.mav.set_mode_send(
            self.m.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            loiter_id
        )

        t0 = time.time()
        while time.time() - t0 < 8:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and hb.custom_mode == loiter_id:
                self._log("Mode = LOITER")
                return True

        self._log("Timeout switching to LOITER")
        return False

    def arm(self):
        self._log("Arming motors...")
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        t0 = time.time()
        while time.time() - t0 < 8:
            if self.m.motors_armed():
                self._log("Armed")
                return True

            msg = self.m.recv_match(
                type=["HEARTBEAT", "STATUSTEXT"],
                blocking=True, timeout=1
            )
            if msg and msg.get_type() == "STATUSTEXT":
                self._log(f"[FCU] {getattr(msg, 'text', '')}")

        self._log("Arm timeout")
        return False

    def takeoff(self, alt):
        self._log(f"Takeoff to {alt} m...")
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            alt
        )

    def land(self):
        self._log("Landing...")
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )

    def send_body_velocity(self, vx, vy, vz):
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |

            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |

            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        ts = int(time.time() * 1000) & 0xFFFFFFFF
        self.m.mav.set_position_target_local_ned_send(
            ts,
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def velocity_z(self, dxp, dyp, dz, found, xmax = 320.0, ymax=240.0):
        if dz != 0 and found:
            s = (dxp/xmax)*(dxp/xmax) + (dyp/ymax)*(dyp/ymax)
            alpha =  0.0 if s >= 1.0 else (1.0 - s)*(1.0 - s)
            if dz < 1: K = 0.06
            else: K = 0.04
            return K*alpha*dz
        else:
            return 0 #chỉnh cho z mượt khi không detect được
    
    def pid_commute(self, dx, dy, dz, dt, found):
        if found:
            dxm = 0 if abs(dx) < 0.1*640 else dz*dx/444
            dym = 0 if abs(dy) < 0.1*480 else dz*dy/444

            # lưu lại để lần sau dùng
            self.dxm_old = dxm
            self.dym_old = dym
        else:
            # dùng lại giá trị cũ
            dxm = self.dxm_old
            dym = self.dym_old
            
        # dxm = dz*dx/444 #Sai lệch theo m
        # dym = dz*dy/444
        if dz < 1.2: i = 2
        else: i = 1
        vx = self.pid_x.commute(dxm, dt, i)
        vy = self.pid_y.commute(dym, dt, i)
        vz = self.velocity_z(dxm, dym, dz, found,i)
        print(f"time: {time.time():.1f}, dx: {dxm}:.3f, vx:{vx}:.3f, dy:{dym}:.3f, vy:{vy}:.3f, dz:{dz}:.3f, vz:{vz}:.3f")
        return vx,vy,vz

