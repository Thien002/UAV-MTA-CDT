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

        # PID cho tracking
        self.pid_x = PID(kp=0.005, ki=0.001, kd=0.001,
                         out_min=-0.5, out_max=0.5)
        self.pid_y = PID(kp=0.002, ki=0.0005, kd=0.005,
                         out_min=-0.5, out_max=0.5)
        self.pid_z = PID(kp=0.1, ki=0.0, kd=0.02,
                         out_min=-0.4, out_max=0.4)

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

            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            # mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
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

    def tracking(self, dx, dy, dz, dt):
        vx = self.pid_x.commute(dx, dt)
        vy = self.pid_y.commute(dy, dt)

        alpha = abs((180 - dx) / 360) + abs((320 - dy) / 640)
        vz = alpha * self.pid_z.commute(dz, dt)

        self.send_body_velocity(vx, vy, vz)
