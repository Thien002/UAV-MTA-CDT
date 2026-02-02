import math
import time
from pymavlink import mavutil
from HCSR04_thread import UltrasonicReader

class state:

    def __init__(self, m, f):
        self.m = m

        #Ultra
        self.ultra_obj = UltrasonicReader(
            port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            baudrate=9600,
            timeout=0.05,
            min_valid_m=0.05,
            max_valid_m=4.0,
        )
        self.ultra_obj.start()

        self.ch6 = None
        self.baro = None
        self.ultra = None
        self.alt = None
        self.lat = None
        self.lon = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self._t0 = time.monotonic()

        # Cờ đánh dấu đã có message mới cho từng loại (so với snapshot gần nhất)
        self._has_new_rc = False
        self._has_new_gpos = False
        self._has_new_att = False

        # Snapshot hợp lệ gần nhất
        self._ready = False
        self._last_snapshot = None

        self._log_file = f
        self._log_file.write("t_ms,ch6,alt,baro,ultra,lat,lon,roll,pitch,yaw\n")

    def _set_message_interval(self, msg_id, hz = 10):
        if hz <= 0:
            interval_us = 0
        else:
            interval_us = int(1_000_000 / hz)

        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    def start_stream(self):
        # RC_CHANNELS (65)
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS)

        # GLOBAL_POSITION_INT (33)
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT)

        # ATTITUDE (30)
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)

    def stop_stream(self):
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 0)
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0)
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 0)

        try:
            self.ultra_obj.stop()
        except:
            pass

        self._log_file.close()

    def _fusion_alt(self):
        self.ultra, _ = self.ultra_obj.latest()

        if self.baro is None:
            return None

        if self.ultra == 0 or self.ultra is None:
            return self.baro
        elif self.ultra <= 1.5:
            return self.ultra
        elif self.ultra < 2.0:
            return 0.6 * self.ultra + 0.4 * self.baro
        else:
            return self.baro

    def poll(self):
        msg = self.m.recv_match(
            type=["RC_CHANNELS", "GLOBAL_POSITION_INT", "ATTITUDE", "STATUSTEXT"],
            blocking=False,
        )

        # Có message mới
        if msg is not None:
            mtype = msg.get_type()

            if mtype == "RC_CHANNELS":
                self.ch6 = msg.chan6_raw
                self._has_new_rc = True

            elif mtype == "GLOBAL_POSITION_INT":
                self.baro = msg.relative_alt / 1000.0
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7
                self._has_new_gpos = True

            elif mtype == "ATTITUDE":
                self.roll  = round(math.degrees(msg.roll), 1)
                self.pitch = round(math.degrees(msg.pitch), 1)
                self.yaw   = round(math.degrees(msg.yaw), 1)
                self._has_new_att = True

            elif mtype == "STATUSTEXT":
                text = getattr(msg, "text", "")
                self._log_file.write(f"[FCU] {text}\n")

        if (self._has_new_rc and
            self._has_new_gpos and
            self._has_new_att):

            t_ms = int((time.monotonic() - self._t0) * 1000)
            # update altitude
            self.alt = self._fusion_alt()

            snapshot = (
                t_ms,
                self.ch6,
                self.alt,
                self.lat,
                self.lon,
                self.roll,
                self.pitch,
                self.yaw,
            )

            self._last_snapshot = snapshot
            self._ready = True

            # reset cờ
            self._has_new_rc = False
            self._has_new_gpos = False
            self._has_new_att = False

            # ghi log
            self._log_file.write(
                f"{t_ms},{self.ch6},{self.alt},{self.baro},{self.ultra},{self.lat},{self.lon},{self.roll},{self.pitch},{self.yaw}\n"
            )

        if self._ready:
            return self._last_snapshot

        return None