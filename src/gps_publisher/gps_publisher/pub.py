import os, glob, serial, pynmea2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from math import inf
from typing import Optional
from datetime import datetime, date, time as dtime
from rclpy.time import Time

class NMEAGPS(Node):
    UERE_METERS = 1.5
    VARIANCE_SENTINEL = 1e8
    COMMON_BAUDS = [9600, 19200, 38400, 57600, 115200]

    def __init__(self):
        super().__init__('nmea_gps')

        # ----- Parameters -----
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('uere_m', float(self.UERE_METERS))

        self.port     = self._resolve_port()
        self.baud     = self.get_parameter('baud').value
        self.rate_hz  = float(self.get_parameter('rate_hz').value or 1.0)
        self.UERE_METERS = float(self.get_parameter('uere_m').value)

        # ----- Serial & state ----- #
        self.ser   = None
        self.hdop  = None
        self.vdop  = None
        self._open_serial()

        self.pub   = self.create_publisher(NavSatFix, 'fix', 10)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-3), self._read)

        self.get_logger().info(
            f"GPS node up - port: {self.port or 'auto-detect'}, "
            f"baud: {self.baud}, rate: {self.rate_hz} Hz, UERE: {self.UERE_METERS} m"
        )

    @classmethod
    def _dop_to_var(cls, dop: Optional[float], uere: Optional[float] = None) -> float:
        """
        Convert a DOP (HDOP / VDOP) to variance (σ², m²).
        Returns sentinel for “unknown” so downstream filters can ignore it.
        """
        uere = uere or cls.UERE_METERS
        if dop is None:
            return cls.VARIANCE_SENTINEL

        sigma = dop * uere
        return sigma * sigma

    def _resolve_port(self) -> str:
        param_port = self.get_parameter('port').value
        if param_port:
            return param_port

        links = sorted(glob.glob('/dev/ublox_gps*'))
        if links:
            return links[0]

        acms = sorted(glob.glob('/dev/ttyACM*'))
        return acms[0] if acms else ''

    # ----------------- #
    #  Serial handling
    # ------------------#
    def _open_serial(self):
        if not self.port:
            self.get_logger().error('No GPS device found')
            self.create_timer(2.0, self._retry_open, oneshot=True)
            return

        if not self.baud:
            for b in self.COMMON_BAUDS:
                try:
                    ser = serial.Serial(self.port, b, timeout=1)
                    ser.reset_input_buffer()
                    line = ser.readline().decode(errors='ignore').strip()
                    if line.startswith('$'):
                        self.ser = ser
                        self.baud = b
                        self.get_logger().info(f'Auto-detected baud {b} on {self.port}')
                        return
                    ser.close()
                except serial.SerialException as e:
                    self.get_logger().debug(f'Baud {b} failed: {e}')

            self.get_logger().error(f'Failed to auto-detect baud on {self.port}')
            self.create_timer(2.0, self._retry_open, oneshot=True)
        else:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                self.ser.reset_input_buffer()
                self.get_logger().info(f'Opened {self.port} @ {self.baud}')

            except serial.SerialException as e:
                self.get_logger().warn(f'Port unavailable: {e}')
                self.create_timer(2.0, self._retry_open, oneshot=True)

    def _retry_open(self):
        if not self.ser or not self.ser.is_open:
            self.port = self._resolve_port()
            self._open_serial()

    def _cleanup(self):
        if self.ser and self.ser.is_open:
            self.get_logger().info('Closing GPS serial port')
            try:
                self.ser.close()
            except Exception:
                pass

    def _nmea_time_to_ros(self, utc_time: dtime, recv_stamp):
        # If no UTC time, keep receive-time
        if utc_time is None:
            return recv_stamp
        today = date.today()
        dt = datetime.combine(today, utc_time)
        # Convert to seconds since epoch
        ts = dt.timestamp()
        sec = int(ts)
        nanosec = int((ts - sec) * 1e9)

        return Time(seconds=sec, nanoseconds=nanosec).to_msg()

    def _read(self):
        if not (self.ser and self.ser.is_open):
            return
        try:
            raw = self.ser.readline()
            if not raw:
                return

            line = raw.decode(errors='ignore').strip()
            if not line.startswith('$'):
                return
            msg = pynmea2.parse(line)

            # Cache DOPs (GSA sentence)
            if isinstance(msg, pynmea2.types.talker.GSA):
                self.hdop = float(msg.hdop) if msg.hdop else None
                self.vdop = float(msg.vdop) if msg.vdop else None
                # Invalidate on no‑fix
                if int(msg.mode_fix_type or 1) == 1:
                    self.hdop = self.vdop = None
                return

            # Position fix (GGA sentence)
            if isinstance(msg, pynmea2.types.talker.GGA):
                stamp = self._nmea_time_to_ros(msg.timestamp, self.get_clock().now().to_msg())
                self._publish_fix(msg, stamp)

        except (serial.SerialException,
                pynmea2.nmea.ChecksumError,
                UnicodeDecodeError) as e:
            self.get_logger().warn(str(e))


    def _publish_fix(self, gga, stamp):
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps'

        fix.status.status  = (
            NavSatStatus.STATUS_FIX
            if int(gga.gps_qual or 0) > 0
            else NavSatStatus.STATUS_NO_FIX
        )

        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude  = gga.latitude
        fix.longitude = gga.longitude
        fix.altitude  = gga.altitude

        # ------ Covariance Information ------ #
        var_h = self._dop_to_var(self.hdop, self.UERE_METERS)
        var_v = self._dop_to_var(self.vdop, self.UERE_METERS)
        fix.position_covariance = [
                var_h, 0.0,   0.0,
                0.0,   var_h, 0.0,
                0.0,   0.0,   var_v,
        ]

        fix.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            if var_h < self.VARIANCE_SENTINEL and var_v < self.VARIANCE_SENTINEL
            else NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )
        # ------ Publish the navigation stuff ------ #
        self.pub.publish(fix)

def main(args=None):
    rclpy.init(args=args)
    node = NMEAGPS()
    rclpy.spin(node)
    node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
