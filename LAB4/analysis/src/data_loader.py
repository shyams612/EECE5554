# data_loader.py
# Parses ROS2 SQLite bags (XCDR1 CDR encoding) and returns structured data.
#
# Confirmed field layout verified against live bag data (Boston GPS + IMU values).
#
# GPS message (gps_msg/msg/Gpsmsg):
#   std_msgs/Header header
#   float64 latitude, longitude, altitude, hdop
#   float64 utm_easting, utm_northing, utc
#   int32   zone
#   string  letter
#
# IMU message (imu_msg/msg/Imumsg):
#   std_msgs/Header      header
#   sensor_msgs/Imu      imu      (header + quat + 9-cov + vec3 gyro + 9-cov + vec3 accel + 9-cov)
#   sensor_msgs/MagField mag_field (header + vec3 mag + 9-cov)
#   string               raw_imu_string

import sqlite3
import struct
import numpy as np
from pathlib import Path


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def load_bag(bag_path: str, topics: list[str]) -> dict[str, list]:
    """
    Read all messages from a ROS2 SQLite bag for the given topic names.
    Returns dict[topic_name -> list[parsed_dict]].
    """
    path = Path(bag_path)
    if not path.exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")

    conn = sqlite3.connect(str(path))
    c = conn.cursor()

    c.execute("SELECT id, name FROM topics")
    name_to_id = {row[1]: row[0] for row in c.fetchall()}

    results = {t: [] for t in topics}

    for topic_name in topics:
        if topic_name not in name_to_id:
            print(f"[data_loader] WARNING: topic '{topic_name}' not found in bag")
            continue
        tid = name_to_id[topic_name]
        c.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (tid,),
        )
        parser = _get_parser(topic_name)
        for bag_ts_ns, raw in c.fetchall():
            parsed = parser(raw, bag_ts_ns)
            if parsed is not None:
                results[topic_name].append(parsed)

    conn.close()
    return results


def to_numpy(records: list[dict], fields: list[str]) -> dict[str, np.ndarray]:
    """Convert a list of parsed message dicts to numpy arrays, one per field."""
    return {f: np.array([r[f] for r in records]) for f in fields}


# ---------------------------------------------------------------------------
# CDR reader  (XCDR1: little-endian, max 4-byte alignment)
# ---------------------------------------------------------------------------

class _CDR:
    """Stateful cursor for reading XCDR1-encoded ROS2 messages."""

    def __init__(self, data: bytes):
        # Byte 1: 0x01 = little-endian, 0x00 = big-endian
        self.le = data[1] == 0x01
        self.data = data
        self.offset = 4  # skip 4-byte CDR encapsulation header

    def _align(self, n: int):
        """Align cursor to n bytes (capped at 4 for XCDR1)."""
        n = min(n, 4)
        r = self.offset % n
        if r:
            self.offset += n - r

    def uint32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<I' if self.le else '>I', self.data, self.offset)[0]
        self.offset += 4
        return v

    def int32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<i' if self.le else '>i', self.data, self.offset)[0]
        self.offset += 4
        return v

    def float64(self) -> float:
        self._align(4)  # XCDR1: float64 aligns to min(8,4)=4
        v = struct.unpack_from('<d' if self.le else '>d', self.data, self.offset)[0]
        self.offset += 8
        return v

    def float64_array(self, n: int) -> list[float]:
        return [self.float64() for _ in range(n)]

    def string(self) -> str:
        length = self.uint32()          # includes null terminator
        s = self.data[self.offset: self.offset + length - 1].decode('utf-8', errors='replace')
        self.offset += length
        return s

    def header(self) -> dict:
        """Read std_msgs/Header: stamp (sec, nanosec) + frame_id string."""
        sec = self.uint32()
        nanosec = self.uint32()
        frame_id = self.string()
        return {'sec': sec, 'nanosec': nanosec, 'frame_id': frame_id}


# ---------------------------------------------------------------------------
# Message parsers
# ---------------------------------------------------------------------------

def _parse_gps_msg(raw: bytes, bag_ts_ns: int) -> dict:
    """
    Parse gps_msg/msg/Gpsmsg.

    Returned keys: bag_ts_ns, sec, nanosec, latitude, longitude, altitude,
                   hdop, utm_easting, utm_northing, utc, zone, letter
    """
    r = _CDR(raw)
    hdr = r.header()
    lat     = r.float64()
    lon     = r.float64()
    alt     = r.float64()
    hdop    = r.float64()
    utm_e   = r.float64()
    utm_n   = r.float64()
    utc     = r.float64()
    zone    = r.int32()
    letter  = r.string()

    return {
        'bag_ts_ns':   bag_ts_ns,
        'sec':         hdr['sec'],
        'nanosec':     hdr['nanosec'],
        'latitude':    lat,
        'longitude':   lon,
        'altitude':    alt,
        'hdop':        hdop,
        'utm_easting': utm_e,
        'utm_northing':utm_n,
        'utc':         utc,
        'zone':        zone,
        'letter':      letter,
    }


def _parse_imu_msg(raw: bytes, bag_ts_ns: int) -> dict:
    """
    Parse imu_msg/msg/Imumsg.

    Returned keys: bag_ts_ns, sec, nanosec,
                   quat_{x,y,z,w},
                   gyro_{x,y,z},  accel_{x,y,z},
                   mag_{x,y,z},
                   roll, pitch, yaw  (Euler angles from the IMU, degrees)
    """
    r = _CDR(raw)

    # --- outer header ---
    outer_hdr = r.header()

    # --- sensor_msgs/Imu ---
    r.header()                          # imu sub-header (same timestamp, skip)
    qx, qy, qz, qw = r.float64_array(4)
    r.float64_array(9)                  # orientation_covariance (unused)
    gx, gy, gz = r.float64_array(3)    # angular_velocity (rad/s)
    r.float64_array(9)                  # angular_velocity_covariance
    ax, ay, az = r.float64_array(3)    # linear_acceleration (m/s²)
    r.float64_array(9)                  # linear_acceleration_covariance

    # --- sensor_msgs/MagneticField ---
    r.header()                          # mag sub-header
    mx, my, mz = r.float64_array(3)    # magnetic_field (Gauss)
    r.float64_array(9)                  # magnetic_field_covariance

    # --- raw VNYMR string (contains Euler angles) ---
    raw_str = r.string()
    roll, pitch, yaw = _parse_euler_from_vnymr(raw_str)

    return {
        'bag_ts_ns': bag_ts_ns,
        'sec':       outer_hdr['sec'],
        'nanosec':   outer_hdr['nanosec'],
        'quat_x': qx, 'quat_y': qy, 'quat_z': qz, 'quat_w': qw,
        'gyro_x': gx, 'gyro_y': gy, 'gyro_z': gz,
        'accel_x': ax, 'accel_y': ay, 'accel_z': az,
        'mag_x': mx, 'mag_y': my, 'mag_z': mz,
        'roll': roll, 'pitch': pitch, 'yaw': yaw,
    }


def _parse_euler_from_vnymr(raw_str: str):
    """
    Extract (roll, pitch, yaw) in degrees from a VNYMR NMEA-style sentence.
    Format: $VNYMR,<yaw>,<pitch>,<roll>,<mag_x>,...*checksum
    Returns (roll, pitch, yaw) as floats, or (nan, nan, nan) on parse failure.
    """
    nan = float('nan')
    try:
        body = raw_str.split('*')[0]
        parts = body.split(',')
        if len(parts) < 4:
            return nan, nan, nan
        yaw   = float(parts[1])
        pitch = float(parts[2])
        roll  = float(parts[3])
        return roll, pitch, yaw
    except (ValueError, IndexError):
        return nan, nan, nan


# ---------------------------------------------------------------------------
# Internal routing
# ---------------------------------------------------------------------------

def _get_parser(topic_name: str):
    parsers = {
        '/gps': _parse_gps_msg,
        '/imu': _parse_imu_msg,
    }
    if topic_name not in parsers:
        raise ValueError(f"No parser registered for topic '{topic_name}'")
    return parsers[topic_name]
