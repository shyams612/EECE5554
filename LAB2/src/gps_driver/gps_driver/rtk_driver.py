#!/usr/bin/env python3

# Standard library
import time as pytime
from dataclasses import dataclass
from datetime import datetime
import argparse

# Third-party
import utm

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from gps_driver.msg import RtkMsg

# -----------------------
# GPS Data model
# -----------------------
@dataclass
class GPSFix:
    latitude: float
    longitude: float
    altitude: float
    hdop: float
    utm_easting: float
    utm_northing: float
    zone: int
    letter: str
    utc_time: datetime
    fix_quality: int
    num_satellites: int

# -----------------------
# RTK Driver
# -----------------------
class RTKDriver:
    def __init__(self):
        self.valid_count = 0
        self.invalid_count = 0
    
    def safe_float(self, value, default=0.0):
        """Convert to float, return default if empty or invalid"""
        if not value or value.strip() == '':
            return default
        try:
            # Check for malformed floats like "0..000"
            if value.count('.') > 1:
                return default
            return float(value)
        except ValueError:
            return default

    def safe_int(self, value, default=0):
        """Convert to int, return default if empty or invalid"""
        if not value or value.strip() == '':
            return default
        try:
            return int(value)
        except ValueError:
            return default
    
    def is_valid_fix(self, fix):
        """Validate GPS fix data"""
        if fix is None:
            return False
        
        # Critical: Must have non-zero coordinates
        if fix.latitude == 0.0 or fix.longitude == 0.0:
            return False
        
        # Quality checks
        if fix.hdop == 0.0:  # No HDOP data
            return False
        
        if fix.hdop > 20.0:  # Poor accuracy
            return False
            
        if fix.zone == 0 or not fix.letter:  # Invalid UTM
            return False
        
        # Sanity checks for coordinate ranges
        if abs(fix.latitude) > 90:  # Latitude out of range
            return False
        
        if abs(fix.longitude) > 180:  # Longitude out of range
            return False
        
        # Optional: Check fix quality (uncomment to enforce valid GPS lock)
        if fix.fix_quality == 0:
            return False
        
        return True

    def parse_line(self, line: str):
        """
        Parse a GNGGA NMEA string and return GPSFix.
        Returns None if line is invalid or not GNGGA.
        """
        if not line.startswith("$GNGGA"):
            return None

        try:
            parts = line.strip().split(",")
            if len(parts) < 15:
                return None

            # Parse fields with safety checks
            utc_str = parts[1]
            latitude_str = parts[2]
            lat_dir = parts[3]
            longitude_str = parts[4]
            lon_dir = parts[5]
            fix_quality = self.safe_int(parts[6], 0)
            num_sats = self.safe_int(parts[7], 0)
            hdop = self.safe_float(parts[8], 0.0)
            altitude = self.safe_float(parts[9], 0.0)

            # Skip if critical fields are missing
            if not utc_str or not latitude_str or not longitude_str:
                return None
            
            # Convert lat/lon to decimal degrees
            latitude = self.safe_float(latitude_str)
            longitude = self.safe_float(longitude_str)
            
            # Skip if coordinates are zero (no GPS fix)
            if latitude == 0.0 or longitude == 0.0:
                return None

            # Convert NMEA format to decimal degrees
            lat_deg = int(latitude / 100)
            lat_min = latitude - lat_deg * 100
            lat = lat_deg + lat_min / 60
            if lat_dir == "S":
                lat = -lat

            lon_deg = int(longitude / 100)
            lon_min = longitude - lon_deg * 100
            lon = lon_deg + lon_min / 60
            if lon_dir == "W":
                lon = -lon

            # Validate coordinate ranges
            if abs(lat) > 90 or abs(lon) > 180:
                return None

            # Convert to UTM
            try:
                utm_easting, utm_northing, zone, letter = utm.from_latlon(lat, lon)
            except Exception:
                # UTM conversion failed for invalid coordinates
                return None

            # Parse UTC time safely
            if len(utc_str) < 6:
                return None

            try:
                hour = int(utc_str[0:2])
                minute = int(utc_str[2:4])
                second = int(utc_str[4:6])

                # Validate time ranges
                if not (0 <= hour < 24 and 0 <= minute < 60 and 0 <= second < 60):
                    return None

                # Handle fractional seconds properly
                if "." in utc_str and len(utc_str) > 6:
                    frac_str = utc_str.split('.')[1]
                    microsecond = int(float("0." + frac_str) * 1e6)
                else:
                    microsecond = 0

                utc_time = datetime(1, 1, 1, hour, minute, second, microsecond)
            except (ValueError, IndexError):
                # Time parsing failed, use default
                utc_time = datetime(1, 1, 1, 0, 0, 0)

            fix = GPSFix(
                latitude=lat,
                longitude=lon,
                altitude=altitude,
                hdop=hdop,
                utm_easting=utm_easting,
                utm_northing=utm_northing,
                zone=zone,
                letter=letter,
                utc_time=utc_time,
                fix_quality=fix_quality,
                num_satellites=num_sats
            )
            
            # Validate the fix before returning
            if self.is_valid_fix(fix):
                self.valid_count += 1
                return fix
            else:
                self.invalid_count += 1
                return None
                
        except Exception as e:
            # Log error for debugging (optional)
            # import traceback
            # traceback.print_exc()
            self.invalid_count += 1
            return None

    def read(self, path):
        """Generator to read streaming lines from GPS device"""
        with open(path, "r") as f:
            while True:
                line = f.readline()
                if not line:
                    pytime.sleep(0.01)
                    continue
                yield line.strip()

# -----------------------
# ROS2 Publisher Node
# -----------------------
class RTKPublisher(Node):
    def __init__(self, path):
        super().__init__("rtk_publisher")

        self.publisher = self.create_publisher(RtkMsg, "/rtk", 10)

        self.driver = RTKDriver()
        self.reader = self.driver.read(path)

        self.timer = self.create_timer(0.01, self.publish_rtk)
        
        # Statistics timer (log every 10 seconds)
        self.stats_timer = self.create_timer(10.0, self.log_statistics)
        
        self.get_logger().info(f"RTK Publisher started, reading from {path}")

    def log_statistics(self):
        """Log RTK parsing statistics"""
        total = self.driver.valid_count + self.driver.invalid_count
        if total > 0:
            success_rate = (self.driver.valid_count / total) * 100
            self.get_logger().info(
                f"RTK Stats - Valid: {self.driver.valid_count}, "
                f"Invalid: {self.driver.invalid_count}, "
                f"Success Rate: {success_rate:.1f}%"
            )

    def publish_rtk(self):
        try:
            line = next(self.reader)
            
            # Parse the line
            fix = self.driver.parse_line(line)
            
            if not fix:
                return

            msg = RtkMsg()

            # Header
            msg.header = Header()
            msg.header.frame_id = "RTK1_Frame"

            msg.header.stamp = Time()
            msg.header.stamp.sec = (
                fix.utc_time.hour * 3600 +
                fix.utc_time.minute * 60 +
                fix.utc_time.second
            )
            msg.header.stamp.nanosec = fix.utc_time.microsecond * 1000

            # Payload
            msg.latitude = fix.latitude
            msg.longitude = fix.longitude
            msg.altitude = fix.altitude
            msg.hdop = fix.hdop
            msg.utm_easting = fix.utm_easting
            msg.utm_northing = fix.utm_northing
            msg.zone = fix.zone
            msg.letter = fix.letter

            # Convert UTC time to float (seconds since midnight)
            msg.utc = (fix.utc_time.hour * 3600.0 + 
                      fix.utc_time.minute * 60.0 + 
                      fix.utc_time.second + 
                      fix.utc_time.microsecond / 1e6)
            msg.fix_quality = fix.fix_quality
            
            self.publisher.publish(msg)
            
            # Log valid RTK data (throttled to once per second)
            self.get_logger().info(
                f"RTK Fix - Lat: {fix.latitude:.6f}, Lon: {fix.longitude:.6f}, "
                f"Alt: {fix.altitude:.1f}m, HDOP: {fix.hdop:.1f}, "
                f"Sats: {fix.num_satellites}, Quality: {fix.fix_quality}",
                throttle_duration_sec=1.0
            )

        except StopIteration:
            pass
        except Exception as e:
            self.get_logger().error(f"Error in publish_rtk: {e}")

# -----------------------
# Main
# -----------------------
def main():
    parser = argparse.ArgumentParser(description="GNGGA → UTM ROS2 publisher for RTK")
    parser.add_argument(
        "-p", "--port",
        required=True,
        help="Path to RTK device or PTY (e.g. /dev/ttyUSB0, /dev/pts/2)"
    )
    # Filter out ROS arguments before parsing
    import sys
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args(sys.argv[1:]))

    rclpy.init()
    node = RTKPublisher(args.port)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()