#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import sys
import math
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_msgs.msg import IMUmsg


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Convert Euler angles (degrees) to quaternion (x, y, z, w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y_, z, w


def validate_checksum(sentence):
    """
    Validate NMEA-style checksum.
    Checksum is XOR of all bytes between $ and * (exclusive).
    Returns True if valid, False otherwise.
    """
    try:
        start = sentence.index('$') + 1
        end = sentence.index('*')
        body = sentence[start:end]
        given = int(sentence[end + 1:end + 3], 16)
        computed = 0
        for ch in body:
            computed ^= ord(ch)
        return computed == given
    except (ValueError, IndexError):
        return False


def parse_vnymr(line):
    """
    Parse a $VNYMR sentence and return a dict of fields.
    Format: $VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ*checksum
    Returns None if checksum fails or parsing fails.
    """
    line = line.strip()
    if not line.startswith('$VNYMR'):
        return None
    if not validate_checksum(line):
        return None
    try:
        # Strip checksum suffix before splitting
        body = line.split('*')[0]
        fields = body.split(',')
        return {
            'yaw':   float(fields[1]),
            'pitch': float(fields[2]),
            'roll':  float(fields[3]),
            'mag_x': float(fields[4]),
            'mag_y': float(fields[5]),
            'mag_z': float(fields[6]),
            'acc_x': float(fields[7]),
            'acc_y': float(fields[8]),
            'acc_z': float(fields[9]),
            'gyr_x': float(fields[10]),
            'gyr_y': float(fields[11]),
            'gyr_z': float(fields[12]),
        }
    except (IndexError, ValueError):
        return None


class IMUDriver(Node):

    def __init__(self, port):
        super().__init__('imu_driver')
        self.pub = self.create_publisher(IMUmsg, 'imu', 10)
        self.ser = self.open_serial(port)
        self.set_rate_40hz()
        self.get_logger().info(f'IMU driver started on port {port}')
        self.run()

    def open_serial(self, port):
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port}')
            return ser
        except serial.SerialException as e:
            self.get_logger().fatal(f'Could not open port {port}: {e}')
            sys.exit(1)

    def set_rate_40hz(self):
        """Write the VectorNav register command to set output rate to 40 Hz."""
        # Register 07 controls the async output frequency
        # The checksum XX is a placeholder; the VN-100 accepts it via emulator
        cmd = '$VNWRG,07,40*XX\r\n'
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().info(f'Sent rate config: {cmd.strip()}')

    def build_message(self, data, raw):
        msg = IMUmsg()

        # --- Header ---
        now = self.get_clock().now().to_msg()
        msg.header = Header()
        msg.header.stamp = now
        msg.header.frame_id = 'IMU1_Frame'

        # --- Quaternion from Euler ---
        qx, qy, qz, qw = euler_to_quaternion(
            data['roll'], data['pitch'], data['yaw']
        )

        # --- IMU message ---
        msg.imu = Imu()
        msg.imu.header = msg.header
        msg.imu.orientation.x = qx
        msg.imu.orientation.y = qy
        msg.imu.orientation.z = qz
        msg.imu.orientation.w = qw
        msg.imu.linear_acceleration.x = data['acc_x']
        msg.imu.linear_acceleration.y = data['acc_y']
        msg.imu.linear_acceleration.z = data['acc_z']
        msg.imu.angular_velocity.x = data['gyr_x']
        msg.imu.angular_velocity.y = data['gyr_y']
        msg.imu.angular_velocity.z = data['gyr_z']

        # --- MagneticField message ---
        msg.mag_field = MagneticField()
        msg.mag_field.header = msg.header
        msg.mag_field.magnetic_field.x = data['mag_x']
        msg.mag_field.magnetic_field.y = data['mag_y']
        msg.mag_field.magnetic_field.z = data['mag_z']

        # --- Raw string ---
        msg.raw_imu_string = raw

        return msg

    def run(self):
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode('utf-8', errors='replace')
                data = parse_vnymr(raw)
                if data is None:
                    continue
                msg = self.build_message(data, raw.strip())
                self.pub.publish(msg)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break


def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: imu_driver <port>')
        sys.exit(1)
    port = sys.argv[1]
    node = IMUDriver(port)
    rclpy.shutdown()


if __name__ == '__main__':
    main()