#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import depthai as dai
import math

def dps_to_rads(v):
    """Degrees per second -> radians per second (agar kabhi chahiye ho)."""
    return v * math.pi / 180.0

def quat_multiply(q1, q2):
    """Hamilton product: q = q1 * q2, with q = [x,y,z,w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
        w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
    ]

def rpy_to_quat(roll, pitch, yaw):
    """RPY (rad) -> quaternion [x,y,z,w], ROS convention (XYZ extrinsic)."""
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    # Z * Y * X (yaw * pitch * roll)
    w = cy*cp*cr + sy*sp*sr
    x = cy*cp*sr - sy*sp*cr
    y = cy*sp*cr + sy*cp*sr
    z = sy*cp*cr - cy*sp*sr
    return [x, y, z, w]

class OakImuOnly(Node):
    def __init__(self):
        super().__init__("oak_imu_only")

        # --- Params ---
        self.declare_parameter("frame_id", "oak_imu_link")   # URDF frame
        self.declare_parameter("rate_hz", 100)

        # correction (radians). Start 0, zarurat pe roll/pitch/yaw adjust karna.
        self.declare_parameter("corr_roll", 0.0)
        self.declare_parameter("corr_pitch", 0.0)
        self.declare_parameter("corr_yaw", 0.0)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.rate_hz = int(self.get_parameter("rate_hz").get_parameter_value().integer_value or 100)
        self.corr_roll  = float(self.get_parameter("corr_roll").value)
        self.corr_pitch = float(self.get_parameter("corr_pitch").value)
        self.corr_yaw   = float(self.get_parameter("corr_yaw").value)
        self.q_corr = rpy_to_quat(self.corr_roll, self.corr_pitch, self.corr_yaw)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Imu, "/oak/imu/data", qos)

        self.get_logger().info(
            f"OAK-D IMU-only | frame_id={self.frame_id} rate={self.rate_hz}Hz "
            f"| corr_rpy=({self.corr_roll:.4f}, {self.corr_pitch:.4f}, {self.corr_yaw:.4f})"
        )

        pipeline = dai.Pipeline()
        imu = pipeline.createIMU()
        xout = pipeline.createXLinkOut()
        xout.setStreamName("imu")

        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, self.rate_hz)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, self.rate_hz)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, self.rate_hz)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(20)
        imu.out.link(xout.input)

        self.dev = dai.Device(pipeline)
        self.q = self.dev.getOutputQueue(name="imu", maxSize=50, blocking=False)

        self.timer = self.create_timer(1.0 / max(50, self.rate_hz), self.spin_once)

    def spin_once(self):
        pkt = self.q.tryGet()
        if pkt is None:
            return
        for p in pkt.packets:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Orientation
            rv = getattr(p, "rotationVector", None)
            if rv is not None:
                q_sens = [rv.i, rv.j, rv.k, rv.real]
                q_fixed = quat_multiply(q_sens, self.q_corr)
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q_fixed

                # 🔥 IMU ko thora soft bana rahe
                msg.orientation_covariance[0] = 0.05
                msg.orientation_covariance[4] = 0.05
                msg.orientation_covariance[8] = 0.05
            else:
                msg.orientation_covariance[0] = -1.0

            # Angular velocity (rad/s)
            gyro = getattr(p, "gyroscope", None) or getattr(p, "gyro", None)
            if gyro:
                msg.angular_velocity.x = gyro.x
                msg.angular_velocity.y = gyro.y
                msg.angular_velocity.z = gyro.z

                msg.angular_velocity_covariance[0] = 0.05
                msg.angular_velocity_covariance[4] = 0.05
                msg.angular_velocity_covariance[8] = 0.05

            # Linear acceleration (m/s^2)
            acc = getattr(p, "accelerometer", None) or getattr(p, "acceleroMeter", None)
            if acc:
                msg.linear_acceleration.x = acc.x
                msg.linear_acceleration.y = acc.y
                msg.linear_acceleration.z = acc.z
                msg.linear_acceleration_covariance[0] = 0.1
                msg.linear_acceleration_covariance[4] = 0.1
                msg.linear_acceleration_covariance[8] = 0.1

            self.pub.publish(msg)

def main():
    rclpy.init()
    node = OakImuOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
