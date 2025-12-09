#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import serial
import math
import threading
import time
import re

# ==========================================
# 🚀 FIXED SERIAL NODE FOR DODO BOT
# ==========================================

ENC_REGEX = re.compile(r"^ENC,l=(-?\d+),r=(-?\d+),dt=(-?\d+)$")

class HoverSerial(Node):
    def __init__(self):
        super().__init__('hover_serial')

        # ====== ROS params ======
        self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_14235333431351619082-if00')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_tf', False) # TF EKF handle karega

        # ====== CALIBRATION ======
        # Updated based on your logs
        self.declare_parameter('ticks_per_meter', 852.0)
        self.declare_parameter('wheel_radius', 0.084)   
        self.declare_parameter('track_width', 0.343)    

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.TPM  = float(self.get_parameter('ticks_per_meter').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.L    = float(self.get_parameter('track_width').value)

        self.get_logger().info(f"✅ Dodo Bot Serial Started. TPM={self.TPM}")

        self.ser = None
        self.connect_serial()

        # ====== ROS pubs/subs ======
        self.pub_odom  = self.create_publisher(Odometry, '/odom', 10) # Remapped to /raw_odom in launch
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.sub_cmd   = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        # ====== State ======
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.pos_left = 0.0
        self.pos_right = 0.0
        
        self.prev_l = 0
        self.prev_r = 0
        self.first_run = True
        self.last_cmd_time = self.get_clock().now()

        # For Velocity Calculation
        self.v_lin = 0.0
        self.v_ang = 0.0

        # PHYSICS LOCK
        self.MAX_SPEED = 1.0 

        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

        # 30 Hz publish
        self.create_timer(1.0/30.0, self.publish_loop)

    def connect_serial(self):
        try:
            if self.ser:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f"✅ Serial Connected: {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Connection Failed: {e}")
            self.ser = None

    def cmd_cb(self, msg: Twist):
        if not self.ser: return
        self.last_cmd_time = self.get_clock().now()

        lin = msg.linear.x
        ang = -msg.angular.z # Check direction if inverted

        scale_x = 300.0
        scale_z = 200.0

        spd = int(lin * scale_x)
        trn = int(ang * scale_z)
        spd = max(min(spd, 1000), -1000)
        trn = max(min(trn, 1000), -1000)

        cmd = f"speed {spd} turn {trn}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception:
            pass

    def read_loop(self):
        while rclpy.ok():
            if not self.ser:
                time.sleep(1.0)
                self.connect_serial()
                continue
            try:
                if self.ser.in_waiting > 512:
                    self.ser.reset_input_buffer()
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line: continue

                m = ENC_REGEX.match(line)
                if m:
                    raw_l = int(m.group(1))
                    raw_r = int(m.group(2))
                    dt_ms = int(m.group(3))
                    dt = dt_ms / 1000.0 
                    self.process_enc(raw_l, raw_r, dt)
            except Exception:
                time.sleep(0.001)

    def process_enc(self, raw_l, raw_r, dt):
        if self.first_run:
            self.prev_l = raw_l
            self.prev_r = raw_r
            self.first_run = False
            return

        dl = raw_l - self.prev_l
        
        # 🔥 CRITICAL FIX: Right Wheel Inverted 🔥
        # Log showed Right wheel goes NEGATIVE when moving FORWARD.
        # We invert it here so FORWARD = POSITIVE for both.
        dr = -(raw_r - self.prev_r) 

        # Junk Filter
        if abs(dl) > self.TPM or abs(dr) > self.TPM: return

        d_left_m  = dl / self.TPM
        d_right_m = dr / self.TPM
        d_center = (d_left_m + d_right_m) / 2.0

        if dt <= 0.001: return 

        # --- Calculate Velocities for EKF ---
        v_l = d_left_m / dt
        v_r = d_right_m / dt
        
        self.v_lin = (v_l + v_r) / 2.0
        self.v_ang = (v_r - v_l) / self.L

        # Physics Lock
        if abs(self.v_lin) > self.MAX_SPEED: return

        # State Update
        self.prev_l = raw_l
        self.prev_r = raw_r

        self.pos_left  += d_left_m  / self.wheel_radius
        self.pos_right += d_right_m / self.wheel_radius

        d_th = (d_right_m - d_left_m) / self.L
        self.th += d_th
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

    def publish_loop(self):
        # Auto-stop
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9:
            if self.ser:
                try: self.ser.write(b"speed 0 turn 0\n")
                except: pass
            self.v_lin = 0.0
            self.v_ang = 0.0

        now = self.get_clock().now().to_msg()
        
        # 1. Joint States
        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.pos_left, self.pos_right]
        self.pub_joint.publish(js)

        # 2. Odom
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = "odom"
        o.child_frame_id = "base_footprint"
        
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.position.z = 0.0
        
        cy = math.cos(self.th * 0.5)
        sy = math.sin(self.th * 0.5)
        o.pose.pose.orientation.z = sy
        o.pose.pose.orientation.w = cy

        # Velocity
        o.twist.twist.linear.x = self.v_lin
        o.twist.twist.angular.z = self.v_ang

        # COVARIANCE TUNING
        # Yaw = DO NOT Trust Wheel Odom (0.5) -> Let IMU handle rotation
        o.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5
        ]

        # Twist Covariance:
        o.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0, 0.0, 0.1
        ]

        self.pub_odom.publish(o)

def main(args=None):
    rclpy.init(args=args)
    node = HoverSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()