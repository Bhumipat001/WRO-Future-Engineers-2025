import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import pyrealsense2 as rs
from ultralytics import YOLO
import numpy as np
import math
import time
import cv2

class WROMissionNode(Node):
    def __init__(self, mode=1, direction=1):
        super().__init__('wro_mission_node')
        self.mode = mode
        if direction not in [1, -1]:
            raise ValueError("direction must be 1 (CW) or -1 (CCW)")
        self.direction = direction
        self.get_logger().info(f"Match direction: {'Clockwise' if self.direction == 1 else 'Counterclockwise'}")
        self.total_laps = 3 if self.direction == 1 else 4
        self.lap_count = 0
        self.last_lap_time = time.time()
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.model = YOLO('yolov8n.pt')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.lidar_ranges = []
        self.state = "mapping" if self.mode == 2 else "autonomous"
        self.map_objects = []
        self.parking_exited = False
        self.parking_position = (0.0, 0.0)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        self.lidar_ranges = np.array(msg.ranges)

    def control_loop(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return
        frame = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        steer = 0.0
        speed = 0.5
        if self.mode == 1:
            steer, speed = self.simple_lap_controller()
        elif self.mode == 2:
            if self.state == "mapping":
                self.mapping_lap(frame, depth)
            else:
                steer, speed = self.autonomous_object_lap()
        elif self.mode == 3:
            steer, speed = self.parking_mission(frame, depth)
        if len(self.lidar_ranges) > 0:
            front = self.lidar_ranges[0:10].min()
            if front < 0.5:
                speed = 0.0
                steer = 0.3
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(drive_msg)
        cv2.imshow('D415 YOLOv8', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def simple_lap_controller(self):
        steer = 0.0
        speed = 1.0
        self.check_lap_completion()
        return steer, speed

    def mapping_lap(self, frame, depth):
        results = self.model(frame)[0]
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = result
            cls_name = self.model.names[int(cls)]
            center_x = int((x1 + x2)/2)
            center_y = int((y1 + y2)/2)
            distance = depth[center_y, center_x]*0.001
            obj_global_x = self.odom_x + distance * math.cos(self.odom_yaw)
            obj_global_y = self.odom_y + distance * math.sin(self.odom_yaw)
            self.map_objects.append((obj_global_x, obj_global_y, cls_name))
        self.check_lap_completion()

    def autonomous_object_lap(self):
        steer = 0.0
        speed = 1.0
        min_distance = 100.0
        for obj_x, obj_y, color in self.map_objects:
            dx = obj_x - self.odom_x
            dy = obj_y - self.odom_y
            distance = math.hypot(dx, dy)
            angle_to_obj = math.atan2(dy, dx) - self.odom_yaw
            if -math.pi/4 < angle_to_obj < math.pi/4 and distance < min_distance:
                min_distance = distance
                if color=="Red":
                    steer = -0.5 * self.direction
                elif color=="Green":
                    steer = 0.5 * self.direction
        self.check_lap_completion()
        return steer, speed

    def parking_mission(self, frame, depth):
        steer = 0.0
        speed = 0.5
        if not self.parking_exited:
            dx = self.odom_x - self.parking_position[0]
            dy = self.odom_y - self.parking_position[1]
            if math.hypot(dx, dy) > 1.0:
                self.parking_exited = True
        else:
            if self.lap_count < self.total_laps:
                steer, speed = self.simple_lap_controller()
            else:
                dx = self.parking_position[0] - self.odom_x
                dy = self.parking_position[1] - self.odom_y
                steer = math.atan2(dy, dx) - self.odom_yaw
                speed = 0.5
        return steer, speed

    def check_lap_completion(self):
        if math.hypot(self.odom_x, self.odom_y) < 0.3:
            if time.time() - self.last_lap_time > 5:
                self.lap_count += 1
                self.last_lap_time = time.time()
                self.get_logger().info(f"Lap {self.lap_count}/{self.total_laps} completed!")
                if self.mode==2 and self.state=="mapping":
                    self.state = "autonomous"
                if self.lap_count >= self.total_laps and self.mode != 3:
                    self.get_logger().info("Mission finished!")

def main(args=None):
    rclpy.init(args=args)
    mode = 2
    direction = 1
    node = WROMissionNode(mode=mode, direction=direction)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pipeline.stop()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()