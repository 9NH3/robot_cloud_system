#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading
import math
import time

class SimpleTestLidarPublisher(Node):
    def __init__(self):
        super().__init__('simple_test_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0, self.publish_simple_lidar_data)  # 每秒发布一次
        self.step = 0
        self.get_logger().info('简单测试激光雷达发布器已启动')

    def publish_simple_lidar_data(self):
        msg = LaserScan()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        
        # 设置激光雷达参数
        msg.angle_min = -math.pi/4  # -45度
        msg.angle_max = math.pi/4   # 45度
        msg.angle_increment = math.pi / 36  # 5度分辨率，共19个点
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # 生成简单的测试数据：距离依次为1,2,3...米
        ranges = []
        for i in range(19):
            ranges.append(float(i + 1))
        
        msg.ranges = ranges
        
        # 发布消息
        self.publisher_.publish(msg)
        self.step += 1
        self.get_logger().info(f'[{self.step}] 发布激光雷达数据: {len(ranges)} 点, 距离: {ranges[0]}-{ranges[-1]} 米')

def main():
    print("启动简单测试激光雷达发布器...")
    
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = SimpleTestLidarPublisher()
    
    # 在后台线程中运行ROS 2
    def spin_ros():
        rclpy.spin(test_node)
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()
    
    try:
        print("简单测试程序已启动，请在浏览器中检查激光雷达显示")
        print("按 Ctrl+C 停止程序")
        time.sleep(30)  # 运行30秒
    except KeyboardInterrupt:
        print("\n正在停止简单测试程序...")
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()