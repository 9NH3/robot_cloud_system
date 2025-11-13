#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading
import math
import time
import sys

class DebugLidarPublisher(Node):
    def __init__(self):
        super().__init__('debug_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.5, self.publish_debug_lidar_data)  # 每500ms发布一次
        self.step = 0
        self.get_logger().info('调试激光雷达发布器已启动')

    def publish_debug_lidar_data(self):
        msg = LaserScan()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        
        # 设置激光雷达参数
        msg.angle_min = -math.pi/4  # -45度
        msg.angle_max = math.pi/4   # 45度
        msg.angle_increment = math.pi / 180  # 1度分辨率
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # 生成简单的测试数据
        ranges = []
        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        
        for i in range(num_points):
            # 简单的固定模式：前20个点距离为1米，后20个点距离为2米，其余为5米
            if i < 20:
                distance = 1.0
            elif i < 40:
                distance = 2.0
            else:
                distance = 5.0
                
            ranges.append(distance)
        
        msg.ranges = ranges
        
        # 发布消息
        self.publisher_.publish(msg)
        self.step += 1
        self.get_logger().info(f'[{self.step}] 发布激光雷达数据: 点数={len(ranges)}, 示例距离={ranges[0]:.2f}m')

def main():
    print("启动调试激光雷达发布器...")
    print("此程序将发布简单的测试数据以帮助诊断激光雷达显示问题")
    
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = DebugLidarPublisher()
    
    # 在后台线程中运行ROS 2
    def spin_ros():
        rclpy.spin(test_node)
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()
    
    try:
        print("调试程序已启动，请在浏览器中检查激光雷达显示")
        print("如果5秒后仍未看到数据，请检查以下几点：")
        print("1. 主程序(app.py)是否正在运行")
        print("2. 浏览器控制台是否有JavaScript错误")
        print("3. 是否正确连接到WebSocket")
        print("按 Ctrl+C 停止程序")
        time.sleep(30)  # 运行30秒
    except KeyboardInterrupt:
        print("\n正在停止调试程序...")
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()