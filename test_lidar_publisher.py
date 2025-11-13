#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading
import math
import time

class TestLidarPublisher(Node):
    def __init__(self):
        super().__init__('test_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_lidar_data)  # 每100ms发布一次
        self.counter = 0
        self.get_logger().info('测试激光雷达发布器已启动，将发布模拟激光雷达数据到/scan主题')

    def publish_fake_lidar_data(self):
        msg = LaserScan()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        
        # 设置激光雷达参数
        msg.angle_min = -math.pi  # -180度
        msg.angle_max = math.pi   # 180度
        msg.angle_increment = math.pi / 180  # 1度分辨率
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # 生成模拟激光雷达数据
        self.counter += 1
        ranges = []
        
        # 生成360个点（360度）
        for i in range(360):
            angle = msg.angle_min + i * msg.angle_increment
            
            # 模拟一些障碍物
            # 1. 固定障碍物在前方1米处
            if abs(angle) < 0.1:
                distance = 1.0
            # 2. 移动障碍物（绕圈）
            elif abs(angle - (self.counter * 0.1) % (2 * math.pi)) < 0.2:
                distance = 1.5 + 0.5 * math.sin(self.counter * 0.2)
            # 3. 圆形墙壁（半径3米）
            else:
                distance = 3.0 + 0.2 * math.sin(3 * angle)  # 墙壁有一些凹凸
            
            # 添加一些噪声
            distance += 0.05 * math.sin(self.counter * 0.5 + angle)
            
            # 确保距离在有效范围内
            distance = max(msg.range_min, min(msg.range_max, distance))
            ranges.append(distance)
        
        msg.ranges = ranges
        # 不设置intensities，保持为空
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布激光雷达数据: 点数={len(ranges)}, 前方距离={ranges[0]:.2f}m')

def main():
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = TestLidarPublisher()
    
    # 在后台线程中运行ROS 2
    def spin_ros():
        rclpy.spin(test_node)
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()
    
    try:
        # 运行60秒或直到用户中断
        time.sleep(60)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()