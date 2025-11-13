#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import threading
import math
import time

class EnhancedTestLidarPublisher(Node):
    def __init__(self):
        super().__init__('enhanced_test_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_lidar_data)  # 每100ms发布一次
        self.counter = 0
        self.get_logger().info('增强版测试激光雷达发布器已启动，将发布模拟激光雷达数据到/scan主题')

    def publish_fake_lidar_data(self):
        msg = LaserScan()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        
        # 设置激光雷达参数
        msg.angle_min = -math.pi/2  # -90度 (减少角度范围以便更容易观察)
        msg.angle_max = math.pi/2   # 90度
        msg.angle_increment = math.pi / 360  # 0.5度分辨率 (提高分辨率)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # 生成模拟激光雷达数据
        self.counter += 1
        ranges = []
        
        # 生成181个点（-90度到90度）
        num_points = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        for i in range(num_points):
            angle = msg.angle_min + i * msg.angle_increment
            
            # 模拟不同类型的障碍物
            distance = msg.range_max  # 默认为最大距离
            
            # 1. 前方固定障碍物（距离2米）
            if abs(angle) < 0.1:
                distance = 2.0
                
            # 2. 左侧移动障碍物（周期性移动）
            elif abs(angle - math.pi/4) < 0.15:  # 45度方向
                distance = 1.5 + math.sin(self.counter * 0.3) * 0.5
                
            # 3. 右侧静态障碍物（距离3米）
            elif abs(angle + math.pi/6) < 0.2:  # -30度方向
                distance = 3.0
                
            # 4. 前方圆形障碍物（模拟圆柱）
            elif abs(angle) < 0.4:
                distance = min(distance, 2.5)
                
            # 5. 添加一些随机噪声
            distance += (0.1 * math.sin(self.counter * 0.5 + angle * 5))
            
            # 确保距离在有效范围内
            distance = max(msg.range_min, min(msg.range_max, distance))
            ranges.append(distance)
        
        msg.ranges = ranges
        # 不设置intensities，保持为空
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布激光雷达数据: 点数={len(ranges)}, 前方距离={ranges[len(ranges)//2]:.2f}m')

def main():
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = EnhancedTestLidarPublisher()
    
    # 在后台线程中运行ROS 2
    def spin_ros():
        rclpy.spin(test_node)
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()
    
    try:
        print("激光雷达测试程序已启动...")
        print("请在浏览器中打开 http://localhost:5000 查看激光雷达可视化")
        print("观察页面中激光雷达区域的点云显示")
        print("按 Ctrl+C 停止程序")
        # 运行60秒或直到用户中断
        time.sleep(60)
    except KeyboardInterrupt:
        print("\n正在停止激光雷达测试程序...")
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()