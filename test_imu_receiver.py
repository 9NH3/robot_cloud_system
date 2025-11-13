#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import time
import math

class TestIMUPublisher(Node):
    def __init__(self):
        super().__init__('test_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_imu_data)  # 每秒发布一次
        self.counter = 0
        self.get_logger().info('测试IMU发布器已启动，将发布模拟IMU数据到/imu主题')

    def publish_fake_imu_data(self):
        msg = Imu()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"
        
        # 模拟方向数据（四元数）
        self.counter += 1
        angle = self.counter * 0.1
        msg.orientation.x = math.sin(angle/2)
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = math.cos(angle/2)
        
        # 模拟角速度数据
        msg.angular_velocity.x = 0.1 * math.sin(angle)
        msg.angular_velocity.y = 0.05 * math.cos(angle)
        msg.angular_velocity.z = 0.02 * math.sin(2*angle)
        
        # 模拟线性加速度数据
        msg.linear_acceleration.x = 0.5 * math.sin(angle)
        msg.linear_acceleration.y = 0.3 * math.cos(angle)
        msg.linear_acceleration.z = 9.8 + 0.1 * math.sin(2*angle)
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布IMU数据: 角度={angle:.2f}, orientation_x={msg.orientation.x:.3f}')

def main():
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = TestIMUPublisher()
    
    # 在后台线程中运行ROS 2
    def spin_ros():
        rclpy.spin(test_node)
    
    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()
    
    try:
        # 运行30秒或直到用户中断
        time.sleep(30)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()