#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import time

class TestMapPublisher(Node):
    def __init__(self):
        super().__init__('test_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每1秒发布一次
        self.counter = 0
        self.get_logger().info('测试地图发布器已启动')
        self.get_logger().info('地图话题: /map')
        self.get_logger().info('发布频率: 1 Hz')

    def timer_callback(self):
        # 创建一个模拟的地图数据
        map_msg = OccupancyGrid()
        
        # 设置消息头
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        
        # 设置地图信息
        map_msg.info.width = 200  # 200个格子宽
        map_msg.info.height = 200  # 200个格子高
        map_msg.info.resolution = 0.05  # 每个格子0.05米
        map_msg.info.origin = Pose()
        map_msg.info.origin.position = Point(x=-5.0, y=-5.0, z=0.0)  # 设置原点偏移
        map_msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # 生成模拟地图数据
        # -1 表示未知区域
        # 0 表示自由空间
        # 100 表示障碍物
        map_data = []
        
        # 创建一个简单的房间布局
        for y in range(map_msg.info.height):
            for x in range(map_msg.info.width):
                # 边界墙
                if x == 0 or x == map_msg.info.width - 1 or y == 0 or y == map_msg.info.height - 1:
                    map_data.append(100)  # 墙壁
                # 中间障碍物
                elif 50 <= x <= 60 and 50 <= y <= 150:
                    map_data.append(100)  # 垂直墙
                elif 60 <= x <= 140 and 100 <= y <= 110:
                    map_data.append(100)  # 水平墙
                # 添加一些随机元素
                elif x > 80 and x < 120 and y > 130 and y < 140:
                    map_data.append(100)  # 额外障碍物
                # 其他区域
                else:
                    # 随机生成一些障碍物和未知区域
                    rand_val = np.random.random()
                    if rand_val < 0.7:
                        map_data.append(0)  # 自由空间
                    elif rand_val < 0.9:
                        map_data.append(-1)  # 未知区域
                    else:
                        map_data.append(100)  # 障碍物
        
        map_msg.data = map_data
        
        # 发布地图
        self.publisher_.publish(map_msg)
        self.get_logger().info(f'发布地图数据 #{self.counter}')
        self.get_logger().info(f'  尺寸: {map_msg.info.width}x{map_msg.info.height}')
        self.get_logger().info(f'  分辨率: {map_msg.info.resolution} m/cell')
        self.get_logger().info(f'  原点: ({map_msg.info.origin.position.x}, {map_msg.info.origin.position.y})')
        self.get_logger().info(f'  数据点数: {len(map_msg.data)}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断发布器')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()