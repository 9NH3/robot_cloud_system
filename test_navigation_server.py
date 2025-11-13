#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
import time
import threading
import math
from collections import deque

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback)
        
        # 订阅地图数据
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.get_logger().info('导航服务器已启动')
        
        # 初始化当前位置为原点
        self.current_x = 0.0
        self.current_y = 0.0
        
        # 地图数据
        self.map_data = None
        self.map_info = None

    def map_callback(self, msg):
        """处理地图数据"""
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(f'收到地图数据: {msg.info.width}x{msg.info.height}')

    def world_to_map(self, wx, wy):
        """将世界坐标转换为地图坐标"""
        if self.map_info is None:
            return None, None
            
        # 计算地图坐标
        mx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # 检查边界
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return None, None
            
        return mx, my

    def map_to_world(self, mx, my):
        """将地图坐标转换为世界坐标"""
        if self.map_info is None:
            return None, None
            
        wx = self.map_info.origin.position.x + (mx + 0.5) * self.map_info.resolution
        wy = self.map_info.origin.position.y + (my + 0.5) * self.map_info.resolution
        
        return wx, wy

    def is_free(self, mx, my):
        """检查地图上的某个位置是否为空闲空间"""
        if self.map_data is None or self.map_info is None:
            return True  # 如果没有地图数据，默认为空闲
            
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False  # 超出边界
            
        index = my * self.map_info.width + mx
        return self.map_data[index] == 0  # 0表示空闲空间

    def heuristic(self, a, b):
        """计算启发式距离（曼哈顿距离）"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        """获取相邻节点"""
        neighbors = []
        # 四个方向：上、右、下、左
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if self.is_free(nx, ny):
                neighbors.append((nx, ny))
                
        return neighbors

    def a_star(self, start, goal):
        """A*路径规划算法"""
        # 将世界坐标转换为地图坐标
        start_mx, start_my = self.world_to_map(start[0], start[1])
        goal_mx, goal_my = self.world_to_map(goal[0], goal[1])
        
        if start_mx is None or start_my is None or goal_mx is None or goal_my is None:
            self.get_logger().warn('起始点或目标点在地图外')
            return []
            
        if not self.is_free(start_mx, start_my):
            self.get_logger().warn('起始点在障碍物上')
            return []
            
        if not self.is_free(goal_mx, goal_my):
            self.get_logger().warn('目标点在障碍物上')
            return []
        
        # A*算法实现
        open_set = deque([(0, (start_mx, start_my))])  # 优先队列：(f_score, node)
        came_from = {}  # 记录路径
        g_score = {(start_mx, start_my): 0}  # 起点到当前点的实际代价
        f_score = {(start_mx, start_my): self.heuristic((start_mx, start_my), (goal_mx, goal_my))}  # 估计总代价
        
        while open_set:
            # 取出f_score最小的节点
            current = open_set.popleft()[1]
            
            # 如果到达目标点
            if current == (goal_mx, goal_my):
                # 重构路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_mx, start_my))
                path.reverse()
                
                # 将地图坐标转换为世界坐标
                world_path = []
                for mx, my in path:
                    wx, wy = self.map_to_world(mx, my)
                    if wx is not None and wy is not None:
                        world_path.append((wx, wy))
                
                return world_path
            
            # 遍历邻居节点
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1  # 假设移动代价为1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 这是到达邻居节点的更好路径
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, (goal_mx, goal_my))
                    
                    # 将邻居节点添加到开放集合中
                    # 简化的排序：按f_score排序
                    inserted = False
                    for i, (fs, node) in enumerate(open_set):
                        if f_score[neighbor] < fs:
                            open_set.insert(i, (f_score[neighbor], neighbor))
                            inserted = True
                            break
                    if not inserted:
                        open_set.append((f_score[neighbor], neighbor))
        
        # 没有找到路径
        self.get_logger().warn('无法找到从起始点到目标点的路径')
        return []

    def execute_callback(self, goal_handle):
        self.get_logger().info('接收导航目标请求')
        
        # 获取目标位置
        target_pose = goal_handle.request.pose
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        self.get_logger().info(f'目标位置: x={target_x}, y={target_y}')
        self.get_logger().info(f'当前位置: x={self.current_x}, y={self.current_y}')
        
        # 接受目标
        goal_handle.succeed()
        
        # 使用保存的当前位置
        current_x = self.current_x
        current_y = self.current_y
        
        # 使用A*算法规划路径
        path = self.a_star((current_x, current_y), (target_x, target_y))
        
        if not path:
            self.get_logger().warn('无法规划路径，将直接移动到目标点')
            path = [(current_x, current_y), (target_x, target_y)]
        
        self.get_logger().info(f'规划路径包含 {len(path)} 个点')
        
        # 模拟导航过程
        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.current_pose = PoseStamped()
        
        # 沿路径移动
        for i, (wx, wy) in enumerate(path):
            # 更新当前位置
            self.current_x = wx
            self.current_y = wy
            
            # 模拟反馈
            feedback_msg.current_pose.pose.position.x = self.current_x
            feedback_msg.current_pose.pose.position.y = self.current_y
            goal_handle.publish_feedback(feedback_msg)
            
            # 控制更新频率
            time.sleep(0.1)
        
        self.get_logger().info(f'导航完成，当前位置: x={self.current_x}, y={self.current_y}')
        return NavigateToPose.Result()

def main():
    rclpy.init()
    navigation_server = NavigationServer()
    
    # 使用多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(navigation_server)
    
    # 在后台线程中运行
    thread = threading.Thread(target=executor.spin)
    thread.daemon = True
    thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()