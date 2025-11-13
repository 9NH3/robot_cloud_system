#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import threading
import time

class TestBatteryPublisher(Node):
    def __init__(self):
        super().__init__('test_battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(5.0, self.publish_fake_battery_data)  # 每5秒发布一次
        self.counter = 0
        self.get_logger().info('测试电池状态发布器已启动，将发布模拟电池数据到/battery_state主题')

    def publish_fake_battery_data(self):
        msg = BatteryState()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery_frame"
        
        # 模拟电池数据
        self.counter += 1
        # 电压在11.5V到12.5V之间波动
        msg.voltage = 11.5 + (self.counter % 11) * 0.1
        # 电量百分比逐渐下降
        msg.percentage = max(0.2, 1.0 - (self.counter * 0.01))
        # 充电状态
        if msg.percentage > 0.8:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif msg.percentage > 0.3:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            
        # 电池电流等其他字段
        msg.current = -1.2  # 放电电流为负值
        msg.capacity = 20.0  # 总容量20Ah
        msg.design_capacity = 20.0
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布电池状态: 电压={msg.voltage:.2f}V, 电量={msg.percentage*100:.1f}%')

def main():
    # 初始化ROS 2
    rclpy.init()
    
    # 创建测试节点
    test_node = TestBatteryPublisher()
    
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