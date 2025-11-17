from flask import Flask, render_template, request, send_from_directory, jsonify
from flask_socketio import SocketIO, emit
import rclpy
from rclpy.node import Node
import threading
import time
from sensor_msgs.msg import Imu, BatteryState, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

# 尝试导入导航相关的模块，如果失败则标记为不可用
try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from action_msgs.msg import GoalStatus
    NAVIGATION_AVAILABLE = True
except ImportError as e:
    print(f"导航模块导入失败: {e}")
    NAVIGATION_AVAILABLE = False

# 导入任务管理模块
from tasks_api import tasks_bp

# 创建Flask应用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_secret_key_here'
socketio = SocketIO(app, cors_allowed_origins="*")

# 注册任务管理API蓝图
app.register_blueprint(tasks_bp, url_prefix='/api')

# 创建ROS 2节点类
class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.get_logger().info('WebNode已初始化')

        # 速度命令发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 电池状态订阅者
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        # IMU数据订阅者
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # 激光雷达相关配置与订阅
        self.declare_parameter('sample_step', 3)
        self.declare_parameter('max_points', 240)
        self.sample_step = self.get_parameter('sample_step').value
        self.max_points = self.get_parameter('max_points').value
        self.last_lidar_send = time.time()
        
        # 创建激光雷达数据订阅器
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info("激光雷达处理功能已初始化")
        
        # 添加地图数据订阅器
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info("地图数据处理功能已初始化")

        # 如果导航模块可用，则初始化导航动作客户端
        if NAVIGATION_AVAILABLE:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info("导航动作客户端已初始化")
        else:
            self.get_logger().warning("导航模块不可用，跳过导航功能初始化")

    # 电池状态订阅回调函数
    def battery_callback(self, msg):
        battery_data = {
            'voltage': msg.voltage,
            'percentage': msg.percentage,
            'status': msg.power_supply_status
        }
        socketio.emit('battery_data', battery_data)
        self.get_logger().info(f'接收到电池状态: 电压={msg.voltage}V, 电量={msg.percentage*100}%')

    # IMU数据处理回调函数
    def imu_callback(self, msg):
        imu_data = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
        socketio.emit('imu_data', imu_data)

    # 激光雷达数据处理回调函数
    def scan_callback(self, msg):
        current_time = time.time()
        # 控制数据传输频率 (10Hz)
        if current_time - self.last_lidar_send < 0.1: 
            return
            
        self.get_logger().info(f"收到激光雷达数据: {len(msg.ranges)} 点, frame_id: {msg.header.frame_id}")
            
        # 处理原始数据
        ranges = self.process_ranges(
            msg.ranges, 
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max
        )
        
        self.get_logger().info(f"处理后数据: {len(ranges)} 点")
        
        # 构建发送数据
        data = {
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'ranges': [float(r) for r in ranges],  # 确保所有值都是float类型
            'frame_id': msg.header.frame_id if msg.header.frame_id else 'laser_frame',
            'timestamp': current_time
        }
        
        try:
            # 发送到WebSocket
            socketio.emit('lidar_data', data)
            self.last_lidar_send = current_time
            self.get_logger().info(f"发送 {len(ranges)} 个激光点到Web")
        except Exception as e:
            self.get_logger().error(f'激光数据发送错误: {e}')
            self.get_logger().error(f'发送的数据: {data}')

    # 激光雷达数据处理函数
    def process_ranges(self, ranges, angle_min, angle_increment, range_min, range_max):
        """过滤并优化激光距离数据"""
        processed = []
        count = 0
        
        # 对原始数据进行采样
        for i in range(0, len(ranges), self.sample_step):
            if count >= self.max_points:
                break
                
            r = ranges[i]
            # 过滤无效数据
            if not (range_min < r < range_max):
                continue
                
            processed.append(r)
            count += 1
            
        return processed

    # 地图数据处理回调函数
    def map_callback(self, msg):
        # 简化地图数据以便传输
        slim_map = {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y
                    }
                }
            },
            'data': list(msg.data)  # 转换为列表
        }
        
        try:
            # 发送到WebSocket
            socketio.emit('map_update', slim_map)
            self.get_logger().info(f"发送地图数据: {msg.info.width}x{msg.info.height}, 分辨率: {msg.info.resolution}")
            self.get_logger().info(f"  原点位置: ({msg.info.origin.position.x}, {msg.info.origin.position.y})")
            self.get_logger().info(f"  数据点数: {len(msg.data)}")
        except Exception as e:
            self.get_logger().error(f'地图数据发送错误: {e}')

    # 速度控制命令发布
    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'发布速度命令: linear={linear_x}, angular={angular_z}')

    # 导航到指定位置
    def send_navigation_goal(self, x, y):
        if not NAVIGATION_AVAILABLE:
            self.get_logger().error('导航模块不可用')
            socketio.emit('navigation_status', {'status': 'error', 'message': '导航模块不可用'})
            return False
            
        # 等待动作服务器可用
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('导航服务器未响应')
            socketio.emit('navigation_status', {'status': 'error', 'message': '导航服务器未响应'})
            return False
        
        # 创建目标消息
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        
        goal_msg.pose = pose
        
        # 发送目标并设置反馈回调
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.navigation_goal_response_callback)
        
        return True

    # 导航目标响应回调
    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('导航目标被拒绝')
            socketio.emit('navigation_status', {'status': 'rejected', 'message': '导航目标被拒绝'})
            return
        
        self.get_logger().info('导航目标已接受')
        socketio.emit('navigation_status', {'status': 'accepted', 'message': '导航目标已接受'})
        
        # 获取结果
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.navigation_result_callback)

    # 导航结果回调
    def navigation_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('导航成功完成')
            socketio.emit('navigation_result', {'status': 'completed', 'message': '导航成功完成'})
        else:
            self.get_logger().info(f'导航失败，状态码: {status}')
            socketio.emit('navigation_result', {'status': 'failed', 'message': f'导航失败，状态码: {status}'})

    # 导航反馈回调
    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 发送导航反馈到Web客户端
        feedback_data = {
            'current_pose': {
                'x': feedback.current_pose.pose.position.x,
                'y': feedback.current_pose.pose.position.y
            }
        }
        socketio.emit('navigation_feedback', feedback_data)
        self.get_logger().info(f'当前位置: ({feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f})')

# 全局ROS 2节点
web_node = None

# SocketIO事件处理
@socketio.on('connect')
def handle_connect():
    print('客户端连接成功:', request.sid)
    emit('connected', {'data': '已连接到ROS 2系统'})

@socketio.on('disconnect')
def handle_disconnect():
    print('客户端断开连接:', request.sid)

@socketio.on('control_command')
def handle_control_command(data):
    linear_x = float(data.get('linear_x', 0.0))
    angular_z = float(data.get('angular_z', 0.0))
    if web_node:
        web_node.publish_cmd_vel(linear_x, angular_z)
        emit('command_response', {'status': 'success', 'message': '控制命令已发送'})

# 添加处理导航点的SocketIO事件
@socketio.on('add_waypoint')
def handle_add_waypoint(data):
    """处理添加导航点事件"""
    print('添加导航点:', data)
    emit('waypoint_added', data, broadcast=True)

@socketio.on('update_waypoint')
def handle_update_waypoint(data):
    """处理更新导航点事件"""
    print('更新导航点:', data)
    emit('waypoint_updated', data, broadcast=True)

@socketio.on('delete_waypoint')
def handle_delete_waypoint(data):
    """处理删除导航点事件"""
    print('删除导航点:', data)
    emit('waypoint_deleted', data, broadcast=True)

@socketio.on('clear_waypoints')
def handle_clear_waypoints():
    """处理清除所有导航点事件"""
    print('清除所有导航点')
    emit('waypoints_cleared', broadcast=True)

@socketio.on('start_mapping')
def handle_start_mapping():
    """处理开始建图事件"""
    print('开始建图')
    if web_node:
        # 这里应该发送命令给ROS节点开始建图
        emit('mapping_started')

@socketio.on('save_map')
def handle_save_map():
    """处理保存地图事件"""
    print('保存地图')
    if web_node:
        # 这里应该发送命令给ROS节点保存地图
        emit('map_saved')

@socketio.on('set_navigation_goal')
def handle_set_navigation_goal(data):
    """处理设置导航目标事件"""
    print('设置导航目标:', data)
    if web_node:
        # 发送导航目标到ROS 2导航栈
        success = web_node.send_navigation_goal(data['x'], data['y'])
        if success:
            emit('navigation_goal_set', data)

# 主路由
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/tasks')
def tasks():
    return render_template('tasks.html')

@app.route('/lidar_debug')
def lidar_debug():
    return send_from_directory('.', 'lidar_debug.html')

@app.route('/canvas_test')
def canvas_test():
    return send_from_directory('.', 'canvas_test.html')

@app.route('/test_canvas')
def test_canvas():
    return send_from_directory('.', 'test_canvas.html')

# 添加地图调试页面路由
@app.route('/map_debug')
def map_debug():
    return send_from_directory('.', 'map_debug.html')

# 添加WebSocket测试页面路由
@app.route('/test_websocket')
def test_websocket():
    return send_from_directory('.', 'test_websocket.html')

# 添加导航API端点
@app.route('/api/navigate', methods=['POST'])
def api_navigate():
    """REST API导航接口"""
    data = request.get_json()
    
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'error': '缺少x或y参数'}), 400
    
    x = data['x']
    y = data['y']
    
    # 发送导航目标
    if web_node:
        success = web_node.send_navigation_goal(x, y)
        if success:
            return jsonify({'status': 'accepted', 'message': '导航目标已接受'})
        else:
            return jsonify({'status': 'rejected', 'message': '导航服务器未响应'}), 500
    else:
        return jsonify({'status': 'error', 'message': 'ROS节点未初始化'}), 500

if __name__ == '__main__':
    # 初始化ROS 2
    rclpy.init()
    web_node = WebNode()

    # 在后台线程中运行ROS 2
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(web_node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.daemon = True
    ros_thread.start()

    # 启动Flask应用
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)