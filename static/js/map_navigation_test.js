// map_navigation.js - 机器人小车地图与导航模块
class MapNavigationSystem {
    constructor() {
        this.canvas = document.getElementById('map-canvas');
        if (!this.canvas) {
            console.error('未找到地图Canvas元素');
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.state = {
            mapData: null,
            scale: 1.0,
            offset: { x: 0, y: 0 },
            mapOrigin: { x: 0, y: 0 },
            resolution: 0.05,
            isDragging: false,
            startDragPos: { x: 0, y: 0 },
            startDragOffset: { x: 0, y: 0 },
            robotPosition: { x: 0, y: 0, theta: 0 }
        };

        // 设置Canvas尺寸
        this.resizeCanvas();
        
        this.waypointManager = new WaypointManager();
        this.setupEventListeners();
        this.connectWebSocket();
        
        // 监听窗口大小变化
        window.addEventListener('resize', () => this.resizeCanvas());
        
        console.log('地图导航系统初始化完成');
    }

    // 调整Canvas尺寸
    resizeCanvas() {
        const container = this.canvas.parentElement;
        if (container) {
            const displayWidth = container.clientWidth;
            const displayHeight = container.clientHeight;
            
            // 检查是否需要调整Canvas尺寸
            if (this.canvas.width !== displayWidth || this.canvas.height !== displayHeight) {
                this.canvas.width = displayWidth;
                this.canvas.height = displayHeight;
                console.log(`Canvas尺寸调整为: ${displayWidth} x ${displayHeight}`);
                // 重新渲染
                this.render();
            }
        }
    }

    // 设置事件监听器
    setupEventListeners() {
        // 缩放控制
        this.canvas.addEventListener('wheel', (e) => this.handleZoom(e));

        // 拖拽控制
        this.canvas.addEventListener('mousedown', (e) => this.startDrag(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleDrag(e));
        this.canvas.addEventListener('mouseup', () => this.endDrag());
        this.canvas.addEventListener('mouseleave', () => this.endDrag());

        // 导航点放置（右键）
        this.canvas.addEventListener('contextmenu', (e) => this.handleWaypointPlacement(e));

        // 导航点编辑（双击）
        this.canvas.addEventListener('dblclick', (e) => this.handleWaypointEdit(e));

        // 控制按钮事件
        document.getElementById('start-mapping')?.addEventListener('click', () => this.startMapping());
        document.getElementById('save-map')?.addEventListener('click', () => this.saveMap());
        document.getElementById('clear-path')?.addEventListener('click', () => this.clearPath());
        document.getElementById('set-goal')?.addEventListener('click', () => this.setNavigationGoal());

        // 键盘快捷键
        document.addEventListener('keydown', (e) => this.handleKeyboard(e));
    }

    // 连接WebSocket接收地图数据
    connectWebSocket() {
        console.log('正在连接WebSocket...');
        this.socket = io.connect(window.location.origin);

        this.socket.on('connect', () => {
            console.log('地图WebSocket连接成功');
            document.getElementById('connection-status').textContent = '已连接';
        });

        this.socket.on('disconnect', () => {
            console.log('地图WebSocket连接断开');
            document.getElementById('connection-status').textContent = '已断开';
        });

        this.socket.on('map_update', (data) => {
            console.log('收到地图数据:', data);
            this.processMapData(data);
        });

        this.socket.on('robot_pose', (data) => {
            console.log('收到机器人位置数据:', data);
            this.updateRobotPosition(data);
        });

        this.socket.on('waypoints_update', (data) => {
            console.log('收到导航点更新:', data);
            this.waypointManager.loadWaypoints(data);
            this.render();
        });
        
        this.socket.on('waypoint_added', (data) => {
            console.log('收到导航点添加:', data);
            this.waypointManager.addWaypoint(data.x, data.y, data.label);
            this.updateWaypointCount();
            this.render();
        });
        
        this.socket.on('waypoint_deleted', (data) => {
            console.log('收到导航点删除:', data);
            this.waypointManager.deleteWaypoint(data.id);
            this.updateWaypointCount();
            this.render();
        });
    }

    // 处理地图数据
    processMapData(data) {
        console.log('处理地图数据:', data);
        this.state.mapData = data;
        this.state.resolution = data.info?.resolution || 0.05;
        
        if (data.info?.origin) {
            this.state.mapOrigin = {
                x: data.info.origin.position.x,
                y: data.info.origin.position.y
            };
        }
        
        // 更新地图信息显示
        if (data.info) {
            document.getElementById('map-size').textContent = `${data.info.width} x ${data.info.height}`;
            document.getElementById('map-resolution').textContent = data.info.resolution.toFixed(3) + ' m/pixel';
        }
        
        this.updateWaypointCount();
        this.render();
    }
    
    // 更新导航点计数显示
    updateWaypointCount() {
        document.getElementById('waypoint-count').textContent = this.waypointManager.waypoints.length;
    }

    // 更新机器人位置
    updateRobotPosition(pose) {
        this.state.robotPosition = {
            x: pose.x,
            y: pose.y,
            theta: pose.theta
        };
        this.render();
    }

    // 缩放处理
    handleZoom(e) {
        e.preventDefault();
        
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        const zoomIntensity = 0.1;
        const wheel = e.deltaY < 0 ? 1 : -1;
        const zoom = Math.exp(wheel * zoomIntensity);

        // 计算鼠标位置对应的地图坐标
        const worldX = (mouseX - this.state.offset.x) / this.state.scale;
        const worldY = (mouseY - this.state.offset.y) / this.state.scale;

        // 应用缩放
        this.state.scale *= zoom;
        this.state.scale = Math.max(0.1, Math.min(5, this.state.scale));

        // 调整偏移保持缩放中心
        this.state.offset.x = mouseX - worldX * this.state.scale;
        this.state.offset.y = mouseY - worldY * this.state.scale;

        this.render();
    }

    // 开始拖拽
    startDrag(e) {
        if (e.button === 0) { // 左键
            this.state.isDragging = true;
            this.state.startDragPos = { x: e.clientX, y: e.clientY };
            this.state.startDragOffset = { ...this.state.offset };
            this.canvas.style.cursor = 'grabbing';
        }
    }

    // 处理拖拽
    handleDrag(e) {
        if (!this.state.isDragging) return;

        const deltaX = e.clientX - this.state.startDragPos.x;
        const deltaY = e.clientY - this.state.startDragPos.y;

        this.state.offset.x = this.state.startDragOffset.x + deltaX;
        this.state.offset.y = this.state.startDragOffset.y + deltaY;

        this.render();
    }

    // 结束拖拽
    endDrag() {
        this.state.isDragging = false;
        this.canvas.style.cursor = 'default';
    }

    // 处理导航点放置
    handleWaypointPlacement(e) {
        e.preventDefault();

        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        // 转换为世界坐标
        const worldPos = this.screenToWorld(mouseX, mouseY);

        // 添加导航点
        const waypoint = this.waypointManager.addWaypoint(worldPos.x, worldPos.y);
        if (waypoint) {
            // 发送到服务器
            this.socket.emit('add_waypoint', waypoint);
            this.updateWaypointCount();
            this.render();
        }
    }
    
    // 处理导航点编辑
    handleWaypointEdit(e) {
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        // 查找最近的导航点
        const clickedPoint = this.findNearestWaypoint(mouseX, mouseY);
        if (clickedPoint) {
            const newLabel = prompt('请输入导航点标签:', clickedPoint.label);
            if (newLabel !== null) {
                this.waypointManager.updateWaypoint(clickedPoint.id, { label: newLabel });
                this.socket.emit('update_waypoint', { id: clickedPoint.id, label: newLabel });
                this.render();
            }
        }
    }
    
    // 查找最近的导航点
    findNearestWaypoint(screenX, screenY) {
        const threshold = 10; // 10像素范围内的点击都算作选中
        let nearestPoint = null;
        let minDist = Infinity;
        
        for (const waypoint of this.waypointManager.waypoints) {
            const screenPos = this.worldToScreen(waypoint.x, waypoint.y);
            const dx = screenPos.x - screenX;
            const dy = screenPos.y - screenY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            
            if (dist < threshold && dist < minDist) {
                minDist = dist;
                nearestPoint = waypoint;
            }
        }
        
        return nearestPoint;
    }

    // 开始建图
    startMapping() {
        this.socket.emit('start_mapping');
        console.log('开始建图命令已发送');
    }

    // 保存地图
    saveMap() {
        this.socket.emit('save_map');
        console.log('保存地图命令已发送');
    }

    // 清除路径
    clearPath() {
        this.waypointManager.clearWaypoints();
        this.socket.emit('clear_waypoints');
        this.updateWaypointCount();
        this.render();
        console.log('路径已清除');
    }

    // 设置导航目标
    setNavigationGoal() {
        const goalX = parseFloat(document.getElementById('goal-x').value);
        const goalY = parseFloat(document.getElementById('goal-y').value);
        const goalTheta = parseFloat(document.getElementById('goal-theta').value);

        const goal = {
            x: goalX,
            y: goalY,
            theta: goalTheta
        };

        this.socket.emit('set_navigation_goal', goal);
        console.log('导航目标已设置:', goal);
    }

    // 处理键盘事件
    handleKeyboard(e) {
        // 空格键清除路径
        if (e.code === 'Space') {
            e.preventDefault();
            this.clearPath();
        }
        // Escape键取消拖拽
        else if (e.code === 'Escape') {
            this.endDrag();
        }
    }

    // 屏幕坐标转世界坐标
    screenToWorld(screenX, screenY) {
        const worldX = (screenX - this.state.offset.x) / this.state.scale;
        const worldY = (screenY - this.state.offset.y) / this.state.scale;
        return { x: worldX, y: worldY };
    }

    // 世界坐标转屏幕坐标
    worldToScreen(worldX, worldY) {
        const screenX = worldX * this.state.scale + this.state.offset.x;
        const screenY = worldY * this.state.scale + this.state.offset.y;
        return { x: screenX, y: screenY };
    }

    // 渲染地图
    render() {
        if (!this.ctx) {
            console.error('Canvas上下文未初始化');
            return;
        }
        
        // 重置Canvas尺寸
        this.resizeCanvas();
        
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        if (this.state.mapData) {
            this.renderMap();
        }

        this.renderGrid();
        this.renderRobot();
        this.renderWaypoints();
        this.renderCoordinateSystem();
    }

    // 渲染地图数据
    renderMap() {
        if (!this.state.mapData) {
            console.warn('没有地图数据可渲染');
            return;
        }
        
        const { mapData } = this.state;
        const width = mapData.info.width;
        const height = mapData.info.height;
        
        console.log(`渲染地图: ${width} x ${height}`);

        // 创建离屏Canvas提升性能
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const tempCtx = tempCanvas.getContext('2d');

        const imgData = tempCtx.createImageData(width, height);
        const COLOR_MAP = {
            100: [0, 0, 0, 255],      // 黑色 - 障碍物
            0: [255, 255, 255, 255],  // 白色 - 空闲区域
            '-1': [128, 128, 128, 255] // 灰色 - 未知区域
        };

        // 填充像素数据
        for (let i = 0, j = 0; i < mapData.data.length; i++, j += 4) {
            const cellValue = mapData.data[i];
            const color = COLOR_MAP[cellValue] || COLOR_MAP['-1'];

            imgData.data[j] = color[0];     // R
            imgData.data[j + 1] = color[1]; // G
            imgData.data[j + 2] = color[2]; // B
            imgData.data[j + 3] = color[3]; // A
        }

        tempCtx.putImageData(imgData, 0, 0);

        // 渲染到主Canvas
        this.ctx.save();
        this.ctx.scale(this.state.scale, this.state.scale);
        this.ctx.translate(this.state.offset.x / this.state.scale, this.state.offset.y / this.state.scale);
        this.ctx.drawImage(tempCanvas, 0, 0);
        this.ctx.restore();
        
        console.log('地图渲染完成');
    }

    // 渲染网格
    renderGrid() {
        this.ctx.save();
        this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
        this.ctx.lineWidth = 1;

        const gridSize = 1.0; // 1米网格
        const scaledGridSize = gridSize * this.state.scale;

        // 计算可见区域
        const startX = Math.floor(-this.state.offset.x / scaledGridSize) * scaledGridSize;
        const startY = Math.floor(-this.state.offset.y / scaledGridSize) * scaledGridSize;
        const endX = this.canvas.width;
        const endY = this.canvas.height;

        // 绘制垂直线
        for (let x = startX; x < endX; x += scaledGridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }

        // 绘制水平线
        for (let y = startY; y < endY; y += scaledGridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }

        this.ctx.restore();
    }

    // 渲染机器人位置
    renderRobot() {
        if (!this.state.robotPosition) return;
        
        const screenPos = this.worldToScreen(this.state.robotPosition.x, this.state.robotPosition.y);
        
        this.ctx.save();
        this.ctx.translate(screenPos.x, screenPos.y);
        this.ctx.rotate(this.state.robotPosition.theta);

        // 机器人主体（三角形）
        this.ctx.fillStyle = '#3498db';
        this.ctx.beginPath();
        this.ctx.moveTo(0, -15);
        this.ctx.lineTo(-10, 10);
        this.ctx.lineTo(10, 10);
        this.ctx.closePath();
        this.ctx.fill();

        // 方向指示器
        this.ctx.strokeStyle = '#e74c3c';
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(0, -20);
        this.ctx.stroke();

        this.ctx.restore();
    }

    // 渲染导航点
    renderWaypoints() {
        this.waypointManager.waypoints.forEach(waypoint => {
            const screenPos = this.worldToScreen(waypoint.x, waypoint.y);
            
            this.ctx.save();
            
            // 导航点圆圈
            this.ctx.fillStyle = waypoint.color;
            this.ctx.beginPath();
            this.ctx.arc(screenPos.x, screenPos.y, 8, 0, Math.PI * 2);
            this.ctx.fill();
            
            // 边框
            this.ctx.strokeStyle = '#2c3e50';
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
            
            // 标签背景
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
            const labelWidth = this.ctx.measureText(waypoint.label).width;
            this.ctx.fillRect(screenPos.x + 12, screenPos.y - 20, labelWidth + 4, 16);
            
            // 标签文字
            this.ctx.fillStyle = '#2c3e50';
            this.ctx.font = '12px Arial';
            this.ctx.fillText(waypoint.label, screenPos.x + 14, screenPos.y - 8);
            
            this.ctx.restore();
        });
    }

    // 渲染坐标系
    renderCoordinateSystem() {
        this.ctx.save();
        
        // 坐标原点标记
        const originScreen = this.worldToScreen(0, 0);
        this.ctx.fillStyle = '#e74c3c';
        this.ctx.beginPath();
        this.ctx.arc(originScreen.x, originScreen.y, 5, 0, Math.PI * 2);
        this.ctx.fill();
        
        // 坐标轴
        this.ctx.strokeStyle = '#e74c3c';
        this.ctx.lineWidth = 2;
        
        // X轴
        this.ctx.beginPath();
        this.ctx.moveTo(originScreen.x, originScreen.y);
        this.ctx.lineTo(originScreen.x + 50, originScreen.y);
        this.ctx.stroke();
        
        // Y轴
        this.ctx.beginPath();
        this.ctx.moveTo(originScreen.x, originScreen.y);
        this.ctx.lineTo(originScreen.x, originScreen.y - 50);
        this.ctx.stroke();
        
        // 标签
        this.ctx.fillStyle = '#e74c3c';
        this.ctx.font = '12px Arial';
        this.ctx.fillText('X', originScreen.x + 55, originScreen.y + 5);
        this.ctx.fillText('Y', originScreen.x - 5, originScreen.y - 55);
        
        this.ctx.restore();
    }
}

// 导航点管理类
class WaypointManager {
    constructor() {
        this.waypoints = [];
        this.minDistance = 0.2; // 防止点过于接近
        this.loadFromLocalStorage();
    }

    addWaypoint(x, y, label = '') {
        const id = this.nextId();
        const newPoint = {
            id,
            x,
            y,
            label: label || `Point_${id}`,
            color: this.getDynamicColor(),
            timestamp: Date.now(),
        };

        // 检查是否过近
        if (this.isTooClose(newPoint)) {
            console.warn('导航点距离过近，添加失败');
            return null;
        }

        this.waypoints.push(newPoint);
        this.saveToLocalStorage();
        return newPoint;
    }

    updateWaypoint(id, properties) {
        const index = this.findIndex(id);
        if (index !== -1) {
            this.waypoints[index] = { ...this.waypoints[index], ...properties };
            this.saveToLocalStorage();
            return this.waypoints[index];
        }
        return null;
    }

    deleteWaypoint(id) {
        const index = this.findIndex(id);
        if (index !== -1) {
            const deleted = this.waypoints.splice(index, 1)[0];
            this.saveToLocalStorage();
            return deleted;
        }
        return null;
    }

    clearWaypoints() {
        this.waypoints = [];
        this.saveToLocalStorage();
    }

    loadWaypoints(data) {
        if (Array.isArray(data)) {
            this.waypoints = data;
        }
    }

    findIndex(id) {
        return this.waypoints.findIndex(wp => wp.id === id);
    }

    nextId() {
        return Date.now() + Math.floor(Math.random() * 1000);
    }

    isTooClose(point) {
        return this.waypoints.some(wp => {
            const dx = wp.x - point.x;
            const dy = wp.y - point.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            return dist < this.minDistance;
        });
    }

    getDynamicColor() {
        const colors = ['#e74c3c', '#3498db', '#2ecc71', '#9b59b6', '#f39c12'];
        return colors[this.waypoints.length % colors.length];
    }

    saveToLocalStorage() {
        localStorage.setItem('robot_waypoints', JSON.stringify(this.waypoints));
    }

    loadFromLocalStorage() {
        const data = localStorage.getItem('robot_waypoints');
        if (data) {
            try {
                this.waypoints = JSON.parse(data);
            } catch (e) {
                console.error('加载导航点数据失败:', e);
                this.waypoints = [];
            }
        }
    }
}

// 页面加载完成后初始化地图系统
document.addEventListener('DOMContentLoaded', function() {
    // 等待导航页面激活时再初始化
    const navigationTab = document.getElementById('navigation-tab');
    if (navigationTab) {
        navigationTab.addEventListener('shown.bs.tab', function() {
            // 延迟初始化确保Canvas已正确渲染
            setTimeout(() => {
                if (!window.mapNavigationSystem) {
                    window.mapNavigationSystem = new MapNavigationSystem();
                    console.log('地图导航系统已启动');
                }
            }, 100);
        });
    }

    // 如果直接打开导航页面，也进行初始化
    if (window.location.hash === '#navigation' || document.getElementById('navigation').classList.contains('active')) {
        setTimeout(() => {
            window.mapNavigationSystem = new MapNavigationSystem();
        }, 500);
    }
    
    // 添加测试按钮事件监听器
    const testBtn = document.getElementById('run-test-btn');
    if (testBtn) {
        testBtn.addEventListener('click', function() {
            // 如果地图导航系统尚未初始化，则初始化它
            if (!window.mapNavigationSystem) {
                window.mapNavigationSystem = new MapNavigationSystem();
            }
            
            // 生成测试数据
            const testData = {
                info: {
                    width: 100,
                    height: 100,
                    resolution: 0.05,
                    origin: {
                        position: {
                            x: 0,
                            y: 0
                        }
                    }
                },
                data: []
            };
            
            // 生成测试地图数据
            for (let i = 0; i < 100 * 100; i++) {
                // 边界设为障碍物
                const x = i % 100;
                const y = Math.floor(i / 100);
                
                if (x === 0 || x === 99 || y === 0 || y === 99) {
                    testData.data.push(100); // 障碍物
                } else if (x > 30 && x < 70 && y > 30 && y < 70) {
                    testData.data.push(100); // 中心障碍物
                } else if (Math.random() > 0.7) {
                    testData.data.push(100); // 随机障碍物
                } else {
                    testData.data.push(0); // 自由空间
                }
            }
            
            // 处理测试数据
            window.mapNavigationSystem.processMapData(testData);
        });
    }
});

// 导出供其他模块使用
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { MapNavigationSystem, WaypointManager };
}

