// map_navigation.js - 地图导航系统
class MapNavigation {
    constructor() {
        // 获取地图Canvas元素
        this.canvas = document.getElementById('map-canvas');
        if (!this.canvas) {
            console.error('未找到地图Canvas元素');
            return;
        }
        
        // 获取2D上下文
        this.ctx = this.canvas.getContext('2d');
        if (!this.ctx) {
            console.error('无法获取Canvas上下文');
            return;
        }
        
        // 初始化状态
        this.state = {
            mapData: null,
            scale: 1.0,
            offset: { x: 0, y: 0 },
            navigationStatus: 'idle', // idle, navigating, completed, failed
            currentPose: null,
            targetPose: null
        };
        
        // 调整Canvas尺寸
        this.resizeCanvas();
        
        // 连接WebSocket
        this.connectWebSocket();
        
        // 绑定UI事件
        this.bindUIEvents();
        
        // 监听窗口大小变化
        window.addEventListener('resize', () => this.handleResize());
        
        console.log('地图导航系统初始化完成');
    }
    
    // 调整Canvas尺寸
    resizeCanvas() {
        const container = this.canvas.parentElement;
        if (container) {
            this.canvas.width = container.clientWidth;
            this.canvas.height = container.clientHeight;
            console.log(`Canvas尺寸调整为: ${this.canvas.width} x ${this.canvas.height}`);
        }
    }
    
    // 处理窗口大小变化
    handleResize() {
        this.resizeCanvas();
        this.render();
    }
    
    // 连接WebSocket
    connectWebSocket() {
        console.log('正在连接WebSocket...');
        this.socket = io.connect(window.location.origin);
        
        this.socket.on('connect', () => {
            console.log('WebSocket连接成功');
        });
        
        this.socket.on('map_update', (data) => {
            console.log('收到地图数据:', data.info.width, 'x', data.info.height);
            this.processMapData(data);
        });
        
        // 导航状态更新
        this.socket.on('navigation_status', (data) => {
            console.log('导航状态更新:', data);
            this.handleNavigationStatus(data);
        });
        
        // 导航结果
        this.socket.on('navigation_result', (data) => {
            console.log('导航结果:', data);
            this.handleNavigationResult(data);
        });
        
        // 导航反馈（当前位置）
        this.socket.on('navigation_feedback', (data) => {
            console.log('导航反馈:', data);
            this.handleNavigationFeedback(data);
        });
    }
    
    // 处理地图数据
    processMapData(data) {
        this.state.mapData = data;
        this.render();
    }
    
    // 处理导航状态更新
    handleNavigationStatus(data) {
        this.state.navigationStatus = data.status;
        // 更新UI状态显示
        const statusElement = document.getElementById('navigation-status');
        if (statusElement) {
            statusElement.textContent = data.message;
            statusElement.className = 'alert alert-info';
        }
    }
    
    // 处理导航结果
    handleNavigationResult(data) {
        this.state.navigationStatus = data.status;
        // 更新UI结果显示
        const statusElement = document.getElementById('navigation-status');
        if (statusElement) {
            statusElement.textContent = data.message;
            if (data.status === 'completed') {
                statusElement.className = 'alert alert-success';
            } else {
                statusElement.className = 'alert alert-danger';
            }
        }
    }
    
    // 处理导航反馈（当前位置）
    handleNavigationFeedback(data) {
        this.state.currentPose = data.current_pose;
        this.render();
    }
    
    // 绑定UI事件
    bindUIEvents() {
        // 设置目标点按钮
        const setGoalButton = document.getElementById('set-goal');
        if (setGoalButton) {
            setGoalButton.addEventListener('click', () => {
                const x = parseFloat(document.getElementById('goal-x').value);
                const y = parseFloat(document.getElementById('goal-y').value);
                
                if (isNaN(x) || isNaN(y)) {
                    alert('请输入有效的坐标值');
                    return;
                }
                
                this.setNavigationGoal(x, y);
            });
        }
        
        // 开始建图按钮
        const startMappingButton = document.getElementById('start-mapping');
        if (startMappingButton) {
            startMappingButton.addEventListener('click', () => {
                this.socket.emit('start_mapping');
            });
        }
        
        // 保存地图按钮
        const saveMapButton = document.getElementById('save-map');
        if (saveMapButton) {
            saveMapButton.addEventListener('click', () => {
                this.socket.emit('save_map');
            });
        }
    }
    
    // 设置导航目标
    setNavigationGoal(x, y) {
        this.state.targetPose = { x, y };
        this.render();
        
        // 发送导航目标到服务器
        this.socket.emit('set_navigation_goal', { x, y });
        
        // 更新UI
        const statusElement = document.getElementById('navigation-status');
        if (statusElement) {
            statusElement.textContent = '正在发送导航目标...';
            statusElement.className = 'alert alert-warning';
        }
    }
    
    // 渲染地图和导航元素
    render() {
        if (!this.ctx || !this.state.mapData) {
            return;
        }
        
        // 调整Canvas尺寸
        this.resizeCanvas();
        
        // 清除Canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 渲染地图
        this.renderMap();
        
        // 渲染导航元素
        this.renderNavigationElements();
    }
    
    // 渲染地图数据
    renderMap() {
        const { mapData } = this.state;
        const { width, height, origin } = mapData.info;
        
        // 创建图像数据
        const imageData = this.ctx.createImageData(width, height);
        const COLOR_MAP = {
            100: [0, 0, 0, 255],      // 黑色 - 障碍物
            0: [255, 255, 255, 255],  // 白色 - 空闲区域
            '-1': [128, 128, 128, 255] // 灰色 - 未知区域
        };
        
        // 填充像素数据
        for (let i = 0, j = 0; i < mapData.data.length; i++, j += 4) {
            const cellValue = mapData.data[i];
            const color = COLOR_MAP[cellValue] || COLOR_MAP['-1'];
            
            imageData.data[j] = color[0];     // R
            imageData.data[j + 1] = color[1]; // G
            imageData.data[j + 2] = color[2]; // B
            imageData.data[j + 3] = color[3]; // A
        }
        
        // 创建临时Canvas来缩放图像
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const tempCtx = tempCanvas.getContext('2d');
        tempCtx.putImageData(imageData, 0, 0);
        
        // 绘制到主Canvas（居中显示）
        const scale = Math.min(this.canvas.width / width, this.canvas.height / height);
        const x = (this.canvas.width - width * scale) / 2;
        const y = (this.canvas.height - height * scale) / 2;
        
        this.ctx.drawImage(tempCanvas, x, y, width * scale, height * scale);
        
        // 保存缩放和偏移信息用于坐标转换
        this.state.scale = scale;
        this.state.offset = { x, y };
        this.state.origin = origin;
    }
    
    // 渲染导航元素（当前位置、目标位置等）
    renderNavigationElements() {
        if (!this.state.mapData) return;
        
        const { width, height, resolution, origin } = this.state.mapData.info;
        const { scale, offset } = this.state;
        
        // 渲染目标点
        if (this.state.targetPose) {
            this.ctx.save();
            this.ctx.fillStyle = '#ff0000'; // 红色
            this.ctx.beginPath();
            
            // 转换世界坐标到Canvas坐标
            // 正确的转换公式：canvasX = offset.x + (worldX - origin.x) / resolution * scale
            const canvasX = offset.x + (this.state.targetPose.x - origin.position.x) / resolution * scale;
            const canvasY = offset.y + (height - (this.state.targetPose.y - origin.position.y) / resolution) * scale;
            
            this.ctx.arc(canvasX, canvasY, 8, 0, 2 * Math.PI);
            this.ctx.fill();
            this.ctx.restore();
        }
        
        // 渲染当前位置
        if (this.state.currentPose) {
            this.ctx.save();
            this.ctx.fillStyle = '#00ff00'; // 绿色
            this.ctx.beginPath();
            
            // 转换世界坐标到Canvas坐标
            const canvasX = offset.x + (this.state.currentPose.x - origin.position.x) / resolution * scale;
            const canvasY = offset.y + (height - (this.state.currentPose.y - origin.position.y) / resolution) * scale;
            
            this.ctx.arc(canvasX, canvasY, 6, 0, 2 * Math.PI);
            this.ctx.fill();
            this.ctx.restore();
        }
    }
}

// 当页面加载完成后初始化地图系统
document.addEventListener('DOMContentLoaded', function() {
    // 监听导航标签页的显示事件
    const navigationTab = document.getElementById('navigation-tab');
    if (navigationTab) {
        navigationTab.addEventListener('shown.bs.tab', function() {
            // 延迟初始化确保DOM完全加载
            setTimeout(() => {
                if (!window.mapNavigation) {
                    window.mapNavigation = new MapNavigation();
                    console.log('地图导航系统已启动');
                }
            }, 100);
        });
    }
    
    // 如果直接访问导航页面也进行初始化
    if (window.location.hash === '#navigation') {
        setTimeout(() => {
            window.mapNavigation = new MapNavigation();
        }, 500);
    }
});