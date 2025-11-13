// map_navigation.js - 简化版地图导航系统
class SimpleMapNavigation {
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
            offset: { x: 0, y: 0 }
        };
        
        // 调整Canvas尺寸
        this.resizeCanvas();
        
        // 连接WebSocket
        this.connectWebSocket();
        
        // 监听窗口大小变化
        window.addEventListener('resize', () => this.handleResize());
        
        console.log('简化版地图导航系统初始化完成');
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
    }
    
    // 处理地图数据
    processMapData(data) {
        this.state.mapData = data;
        this.render();
    }
    
    // 渲染地图
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
    }
    
    // 渲染地图数据
    renderMap() {
        const { mapData } = this.state;
        const { width, height } = mapData.info;
        
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
                if (!window.simpleMapNavigation) {
                    window.simpleMapNavigation = new SimpleMapNavigation();
                    console.log('简化版地图导航系统已启动');
                }
            }, 100);
        });
    }
    
    // 如果直接访问导航页面也进行初始化
    if (window.location.hash === '#navigation') {
        setTimeout(() => {
            window.simpleMapNavigation = new SimpleMapNavigation();
        }, 500);
    }
});