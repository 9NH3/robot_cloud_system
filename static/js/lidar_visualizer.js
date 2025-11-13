// 激光雷达可视化主类
class LidarVisualizer {
  constructor() {
    console.log("开始构造LidarVisualizer对象");
    
    // 初始化DOM元素
    this.canvas = document.getElementById('lidar-canvas');
    console.log("Canvas元素:", this.canvas);
    
    if (!this.canvas) {
      console.error('未找到id为"lidar-canvas"的Canvas元素');
      return;
    }
    
    this.ctx = this.canvas.getContext('2d');
    console.log("Canvas 2D上下文:", this.ctx);
    
    if (!this.ctx) {
      console.error('无法获取Canvas 2D上下文');
      return;
    }
    
    this.pointCountEl = document.getElementById('point-count');
    this.frameIdEl = document.getElementById('frame-id');
    this.timestampEl = document.getElementById('timestamp');
    this.showBeamsEl = document.getElementById('show-beams');
    
    console.log("DOM元素检查:", {
      pointCountEl: this.pointCountEl,
      frameIdEl: this.frameIdEl,
      timestampEl: this.timestampEl,
      showBeamsEl: this.showBeamsEl
    });

    // 画布配置
    this.centerX = this.canvas.width / 2;  // 中心点X（机器人位置）
    this.centerY = this.canvas.height / 2; // 中心点Y（机器人位置）
    this.scale = 80;                       // 缩放比例：1米 = 80像素
    this.pointSize = 5;                    // 增大点大小：从2增加到5，使点更容易看到
    this.lastPoints = [];                  // 缓存上一帧点云

    // 初始化WebSocket连接
    this.initWebSocket();
    // 绘制背景网格
    this.drawBackgroundGrid();
    
    console.log('激光雷达可视化器初始化完成');
  }

  // 初始化WebSocket连接
  initWebSocket() {
    // 连接当前页面的后端服务
    const socket = io.connect(window.location.origin);

    // 连接成功回调
    socket.on('connect', () => {
      console.log('WebSocket连接成功');
    });

    // 接收激光雷达数据
    socket.on('lidar_data', (data) => {
      this.processLidarData(data);
    });

    // 连接断开回调
    socket.on('disconnect', () => {
      console.log('WebSocket连接断开，尝试重连...');
    });
  }

  // 处理激光雷达数据
  processLidarData(data) {
    console.log('处理激光雷达数据:', data);
    
    // 更新状态信息
    this.pointCountEl.textContent = data.ranges.length;
    this.frameIdEl.textContent = data.frame_id || 'unknown';
    this.timestampEl.textContent = new Date(data.timestamp * 1000).toLocaleTimeString();

    // 计算点云屏幕坐标
    const points = [];
    // 使用角度索引方式计算点坐标
    for (let i = 0; i < data.ranges.length; i++) {
      const distance = data.ranges[i];
      // 计算当前点的角度（极坐标）
      const angle = data.angle_min + i * data.angle_increment;
      
      // 极坐标转屏幕坐标（Y轴反转，因为Canvas向下为正方向）
      const x = this.centerX + distance * this.scale * Math.cos(angle);
      const y = this.centerY - distance * this.scale * Math.sin(angle); // 减号反转Y轴
      
      // 检查坐标是否有效
      if (!isNaN(x) && !isNaN(y)) {
        points.push({ x, y, distance });
      }
    }
    
    console.log('转换后的点:', points);

    // 缓存点云并绘制
    this.lastPoints = points;
    this.render();
  }

  // 渲染画布（点云和激光束）
  render() {
    if (!this.ctx) {
      console.error('Canvas 2D上下文不可用');
      return;
    }
    
    console.log("开始渲染");
    
    // 清空画布
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    console.log("画布已清空");

    // 重绘背景网格
    this.drawBackgroundGrid();

    // 绘制激光束（如果启用）
    if (this.showBeamsEl && this.showBeamsEl.checked) {
      console.log("绘制激光束");
      this.drawLaserBeams();
    }

    // 绘制点云
    this.drawPointCloud();
    
    console.log('完成渲染，点数:', this.lastPoints.length);
  }

  // 绘制点云
  drawPointCloud() {
    if (!this.ctx || !this.lastPoints) {
      console.log("无法绘制点云: ctx或lastPoints不存在");
      return;
    }
    
    console.log('绘制点云，点数:', this.lastPoints.length);
    console.log('前几个点的坐标:', this.lastPoints.slice(0, 3));
    
    // 先保存当前的绘图状态
    this.ctx.save();
    
    this.lastPoints.forEach((point, index) => {
      // 根据距离设置颜色（渐变色）
      const color = this.getColorByDistance(point.distance);
      
      console.log(`绘制第${index}个点:`, {x: point.x, y: point.y, color: color});

      // 设置填充样式
      this.ctx.fillStyle = color;
      
      // 确保路径是新的
      this.ctx.beginPath();
      
      // 绘制点
      this.ctx.arc(point.x, point.y, this.pointSize, 0, Math.PI * 2);
      
      // 填充点
      this.ctx.fill();
      
      // 可选：添加描边使点更明显
      this.ctx.strokeStyle = 'rgba(0, 0, 0, 0.5)';
      this.ctx.lineWidth = 0.5;
      this.ctx.stroke();
    });
    
    // 恢复绘图状态
    this.ctx.restore();
    
    console.log("点云绘制完成");
  }

  // 绘制激光束（从中心到每个点的连线）
  drawLaserBeams() {
    // 保存当前的绘图状态
    this.ctx.save();
    
    this.ctx.strokeStyle = 'rgba(50, 180, 255, 0.1)'; // 半透明蓝色
    this.ctx.lineWidth = 1;

    this.lastPoints.forEach((point) => {
      this.ctx.beginPath();
      this.ctx.moveTo(this.centerX, this.centerY); // 起点：机器人中心
      this.ctx.lineTo(point.x, point.y);           // 终点：激光点
      this.ctx.stroke();
    });
    
    // 恢复绘图状态
    this.ctx.restore();
  }

  // 绘制背景网格
  drawBackgroundGrid() {
    console.log("开始绘制背景网格");
    
    // 保存当前的绘图状态
    this.ctx.save();
    
    // 网格样式
    this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
    this.ctx.lineWidth = 1;

    // 绘制同心圆（每隔0.5米）
    for (let r = 0.5; r <= 5; r += 0.5) { // 最大显示5米
      const radius = r * this.scale;
      this.ctx.beginPath();
      this.ctx.arc(this.centerX, this.centerY, radius, 0, Math.PI * 2);
      this.ctx.stroke();
    }

    // 绘制十字线（坐标轴）
    this.ctx.strokeStyle = 'rgba(255, 0, 0, 0.5)'; // 红色十字线
    // 水平线（前后方向）
    this.ctx.beginPath();
    this.ctx.moveTo(0, this.centerY);
    this.ctx.lineTo(this.canvas.width, this.centerY);
    this.ctx.stroke();
    // 垂直线（左右方向）
    this.ctx.beginPath();
    this.ctx.moveTo(this.centerX, 0);
    this.ctx.lineTo(this.centerX, this.canvas.height);
    this.ctx.stroke();
    
    // 恢复绘图状态
    this.ctx.restore();
    
    console.log("背景网格绘制完成");
  }

  // 根据距离生成颜色（渐变色）
  getColorByDistance(distance) {
    // 距离0-3米：红色→黄色→绿色
    if (distance < 1) {
      // 近距离：红色
      return `rgba(255, ${Math.floor(distance * 255)}, 0, 0.8)`;
    } else if (distance < 3) {
      // 中距离：黄色→绿色
      const green = 255;
      const red = Math.floor(255 - (distance - 1) * 127.5);
      return `rgba(${red}, ${green}, 0, 0.8)`;
    } else {
      // 远距离：蓝色
      return 'rgba(0, 100, 255, 0.6)';
    }
  }
}

// 页面加载完成后初始化可视化器
window.onload = () => {
  // 检查Canvas是否存在
  if (document.getElementById('lidar-canvas')) {
    const visualizer = new LidarVisualizer();
    
    // 监听激光束显示开关变化
    document.getElementById('show-beams').addEventListener('change', () => {
      visualizer.render();
    });
  } else {
    console.error('未找到id为"lidar-canvas"的Canvas元素，请检查HTML结构');
  }
};