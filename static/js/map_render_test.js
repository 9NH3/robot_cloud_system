// 简单的地图渲染测试函数
function testMapRendering() {
    console.log("开始测试地图渲染功能");
    
    // 获取Canvas元素
    const canvas = document.getElementById('test-map-canvas');
    if (!canvas) {
        console.error("未找到测试Canvas元素");
        return;
    }
    
    const ctx = canvas.getContext('2d');
    if (!ctx) {
        console.error("无法获取Canvas上下文");
        return;
    }
    
    // 设置Canvas尺寸
    canvas.width = 400;
    canvas.height = 400;
    
    // 清除Canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // 创建测试地图数据
    const width = 40;
    const height = 40;
    const testData = [];
    
    // 生成测试数据
    for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
            // 创建边界
            if (x === 0 || x === width-1 || y === 0 || y === height-1) {
                testData.push(100); // 障碍物 - 黑色
            }
            // 创建一个简单的形状
            else if (x > 10 && x < 30 && y > 10 && y < 30) {
                testData.push(100); // 障碍物 - 黑色
            }
            else if (x > 15 && x < 25 && y > 15 && y < 25) {
                testData.push(0); // 自由空间 - 白色
            }
            else {
                testData.push(-1); // 未知区域 - 灰色
            }
        }
    }
    
    console.log("生成测试地图数据:", testData.length, "个点");
    
    // 创建图像数据
    const imageData = ctx.createImageData(width, height);
    const COLOR_MAP = {
        100: [0, 0, 0, 255],      // 黑色 - 障碍物
        0: [255, 255, 255, 255],  // 白色 - 空闲区域
        '-1': [128, 128, 128, 255] // 灰色 - 未知区域
    };
    
    // 填充像素数据
    for (let i = 0, j = 0; i < testData.length; i++, j += 4) {
        const cellValue = testData[i];
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
    
    // 绘制到主Canvas（放大显示）
    const scale = Math.min(canvas.width / width, canvas.height / height);
    const x = (canvas.width - width * scale) / 2;
    const y = (canvas.height - height * scale) / 2;
    
    ctx.drawImage(tempCanvas, x, y, width * scale, height * scale);
    
    console.log("地图渲染测试完成");
}

// 页面加载完成后执行测试
document.addEventListener('DOMContentLoaded', function() {
    console.log("页面加载完成，准备测试地图渲染");
    
    // 创建测试Canvas
    const testContainer = document.createElement('div');
    testContainer.id = 'map-test-container';
    testContainer.style.position = 'fixed';
    testContainer.style.bottom = '10px';
    testContainer.style.right = '10px';
    testContainer.style.zIndex = '10000';
    testContainer.style.backgroundColor = 'white';
    testContainer.style.padding = '10px';
    testContainer.style.border = '1px solid #ccc';
    testContainer.style.boxShadow = '0 0 10px rgba(0,0,0,0.5)';
    
    testContainer.innerHTML = `
        <h4>地图渲染测试</h4>
        <canvas id="test-map-canvas" width="400" height="400" style="border: 1px solid #999;"></canvas>
        <button id="run-test-btn" style="margin-top: 10px;">运行测试</button>
    `;
    
    document.body.appendChild(testContainer);
    
    // 添加按钮事件
    document.getElementById('run-test-btn').addEventListener('click', testMapRendering);
    
    console.log("测试控件已添加到页面");
});