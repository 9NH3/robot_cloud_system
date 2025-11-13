from flask import Blueprint, render_template

bp = Blueprint('routes', __name__)

@bp.route('/dashboard')
def dashboard():
    # 渲染一个仪表板模板（templates/dashboard.html）
    return render_template('dashboard.html', title='仪表板')

@bp.route('/tasks')
def tasks():
    # 简单示例：任务管理页面
    return render_template('tasks.html', title='任务管理')

# 保留一个简单测试路由（可选）
@bp.route('/hello')
def hello():
    return "Hello from robot_dashboard routes!"
