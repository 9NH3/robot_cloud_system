import sqlite3
from datetime import datetime

def init_database():
    """初始化任务管理数据库"""
    conn = sqlite3.connect('tasks.db')
    c = conn.cursor()
    
    # 创建任务表
    c.execute('''CREATE TABLE IF NOT EXISTS tasks
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  name TEXT NOT NULL,
                  type TEXT NOT NULL,
                  priority TEXT NOT NULL,
                  status TEXT NOT NULL DEFAULT 'pending',
                  parameters TEXT,
                  created_at TIMESTAMP NOT NULL,
                  updated_at TIMESTAMP NOT NULL)''')
    
    # 创建任务状态记录表
    c.execute('''CREATE TABLE IF NOT EXISTS task_status_history
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  task_id INTEGER NOT NULL,
                  status TEXT NOT NULL,
                  timestamp TIMESTAMP NOT NULL,
                  message TEXT,
                  FOREIGN KEY (task_id) REFERENCES tasks (id))''')
    
    # 创建索引以提高查询性能
    c.execute('CREATE INDEX IF NOT EXISTS idx_tasks_status ON tasks (status)')
    c.execute('CREATE INDEX IF NOT EXISTS idx_tasks_priority ON tasks (priority)')
    c.execute('CREATE INDEX IF NOT EXISTS idx_status_history_task_id ON task_status_history (task_id)')
    c.execute('CREATE INDEX IF NOT EXISTS idx_status_history_timestamp ON task_status_history (timestamp)')
    
    conn.commit()
    conn.close()
    
    print("任务管理数据库初始化完成")

if __name__ == '__main__':
    init_database()