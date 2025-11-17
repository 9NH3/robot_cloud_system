#!/bin/bash

# 清理本地数据库文件脚本
# 该脚本用于清除本地生成的数据库文件，以便在分享代码时保持代码库的干净

echo "开始清理本地数据库文件..."

# 删除数据库文件
if [ -f "tasks.db" ]; then
    echo "删除数据库文件: tasks.db"
    rm tasks.db
else
    echo "数据库文件 tasks.db 不存在"
fi

echo "清理完成！"
echo ""
echo "注意：只删除了数据库文件，其他代码文件保持不变"
echo ""
echo "其他人使用时需要："
echo "  1. 运行 'python3 init_tasks_db.py' 重新初始化数据库"
echo "  2. 运行 'python3 app.py' 启动应用"