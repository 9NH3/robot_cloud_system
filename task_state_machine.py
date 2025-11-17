import sqlite3


class TaskStateMachine:
    def __init__(self):
        # 定义状态转换规则
        self.transition_rules = {
            'pending': ['in_progress', 'cancelled'],
            'in_progress': ['completed', 'failed', 'paused', 'cancelled'],
            'paused': ['in_progress', 'cancelled'],
            'completed': [],  # 终态
            'failed': ['pending', 'cancelled'],
            'cancelled': []  # 终态
        }
    
    def can_transition(self, from_state, to_state):
        """检查状态转换是否允许"""
        if from_state not in self.transition_rules:
            return False
        
        return to_state in self.transition_rules[from_state]
    
    def get_possible_transitions(self, current_state):
        """获取当前状态可能的状态转换"""
        if current_state not in self.transition_rules:
            return []
        
        return self.transition_rules[current_state]
    
    def validate_transition(self, task_id, new_state, db_conn=None):
        """验证状态转换是否有效"""
        # 获取当前状态
        if db_conn is None:
            conn = sqlite3.connect('tasks.db')
        else:
            conn = db_conn
        
        c = conn.cursor()
        c.execute('SELECT status FROM tasks WHERE id = ?', (task_id,))
        result = c.fetchone()
        
        if result is None:
            if db_conn is None:
                conn.close()
            return False, "任务不存在"
        
        current_state = result[0]
        
        # 检查状态转换是否允许
        if not self.can_transition(current_state, new_state):
            if db_conn is None:
                conn.close()
            return False, f"不允许从 {current_state} 状态转换到 {new_state} 状态"
        
        if db_conn is None:
            conn.close()
        
        return True, "状态转换有效"


# 全局状态机实例
task_state_machine = TaskStateMachine()