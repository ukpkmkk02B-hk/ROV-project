# state_machine/task_scheduler.py
import threading
import time
import logging
from enum import Enum
from typing import Dict, Any, Optional, Callable
import queue

class TaskState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"

class SystemState(Enum):
    IDLE = "system_idle"
    TASK_RUNNING = "task_running"
    ERROR = "system_error"
    SHUTDOWN = "shutdown"

class TaskScheduler:
    """任务调度状态机"""
    
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # 系统状态
        self.system_state = SystemState.IDLE
        self.current_task = None
        self.task_registry = {}  # 注册的任务
        self.task_instances = {}  # 任务实例
        
        # 线程控制
        self.running = False
        self.scheduler_thread = None
        self.lock = threading.RLock()
        
        # 状态回调
        self.state_callbacks = {}
        
        # 任务队列
        self.task_queue = queue.Queue()
        
        # 状态历史
        self.state_history = []
        self.max_history = 100
        
    def register_task(self, task_name: str, task_class: type, **kwargs):
        """注册任务类型"""
        with self.lock:
            self.task_registry[task_name] = {
                'class': task_class,
                'kwargs': kwargs
            }
            self.logger.info(f"已注册任务: {task_name}")
    
    def register_callback(self, event: str, callback: Callable):
        """注册状态变化回调"""
        if event not in self.state_callbacks:
            self.state_callbacks[event] = []
        self.state_callbacks[event].append(callback)
        
    def _trigger_callback(self, event: str, data: Any = None):
        """触发回调"""
        if event in self.state_callbacks:
            for callback in self.state_callbacks[event]:
                try:
                    callback(data)
                except Exception as e:
                    self.logger.error(f"回调执行失败 {event}: {e}")
    
    def start(self):
        """启动调度器"""
        if self.running:
            self.logger.warning("调度器已在运行")
            return
            
        self.running = True
        self.scheduler_thread = threading.Thread(target=self._scheduler_loop, daemon=True)
        self.scheduler_thread.start()
        self.logger.info("任务调度器已启动")
        
    def stop(self):
        """停止调度器"""
        self.running = False
        
        # 停止当前任务
        if self.current_task:
            self.stop_current_task()
            
        # 等待线程结束
        if self.scheduler_thread and self.scheduler_thread.is_alive():
            self.scheduler_thread.join(timeout=5.0)
            
        self.system_state = SystemState.SHUTDOWN
        self.logger.info("任务调度器已停止")
    
    def _scheduler_loop(self):
        """调度器主循环"""
        while self.running:
            try:
                # 检查任务队列
                if not self.task_queue.empty() and self.system_state == SystemState.IDLE:
                    task_request = self.task_queue.get_nowait()
                    self._start_task_internal(task_request['name'], task_request.get('params', {}))
                
                # 更新当前任务状态
                if self.current_task:
                    self._update_current_task()
                    
                time.sleep(0.05)  # 10Hz更新频率
                
            except Exception as e:
                self.logger.exception("调度器循环异常")
                time.sleep(1.0)
    
    def start_task(self, task_name: str, params: Dict[str, Any] = None):
        """启动任务（异步）"""
        if task_name not in self.task_registry:
            self.logger.error(f"未注册的任务: {task_name}")
            return False
            
        # 添加到任务队列
        self.task_queue.put({
            'name': task_name,
            'params': params or {}
        })
        
        self.logger.info(f"任务已加入队列: {task_name}")
        return True
    
    def has_pending_task(self, task_name: str) -> bool:
        """Return True when a task is queued but not started yet."""
        with self.task_queue.mutex:
            return any(item.get("name") == task_name for item in self.task_queue.queue)

    def has_pending_tasks(self, task_names) -> bool:
        """Return True when any task in task_names is queued but not started yet."""
        names = set(task_names)
        with self.task_queue.mutex:
            return any(item.get("name") in names for item in self.task_queue.queue)

    def clear_pending_tasks(self, task_names) -> int:
        """Remove queued tasks by name and return the number removed."""
        names = set(task_names)
        removed = 0
        with self.task_queue.mutex:
            pending = list(self.task_queue.queue)
            self.task_queue.queue.clear()
            for item in pending:
                if item.get("name") in names:
                    removed += 1
                else:
                    self.task_queue.queue.append(item)
        return removed

    def _start_task_internal(self, task_name: str, params: Dict[str, Any]):
        """内部启动任务"""
        with self.lock:
            if self.system_state != SystemState.IDLE:
                self.logger.warning(f"系统忙碌，无法启动任务: {task_name}")
                return False
                
            try:
                # 创建任务实例
                task_config = self.task_registry[task_name]
                task_kwargs = task_config['kwargs'].copy()
                task_kwargs.update(params)
                
                # 传递状态机引用
                task_kwargs['state_machine'] = self
                
                task_instance = task_config['class'](**task_kwargs)
                
                # 启动任务
                task_instance.start()
                
                # 更新状态
                self.current_task = {
                    'name': task_name,
                    'instance': task_instance,
                    'start_time': time.time()
                }
                
                self.task_instances[task_name] = task_instance
                self.system_state = SystemState.TASK_RUNNING
                
                # 记录状态变化
                self._add_to_history('task_started', task_name)
                self._trigger_callback('task_started', task_name)
                
                self.logger.info(f"任务已启动: {task_name}")
                return True
                
            except Exception as e:
                self.logger.exception(f"启动任务失败: {task_name}")
                self.system_state = SystemState.ERROR
                return False
    
    def _update_current_task(self):
        """更新当前任务状态"""
        if not self.current_task:
            return
            
        task_instance = self.current_task['instance']
        
        try:
            # 运行任务逻辑
            if hasattr(task_instance, 'run'):
                task_instance.run()
                
            # 检查任务状态
            if hasattr(task_instance, 'get_status'):
                status = task_instance.get_status()
                task_status = status.get('status', 'unknown')
                
                if task_status == 'completed':
                    self._handle_task_completed()
                elif task_status == 'failed':
                    self._handle_task_failed()
                elif task_status == 'stopped':
                    self._handle_task_stopped()
                    
        except Exception as e:
            self.logger.exception("更新任务状态时异常")
            self._handle_task_failed()
    
    def _handle_task_completed(self):
        """处理任务完成"""
        task_name = self.current_task['name']
        self.logger.info(f"任务完成: {task_name}")
        
        self._add_to_history('task_completed', task_name)
        self._trigger_callback('task_completed', task_name)
        
        self._cleanup_current_task()
    
    def _handle_task_failed(self):
        """处理任务失败"""
        task_name = self.current_task['name']
        self.logger.error(f"任务失败: {task_name}")
        
        self._add_to_history('task_failed', task_name)
        self._trigger_callback('task_failed', task_name)
        
        self._cleanup_current_task()
        self.system_state = SystemState.ERROR
    
    def _handle_task_stopped(self):
        """处理任务停止"""
        task_name = self.current_task['name']
        self.logger.info(f"任务停止: {task_name}")
        
        self._add_to_history('task_stopped', task_name)
        self._trigger_callback('task_stopped', task_name)
        
        self._cleanup_current_task()
    
    def _cleanup_current_task(self):
        """清理当前任务"""
        if self.current_task:
            task_name = self.current_task['name']
            
            # 从实例字典中移除
            if task_name in self.task_instances:
                del self.task_instances[task_name]
                
            self.current_task = None
            
        # 重置系统状态
        if self.system_state == SystemState.TASK_RUNNING:
            self.system_state = SystemState.IDLE
    
    def stop_current_task(self):
        """停止当前任务"""
        with self.lock:
            if not self.current_task:
                self.logger.warning("没有正在运行的任务")
                return False
                
            task_instance = self.current_task['instance']
            task_name = self.current_task['name']
            
            try:
                if hasattr(task_instance, 'stop'):
                    task_instance.stop()
                    
                self.logger.info(f"已停止任务: {task_name}")
                return True
                
            except Exception as e:
                self.logger.exception(f"停止任务失败: {task_name}")
                return False

    def promote_current_task(self, task_name: str):
        """Rename the running task without stopping its instance."""
        with self.lock:
            if not self.current_task:
                self.logger.warning("没有正在运行的任务")
                return False
            old_name = self.current_task["name"]
            task_instance = self.current_task["instance"]
            self.current_task["name"] = task_name
            if hasattr(task_instance, "name"):
                task_instance.name = task_name
            if old_name in self.task_instances:
                del self.task_instances[old_name]
            self.task_instances[task_name] = task_instance
            self._add_to_history("task_promoted", {"from": old_name, "to": task_name})
            self._trigger_callback("task_promoted", {"from": old_name, "to": task_name})
            return True
    
    def get_system_status(self):
        """获取系统状态"""
        with self.lock:
            status = {
                'system_state': self.system_state.value,
                'current_task': None,
                'registered_tasks': list(self.task_registry.keys()),
                'task_queue_size': self.task_queue.qsize()
            }
            
            if self.current_task:
                task_instance = self.current_task['instance']
                status['current_task'] = {
                    'name': self.current_task['name'],
                    'start_time': self.current_task['start_time'],
                    'duration': time.time() - self.current_task['start_time']
                }
                
                # 获取任务详细状态
                if hasattr(task_instance, 'get_status'):
                    try:
                        task_status = task_instance.get_status()
                        status['current_task'].update(task_status)
                    except Exception as e:
                        self.logger.warning(f"获取任务状态失败: {e}")
                        
            return status
    
    def _add_to_history(self, event: str, data: Any):
        """添加到状态历史"""
        self.state_history.append({
            'timestamp': time.time(),
            'event': event,
            'data': data
        })
        
        # 限制历史记录长度
        if len(self.state_history) > self.max_history:
            self.state_history.pop(0)
    
    def get_history(self, limit: int = 10):
        """获取状态历史"""
        return self.state_history[-limit:]
    
    def reset_error_state(self):
        """重置错误状态"""
        with self.lock:
            if self.system_state == SystemState.ERROR:
                self.system_state = SystemState.IDLE
                self.logger.info("已重置错误状态")
                return True
            return False
    
    # 以下是任务调用的接口方法
    def notify_task_start(self, task_name: str):
        """任务通知：已启动"""
        self.logger.debug(f"任务通知 - 启动: {task_name}")
        
    def notify_task_stop(self, task_name: str):
        """任务通知：已停止"""
        self.logger.debug(f"任务通知 - 停止: {task_name}")
        
    def notify_task_completed(self, task_name: str):
        """任务通知：已完成"""
        self.logger.debug(f"任务通知 - 完成: {task_name}")
        
    def notify_task_failed(self, task_name: str):
        """任务通知：失败"""
        self.logger.debug(f"任务通知 - 失败: {task_name}")


# 使用示例和测试代码
if __name__ == "__main__":
    import logging
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # 创建调度器
    scheduler = TaskScheduler()
    
    # 简单测试任务
    class TestTask:
        def __init__(self, name="test", duration=5, state_machine=None):
            self.name = name
            self.duration = duration
            self.state_machine = state_machine
            self.start_time = None
            self.status = "idle"
            
        def start(self):
            self.start_time = time.time()
            self.status = "running"
            print(f"[{self.name}] 任务启动")
            
        def stop(self):
            self.status = "stopped"
            print(f"[{self.name}] 任务停止")
            
        def run(self):
            if self.status != "running":
                return
                
            elapsed = time.time() - self.start_time
            if elapsed >= self.duration:
                self.status = "completed"
                print(f"[{self.name}] 任务完成")
            else:
                print(f"[{self.name}] 运行中... {elapsed:.1f}s/{self.duration}s")
                
        def get_status(self):
            return {
                "name": self.name,
                "status": self.status,
                "duration": self.duration
            }
    
    # 注册测试任务
    scheduler.register_task("test_task", TestTask, name="测试任务", duration=3)
    
    # 注册回调
    def on_task_completed(task_name):
        print(f"✅ 任务完成回调: {task_name}")
        
    scheduler.register_callback("task_completed", on_task_completed)
    
    try:
        # 启动调度器
        scheduler.start()
        print("📊 调度器已启动")
        
        # 启动测试任务
        scheduler.start_task("test_task")
        
        # 监控状态
        for i in range(20):
            status = scheduler.get_system_status()
            print(f"系统状态: {status['system_state']}")
            if status['current_task']:
                print(f"当前任务: {status['current_task']['name']} - {status['current_task'].get('status', 'unknown')}")
            time.sleep(0.5)
            
        print("🛑 停止调度器")
        scheduler.stop()
        
    except KeyboardInterrupt:
        print("\n用户中断")
        scheduler.stop()
