# RH16 Python版本电机控制修复

## 问题描述
用户反映Python版本的RH16控制代码无法让手部硬件做出相应动作，而C++版本工作正常。

## 根本原因分析
1. **CAN通信协议不匹配** - Python版本的CAN消息格式与C++版本不一致
2. **电机控制命令格式错误** - 没有使用正确的RyMotion_ServoMove_Mix协议
3. **线程处理方式不同** - Python版本缺少类似C++的高优先级线程处理
4. **缺少电机故障清除** - 启动时没有清除电机故障状态

## 修复内容

### 1. 修复CAN消息格式 (`rh16_can_node.py`)
- **修复前**: 使用错误的CAN帧格式
- **修复后**: 使用正确的CAN帧格式，与C++版本一致
```python
# 正确的CAN帧格式
can_frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b'\x00'))
```

### 2. 修复电机控制协议 (`rh16_can_node.py`)
- **修复前**: 使用简化的电机控制格式
- **修复后**: 使用RyMotion_ServoMove_Mix协议格式
```python
# 正确的混合控制命令格式
data = struct.pack('<BHHH', 0xAA, pos, spd, cur & 0xFFFF)
# 0xAA: 混合控制命令码
# pos: 位置 (0-4095)
# spd: 速度 (0-65535)  
# cur: 电流限制 (-1000到1000)
```

### 3. 添加电机故障清除 (`rh16_ctrl_node.py`)
- **新增功能**: 启动时自动清除所有电机故障
```python
def clear_motor_fault(self, motor_id: int) -> bool:
    can_id = motor_id
    data = struct.pack('<B', 0xA1)  # 清除故障命令码
    return self.can_socket.send_message(can_id, data)
```

### 4. 改进线程处理 (`rh16_ctrl_node.py`)
- **修复前**: 使用简单的Python线程
- **修复后**: 模拟C++的高优先级线程处理
```python
# 全局线程控制 (类似C++的thread_go)
thread_go = True

# 高优先级线程函数 (类似C++的CanRx_and_uwTick_thread)
def can_rx_and_uwtick_thread(node):
    # 设置线程优先级
    # 执行CAN通信和时间节拍任务
```

### 5. 添加详细日志输出
- **新增**: 类似C++版本的详细启动日志
```python
self.get_logger().info("Using CAN interface: can0")
self.get_logger().info("CAN socket opened successfully on interface: can0")
self.get_logger().info("Receive buffer size: 212992 bytes")
self.get_logger().info("Send buffer size: 212992 bytes")
self.get_logger().info("线程调度策略: SCHED_FIFO")
self.get_logger().info("线程优先级: 80")
```

### 6. 改进CAN消息处理
- **新增**: 实际的CAN消息接收和处理
```python
def process_received_can_message(self, can_id: int, data: bytes):
    # 解析电机状态数据
    # 更新电机位置、电流、状态信息
```

## 测试验证
创建了 `test_motor_control.py` 测试脚本，验证：
- ✅ CAN消息格式正确性
- ✅ 电机参数范围检查
- ✅ 清除故障命令格式
- ✅ 多电机命令模拟

## 使用说明

### 在WSL中运行
1. 确保CAN接口已配置 (`can0`)
2. 构建Python包：
```bash
cd ryhand_sdk/py
colcon build --packages-select rh16_ctrl_py
```
3. 运行控制节点：
```bash
source install/setup.bash
ros2 run rh16_ctrl_py rh16_ctrl_node
```

### 预期日志输出
启动时应该看到类似C++版本的详细日志：
```
[INFO] Using CAN interface: can0
[INFO] CAN socket opened successfully on interface: can0
[INFO] 高优先级程序启动成功
[INFO] 线程调度策略: SCHED_FIFO
[INFO] 线程优先级: 80
[INFO] 清除电机故障...
[INFO] CAN接收和时间节拍线程已启动
```

## 关键改进点
1. **协议一致性** - 确保Python版本使用与C++版本相同的CAN协议
2. **硬件兼容性** - 正确的电机控制命令格式
3. **错误处理** - 启动时清除电机故障状态
4. **实时性** - 改进的线程处理和CAN通信
5. **调试支持** - 详细的日志输出便于问题诊断

这些修复应该能解决手部硬件无响应的问题，使Python版本的控制效果与C++版本一致。
