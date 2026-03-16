# 电机5和电机6同步控制修改说明

## 修改目的
电机5（Motor5）和电机6（Motor6）共同用于顶马桶功能，需要实现同步运动：
- M5前进 → M6同时前进
- M5后退 → M6同时后退
- M5停止 → M6同时停止

## 修改内容

### 1. 状态机标志结构体 (StateFlags)
**位置**: 第229-245行

**修改**:
- 添加 `bool motor6_done;` - 电机6完成标志
- 添加 `bool motor6_forward;` - 电机6方向标志

### 2. 如厕指令 - 第三阶段启动
**位置**: 第1016-1026行

**修改**:
```cpp
// 原来: 只启动M5→4500
// 现在: 同时启动M5和M6→4500
stateFlags.motor5_done = !startMotorToPosition(5, 4500, &stateFlags.motor5_forward);
stateFlags.motor6_done = !startMotorToPosition(6, 4500, &stateFlags.motor6_forward);
```

### 3. 如厕指令 - 第三阶段执行
**位置**: 第1043-1067行

**修改**:
- 添加M6电流监测
- 添加M6目标位置检测
- M6到达4500位置后停止

### 4. 如厕指令 - 完成条件
**位置**: 第1084行

**修改**:
```cpp
// 原来: motor5_done && motor1_done && motor2_done
// 现在: motor5_done && motor6_done && motor1_done && motor2_done
```
必须等待M5和M6都完成才能结束如厕动作

### 5. 电机5单独控制指令（同步控制M6）
**位置**: 第1267-1313行

**修改**: 所有M5的控制指令都同步控制M6

#### 停止指令 (cmdMotor5Stop)
```cpp
motorStop(5);
motorStop(6);
setPCA9685PWM(motors[5].pwmChannel, 0);
setPCA9685PWM(motors[6].pwmChannel, 0);
```

#### 前进指令 (cmdMotor5Forward)
```cpp
motorForward(5);
motorForward(6);
setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
```

#### 后退指令 (cmdMotor5Reverse)
```cpp
motorReverse(5);
motorReverse(6);
setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
```

#### 位置重置指令 (cmdResetMotor5)
```cpp
// 同时重置M5和M6的位置、计数器、保存状态
motors[5].position = 0;
motors[6].position = 0;
savePositionToEEPROM(5, 0);
savePositionToEEPROM(6, 0);
```

### 6. 如厕指令初始化
**位置**: 第1688-1692行

**修改**:
- 添加 `stateFlags.motor6_done = false;` 初始化

## 功能验证

### 测试项目
1. ✅ **单独控制测试**
   - 发送M5前进指令 → M5和M6应同时前进
   - 发送M5后退指令 → M5和M6应同时后退
   - 发送M5停止指令 → M5和M6应同时停止

2. ✅ **如厕复合指令测试**
   - 发送如厕指令
   - 第三阶段应同时移动M5和M6到4500位置
   - 两个电机都到达后才完成如厕动作

3. ✅ **位置重置测试**
   - 发送M5重置指令 → M5和M6位置都应归零

## 注意事项
1. **电机6独立控制**: M6的单独控制指令(cmdMotor6Forward/Reverse/Stop)仍然保留，可以独立控制M6
2. **同步精度**: 两个电机通过各自的霍尔编码器独立反馈位置，可能存在微小偏差
3. **过流保护**: M5和M6各自独立监测电流，任一电机过流都会自动停止
4. **EEPROM保存**: M5和M6的位置分别保存在不同地址

## 修改日期
2026-03-16
