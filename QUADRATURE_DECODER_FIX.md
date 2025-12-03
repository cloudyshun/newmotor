# newmotor.ino 正交解码问题修复报告

## 问题描述

G口的计数和频率始终不变，而H口的都会正常变化。G、H口都是用于检测霍尔脉冲信号的正交编码输入。

## 根本原因

### 74HC165 读取时序问题

在 `read165_HGF()` 函数中，**CLK脉冲后立即读取QH引脚**，没有给芯片足够的传播延迟时间。

#### 原代码（有问题）：
```cpp
uint8_t read165_HGF() {
  digitalWrite(PIN_165_SHLD, LOW);            // 并行装载
  delayMicroseconds(1);
  digitalWrite(PIN_165_SHLD, HIGH);           // 进入移位
  uint8_t H = digitalRead(PIN_165_QH);        // 此时为 H ✓
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  uint8_t G = digitalRead(PIN_165_QH);        // ✗ 立即读取，可能读到旧数据
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  uint8_t F = digitalRead(PIN_165_QH);        // ✗ 立即读取，可能读到旧数据
  return (uint8_t)((H<<2)|(G<<1)|(F<<0));
}
```

### 时序分析

1. **H位读取正常**：
   - `SHLD`从LOW到HIGH后，第一个数据(H)已经在QH引脚上
   - 直接读取QH得到正确的H值 ✓

2. **G位读取异常**：
   - CLK脉冲刚完成，数据还在内部移位
   - 74HC165需要传播延迟（典型值：几十纳秒，最大可达100ns）
   - 立即读取可能得到的**还是H的旧值**
   - 导致 `current_g` 经常等于 `last_g`，无法触发变化检测 ✗

3. **F位读取同样有问题**：
   - 同样的时序问题 ✗

## 修复方案

在每次CLK脉冲**之后**、读取QH**之前**，添加 `delayMicroseconds(1)` 延迟。

#### 修复后代码：
```cpp
uint8_t read165_HGF() {
  digitalWrite(PIN_165_SHLD, LOW);            // 并行装载
  delayMicroseconds(1);
  digitalWrite(PIN_165_SHLD, HIGH);           // 进入移位
  delayMicroseconds(1);                       // ✓ 等待数据稳定
  uint8_t H = digitalRead(PIN_165_QH);        // 此时为 H
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  delayMicroseconds(1);                       // ✓ 等待移位完成，数据稳定
  uint8_t G = digitalRead(PIN_165_QH);        // 移到 G
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  delayMicroseconds(1);                       // ✓ 等待移位完成，数据稳定
  uint8_t F = digitalRead(PIN_165_QH);        // 移到 F
  return (uint8_t)((H<<2)|(G<<1)|(F<<0));
}
```

### 1微秒延迟的选择理由

1. **74HC165数据手册参数**：
   - 典型传播延迟：25ns @ 5V
   - 最大传播延迟：100ns @ 4.5V
   
2. **STM32实际情况**：
   - `digitalWrite()` 本身有一定开销（几百纳秒）
   - 但在高速时钟下可能不够
   - 1微秒延迟既能保证可靠性，又不会显著影响轮询频率

3. **影响评估**：
   - 每次 `read165_HGF()` 增加约3微秒
   - loop轮询频率从约300kHz降到约200kHz
   - 对于霍尔脉冲信号（通常<10kHz）完全足够

## 正交解码逻辑验证

### 当前实现（第360-375行）：

```cpp
// 检测G相变化 (A相)
if (current_g != last_g) {
    g_count++;
    // A相变化时，读取B相电平判断方向
    if (current_g == current_h) {
        position++;  // 正转
    } else {
        position--;  // 反转
    }
    last_g = current_g;
}

// 检测H相变化 (B相)
if (current_h != last_h) {
    h_count++;
    last_h = current_h;
}
```

### 解码方法：单边沿解码

- **只在G相(A相)变化时检测方向**
- 通过比较 `current_g` 和 `current_h` 判断旋转方向
- H相(B相)变化只递增计数，不影响位置

### 方向判断逻辑

这种逻辑的正确性**取决于霍尔传感器的相位关系和接线方式**：

#### 情况1：标准正交编码（90度相位差，正转时A超前B）
正转序列：`00 → 10 → 11 → 01 → 00`
- `00→10`: G变化, G=1, H=0, G≠H → position--
- `11→01`: G变化, G=0, H=1, G≠H → position--

**结果：正转时position减小** → 如果实际正转时position在减小，说明逻辑正确

#### 情况2：反相或不同接线
如果实际测试时发现方向反了，只需将判断条件取反：
```cpp
if (current_g != current_h) {  // 原来是 ==
    position++;
} else {
    position--;
}
```

## 测试建议

1. **烧录修复后的代码**
2. **观察串口输出**（每秒输出一次）：
   ```
   HGF=0b... G=... H=... last_g=... last_h=... g_count=... h_count=...
   ```
3. **验证G口计数**：
   - 手动转动电机或霍尔传感器
   - 观察 `g_count` 和 `h_count` 是否都在增加
4. **验证方向判断**：
   - 正转时观察 `position` 的变化
   - 反转时观察 `position` 的变化
   - 如果方向反了，修改判断条件

## 修复的文件

1. **newmotor.ino** - 主要的电机控制和正交解码程序
2. **store8.txt** - 74HC165/Hall监控程序（也存在相同问题）

两个文件中的 `read165_HGF()` 函数都已修复。

## 总结

**主要问题**：74HC165的SPI读取时序不正确，导致G位和F位读取失败。

**根本解决方案**：在CLK脉冲后添加1微秒延迟，确保数据稳定后再读取。

**次要注意事项**：正交解码的方向判断逻辑需要根据实际硬件接线进行验证和调整。

**影响范围**：
- newmotor.ino中的G口和H口正交解码
- store8.txt中的H、G、F限位开关检测
