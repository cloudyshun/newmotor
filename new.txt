/*
 * 项目名称: STM32F103C8T6 智能马桶全功能控制系统
 * 版本: V2.0 (集成过零检测开关)
 * 
 * 硬件连接说明:
 * 1. 强电控制 (低电平有效，通过 MOC3023 驱动):
 *    - 风扇加热丝: PA1
 *    - 水泵:       PA2
 *    - 水即热装置: PA3
 *    - 粉碎泵:     PA4
 *    - [关键] 过零信号输入: PA0 (连接 LTV-814S 光耦输出)
 * 
 * 2. 步进电机 (通过 74HC595 + ULN2803):
 *    - 595 Data (SER):   PA5
 *    - 595 Latch (RCLK): PA6
 *    - 595 Clock (SRCLK): PA7
 *    - 电机1: 对应 595 输出的低4位 (Q0-Q3)
 *    - 电机2: 对应 595 输出的高4位 (Q4-Q7)
 * 
 * 3. RS485 通信:
 *    - 控制脚 (DE/RE): PA8
 *    - TX/RX: Serial1 (PA9/PA10)
 */

#include <Arduino.h>

// ================= 1. 引脚定义 =================
// 强电与过零检测
#define PIN_ZERO_CROSS PA0  // 过零中断输入
#define PIN_FAN        PA1  // 风扇加热丝
#define PIN_PUMP       PA2  // 水泵
#define PIN_HEATER     PA3  // 水即热装置
#define PIN_MACERATOR  PA4  // 粉碎泵

// 步进电机 (74HC595)
#define PIN_595_DATA   PA5
#define PIN_595_LATCH  PA6
#define PIN_595_CLOCK  PA7

// RS485
#define DE_RE_Pin      PA8

// 继电器控制引脚
#define PIN_DRY_FAN    PB8  // 烘干风扇
#define PIN_DEODOR_FAN PB9  // 除臭风扇
#define PIN_VALVE1     PB10 // 电磁阀1
#define PIN_VALVE2     PB11 // 电磁阀2
#define PIN_VALVE3     PB12 // 电磁阀3

// NTC温度传感器
#define NTC_PIN PB0          // NTC连接到PB0 (ADC输入)
#define SERIES_RESISTOR 10000 // 上拉电阻10kΩ
#define NTC_NOMINAL 10000     // NTC标称阻值10kΩ (25°C时)
#define TEMPERATURE_NOMINAL 25   // 标称温度25°C
#define B_COEFFICIENT 3950    // NTC的B值系数
#define ADC_RESOLUTION 4095   // STM32的12位ADC分辨率

// ================= 2. 参数设置 =================
#define STEPS_PER_REVOLUTION 300  // 电机转一圈的步数 (根据实际电机调整)
#define STEP_DELAY 5              // 步进速度 (毫秒/步)

// ================= 3. 全局变量 =================

// --- [核心] 强电负载的目标状态 (volatile 必不可少，因为在中断中读取) ---
// true = 开启(输出低电平), false = 关闭(输出高电平)
volatile bool targetStatePump = false;
volatile bool targetStateMacerator = false;

// --- [新增] 风扇加热丝移相调压控制 ---
volatile uint16_t fanDelayMicros = 0;  // 延时时间(微秒)，0=关闭
volatile bool fanTriggerEnabled = false; // 是否启用风扇加热丝

// --- [新增] 水即热装置移相调压控制 ---
volatile uint16_t heaterDelayMicros = 0;  // 延时时间(微秒)，0=关闭，100=立即触发(100%)
volatile bool heaterTriggerEnabled = false; // 是否启用加热器

// --- 温度监控相关变量 ---
float currentTemperature = 0.0;
unsigned long lastTempTime = 0;    // 上次温度更新时间
unsigned long lastUploadTime = 0;  // 上次温度上传时间

// --- 步进电机相关变量 ---
byte currentShiftOutput = 0; // 595当前的输出字节
byte motor1Nibble = 0;       // 电机1的4位状态
byte motor2Nibble = 0;       // 电机2的4位状态

int nextTargetSteps1 = STEPS_PER_REVOLUTION; 
int nextTargetSteps2 = STEPS_PER_REVOLUTION; 

// 复合动作指令的状态机
int command1Status = 0; // cmdAction1 状态标志
int command2Status = 0; // cmdAction2 状态标志
int command3Status = 0; 

// 电机1 运行时参数
bool motor1Running = false;
int motor1StepPhase = 0;      // 当前相位 (0-3)
int motor1StepCycle = 0;      // 当前步数计数
int motor1TargetSteps = 0;    // 目标总步数
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  // true=正转, false=反转
bool motor1Enabled = false;   
int motor1RevolutionCount = 0;

// 电机2 运行时参数
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true;  
bool motor2Enabled = false;   
int motor2RevolutionCount = 0;

// RS485 接收缓存
byte rs485Buffer[8];
int rs485Index = 0;

// 双相励磁序列表 (两相通电，扭矩大)
// 对应二进制: 1100, 0110, 0011, 1001
const byte stepPatternHex[4] = {0x0C, 0x06, 0x03, 0x09};
const byte stepPatternHexRev[4] = {0x09, 0x03, 0x06, 0x0C};

// ================= 4. 指令集定义 (Hex) =================
// 步进电机指令 (1-4)
const byte motor1FwdCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // M1 正转
const byte motor1RevCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00}; // M1 反转
const byte motor2FwdCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00}; // M2 正转
const byte motor2RevCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00}; // M2 反转
const byte cmdAction1[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}; // M1逆 M2正
const byte cmdAction2[8]   = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00}; // 全逆
const byte cmdAction3[8]   = {0xEE, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}; // 复合动作

// 强电负载指令 (5-12)
const byte cmdFanOn[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00}; // 风扇开
const byte cmdFanOff[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01}; // 风扇关
const byte cmdPumpOn[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00}; // 水泵开
const byte cmdPumpOff[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x01}; // 水泵关
const byte cmdMacOn[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00}; // 粉碎开
const byte cmdMacOff[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x01}; // 粉碎关
const byte cmdDryFanOn[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x05, 0x00}; // 烘干风扇开
const byte cmdDryFanOff[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x05, 0x01}; // 烘干风扇关
const byte cmdDeodorFanOn[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x06, 0x00}; // 除臭风扇开
const byte cmdDeodorFanOff[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x06, 0x01}; // 除臭风扇关
const byte cmdValve1On[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x07, 0x00}; // 电磁阀1开
const byte cmdValve1Off[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x07, 0x01}; // 电磁阀1关
const byte cmdValve2On[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x08, 0x00}; // 电磁阀2开
const byte cmdValve2Off[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x08, 0x01}; // 电磁阀2关
const byte cmdValve3On[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x09, 0x00}; // 电磁阀3开
const byte cmdValve3Off[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x09, 0x01}; // 电磁阀3关

// 水即热功率控制指令 (使用原来的加热指令，但增加功率等级)
// 格式: 0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, [功率等级]
// 功率等级: 0x00=关闭, 0x01=20%, 0x02=40%, 0x03=60%, 0x04=80%, 0x05=100%
const byte cmdHeatOff_New[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x00}; // 加热关
const byte cmdHeat20[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x01}; // 20%功率
const byte cmdHeat40[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x02}; // 40%功率
const byte cmdHeat60[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x03}; // 60%功率
const byte cmdHeat80[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x04}; // 80%功率
const byte cmdHeat100[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x05}; // 100%功率


// ================= 5. 函数声明 =================
void zeroCrossingISR(); // 中断服务函数
void heaterTimerCallback(); // 定时器回调函数（水即热）
void fanTimerCallback(); // 定时器回调函数（风扇加热丝）
void updateShiftRegister();
void stopMotor1(); void stopMotor2();
void startMotor1(); void startMotor2();
void calculatePhaseMotor1(); void calculatePhaseMotor2();
void finishRevolutionMotor1(); void finishRevolutionMotor2();
void handleStepperMotor(unsigned long currentTime);
void handleRS485Commands();
void processHexCommand(byte cmd[8]);
void sendHex485(byte data[8]);
float readTemperature(); // 温度读取函数
void temperatureToTwoBytes(float temp, byte &highByte, byte &lowByte); // 温度转换函数
void uploadTemperatureData(); // 温度上传函数

// ================= 6. 定时器回调函数 =================
// 定时器中断：延时后触发可控硅（水即热）
void heaterTimerCallback() {
  // 输出100微秒的低电平脉冲触发可控硅
  digitalWrite(PIN_HEATER, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_HEATER, HIGH);

  // 停止定时器，等待下次过零触发
  timer_pause(TIMER2);
}

// 定时器中断：延时后触发可控硅（风扇加热丝）
void fanTimerCallback() {
  // 输出100微秒的低电平脉冲触发可控硅
  digitalWrite(PIN_FAN, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_FAN, HIGH);

  // 停止定时器，等待下次过零触发
  timer_pause(TIMER3);
}

// ================= 7. Setup 初始化 =================
void setup() {
  // 调试串口 (USB)
  Serial.begin(9600);
  delay(1000);
  Serial.println("System Initializing...");

  // RS485 初始化
  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW); // 默认接收模式
  Serial1.begin(9600);          // PA9/PA10
  
  // 步进电机引脚
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);

  // --- 强电负载初始化 ---
  // 上电默认关闭 (高电平)
  pinMode(PIN_FAN, OUTPUT); 
  digitalWrite(PIN_FAN, HIGH);
  
  pinMode(PIN_PUMP, OUTPUT); 
  digitalWrite(PIN_PUMP, HIGH);
  
  pinMode(PIN_HEATER, OUTPUT); 
  digitalWrite(PIN_HEATER, HIGH);
  
  pinMode(PIN_MACERATOR, OUTPUT);
  digitalWrite(PIN_MACERATOR, HIGH);

  // --- 继电器控制引脚初始化 ---
  pinMode(PIN_DRY_FAN, OUTPUT);
  digitalWrite(PIN_DRY_FAN, LOW); // 默认关闭

  pinMode(PIN_DEODOR_FAN, OUTPUT);
  digitalWrite(PIN_DEODOR_FAN, LOW); // 默认关闭

  pinMode(PIN_VALVE1, OUTPUT);
  digitalWrite(PIN_VALVE1, LOW); // 默认关闭

  pinMode(PIN_VALVE2, OUTPUT);
  digitalWrite(PIN_VALVE2, LOW); // 默认关闭

  pinMode(PIN_VALVE3, OUTPUT);
  digitalWrite(PIN_VALVE3, LOW); // 默认关闭

  // --- 初始化ADC用于NTC温度读取 ---
  pinMode(NTC_PIN, INPUT_ANALOG);
  Serial.println("NTC Temperature Sensor Initialized");

  // --- [关键] 配置定时器2用于水即热延时触发 ---
  // 使用libmaple原生API配置定时器
  timer_pause(TIMER2);
  timer_set_prescaler(TIMER2, 71); // 72MHz / (71+1) = 1MHz，即1微秒计数
  timer_set_mode(TIMER2, TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer_set_reload(TIMER2, 10000); // 默认10ms
  timer_attach_interrupt(TIMER2, TIMER_CH1, heaterTimerCallback);
  // 注意：不在这里启动定时器，而是在过零中断中按需启动

  // --- [关键] 配置定时器3用于风扇加热丝延时触发 ---
  timer_pause(TIMER3);
  timer_set_prescaler(TIMER3, 71); // 72MHz / (71+1) = 1MHz，即1微秒计数
  timer_set_mode(TIMER3, TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer_set_reload(TIMER3, 10000); // 默认10ms
  timer_attach_interrupt(TIMER3, TIMER_CH1, fanTimerCallback);
  // 注意：不在这里启动定时器，而是在过零中断中按需启动

  // --- [关键] 开启 PA0 过零检测中断 ---
  // PA0 连接光耦，平时高电平，过零时低电平。
  // 检测下降沿 (FALLING) 代表进入过零点。
  pinMode(PIN_ZERO_CROSS, INPUT); 
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), zeroCrossingISR, FALLING);

  // 步进电机初始归零
  stopMotor1();
  stopMotor2();
  updateShiftRegister(); 
  
  // 默认启动参数
  nextTargetSteps1 = STEPS_PER_REVOLUTION;
  nextTargetSteps2 = STEPS_PER_REVOLUTION;
  motor1Direction = true; 
  motor2Direction = true; 
  motor1Enabled = true;
  motor2Enabled = true;
  command3Status = 0; 
  
  Serial.println("System Ready: Zero-Crossing Enabled");
}

// ================= 7. 主循环 =================
void loop() {
  unsigned long currentTime = millis();

  // 步进电机是非阻塞的，需要不断调用
  handleStepperMotor(currentTime);

  // 检查 RS485 指令
  handleRS485Commands();

  // 温度读取 (每2秒)
  if(currentTime - lastTempTime >= 2000) {
    currentTemperature = readTemperature();
    lastTempTime = currentTime;
    Serial.print("Current Temperature: ");
    Serial.print(currentTemperature, 1);
    Serial.println(" C");
  }

  // 温度数据上传 (每5秒)
  if(currentTime - lastUploadTime >= 5000) {
    uploadTemperatureData();
    lastUploadTime = currentTime;
  }
}

// ================= 8. 中断服务函数 (ISR) =================
// 只要交流电有电，该函数每 10ms (50Hz) 会自动执行一次
// 作用：将 targetState 的状态应用到 IO 口，实现过零开关
void zeroCrossingISR() {
  // 风扇加热丝：使用移相调压控制
  if (fanTriggerEnabled && fanDelayMicros > 0) {
    // 确保PA1为高电平（可控硅关断状态）
    digitalWrite(PIN_FAN, HIGH);

    // 设置定时器延时时间并启动
    timer_set_reload(TIMER3, fanDelayMicros);
    timer_set_count(TIMER3, 0); // 重置计数器
    timer_resume(TIMER3); // 启动定时器
  } else {
    // 关闭状态：保持高电平
    digitalWrite(PIN_FAN, HIGH);
  }

  // 其他强电负载：立即在过零点切换 (低电平有效)
  digitalWrite(PIN_PUMP,      targetStatePump      ? LOW : HIGH);
  digitalWrite(PIN_MACERATOR, targetStateMacerator ? LOW : HIGH);

  // 水即热装置：使用移相调压控制
  if (heaterTriggerEnabled && heaterDelayMicros > 0) {
    // 确保PA3为高电平（可控硅关断状态）
    digitalWrite(PIN_HEATER, HIGH);

    // 设置定时器延时时间并启动
    timer_set_reload(TIMER2, heaterDelayMicros);
    timer_set_count(TIMER2, 0); // 重置计数器
    timer_resume(TIMER2); // 启动定时器
  } else {
    // 关闭状态：保持高电平
    digitalWrite(PIN_HEATER, HIGH);
  }
}

// ================= 9. RS485 通信处理 =================

void handleRS485Commands() {
  while (Serial1.available()) {
    byte receivedByte = Serial1.read();
    // 简单的帧头检测 (0xEE)
    if (receivedByte == 0xEE) {
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      // 读取剩余7字节，带超时
      while (rs485Index < 8) {
        if (Serial1.available()) {
          rs485Buffer[rs485Index++] = Serial1.read();
        }
        if (millis() - startTime > 20) { // 20ms 超时
          rs485Index = 0; 
          return; 
        }
      }
      // 收到完整8字节，处理
      if (rs485Index == 8) {
        processHexCommand(rs485Buffer);
      }
      rs485Index = 0;
    }
  }
}

void sendHex485(byte data[8]) {
  digitalWrite(DE_RE_Pin, HIGH); // 发送模式
  delayMicroseconds(20);         // 等待电平稳定
  Serial1.write(data, 8);
  Serial1.flush();               // 等待发送完成
  delayMicroseconds(20);
  digitalWrite(DE_RE_Pin, LOW);  // 切换回接收模式
  Serial.println("Ack Sent");
}

void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;
  
  // 重置电机默认目标
  nextTargetSteps1 = STEPS_PER_REVOLUTION; 
  nextTargetSteps2 = STEPS_PER_REVOLUTION; 
  // 注意：command3Status 在收到新指令时需要根据情况重置，但在电机未跑完时不建议打断

  // --- A. 强电控制指令 (只修改变量，不操作IO) ---
  
  // 1. 风扇控制
  if (memcmp(cmd, cmdFanOn, 8) == 0) {
    fanTriggerEnabled = true;
    fanDelayMicros = 8000; // 延时8ms，导通2ms，功率约20%
    Serial.println("CMD: Fan ON 20% (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdFanOff, 8) == 0) {
    fanTriggerEnabled = false;
    fanDelayMicros = 0;
    Serial.println("CMD: Fan OFF (Wait ZC)");
    sendHex485(cmd); return;
  }
  
  // 2. 水泵控制
  else if (memcmp(cmd, cmdPumpOn, 8) == 0) {
    targetStatePump = true;
    Serial.println("CMD: Pump ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdPumpOff, 8) == 0) {
    targetStatePump = false;
    Serial.println("CMD: Pump OFF (Wait ZC)");
    sendHex485(cmd); return;
  }
  
  // 3. 水即热控制 (新版：支持功率调节)
  else if (memcmp(cmd, cmdHeatOff_New, 8) == 0) {
    heaterTriggerEnabled = false;
    heaterDelayMicros = 0;
    Serial.println("CMD: Heater OFF");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeat20, 8) == 0) {
    heaterTriggerEnabled = true;
    heaterDelayMicros = 8000; // 延时8ms，导通2ms，功率约20%
    Serial.println("CMD: Heater 20%");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeat40, 8) == 0) {
    heaterTriggerEnabled = true;
    heaterDelayMicros = 6000; // 延时6ms，导通4ms，功率约40%
    Serial.println("CMD: Heater 40%");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeat60, 8) == 0) {
    heaterTriggerEnabled = true;
    heaterDelayMicros = 4000; // 延时4ms，导通6ms，功率约60%
    Serial.println("CMD: Heater 60%");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeat80, 8) == 0) {
    heaterTriggerEnabled = true;
    heaterDelayMicros = 2000; // 延时2ms，导通8ms，功率约80%
    Serial.println("CMD: Heater 80%");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeat100, 8) == 0) {
    heaterTriggerEnabled = true;
    heaterDelayMicros = 100; // 几乎立即触发，功率约100%
    Serial.println("CMD: Heater 100%");
    sendHex485(cmd); return;
  }

  // 4. 粉碎泵控制
  else if (memcmp(cmd, cmdMacOn, 8) == 0) {
    targetStateMacerator = true;
    Serial.println("CMD: Macerator ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdMacOff, 8) == 0) {
    targetStateMacerator = false;
    Serial.println("CMD: Macerator OFF (Wait ZC)");
    sendHex485(cmd); return;
  }

  // 5. 烘干风扇控制
  else if (memcmp(cmd, cmdDryFanOn, 8) == 0) {
    digitalWrite(PIN_DRY_FAN, HIGH);
    Serial.println("CMD: Dry Fan ON");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdDryFanOff, 8) == 0) {
    digitalWrite(PIN_DRY_FAN, LOW);
    Serial.println("CMD: Dry Fan OFF");
    sendHex485(cmd); return;
  }

  // 6. 除臭风扇控制
  else if (memcmp(cmd, cmdDeodorFanOn, 8) == 0) {
    digitalWrite(PIN_DEODOR_FAN, HIGH);
    Serial.println("CMD: Deodorizing Fan ON");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdDeodorFanOff, 8) == 0) {
    digitalWrite(PIN_DEODOR_FAN, LOW);
    Serial.println("CMD: Deodorizing Fan OFF");
    sendHex485(cmd); return;
  }

  // 7. 电磁阀1控制
  else if (memcmp(cmd, cmdValve1On, 8) == 0) {
    digitalWrite(PIN_VALVE1, HIGH);
    Serial.println("CMD: Valve 1 ON");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdValve1Off, 8) == 0) {
    digitalWrite(PIN_VALVE1, LOW);
    Serial.println("CMD: Valve 1 OFF");
    sendHex485(cmd); return;
  }

  // 8. 电磁阀2控制
  else if (memcmp(cmd, cmdValve2On, 8) == 0) {
    digitalWrite(PIN_VALVE2, HIGH);
    Serial.println("CMD: Valve 2 ON");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdValve2Off, 8) == 0) {
    digitalWrite(PIN_VALVE2, LOW);
    Serial.println("CMD: Valve 2 OFF");
    sendHex485(cmd); return;
  }

  // 9. 电磁阀3控制
  else if (memcmp(cmd, cmdValve3On, 8) == 0) {
    digitalWrite(PIN_VALVE3, HIGH);
    Serial.println("CMD: Valve 3 ON");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdValve3Off, 8) == 0) {
    digitalWrite(PIN_VALVE3, LOW);
    Serial.println("CMD: Valve 3 OFF");
    sendHex485(cmd); return;
  }

  // --- B. 步进电机控制指令 ---

  // 收到新电机指令，先重置电机状态机
  command1Status = 0;
  command2Status = 0;
  command3Status = 0; 
  
  if (memcmp(cmd, cmdAction1, 8) == 0) {
    motor1Running = false; motor2Running = false;
    command1Status = 1;      // 激活 cmdAction1 状态
    motor1Direction = false; // M1 逆
    motor2Direction = true;  // M2 正
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, cmdAction2, 8) == 0) {
    motor1Running = false; motor2Running = false;
    command2Status = 1;      // 激活 cmdAction2 状态
    motor1Direction = false; // M1 逆
    motor2Direction = false; // M2 逆
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, cmdAction3, 8) == 0) {
    motor1Running = false; motor2Running = false;
    command3Status = 1;      // 激活复合动作，进入阶段一
    motor1Direction = true; // M1 顺时针转（缩回）
    motor2Direction = true; // M2 顺时针转（缩回）
    nextTargetSteps1 = 300;  // 1圈
    nextTargetSteps2 = 300;  // 1圈
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1FwdCmd, 8) == 0) {
    motor1Running = false; 
    motor1Direction = true; 
    motor1Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1RevCmd, 8) == 0) {
    motor1Running = false; 
    motor1Direction = false; 
    motor1Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2FwdCmd, 8) == 0) {
    motor2Running = false; 
    motor2Direction = true; 
    motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2RevCmd, 8) == 0) {
    motor2Running = false; 
    motor2Direction = false; 
    motor2Enabled = true;
    sendHex485(cmd);
  }
}

// ================= 10. 步进电机逻辑 =================

// 将缓存数据写入 74HC595
void updateShiftRegister() {
  currentShiftOutput = (motor1Nibble << 4) | (motor2Nibble & 0x0F);
  digitalWrite(PIN_595_LATCH, LOW);
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, LSBFIRST, currentShiftOutput);
  digitalWrite(PIN_595_LATCH, HIGH);
}

// 电机主处理函数 (在 loop 中调用)
void handleStepperMotor(unsigned long currentTime) {
  // --- 电机 1 逻辑 ---
  if(!motor1Running && motor1Enabled) {
    startMotor1();
  }
  if(motor1Running && (currentTime - motor1LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor1(); 
    motor1StepPhase++;
    if(motor1StepPhase >= 4) {
      motor1StepPhase = 0;
      motor1StepCycle++;
      if(motor1StepCycle >= motor1TargetSteps) {
        finishRevolutionMotor1();
      }
    }
    motor1LastStepTime = currentTime;
    updateShiftRegister(); 
  }

  // --- 电机 2 逻辑 ---
  if(!motor2Running && motor2Enabled) {
    startMotor2();
  }
  if(motor2Running && (currentTime - motor2LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor2(); 
    motor2StepPhase++;
    if(motor2StepPhase >= 4) {
      motor2StepPhase = 0;
      motor2StepCycle++;
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
    updateShiftRegister(); 
  }
}

// 计算电机1的励磁状态
void calculatePhaseMotor1() {
  if (motor1Direction) motor1Nibble = stepPatternHex[motor1StepPhase];
  else motor1Nibble = stepPatternHexRev[motor1StepPhase];
}

// 计算电机2的励磁状态
void calculatePhaseMotor2() {
  if (motor2Direction) motor2Nibble = stepPatternHex[motor2StepPhase];
  else motor2Nibble = stepPatternHexRev[motor2StepPhase];
}

// 启动电机1
void startMotor1() {
  if(motor1Running) return; 
  motor1RevolutionCount++;
  motor1StepCycle = 0;
  motor1StepPhase = 0;
  motor1TargetSteps = nextTargetSteps1; 
  motor1Running = true;
}

// 启动电机2
void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = nextTargetSteps2; 
  motor2Running = true;
}

// 电机1 停止逻辑
void finishRevolutionMotor1() {
  motor1Running = false;
  motor1Enabled = false;
  stopMotor1();
  updateShiftRegister();

  // 检查是否是 cmdAction1 或 cmdAction2，且两个电机都已停止
  if ((command1Status == 1 || command2Status == 1) && !motor1Running && !motor2Running) {
    digitalWrite(PIN_VALVE3, HIGH); // 打开电磁阀3
    Serial.println("CMD1/2: Both motors stopped, Valve 3 ON");
    command1Status = 0; // 重置状态
    command2Status = 0;
  }
}

// 电机2 停止逻辑 (包含复杂动作 Command3)
void finishRevolutionMotor2() {
  motor2Running = false;

  // Command 3 特殊阶段处理
  if (command3Status == 1) {
    command3Status = 2;     // 进入阶段二
    motor2Direction = false; // 变反转
    nextTargetSteps2 = 190; // 0.4圈
    startMotor2();          // 立即重启
    Serial.println("CMD3: Phase 2 Start");
    updateShiftRegister();
    return; // 退出，保持运行
  }

  if (command3Status == 2) {
      Serial.println("CMD3: Done");
      command3Status = 3; // 标记为完成状态，等待检查
  }

  motor2Enabled = false;
  stopMotor2();
  updateShiftRegister();

  // 检查是否是 cmdAction1 或 cmdAction2，且两个电机都已停止
  if ((command1Status == 1 || command2Status == 1) && !motor1Running && !motor2Running) {
    digitalWrite(PIN_VALVE3, HIGH); // 打开电磁阀3
    Serial.println("CMD1/2: Both motors stopped, Valve 3 ON");
    command1Status = 0; // 重置状态
    command2Status = 0;
  }

  // 检查是否是 cmdAction3 完成，且两个电机都已停止
  if (command3Status == 3 && !motor1Running && !motor2Running) {
    digitalWrite(PIN_VALVE3, HIGH); // 打开电磁阀3
    Serial.println("CMD3: Both motors stopped, Valve 3 ON");
    command3Status = 0; // 重置状态
  }
}

void stopMotor1() { motor1Nibble = 0x00; }
void stopMotor2() { motor2Nibble = 0x00; }

// ================= 11. 温度监控功能 =================

// 读取NTC温度传感器
float readTemperature() {
  int adcValue = analogRead(NTC_PIN);

  // 计算NTC电阻值
  float resistance = SERIES_RESISTOR / ((ADC_RESOLUTION / (float)adcValue) - 1);

  // 使用Steinhart-Hart方程计算温度
  float steinhart;
  steinhart = resistance / NTC_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);               // ln(R/Ro)
  steinhart /= B_COEFFICIENT;               // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;              // 倒数
  steinhart -= 273.15;                      // 转换为摄氏度

  return steinhart;
}

// 温度值转换为补码的辅助函数
void temperatureToTwoBytes(float temp, byte &highByte, byte &lowByte) {
  int16_t tempInt = (int16_t)(temp * 10);  // 温度×10,转为整数
  highByte = (tempInt >> 8) & 0xFF;         // 高字节
  lowByte = tempInt & 0xFF;                 // 低字节
}

// 温度数据上传函数
void uploadTemperatureData() {
  // 构建8字节十六进制温度数据帧
  // 格式: EE 10 00 01 00 01 [温度高字节] [温度低字节]
  byte tempFrame[8];
  tempFrame[0] = 0xEE;
  tempFrame[1] = 0x10;
  tempFrame[2] = 0x00;
  tempFrame[3] = 0x01;
  tempFrame[4] = 0x00;
  tempFrame[5] = 0x01;

  // 将温度转换为补码形式的两个字节
  temperatureToTwoBytes(currentTemperature, tempFrame[6], tempFrame[7]);

  // 通过RS485发送温度数据
  sendHex485(tempFrame);

  Serial.print("Temperature uploaded: ");
  Serial.print(currentTemperature, 1);
  Serial.println(" C");
}