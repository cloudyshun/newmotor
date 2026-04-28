/* STM32F103C8T6 + 74HC595 x2 + 74HC165 + AT24C02 + PCA9685 + OLED
   功能：电机控制、正交解码、电流监测、位置记忆（EEPROM 自动保存版）
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>

// ---- 引脚定义 ----
#define PIN_SER     PB13
#define PIN_OE      PB5
#define PIN_RCLK    PB1
#define PIN_SRCLK   PB14

#define PIN_4051_S0 PA8
#define PIN_4051_S1 PB3
#define PIN_4051_S2 PB4

#define PIN_ADC_CURRENT PB0
#define SHUNT_RESISTANCE 0.05
#define CURRENT_SAMPLE_COUNT 99
#define CURRENT_UPDATE_INTERVAL 1000

#define SERIAL_SCREEN Serial1

// ---- AT24C02 EEPROM 配置 ----
#define EEPROM_ADDR 0x50           // AT24C02 器件地址
#define ADDR_MAGIC_NUMBER 0x00     // MagicNumber 地址
#define MAGIC_NUMBER 0x55          // 初始化标志
#define ADDR_MOTOR1_POS 0x01       // 电机1位置 (0x01-0x02)
#define ADDR_MOTOR2_POS 0x03       // 电机2位置 (0x03-0x04)
#define ADDR_MOTOR3_POS 0x05       // 电机3位置 (0x05-0x06)
#define ADDR_MOTOR4_POS 0x07       // 电机4位置 (0x07-0x08)
#define ADDR_MOTOR5_POS 0x09       // 电机5位置 (0x09-0x0A)
#define ADDR_MOTOR6_POS 0x0B       // 电机6位置 (0x0B-0x0C)
#define ADDR_MOTOR7_POS 0x0D       // 电机7位置 (0x0D-0x0E)

// 电流采样全局变量
#define FILTER_N 50  // 滤波长度

// ---- Motor 结构体定义 ----
struct Motor {
  // EEPROM 存储地址
  uint8_t eepromAddr;           // 位置存储起始地址（占用2字节）

  // 74HC595 控制位掩码
  uint8_t forwardMaskHigh;      // 前进：高字节掩码
  uint8_t forwardMaskLow;       // 前进：低字节掩码
  uint8_t reverseMaskHigh;      // 后退：高字节掩码
  uint8_t reverseMaskLow;       // 后退：低字节掩码

  // 霍尔信号引脚
  uint8_t hallPinA;             // 霍尔信号A相引脚
  uint8_t hallPinB;             // 霍尔信号B相引脚

  // 多路复用器和PWM通道
  uint8_t adcChannel;           // CD4051 通道号（0-7）
  uint8_t pwmChannel;           // PCA9685 PWM 通道号（0-15）

  // 运行时数据
  volatile int16_t position;    // 当前位置
  volatile uint32_t hallA_count; // A相脉冲计数
  volatile uint32_t hallB_count; // B相脉冲计数
  volatile uint8_t lastState;   // 上一次的AB状态 (bit1=A, bit0=B)

  // 电流采样滤波
  float current_buffer[FILTER_N]; // 电流滑动平均缓冲区
  int buffer_index;              // 缓冲区索引
  float sum_current;             // 电流累加和
  bool buffer_filled;            // 缓冲区是否已填满
  float current_amps;            // 当前电流值（A）

  // 频率统计
  uint32_t last_hallA_cnt;      // 上次A相计数
  uint32_t last_hallB_cnt;      // 上次B相计数
  float freq_hallA;             // A相频率（Hz）
  float freq_hallB;             // B相频率（Hz）

  // 自动保存相关
  int16_t lastSavedPosition;    // 上次保存到EEPROM的位置
  int16_t lastLoopPosition;     // 上次循环检测的位置
  unsigned long lastPosChangeTime; // 位置最后变化时间戳

  // 过流保护
  float maxCurrent;             // 最大允许电流（A）
};

// ---- 7个电机实例 ----
Motor motors[7] = {
  // motor0
  {
    0x01,                    // eepromAddr
    0x01, 0x00,             // forward: 0x01, 0x00
    0x00, 0x01,             // reverse: 0x00, 0x01
    PA0, PA1,               // hallPinA, hallPinB
    0,                      // adcChannel
    0,                      // pwmChannel
    0, 0, 0, 0,             // position, hallA_count, hallB_count, lastState
    {0}, 0, 0.0, false, 0.0, // current_buffer, buffer_index, sum_current, buffer_filled, current_amps
    0, 0, 0.0, 0.0,         // last_hallA_cnt, last_hallB_cnt, freq_hallA, freq_hallB
    0, 0, 0,                // lastSavedPosition, lastLoopPosition, lastPosChangeTime
    6.0                     // maxCurrent (A)
  },
  // motor1
  {
    0x03,
    0x02, 0x00,
    0x00, 0x02,
    PA2, PA3,
    1, 1,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  },
  // motor2
  {
    0x05,
    0x04, 0x00,
    0x00, 0x04,
    PA4, PA5,
    2, 2,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  },
  // motor3
  {
    0x07,
    0x08, 0x00,
    0x00, 0x08,
    PA6, PA7,
    3, 3,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  },
  // motor4
  {
    0x09,
    0x10, 0x00,
    0x00, 0x10,
    PB8, PB9,
    4, 4,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  },
  // motor5
  {
    0x0B,
    0x20, 0x00,
    0x00, 0x20,
    PB10, PB11,
    5, 5,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  },
  // motor6
  {
    0x0D,
    0x40, 0x00,
    0x00, 0x40,
    PB12, PB15,
    6, 6,
    0, 0, 0, 0,
    {0}, 0, 0.0, false, 0.0,
    0, 0, 0.0, 0.0,
    0, 0, 0,
    6.0
  }
};

// 74HC595 全局状态变量
uint16_t hc595_state = 0x0000;

// 电机2方向标志（由motorForward/motorReverse设置，ISR读取）
volatile int8_t motor2_dir = 0;  // 1=前进, -1=后退, 0=停止

// ---- 状态机定义 ----
enum MotionState {
  STATE_IDLE = 0,

  // 左翻身状态
  STATE_LEFT_TURN_RUNNING,
  STATE_LEFT_TURN_WAIT_STOP,

  // 右翻身状态
  STATE_RIGHT_TURN_RUNNING,

  // 平躺状态
  STATE_FLAT_MOTOR2_ONLY,
  STATE_FLAT_AFTER_TURN,
  STATE_FLAT_AFTER_RAISE,
  STATE_FLAT_WAIT_M1_LIMIT,

  // 起背状态
  STATE_RAISE_BACK_RUNNING,
  STATE_RAISE_BACK_RETURN_RUNNING,

  // 抬腿状态
  STATE_RAISE_LEG_RUNNING,
  STATE_RAISE_LEG_RETURN_RUNNING,
  STATE_RAISE_LEG_ZERO_RUNNING,

  // 坐立状态
  STATE_SIT_RUNNING,
  STATE_SIT_WAIT_COMPLETE,

  // 如厕状态
  STATE_TOILET_STAGE1,
  STATE_TOILET_STAGE2,
  STATE_TOILET_STAGE3,
  STATE_TOILET_WAIT_COMPLETE,

  // 结束如厕状态
  STATE_END_TOILET_STAGE1,
  STATE_END_TOILET_STAGE2,
  STATE_END_TOILET_STAGE3
};

// 状态机全局变量
MotionState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long lastPosChangeTime = 0;
int16_t lastMonitoredPos = 0;

// 状态机运行标志
struct StateFlags {
  bool motor0_done;
  bool motor1_done;
  bool motor2_done;
  bool motor3_done;
  bool motor4_done;
  bool motor5_done;
  bool motor6_done;
  bool motor0_forward;
  bool motor2_forward;
  bool motor3_forward;
  bool motor4_forward;
  bool motor5_forward;
  bool motor6_forward;
  int16_t motor1_lastPos;
  unsigned long motor1_lastPosChange;
  int16_t motor3_lastPos;
  unsigned long motor3_lastPosChange;
  int16_t motor4_lastPos;
  unsigned long motor4_lastPosChange;
  int16_t motor5_lastPos;
  unsigned long motor5_lastPosChange;
  int16_t motor6_lastPos;
  unsigned long motor6_lastPosChange;
};

StateFlags stateFlags;

// ---- 电机控制命令定义 ----
#define CMD_STOP    0x00
#define CMD_FORWARD 0x01
#define CMD_REVERSE 0x02
#define CMD_RESET_POS 0x07
#define CMD_LEFT_TURN 0x08

// ---- 8字节指令集定义 ----
// 电机0指令
const byte cmdMotor0Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x10, 0x00};
const byte cmdMotor0Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x10, 0x01};
const byte cmdMotor0Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x10, 0x02};
const byte cmdResetMotor0[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x07};

// 电机1指令
const byte cmdMotor1Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x00};
const byte cmdMotor1Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x01};
const byte cmdMotor1Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x02};
const byte cmdResetMotor1[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x17};

// 电机2指令
const byte cmdMotor2Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x12, 0x00};
const byte cmdMotor2Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x12, 0x01};
const byte cmdMotor2Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x12, 0x02};
const byte cmdResetMotor2[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x27};

// 电机3指令
const byte cmdMotor3Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x13, 0x00};
const byte cmdMotor3Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x13, 0x01};
const byte cmdMotor3Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x13, 0x02};
const byte cmdResetMotor3[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x37};

// 电机4指令
const byte cmdMotor4Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x14, 0x00};
const byte cmdMotor4Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x14, 0x01};
const byte cmdMotor4Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x14, 0x02};
const byte cmdResetMotor4[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x47};

// 电机5指令
const byte cmdMotor5Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x15, 0x00};
const byte cmdMotor5Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x15, 0x01};
const byte cmdMotor5Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x15, 0x02};
const byte cmdResetMotor5[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x57};

// 电机6指令
const byte cmdMotor6Stop[8]    = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x16, 0x00};
const byte cmdMotor6Forward[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x16, 0x01};
const byte cmdMotor6Reverse[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x16, 0x02};
const byte cmdResetMotor6[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x11, 0x67};

// 复合动作指令
const byte cmdLeftTurn[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x10};
const byte cmdRightTurn[8]  = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x11};
const byte cmdFlat[8]       = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x12};
const byte cmdRaiseBack[8]  = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x13};
const byte cmdRaiseLeg[8]   = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x14};
const byte cmdSit[8]        = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x15};
const byte cmdToilet[8]     = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x16};
const byte cmdEndToilet[8]  = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x17};
const byte cmdRaiseBackReturn[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x23};
const byte cmdRaiseLegReturn[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x24};
const byte cmdRaiseLegZero[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0x08, 0x34};
const byte cmdEmergencyStop[8] = {0xEE, 0x02, 0x11, 0x00, 0x11, 0x00, 0xFF, 0xFF};

// ---- 串口中断接收缓冲区 ----
volatile byte cmdBuffer[8];           // 8字节命令缓冲区（中断写入）
volatile int cmdIndex = 0;            // 当前接收索引
volatile bool cmdReady = false;       // 命令接收完成标志
volatile unsigned long lastRxTime = 0; // 上次接收时间（用于超时检测）

// ---- 外设对象 ----
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_RESET   -1
const uint8_t OLED_ADDR = 0x3C;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 600
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ---- 全局变量 ----
unsigned long lastOLEDUpdate = 0;
uint32_t last_ms = 0;

const uint8_t SPEED_PERCENT = 100;

// ================= AT24C02 存储逻辑 =================

// 将 16 位 position 写入 EEPROM (2字节补码)
void savePositionToEEPROM(uint8_t motorIndex, int16_t pos) {
  if (motorIndex >= 7) return;

  uint8_t data[2];
  data[0] = (pos >> 8) & 0xFF;  // 高字节
  data[1] = pos & 0xFF;          // 低字节

  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(motors[motorIndex].eepromAddr);
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.endTransmission();
  delay(5); // AT24C02 写周期延时（必须！）
  Serial.print("Auto-Save Motor");
  Serial.print(motorIndex);
  Serial.print(" Position: ");
  Serial.println(pos);
}

// 从 EEPROM 读取 16 位 position (2字节补码)
int16_t readPositionFromEEPROM(uint8_t motorIndex) {
  if (motorIndex >= 7) return 0;

  uint8_t data[2] = {0, 0};
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(motors[motorIndex].eepromAddr);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDR, (uint8_t)2);
  for (int i = 0; i < 2 && Wire.available(); i++) {
    data[i] = Wire.read();
  }

  return ((int16_t)data[0] << 8) | (int16_t)data[1];
}

// 初始化 EEPROM（首次上电检测）
void initEEPROM() {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(ADDR_MAGIC_NUMBER);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDR, (uint8_t)1);
  uint8_t magic = 0;
  if (Wire.available()) {
    magic = Wire.read();
  }

  if (magic != MAGIC_NUMBER) {
    // 首次上电：写入 MagicNumber 和初始化所有位置为 0
    Serial.println("First boot detected. Initializing EEPROM...");

    // 写入 MagicNumber
    Wire.beginTransmission(EEPROM_ADDR);
    Wire.write(ADDR_MAGIC_NUMBER);
    Wire.write(MAGIC_NUMBER);
    Wire.endTransmission();
    delay(5);

    // 初始化 7 个电机位置为 0
    uint8_t motorAddrs[7] = {ADDR_MOTOR1_POS, ADDR_MOTOR2_POS, ADDR_MOTOR3_POS,
                              ADDR_MOTOR4_POS, ADDR_MOTOR5_POS, ADDR_MOTOR6_POS,
                              ADDR_MOTOR7_POS};
    for (int i = 0; i < 7; i++) {
      Wire.beginTransmission(EEPROM_ADDR);
      Wire.write(motorAddrs[i]);
      Wire.write(0x00);  // 高字节
      Wire.write(0x00);  // 低字节
      Wire.endTransmission();
      delay(5);
    }

    Serial.println("EEPROM initialized successfully.");
  } else {
    Serial.println("EEPROM already initialized.");
  }
}

// ================= 硬件控制基础函数 =================

inline void pulseSRCLK() { digitalWrite(PIN_SRCLK, HIGH); digitalWrite(PIN_SRCLK, LOW); }
inline void pulseRCLK()  { digitalWrite(PIN_RCLK,  HIGH); digitalWrite(PIN_RCLK,  LOW); }

void shiftOut16(uint8_t highByte, uint8_t lowByte) {
  digitalWrite(PIN_OE, LOW);
  for (int8_t i = 7; i >= 0; --i) { digitalWrite(PIN_SER, (highByte >> i) & 0x01); pulseSRCLK(); }
  for (int8_t i = 7; i >= 0; --i) { digitalWrite(PIN_SER, (lowByte  >> i) & 0x01); pulseSRCLK(); }
  pulseRCLK();
}

// 电机前进：只设置该电机的前进位，清除后退位
void motorForward(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  uint16_t forwardMask = ((uint16_t)motors[motorIndex].forwardMaskHigh << 8) | motors[motorIndex].forwardMaskLow;
  uint16_t reverseMask = ((uint16_t)motors[motorIndex].reverseMaskHigh << 8) | motors[motorIndex].reverseMaskLow;

  hc595_state &= ~reverseMask;  // 清除后退位
  hc595_state |= forwardMask;   // 设置前进位

  if (motorIndex == 2) motor2_dir = 1;  // 设置电机2方向标志

  shiftOut16((hc595_state >> 8) & 0xFF, hc595_state & 0xFF);
}

// 电机后退：只设置该电机的后退位，清除前进位
void motorReverse(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  uint16_t forwardMask = ((uint16_t)motors[motorIndex].forwardMaskHigh << 8) | motors[motorIndex].forwardMaskLow;
  uint16_t reverseMask = ((uint16_t)motors[motorIndex].reverseMaskHigh << 8) | motors[motorIndex].reverseMaskLow;

  hc595_state &= ~forwardMask;  // 清除前进位
  hc595_state |= reverseMask;   // 设置后退位

  if (motorIndex == 2) motor2_dir = -1;  // 设置电机2方向标志

  shiftOut16((hc595_state >> 8) & 0xFF, hc595_state & 0xFF);
}

// 电机停止：清除该电机的前进和后退位
void motorStop(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  uint16_t forwardMask = ((uint16_t)motors[motorIndex].forwardMaskHigh << 8) | motors[motorIndex].forwardMaskLow;
  uint16_t reverseMask = ((uint16_t)motors[motorIndex].reverseMaskHigh << 8) | motors[motorIndex].reverseMaskLow;

  hc595_state &= ~forwardMask;  // 清除前进位
  hc595_state &= ~reverseMask;  // 清除后退位

  if (motorIndex == 2) motor2_dir = 0;  // 清除电机2方向标志

  shiftOut16((hc595_state >> 8) & 0xFF, hc595_state & 0xFF);
}

inline uint8_t pctToDuty(uint8_t p){ return (uint8_t)((p * 255UL) / 100UL); }

void setPCA9685PWM(uint8_t channel, uint8_t dutyCycle) {
  if (channel > 15) return;
  uint16_t pwmValue = (uint16_t)((dutyCycle * 4095UL) / 255UL);
  pca9685.setPWM(channel, 0, pwmValue);
}

void select4051Channel(uint8_t channel) {
  digitalWrite(PIN_4051_S0, channel & 0x01);
  digitalWrite(PIN_4051_S1, (channel >> 1) & 0x01);
  digitalWrite(PIN_4051_S2, (channel >> 2) & 0x01);
}

// 更新指定电机的电流读数（滑动平均滤波）
void updateCurrentReading(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  // 1. 切换到该电机的ADC通道
  select4051Channel(motors[motorIndex].adcChannel);
  delayMicroseconds(50); // 增加延迟，等待多路复用器稳定

  // 2. 读取多次ADC并取平均（减少噪声）
  int adc_sum = 0;
  for (int i = 0; i < 3; i++) {
    adc_sum += analogRead(PIN_ADC_CURRENT);
    delayMicroseconds(10);
  }
  int adc_value = adc_sum / 3;

  float voltage = (adc_value / 4095.0) * 3.3;
  float new_amp = voltage / SHUNT_RESISTANCE;

  // 3. 递推平均算法
  motors[motorIndex].sum_current -= motors[motorIndex].current_buffer[motors[motorIndex].buffer_index];
  motors[motorIndex].current_buffer[motors[motorIndex].buffer_index] = new_amp;
  motors[motorIndex].sum_current += new_amp;

  // 4. 更新索引
  motors[motorIndex].buffer_index++;
  if (motors[motorIndex].buffer_index >= FILTER_N) {
    motors[motorIndex].buffer_index = 0;
    motors[motorIndex].buffer_filled = true;
  }

  // 5. 计算平均值
  if (motors[motorIndex].buffer_filled) {
    motors[motorIndex].current_amps = motors[motorIndex].sum_current / FILTER_N;
  } else {
    motors[motorIndex].current_amps = motors[motorIndex].sum_current / motors[motorIndex].buffer_index;
  }
}

// ================= 状态机辅助函数 =================

// 紧急停止所有电机
void emergencyStopAll() {
  Serial.println("!!! EMERGENCY STOP ALL MOTORS !!!");

  // 停止所有电机
  for (int i = 0; i < 7; i++) {
    motorStop(i);
    setPCA9685PWM(motors[i].pwmChannel, 0);
  }

  // 清除所有74HC595输出
  hc595_state = 0x0000;
  shiftOut16(0x00, 0x00);

  // 重置状态机
  currentState = STATE_IDLE;

  // 清除所有状态标志（防止残留影响后续执行）
  stateFlags.motor0_done = false;
  stateFlags.motor1_done = false;
  stateFlags.motor2_done = false;
  stateFlags.motor3_done = false;
  stateFlags.motor4_done = false;
  stateFlags.motor5_done = false;
  stateFlags.motor6_done = false;
  stateFlags.motor0_forward = false;
  stateFlags.motor2_forward = false;
  stateFlags.motor3_forward = false;
  stateFlags.motor4_forward = false;
  stateFlags.motor5_forward = false;
  stateFlags.motor6_forward = false;
  stateFlags.motor1_lastPos = 0;
  stateFlags.motor1_lastPosChange = 0;
  stateFlags.motor3_lastPos = 0;
  stateFlags.motor3_lastPosChange = 0;
  stateFlags.motor4_lastPos = 0;
  stateFlags.motor4_lastPosChange = 0;
  stateFlags.motor5_lastPos = 0;
  stateFlags.motor5_lastPosChange = 0;
  stateFlags.motor6_lastPos = 0;
  stateFlags.motor6_lastPosChange = 0;

  // 立即保存所有电机位置（防止断电丢失）
  Serial.println("=> Saving all motor positions to EEPROM...");
  for (int i = 0; i < 7; i++) {
    if (motors[i].position != motors[i].lastSavedPosition) {
      savePositionToEEPROM(i, motors[i].position);
      motors[i].lastSavedPosition = motors[i].position;
    }
  }
  Serial.println("=> All positions saved successfully");
}

// 读取电机位置（原子操作）
int16_t getMotorPosition(uint8_t motorIndex) {
  int16_t pos;
  noInterrupts();
  pos = motors[motorIndex].position;
  interrupts();
  return pos;
}

// 启动电机到目标位置（返回false表示已在目标位置）
bool startMotorToPosition(uint8_t motorIndex, int16_t targetPos, bool *forwardFlag) {
  int16_t currentPos = getMotorPosition(motorIndex);

  // 使用容差范围判断（±10单位）
  if (abs(currentPos - targetPos) <= 10) {
    return false; // 已在目标位置（容差范围内）
  }

  if (currentPos < targetPos) {
    motorForward(motorIndex);
    *forwardFlag = true;
  } else {
    motorReverse(motorIndex);
    *forwardFlag = false;
  }
  setPCA9685PWM(motors[motorIndex].pwmChannel, pctToDuty(SPEED_PERCENT));
  return true;
}

// 检查电机是否到达目标位置
bool checkMotorReachedTarget(uint8_t motorIndex, int16_t targetPos, bool forward) {
  int16_t currentPos = getMotorPosition(motorIndex);

  // 使用容差范围判断（±10单位）
  if (abs(currentPos - targetPos) <= 10) {
    return true;
  }

  // 原有的方向检查（防止越过目标）
  if (forward && currentPos >= targetPos) {
    return true;
  } else if (!forward && currentPos <= targetPos) {
    return true;
  }
  return false;
}

// 检查电机是否到达极限位置（2秒内位置不变）
bool checkMotorReachedLimit(uint8_t motorIndex, unsigned long *lastChangeTime, int16_t *lastPos) {
  int16_t currentPos = getMotorPosition(motorIndex);

  if (currentPos != *lastPos) {
    *lastPos = currentPos;
    *lastChangeTime = millis();
    return false;
  }

  if (millis() - *lastChangeTime >= 2000) {
    return true;
  }
  return false;
}

// ================= 中断处理 =================

// 4倍频正交解码中断处理函数（通用）
// Motor0 (PA0, PA1)
void ISR_motor0_AB() {
  uint8_t A = digitalRead(motors[0].hallPinA);
  uint8_t B = digitalRead(motors[0].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[0].lastState;

  // 状态转换表判断方向（格雷码序列）
  // 正转: 00 -> 01 -> 11 -> 10 -> 00
  // 反转: 00 -> 10 -> 11 -> 01 -> 00
  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[0].position--;  // 正转（根据实际情况可能需要调整方向）
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[0].position++;  // 反转
  }
  // 其他情况：非法转换或噪声，忽略

  motors[0].lastState = currentState;

  // 更新计数（用于频率统计）
  if (A != (lastState >> 1)) motors[0].hallA_count++;
  if (B != (lastState & 0x01)) motors[0].hallB_count++;
}

// Motor1 (PA2, PA3)
void ISR_motor1_AB() {
  uint8_t A = digitalRead(motors[1].hallPinA);
  uint8_t B = digitalRead(motors[1].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[1].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[1].position--;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[1].position++;
  }

  motors[1].lastState = currentState;
  if (A != (lastState >> 1)) motors[1].hallA_count++;
  if (B != (lastState & 0x01)) motors[1].hallB_count++;
}

// Motor2 (PA4, PA5) - 用驱动方向标志计数，A相和B相CHANGE触发（4倍频）
void ISR_motor2_A() {
  if (motor2_dir == 1) {
    motors[2].position++;
  } else if (motor2_dir == -1) {
    motors[2].position--;
  }
  motors[2].hallA_count++;
}

void ISR_motor2_B() {
  if (motor2_dir == 1) {
    motors[2].position++;
  } else if (motor2_dir == -1) {
    motors[2].position--;
  }
  motors[2].hallB_count++;
}

// Motor3 (PA6, PA7)
void ISR_motor3_AB() {
  uint8_t A = digitalRead(motors[3].hallPinA);
  uint8_t B = digitalRead(motors[3].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[3].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[3].position--;  // 方向已反转：伸出递增
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[3].position++;  // 方向已反转：缩回递减
  }

  motors[3].lastState = currentState;
  if (A != (lastState >> 1)) motors[3].hallA_count++;
  if (B != (lastState & 0x01)) motors[3].hallB_count++;
}

// Motor4 (PB8, PB9)
void ISR_motor4_AB() {
  uint8_t A = digitalRead(motors[4].hallPinA);
  uint8_t B = digitalRead(motors[4].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[4].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[4].position--;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[4].position++;
  }

  motors[4].lastState = currentState;
  if (A != (lastState >> 1)) motors[4].hallA_count++;
  if (B != (lastState & 0x01)) motors[4].hallB_count++;
}

// Motor5 (PB10, PB11)
void ISR_motor5_AB() {
  uint8_t A = digitalRead(motors[5].hallPinA);
  uint8_t B = digitalRead(motors[5].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[5].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[5].position--;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[5].position++;
  }

  motors[5].lastState = currentState;
  if (A != (lastState >> 1)) motors[5].hallA_count++;
  if (B != (lastState & 0x01)) motors[5].hallB_count++;
}

// Motor6 (PB12, PB15)
void ISR_motor6_AB() {
  uint8_t A = digitalRead(motors[6].hallPinA);
  uint8_t B = digitalRead(motors[6].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[6].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[6].position--;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[6].position++;
  }

  motors[6].lastState = currentState;
  if (A != (lastState >> 1)) motors[6].hallA_count++;
  if (B != (lastState & 0x01)) motors[6].hallB_count++;
}

// ================= 状态机更新函数 =================

void updateStateMachine() {
  if (currentState == STATE_IDLE) {
    return; // 空闲状态，无需处理
  }

  // 左翻身状态机
  if (currentState == STATE_LEFT_TURN_RUNNING) {
    updateCurrentReading(0);

    // 检查是否到达极限位置
    if (checkMotorReachedLimit(0, &lastPosChangeTime, &lastMonitoredPos)) {
      motorStop(0);
      setPCA9685PWM(motors[0].pwmChannel, 0);

      // 清零位置并保存
      noInterrupts();
      motors[0].position = 0;
      motors[0].hallA_count = 0;
      motors[0].hallB_count = 0;
      interrupts();
      savePositionToEEPROM(0, 0);
      motors[0].lastSavedPosition = 0;
      motors[0].lastLoopPosition = 0;
      motors[0].last_hallA_cnt = 0;
      motors[0].last_hallB_cnt = 0;

      Serial.println("=> LEFT TURN completed, Motor0 position reset to 0");
      currentState = STATE_IDLE;
    }
  }

  // 右翻身状态机
  else if (currentState == STATE_RIGHT_TURN_RUNNING) {
    updateCurrentReading(0);

    if (checkMotorReachedTarget(0, 6400, true)) {
      motorStop(0);
      setPCA9685PWM(motors[0].pwmChannel, 0);
      Serial.println("=> RIGHT TURN completed, Motor0 reached position 6400");
      currentState = STATE_IDLE;
    }
  }

  // 平躺 - 只移动电机2
  else if (currentState == STATE_FLAT_MOTOR2_ONLY) {
    updateCurrentReading(2);

    if (checkMotorReachedTarget(2, 5000, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      Serial.println("=> Motor2 reached position 5000");
      Serial.println("=> FLAT completed");
      currentState = STATE_IDLE;
    }
  }

  // 平躺 - 翻身后归位（移动电机0和2）
  else if (currentState == STATE_FLAT_AFTER_TURN) {
    if (!stateFlags.motor0_done) {
      updateCurrentReading(0);
      if (checkMotorReachedTarget(0, 4500, stateFlags.motor0_forward)) {
        motorStop(0);
        setPCA9685PWM(motors[0].pwmChannel, 0);
        stateFlags.motor0_done = true;
        Serial.println("=> Motor0 reached position 4500");
      }
    }

    if (!stateFlags.motor2_done) {
      updateCurrentReading(2);
      if (checkMotorReachedTarget(2, 5000, stateFlags.motor2_forward)) {
        motorStop(2);
        setPCA9685PWM(motors[2].pwmChannel, 0);
        stateFlags.motor2_done = true;
        Serial.println("=> Motor2 reached position 5000");
      }
    }

    if (stateFlags.motor0_done && stateFlags.motor2_done) {
      Serial.println("=> FLAT completed");
      currentState = STATE_IDLE;
    }
  }

  // 平躺 - 起背后归位（移动电机1到极限和电机2）
  else if (currentState == STATE_FLAT_AFTER_RAISE) {
    if (!stateFlags.motor1_done) {
      updateCurrentReading(1);
      if (checkMotorReachedLimit(1, &stateFlags.motor1_lastPosChange, &stateFlags.motor1_lastPos)) {
        motorStop(1);
        setPCA9685PWM(motors[1].pwmChannel, 0);
        stateFlags.motor1_done = true;
        Serial.println("=> Motor1 reached limit position");
      }
    }

    if (!stateFlags.motor2_done) {
      updateCurrentReading(2);
      if (checkMotorReachedTarget(2, 5000, stateFlags.motor2_forward)) {
        motorStop(2);
        setPCA9685PWM(motors[2].pwmChannel, 0);
        stateFlags.motor2_done = true;
        Serial.println("=> Motor2 reached position 5000");
      }
    }

    if (stateFlags.motor1_done && stateFlags.motor2_done) {
      Serial.println("=> FLAT completed");
      currentState = STATE_IDLE;
    }
  }

  // 起背状态机
  else if (currentState == STATE_RAISE_BACK_RUNNING) {
    updateCurrentReading(1);

    // 检查是否到达极限位置（0.5秒内位置不变）
    int16_t currentPos = getMotorPosition(1);
    if (currentPos != lastMonitoredPos) {
      lastMonitoredPos = currentPos;
      lastPosChangeTime = millis();
    }

    if (millis() - lastPosChangeTime >= 500) {  // 0.5秒
      motorStop(1);
      setPCA9685PWM(motors[1].pwmChannel, 0);
      Serial.println("=> RAISE BACK completed, Motor1 reached limit position");
      currentState = STATE_IDLE;
    }
  }

  // 起背回归状态机
  else if (currentState == STATE_RAISE_BACK_RETURN_RUNNING) {
    updateCurrentReading(1);

    // 检查是否到达极限位置（0.5秒内位置不变）
    int16_t currentPos = getMotorPosition(1);
    if (currentPos != lastMonitoredPos) {
      lastMonitoredPos = currentPos;
      lastPosChangeTime = millis();
    }

    if (millis() - lastPosChangeTime >= 500) {  // 0.5秒
      motorStop(1);
      setPCA9685PWM(motors[1].pwmChannel, 0);
      Serial.println("=> RAISE BACK RETURN completed, Motor1 reached limit position");
      currentState = STATE_IDLE;
    }
  }

  // 抬腿状态机
  else if (currentState == STATE_RAISE_LEG_RUNNING) {
    updateCurrentReading(2);

    if (checkMotorReachedTarget(2, 6700, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      Serial.print("=> Motor2 reached position: ");
      Serial.println(getMotorPosition(2));
      Serial.println("=> RAISE LEG completed");
      currentState = STATE_IDLE;
    }
  }

  // 抬腿回归状态机
  else if (currentState == STATE_RAISE_LEG_RETURN_RUNNING) {
    updateCurrentReading(2);

    if (checkMotorReachedTarget(2, 5000, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      Serial.print("=> Motor2 reached position: ");
      Serial.println(getMotorPosition(2));
      Serial.println("=> RAISE LEG RETURN completed");
      currentState = STATE_IDLE;
    }
  }

  // 抬腿归零状态机
  else if (currentState == STATE_RAISE_LEG_ZERO_RUNNING) {
    updateCurrentReading(2);

    if (checkMotorReachedTarget(2, 0, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      Serial.print("=> Motor2 reached position: ");
      Serial.println(getMotorPosition(2));
      Serial.println("=> RAISE LEG ZERO completed");
      currentState = STATE_IDLE;
    }
  }

  // 坐立状态机
  else if (currentState == STATE_SIT_RUNNING) {
    if (!stateFlags.motor1_done) {
      updateCurrentReading(1);
      if (checkMotorReachedLimit(1, &stateFlags.motor1_lastPosChange, &stateFlags.motor1_lastPos)) {
        motorStop(1);
        setPCA9685PWM(motors[1].pwmChannel, 0);
        stateFlags.motor1_done = true;
        Serial.println("=> Motor1 reached limit position");
      }
    }

    if (!stateFlags.motor2_done) {
      updateCurrentReading(2);
      if (checkMotorReachedTarget(2, 0, stateFlags.motor2_forward)) {
        motorStop(2);
        setPCA9685PWM(motors[2].pwmChannel, 0);
        stateFlags.motor2_done = true;
        Serial.println("=> Motor2 reached position 0");
      }
    }

    if (stateFlags.motor1_done && stateFlags.motor2_done) {
      Serial.println("=> RAISE BACK + LEG ZERO completed");
      currentState = STATE_IDLE;
    }
  }

  // 如厕状态机
  else if (currentState == STATE_TOILET_STAGE1) {
    // 更新电流读数
    if (!stateFlags.motor4_done) updateCurrentReading(4);
    if (stateFlags.motor1_done == false) updateCurrentReading(1);  // 坐起：M1
    if (stateFlags.motor2_done == false) updateCurrentReading(2);  // 坐起：M2

    // 检查M4进度（两个条件：位置不变 + 位置在范围内）
    if (!stateFlags.motor4_done) {
      int16_t pos4 = getMotorPosition(4);
      
      // 条件1：检查位置是否变化
      if (pos4 != stateFlags.motor4_lastPos) {
        stateFlags.motor4_lastPos = pos4;
        stateFlags.motor4_lastPosChange = millis();
      }
      
      // 条件2：位置0.5秒内不变 且 在[-200, 200]范围内
      if ((millis() - stateFlags.motor4_lastPosChange >= 500) && (abs(pos4) <= 200)) {
        motorStop(4);
        setPCA9685PWM(motors[4].pwmChannel, 0);
        stateFlags.motor4_done = true;
        Serial.println("=> Motor4 reached position 0 / Stage 1 completed");
      }
    }

    // M4完成后启动阶段2（无论是刚到达还是本来就在位置）
    if (stateFlags.motor4_done) {
      // 启动阶段2: M3→0
      currentState = STATE_TOILET_STAGE2;
      Serial.println("=> Stage 2: Moving Motor3 to position 0...");
      stateFlags.motor3_done = !startMotorToPosition(3, 0, &stateFlags.motor3_forward);
      if (stateFlags.motor3_done) Serial.println("=> Motor3 already at position 0");
      return; // 重要：立即返回，下次循环处理STAGE2
    }

    // 检查坐起进度（并行执行）
    if (!stateFlags.motor1_done && checkMotorReachedLimit(1, &stateFlags.motor1_lastPosChange, &stateFlags.motor1_lastPos)) {
      motorStop(1);
      setPCA9685PWM(motors[1].pwmChannel, 0);
      stateFlags.motor1_done = true;
      Serial.println("=> Motor1 reached limit position (SIT)");
    }
    if (!stateFlags.motor2_done && checkMotorReachedTarget(2, 0, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      stateFlags.motor2_done = true;
      Serial.println("=> Motor2 reached position 0 (SIT)");
    }
  }

  else if (currentState == STATE_TOILET_STAGE2) {
    if (!stateFlags.motor3_done) updateCurrentReading(3);
    if (!stateFlags.motor1_done) updateCurrentReading(1);
    if (!stateFlags.motor2_done) updateCurrentReading(2);

    // 检查M3进度（两个条件：位置不变 + 位置在范围内）
    if (!stateFlags.motor3_done) {
      int16_t pos3 = getMotorPosition(3);
      
      // 条件1：检查位置是否变化
      if (pos3 != stateFlags.motor3_lastPos) {
        stateFlags.motor3_lastPos = pos3;
        stateFlags.motor3_lastPosChange = millis();
      }
      
      // 条件2：位置0.5秒内不变 且 在[-200, 200]范围内
      if ((millis() - stateFlags.motor3_lastPosChange >= 500) && (abs(pos3) <= 200)) {
        motorStop(3);
        setPCA9685PWM(motors[3].pwmChannel, 0);
        stateFlags.motor3_done = true;
        Serial.println("=> Motor3 reached position 0 / Stage 2 completed");
      }
    }

    // M3完成后启动阶段3
    if (stateFlags.motor3_done) {
      // 启动阶段3: M5+M6→4500（同步）
      currentState = STATE_TOILET_STAGE3;
      Serial.println("=> Stage 3: Moving Motor5 and Motor6 to position 4500...");
      stateFlags.motor5_done = !startMotorToPosition(5, 4500, &stateFlags.motor5_forward);
      stateFlags.motor6_done = !startMotorToPosition(6, 4500, &stateFlags.motor6_forward);
      if (stateFlags.motor5_done) Serial.println("=> Motor5 already at position 4500");
      if (stateFlags.motor6_done) Serial.println("=> Motor6 already at position 4500");
      return; // 重要：立即返回，下次循环处理STAGE3
    }

    // 检查坐起进度
    if (!stateFlags.motor1_done && checkMotorReachedLimit(1, &stateFlags.motor1_lastPosChange, &stateFlags.motor1_lastPos)) {
      motorStop(1);
      setPCA9685PWM(motors[1].pwmChannel, 0);
      stateFlags.motor1_done = true;
      Serial.println("=> Motor1 reached limit position (SIT)");
    }
    if (!stateFlags.motor2_done && checkMotorReachedTarget(2, 0, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      stateFlags.motor2_done = true;
      Serial.println("=> Motor2 reached position 0 (SIT)");
    }
  }

  else if (currentState == STATE_TOILET_STAGE3) {
    if (!stateFlags.motor5_done) updateCurrentReading(5);
    if (!stateFlags.motor6_done) updateCurrentReading(6);
    if (!stateFlags.motor1_done) updateCurrentReading(1);
    if (!stateFlags.motor2_done) updateCurrentReading(2);

    // 检查M5进度
    if (!stateFlags.motor5_done) {
      if (checkMotorReachedTarget(5, 4500, stateFlags.motor5_forward)) {
        motorStop(5);
        setPCA9685PWM(motors[5].pwmChannel, 0);
        stateFlags.motor5_done = true;
        Serial.println("=> Motor5 reached position 4500");
      }
    }

    // 检查M6进度
    if (!stateFlags.motor6_done) {
      if (checkMotorReachedTarget(6, 4500, stateFlags.motor6_forward)) {
        motorStop(6);
        setPCA9685PWM(motors[6].pwmChannel, 0);
        stateFlags.motor6_done = true;
        Serial.println("=> Motor6 reached position 4500");
      }
    }

    // 检查坐起进度
    if (!stateFlags.motor1_done && checkMotorReachedLimit(1, &stateFlags.motor1_lastPosChange, &stateFlags.motor1_lastPos)) {
      motorStop(1);
      setPCA9685PWM(motors[1].pwmChannel, 0);
      stateFlags.motor1_done = true;
      Serial.println("=> Motor1 reached limit position (SIT)");
    }
    if (!stateFlags.motor2_done && checkMotorReachedTarget(2, 0, stateFlags.motor2_forward)) {
      motorStop(2);
      setPCA9685PWM(motors[2].pwmChannel, 0);
      stateFlags.motor2_done = true;
      Serial.println("=> Motor2 reached position 0 (SIT)");
    }

    // 检查如厕主流程是否完成（只检查M5和M6）
    if (stateFlags.motor5_done && stateFlags.motor6_done) {
      Serial.println("=> TOILET sequence completed successfully");
      currentState = STATE_IDLE;
    }
  }

  // 结束如厕状态机
  else if (currentState == STATE_END_TOILET_STAGE1) {
    if (!stateFlags.motor5_done) updateCurrentReading(5);
    if (!stateFlags.motor6_done) updateCurrentReading(6);

    // 检查M5进度（两个条件：位置不变 + 位置在范围内）
    if (!stateFlags.motor5_done) {
      int16_t pos5 = getMotorPosition(5);
      
      // 条件1：检查位置是否变化
      if (pos5 != stateFlags.motor5_lastPos) {
        stateFlags.motor5_lastPos = pos5;
        stateFlags.motor5_lastPosChange = millis();
      }
      
      // 条件2：位置0.5秒内不变 且 在[-200, 200]范围内
      if ((millis() - stateFlags.motor5_lastPosChange >= 500) && (abs(pos5) <= 200)) {
        motorStop(5);
        setPCA9685PWM(motors[5].pwmChannel, 0);
        stateFlags.motor5_done = true;
        Serial.println("=> Motor5 reached position 0");
      }
    }

    // 检查M6进度（两个条件：位置不变 + 位置在范围内）
    if (!stateFlags.motor6_done) {
      int16_t pos6 = getMotorPosition(6);
      
      // 条件1：检查位置是否变化
      if (pos6 != stateFlags.motor6_lastPos) {
        stateFlags.motor6_lastPos = pos6;
        stateFlags.motor6_lastPosChange = millis();
      }
      
      // 条件2：位置0.5秒内不变 且 在[-200, 200]范围内
      if ((millis() - stateFlags.motor6_lastPosChange >= 500) && (abs(pos6) <= 200)) {
        motorStop(6);
        setPCA9685PWM(motors[6].pwmChannel, 0);
        stateFlags.motor6_done = true;
        Serial.println("=> Motor6 reached position 0");
      }
    }

    // M5和M6都完成后启动阶段2
    if (stateFlags.motor5_done && stateFlags.motor6_done) {
      // 启动阶段2: M3→12500
      currentState = STATE_END_TOILET_STAGE2;
      Serial.println("=> Stage 1 completed");
      Serial.println("=> Stage 2: Moving Motor3 to position 12500...");
      stateFlags.motor3_done = !startMotorToPosition(3, 12500, &stateFlags.motor3_forward);
      if (stateFlags.motor3_done) Serial.println("=> Motor3 already at position 12500");
      return; // 重要：立即返回，下次循环处理STAGE2
    }
  }

  else if (currentState == STATE_END_TOILET_STAGE2) {
    if (!stateFlags.motor3_done) updateCurrentReading(3);

    // 检查M3进度
    if (!stateFlags.motor3_done) {
      if (checkMotorReachedTarget(3, 12500, stateFlags.motor3_forward)) {
        motorStop(3);
        setPCA9685PWM(motors[3].pwmChannel, 0);
        stateFlags.motor3_done = true;
        Serial.println("=> Motor3 reached position 12500");
      }
    }

    // M3完成后启动阶段3
    if (stateFlags.motor3_done) {
      // 启动阶段3: M4→7300
      currentState = STATE_END_TOILET_STAGE3;
      Serial.println("=> Stage 2 completed");
      Serial.println("=> Stage 3: Moving Motor4 to position 7300...");
      stateFlags.motor4_done = !startMotorToPosition(4, 7300, &stateFlags.motor4_forward);
      if (stateFlags.motor4_done) Serial.println("=> Motor4 already at position 7300");
      return; // 重要：立即返回，下次循环处理STAGE3
    }
  }

  else if (currentState == STATE_END_TOILET_STAGE3) {
    if (!stateFlags.motor4_done) updateCurrentReading(4);

    // 检查M4进度
    if (!stateFlags.motor4_done) {
      if (checkMotorReachedTarget(4, 7300, stateFlags.motor4_forward)) {
        motorStop(4);
        setPCA9685PWM(motors[4].pwmChannel, 0);
        stateFlags.motor4_done = true;
        Serial.println("=> Motor4 reached position 7300");
      }
    }

    // 检查是否全部完成
    if (stateFlags.motor4_done) {
      Serial.println("=> Stage 3 completed");
      Serial.println("=> END TOILET sequence completed successfully");
      currentState = STATE_IDLE;
    }
  }
}

// ================= 指令解析与执行 =================

void processCommand(byte cmd[8]) {
  // 检查帧头
  if (cmd[0] != 0xEE) return;

  // ---- 电机0指令 ----
  if (memcmp(cmd, cmdMotor0Stop, 8) == 0) {
    Serial.println("=> Motor0 STOP");
    motorStop(0);
    setPCA9685PWM(motors[0].pwmChannel, 0);
    if (currentState == STATE_LEFT_TURN_RUNNING ||
        currentState == STATE_RIGHT_TURN_RUNNING) {
      Serial.println("=> Turn state aborted by Motor0 STOP");
      currentState = STATE_IDLE;
    }
    return;
  }
  else if (memcmp(cmd, cmdMotor0Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_LEFT_TURN_RUNNING || currentState == STATE_RIGHT_TURN_RUNNING) {
      Serial.println("=> Turn state aborted by Motor0 FORWARD");
      currentState = STATE_IDLE;
    }
    int16_t pos1 = getMotorPosition(1);
    if (pos1 < 6500 || pos1 > 6900) {
      Serial.print("=> Motor0 FORWARD: Motor1 position out of range: ");
      Serial.print(pos1);
      Serial.println(" (required: 6500-6900), command ignored");
      return;
    }
    Serial.println("=> Motor0 FORWARD");
    motorForward(0);
    setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor0Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_LEFT_TURN_RUNNING || currentState == STATE_RIGHT_TURN_RUNNING) {
      Serial.println("=> Turn state aborted by Motor0 REVERSE");
      currentState = STATE_IDLE;
    }
    int16_t pos1 = getMotorPosition(1);
    if (pos1 < 6500 || pos1 > 6900) {
      Serial.print("=> Motor0 REVERSE: Motor1 position out of range: ");
      Serial.print(pos1);
      Serial.println(" (required: 6500-6900), command ignored");
      return;
    }
    Serial.println("=> Motor0 REVERSE");
    motorReverse(0);
    setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor0, 8) == 0) {
    Serial.println("=> Motor0 RESET POSITION TO 0");
    noInterrupts();
    motors[0].position = 0;
    motors[0].hallA_count = 0;
    motors[0].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(0, 0);
    motors[0].lastSavedPosition = 0;
    motors[0].lastLoopPosition = 0;
    motors[0].last_hallA_cnt = 0;
    motors[0].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机1指令 ----
  else if (memcmp(cmd, cmdMotor1Stop, 8) == 0) {
    Serial.println("=> Motor1 STOP");
    motorStop(1);
    setPCA9685PWM(motors[1].pwmChannel, 0);
    if (currentState == STATE_RAISE_BACK_RUNNING) {
      Serial.println("=> Raise back state aborted by Motor1 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_BACK_RETURN_RUNNING) {
      Serial.println("=> Raise back return state aborted by Motor1 STOP");
      currentState = STATE_IDLE;
    }
    return;
  }
  else if (memcmp(cmd, cmdMotor1Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_RAISE_BACK_RUNNING || currentState == STATE_RAISE_BACK_RETURN_RUNNING) {
      Serial.println("=> Raise back state aborted by Motor1 FORWARD");
      currentState = STATE_IDLE;
    }
    int16_t pos0 = getMotorPosition(0);
    if (pos0 < 4200 || pos0 > 4800) {
      Serial.print("=> Motor1 FORWARD: Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800), command ignored");
      return;
    }
    Serial.println("=> Motor1 FORWARD");
    motorForward(1);
    setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor1Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_RAISE_BACK_RUNNING || currentState == STATE_RAISE_BACK_RETURN_RUNNING) {
      Serial.println("=> Raise back state aborted by Motor1 REVERSE");
      currentState = STATE_IDLE;
    }
    int16_t pos0 = getMotorPosition(0);
    if (pos0 < 4200 || pos0 > 4800) {
      Serial.print("=> Motor1 REVERSE: Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800), command ignored");
      return;
    }
    Serial.println("=> Motor1 REVERSE");
    motorReverse(1);
    setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor1, 8) == 0) {
    Serial.println("=> Motor1 RESET POSITION TO 0");
    noInterrupts();
    motors[1].position = 0;
    motors[1].hallA_count = 0;
    motors[1].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(1, 0);
    motors[1].lastSavedPosition = 0;
    motors[1].lastLoopPosition = 0;
    motors[1].last_hallA_cnt = 0;
    motors[1].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机2指令 ----
  else if (memcmp(cmd, cmdMotor2Stop, 8) == 0) {
    Serial.println("=> Motor2 STOP");
    motorStop(2);
    setPCA9685PWM(motors[2].pwmChannel, 0);
    if (currentState == STATE_RAISE_LEG_RUNNING) {
      Serial.println("=> Raise leg state aborted by Motor2 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_RETURN_RUNNING) {
      Serial.println("=> Raise leg return state aborted by Motor2 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_ZERO_RUNNING) {
      Serial.println("=> Raise leg zero state aborted by Motor2 STOP");
      currentState = STATE_IDLE;
    }
    return;
  }
  else if (memcmp(cmd, cmdMotor2Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_RAISE_LEG_RUNNING) {
      Serial.println("=> Raise leg state aborted by Motor2 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_RETURN_RUNNING) {
      Serial.println("=> Raise leg return state aborted by Motor2 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_ZERO_RUNNING) {
      Serial.println("=> Raise leg zero state aborted by Motor2 FORWARD");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor2 FORWARD");
    motorForward(2);
    setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor2Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_RAISE_LEG_RUNNING) {
      Serial.println("=> Raise leg state aborted by Motor2 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_RETURN_RUNNING) {
      Serial.println("=> Raise leg return state aborted by Motor2 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_RAISE_LEG_ZERO_RUNNING) {
      Serial.println("=> Raise leg zero state aborted by Motor2 REVERSE");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor2 REVERSE");
    motorReverse(2);
    setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor2, 8) == 0) {
    Serial.println("=> Motor2 RESET POSITION TO 0");
    noInterrupts();
    motors[2].position = 0;
    motors[2].hallA_count = 0;
    motors[2].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(2, 0);
    motors[2].lastSavedPosition = 0;
    motors[2].lastLoopPosition = 0;
    motors[2].last_hallA_cnt = 0;
    motors[2].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机3指令 ----
  else if (memcmp(cmd, cmdMotor3Stop, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE2) {
      Serial.println("=> Toilet stage 2 aborted by Motor3 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE2) {
      Serial.println("=> End toilet stage 2 aborted by Motor3 STOP");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor3 STOP");
    motorStop(3);
    setPCA9685PWM(motors[3].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor3Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE2) {
      Serial.println("=> Toilet stage 2 aborted by Motor3 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE2) {
      Serial.println("=> End toilet stage 2 aborted by Motor3 FORWARD");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor3 FORWARD");
    motorForward(3);
    setPCA9685PWM(motors[3].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor3Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE2) {
      Serial.println("=> Toilet stage 2 aborted by Motor3 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE2) {
      Serial.println("=> End toilet stage 2 aborted by Motor3 REVERSE");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor3 REVERSE");
    motorReverse(3);
    setPCA9685PWM(motors[3].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor3, 8) == 0) {
    Serial.println("=> Motor3 RESET POSITION TO 0");
    noInterrupts();
    motors[3].position = 0;
    motors[3].hallA_count = 0;
    motors[3].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(3, 0);
    motors[3].lastSavedPosition = 0;
    motors[3].lastLoopPosition = 0;
    motors[3].last_hallA_cnt = 0;
    motors[3].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机4指令 ----
  else if (memcmp(cmd, cmdMotor4Stop, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE1) {
      Serial.println("=> Toilet stage 1 aborted by Motor4 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE3) {
      Serial.println("=> End toilet stage 3 aborted by Motor4 STOP");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor4 STOP");
    motorStop(4);
    setPCA9685PWM(motors[4].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor4Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE1) {
      Serial.println("=> Toilet stage 1 aborted by Motor4 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE3) {
      Serial.println("=> End toilet stage 3 aborted by Motor4 FORWARD");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor4 FORWARD");
    motorForward(4);
    setPCA9685PWM(motors[4].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor4Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE1) {
      Serial.println("=> Toilet stage 1 aborted by Motor4 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE3) {
      Serial.println("=> End toilet stage 3 aborted by Motor4 REVERSE");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor4 REVERSE");
    motorReverse(4);
    setPCA9685PWM(motors[4].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor4, 8) == 0) {
    Serial.println("=> Motor4 RESET POSITION TO 0");
    noInterrupts();
    motors[4].position = 0;
    motors[4].hallA_count = 0;
    motors[4].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(4, 0);
    motors[4].lastSavedPosition = 0;
    motors[4].lastLoopPosition = 0;
    motors[4].last_hallA_cnt = 0;
    motors[4].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机5指令（同步控制电机6）----
  else if (memcmp(cmd, cmdMotor5Stop, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor5+6 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor5+6 STOP");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor5+6 STOP (synchronized)");
    motorStop(5);
    motorStop(6);
    setPCA9685PWM(motors[5].pwmChannel, 0);
    setPCA9685PWM(motors[6].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor5Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor5+6 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor5+6 FORWARD");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor5+6 FORWARD (synchronized)");
    motorForward(5);
    motorForward(6);
    setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
    setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor5Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor5+6 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor5+6 REVERSE");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor5+6 REVERSE (synchronized)");
    motorReverse(5);
    motorReverse(6);
    setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
    setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor5, 8) == 0) {
    Serial.println("=> Motor5+6 RESET POSITION TO 0 (synchronized)");
    noInterrupts();
    motors[5].position = 0;
    motors[5].hallA_count = 0;
    motors[5].hallB_count = 0;
    motors[6].position = 0;
    motors[6].hallA_count = 0;
    motors[6].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(5, 0);
    savePositionToEEPROM(6, 0);
    motors[5].lastSavedPosition = 0;
    motors[5].lastLoopPosition = 0;
    motors[5].last_hallA_cnt = 0;
    motors[5].last_hallB_cnt = 0;
    motors[6].lastSavedPosition = 0;
    motors[6].lastLoopPosition = 0;
    motors[6].last_hallA_cnt = 0;
    motors[6].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机6指令 ----
  else if (memcmp(cmd, cmdMotor6Stop, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor6 STOP");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor6 STOP");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor6 STOP");
    motorStop(6);
    setPCA9685PWM(motors[6].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor6Forward, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor6 FORWARD");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor6 FORWARD");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor6 FORWARD");
    motorForward(6);
    setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor6Reverse, 8) == 0) {
    // 检查并中断相关状态机
    if (currentState == STATE_TOILET_STAGE3) {
      Serial.println("=> Toilet stage 3 aborted by Motor6 REVERSE");
      currentState = STATE_IDLE;
    }
    if (currentState == STATE_END_TOILET_STAGE1) {
      Serial.println("=> End toilet stage 1 aborted by Motor6 REVERSE");
      currentState = STATE_IDLE;
    }
    Serial.println("=> Motor6 REVERSE");
    motorReverse(6);
    setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor6, 8) == 0) {
    Serial.println("=> Motor6 RESET POSITION TO 0");
    noInterrupts();
    motors[6].position = 0;
    motors[6].hallA_count = 0;
    motors[6].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(6, 0);
    motors[6].lastSavedPosition = 0;
    motors[6].lastLoopPosition = 0;
    motors[6].last_hallA_cnt = 0;
    motors[6].last_hallB_cnt = 0;
    return;
  }

  // ---- 复合动作指令 ----

  // 紧急停止
  else if (memcmp(cmd, cmdEmergencyStop, 8) == 0) {
    emergencyStopAll();
    return;
  }

  // 左翻身
  else if (memcmp(cmd, cmdLeftTurn, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> LEFT TURN: State machine busy, command ignored");
      return;
    }

    Serial.println("=> LEFT TURN: Checking Motor1 position...");
    int16_t pos1 = getMotorPosition(1);

    // 检查电机1位置是否在6500-6900范围内
    if (pos1 < 6500 || pos1 > 6900) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(pos1);
      Serial.println(" (required: 6500-6900)");
      Serial.println("=> LEFT TURN: Command ignored");
      return;
    }

    Serial.print("=> Motor1 position OK (");
    Serial.print(pos1);
    Serial.println("), starting LEFT TURN...");

    // 启动电机0反转
    motorReverse(0);
    setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));

    // 初始化状态机
    currentState = STATE_LEFT_TURN_RUNNING;
    stateStartTime = millis();
    lastPosChangeTime = millis();
    lastMonitoredPos = getMotorPosition(0);

    return;
  }

  // 右翻身
  else if (memcmp(cmd, cmdRightTurn, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RIGHT TURN: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RIGHT TURN: Checking Motor1 position...");
    int16_t pos1 = getMotorPosition(1);

    // 检查电机1位置是否在6500-6900范围内
    if (pos1 < 6500 || pos1 > 6900) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(pos1);
      Serial.println(" (required: 6500-6900)");
      Serial.println("=> RIGHT TURN: Command ignored");
      return;
    }

    Serial.print("=> Motor1 position OK (");
    Serial.print(pos1);
    Serial.println("), starting RIGHT TURN...");

    int16_t pos0 = getMotorPosition(0);
    if (pos0 < 6400) {
      motorForward(0);
      setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));

      currentState = STATE_RIGHT_TURN_RUNNING;
      stateStartTime = millis();
    } else {
      Serial.println("=> Motor0 already at or beyond position 6400");
    }
    return;
  }

  // 平躺
  else if (memcmp(cmd, cmdFlat, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> FLAT: State machine busy, command ignored");
      return;
    }

    Serial.println("=> FLAT: Checking bed state...");
    int16_t pos0 = getMotorPosition(0);
    int16_t pos1 = getMotorPosition(1);
    int16_t pos2 = getMotorPosition(2);

    bool motor0_in_range = (pos0 >= 4200 && pos0 <= 4800);
    bool motor1_in_range = (pos1 >= 6500 && pos1 <= 6900);

    // 初始化标志（防止上次残留）
    stateFlags.motor0_done = false;
    stateFlags.motor1_done = false;
    stateFlags.motor2_done = false;

    // 情况1：两个条件都满足 - 只需要移动电机2
    if (motor0_in_range && motor1_in_range) {
      Serial.println("=> State: Both turn and raise are homed, only moving Motor2...");

      stateFlags.motor2_done = !startMotorToPosition(2, 5000, &stateFlags.motor2_forward);
      if (stateFlags.motor2_done) {
        Serial.println("=> Motor2 already at position 5000");
        Serial.println("=> FLAT completed");
        return;
      }

      currentState = STATE_FLAT_MOTOR2_ONLY;
      stateStartTime = millis();
      return;
    }

    // 情况2：状态1 - 翻身后归位（电机1在范围内）
    if (motor1_in_range) {
      Serial.println("=> State 1: After turn, moving Motor0 and Motor2...");

      stateFlags.motor0_done = !startMotorToPosition(0, 4500, &stateFlags.motor0_forward);
      stateFlags.motor2_done = !startMotorToPosition(2, 5000, &stateFlags.motor2_forward);

      if (stateFlags.motor0_done) Serial.println("=> Motor0 already at position 4500");
      if (stateFlags.motor2_done) Serial.println("=> Motor2 already at position 5000");

      if (stateFlags.motor0_done && stateFlags.motor2_done) {
        Serial.println("=> FLAT completed");
        return;
      }

      currentState = STATE_FLAT_AFTER_TURN;
      stateStartTime = millis();
      return;
    }

    // 情况3：状态2 - 起背后归位（电机0在范围内）
    if (motor0_in_range) {
      Serial.println("=> State 2: After raise, moving Motor1 to limit and Motor2...");

      // 启动电机1前进到极限位置
      motorForward(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 启动电机2移动到5000
      stateFlags.motor2_done = !startMotorToPosition(2, 5000, &stateFlags.motor2_forward);
      if (stateFlags.motor2_done) Serial.println("=> Motor2 already at position 5000");

      // 初始化M1的极限位置检测
      stateFlags.motor1_done = false;
      stateFlags.motor1_lastPos = getMotorPosition(1);
      stateFlags.motor1_lastPosChange = millis();

      currentState = STATE_FLAT_AFTER_RAISE;
      stateStartTime = millis();
      return;
    }

    // 情况4：两个条件都不满足 - 拒绝执行
    Serial.println("=> ERROR: Bed state invalid for FLAT command");
    Serial.print("=> Motor0 position: ");
    Serial.print(pos0);
    Serial.println(" (required: 4200-4800 for State 2)");
    Serial.print("=> Motor1 position: ");
    Serial.print(pos1);
    Serial.println(" (required: 6500-6900 for State 1)");
    Serial.println("=> FLAT command ignored");
    return;
  }

  // 起背
  else if (memcmp(cmd, cmdRaiseBack, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE BACK: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE BACK: Checking Motor0 position...");
    int16_t pos0 = getMotorPosition(0);

    // 检查电机0位置是否在4200-4800范围内
    if (pos0 >= 4200 && pos0 <= 4800) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(pos0);
      Serial.println("), Motor1 reversing...");

      motorReverse(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 初始化极限位置检测
      currentState = STATE_RAISE_BACK_RUNNING;
      stateStartTime = millis();
      lastPosChangeTime = millis();
      lastMonitoredPos = getMotorPosition(1);
      Serial.println("=> RAISE BACK: Motor1 reversing (will stop at limit)");
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> RAISE BACK: Command ignored");
    }
    return;
  }

  // 起背回归
  else if (memcmp(cmd, cmdRaiseBackReturn, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE BACK RETURN: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE BACK RETURN: Checking Motor0 position...");
    int16_t pos0 = getMotorPosition(0);

    // 检查电机0位置是否在4200-4800范围内
    if (pos0 >= 4200 && pos0 <= 4800) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(pos0);
      Serial.println("), Motor1 forwarding...");

      motorForward(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 初始化极限位置检测
      currentState = STATE_RAISE_BACK_RETURN_RUNNING;
      stateStartTime = millis();
      lastPosChangeTime = millis();
      lastMonitoredPos = getMotorPosition(1);
      Serial.println("=> RAISE BACK RETURN: Motor1 forwarding (will stop at limit)");
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> RAISE BACK RETURN: Command ignored");
    }
    return;
  }

  // 抬腿
  else if (memcmp(cmd, cmdRaiseLeg, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE LEG: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE LEG: Moving Motor2 to position 6700...");
    int16_t pos2 = getMotorPosition(2);

    // 检查是否已在目标范围内
    if (pos2 >= 6500 && pos2 <= 6900) {
      Serial.print("=> Motor2 already in target range: ");
      Serial.println(pos2);
      Serial.println("=> RAISE LEG completed");
      return;
    }

    // 初始化标志（防止上次残留）
    stateFlags.motor2_done = false;

    stateFlags.motor2_done = !startMotorToPosition(2, 6700, &stateFlags.motor2_forward);
    if (stateFlags.motor2_done) {
      Serial.println("=> Motor2 already at position 6700");
      Serial.println("=> RAISE LEG completed");
      return;
    }

    if (stateFlags.motor2_forward) {
      Serial.println("=> Motor2 moving forward to 6700");
    } else {
      Serial.println("=> Motor2 moving reverse to 6700");
    }

    currentState = STATE_RAISE_LEG_RUNNING;
    stateStartTime = millis();
    return;
  }

  // 抬腿回归
  else if (memcmp(cmd, cmdRaiseLegReturn, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE LEG RETURN: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE LEG RETURN: Moving Motor2 to position 5000...");
    int16_t pos2 = getMotorPosition(2);

    // 检查是否已在目标范围内
    if (pos2 >= 4800 && pos2 <= 5200) {
      Serial.print("=> Motor2 already in target range: ");
      Serial.println(pos2);
      Serial.println("=> RAISE LEG RETURN completed");
      return;
    }

    // 初始化标志（防止上次残留）
    stateFlags.motor2_done = false;

    stateFlags.motor2_done = !startMotorToPosition(2, 5000, &stateFlags.motor2_forward);
    if (stateFlags.motor2_done) {
      Serial.println("=> Motor2 already at position 5000");
      Serial.println("=> RAISE LEG RETURN completed");
      return;
    }

    if (stateFlags.motor2_forward) {
      Serial.println("=> Motor2 moving forward to 5000");
    } else {
      Serial.println("=> Motor2 moving reverse to 5000");
    }

    currentState = STATE_RAISE_LEG_RETURN_RUNNING;
    stateStartTime = millis();
    return;
  }

  // 抬腿归零
  else if (memcmp(cmd, cmdRaiseLegZero, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE LEG ZERO: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE LEG ZERO: Moving Motor2 to position 0...");
    int16_t pos2 = getMotorPosition(2);

    // 检查是否已在目标范围内
    if (pos2 >= -200 && pos2 <= 200) {
      Serial.print("=> Motor2 already in target range: ");
      Serial.println(pos2);
      Serial.println("=> RAISE LEG ZERO completed");
      return;
    }

    // 初始化标志（防止上次残留）
    stateFlags.motor2_done = false;

    stateFlags.motor2_done = !startMotorToPosition(2, 0, &stateFlags.motor2_forward);
    if (stateFlags.motor2_done) {
      Serial.println("=> Motor2 already at position 0");
      Serial.println("=> RAISE LEG ZERO completed");
      return;
    }

    if (stateFlags.motor2_forward) {
      Serial.println("=> Motor2 moving forward to 0");
    } else {
      Serial.println("=> Motor2 moving reverse to 0");
    }

    currentState = STATE_RAISE_LEG_ZERO_RUNNING;
    stateStartTime = millis();
    return;
  }

  // 坐立（起背+屈腿归零）
  else if (memcmp(cmd, cmdSit, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> RAISE BACK + LEG ZERO: State machine busy, command ignored");
      return;
    }

    Serial.println("=> RAISE BACK + LEG ZERO: Checking Motor0 position...");
    int16_t pos0 = getMotorPosition(0);
    int16_t pos2 = getMotorPosition(2);

    // 检查电机0位置是否在4200-4800范围内
    if (pos0 < 4200 || pos0 > 4800) {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> RAISE BACK + LEG ZERO: Command ignored");
      return;
    }

    Serial.print("=> Motor0 position OK (");
    Serial.print(pos0);
    Serial.println("), starting RAISE BACK + LEG ZERO...");

    // 初始化标志（防止上次残留）
    stateFlags.motor1_done = false;
    stateFlags.motor2_done = false;

    // 启动电机1反转（起背）
    motorReverse(1);
    setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

    // 启动电机2移动到0
    stateFlags.motor2_done = !startMotorToPosition(2, 0, &stateFlags.motor2_forward);
    if (stateFlags.motor2_done) {
      Serial.println("=> Motor2 already at position 0");
    } else {
      if (stateFlags.motor2_forward) {
        Serial.println("=> Motor2 moving forward to 0");
      } else {
        Serial.println("=> Motor2 moving reverse to 0");
      }
    }

    // 初始化M1的极限位置检测变量
    stateFlags.motor1_lastPos = getMotorPosition(1);
    stateFlags.motor1_lastPosChange = millis();

    currentState = STATE_SIT_RUNNING;
    stateStartTime = millis();
    return;
  }

  // 如厕（三阶段顺序执行：M4→0, M3→0, M5→4500，同时并行执行坐起：M1反转+M2→0）
  else if (memcmp(cmd, cmdToilet, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> TOILET: State machine busy, command ignored");
      return;
    }

    Serial.println("=> TOILET: Starting parallel execution...");
    int16_t pos0 = getMotorPosition(0);

    // 初始化所有标志（必须在最前面，防止上次残留）
    stateFlags.motor0_done = false;
    stateFlags.motor1_done = false;
    stateFlags.motor2_done = false;
    stateFlags.motor3_done = false;
    stateFlags.motor4_done = false;
    stateFlags.motor5_done = false;
    stateFlags.motor6_done = false;
    stateFlags.motor0_forward = false;
    stateFlags.motor2_forward = false;
    stateFlags.motor3_forward = false;
    stateFlags.motor4_forward = false;
    stateFlags.motor5_forward = false;
    stateFlags.motor6_forward = false;
    stateFlags.motor1_lastPos = getMotorPosition(1);
    stateFlags.motor1_lastPosChange = millis();
    stateFlags.motor3_lastPos = getMotorPosition(3);
    stateFlags.motor3_lastPosChange = millis();
    stateFlags.motor4_lastPos = getMotorPosition(4);
    stateFlags.motor4_lastPosChange = millis();
    stateFlags.motor5_lastPos = getMotorPosition(5);
    stateFlags.motor5_lastPosChange = millis();
    stateFlags.motor6_lastPos = getMotorPosition(6);
    stateFlags.motor6_lastPosChange = millis();

    // 检查并启动坐起动作
    if (pos0 >= 4200 && pos0 <= 4800) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(pos0);
      Serial.println("), starting SIT action (M1+M2)...");

      // 启动M1反转（起背）
      motorReverse(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));
      stateFlags.motor1_lastPosChange = millis();
      stateFlags.motor1_lastPos = getMotorPosition(1);
      stateFlags.motor1_done = false;

      // 启动M2→0
      stateFlags.motor2_done = !startMotorToPosition(2, 0, &stateFlags.motor2_forward);
      if (stateFlags.motor2_done) {
        Serial.println("=> Motor2 already at position 0");
      } else {
        if (stateFlags.motor2_forward) {
          Serial.println("=> Motor2 moving forward to 0");
        } else {
          Serial.println("=> Motor2 moving reverse to 0");
        }
      }
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(pos0);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> SIT action skipped");
      stateFlags.motor1_done = true;  // 标记为已完成（跳过）
      stateFlags.motor2_done = true;
    }

    // 启动阶段1: M4→0
    Serial.println("=> Stage 1: Moving Motor4 to position 0...");
    stateFlags.motor4_done = !startMotorToPosition(4, 0, &stateFlags.motor4_forward);
    if (stateFlags.motor4_done) {
      Serial.println("=> Motor4 already at position 0");
    }

    currentState = STATE_TOILET_STAGE1;
    stateStartTime = millis();
    return;
  }

  // 结束如厕（三阶段顺序执行：M5+M6→0, M3→12900, M4→7300）
  else if (memcmp(cmd, cmdEndToilet, 8) == 0) {
    if (currentState != STATE_IDLE) {
      Serial.println("=> END TOILET: State machine busy, command ignored");
      return;
    }

    Serial.println("=> END TOILET: Starting sequence...");

    // 初始化所有标志（必须在最前面，防止上次残留）
    stateFlags.motor0_done = false;
    stateFlags.motor1_done = false;
    stateFlags.motor2_done = false;
    stateFlags.motor3_done = false;
    stateFlags.motor4_done = false;
    stateFlags.motor5_done = false;
    stateFlags.motor6_done = false;
    stateFlags.motor0_forward = false;
    stateFlags.motor2_forward = false;
    stateFlags.motor3_forward = false;
    stateFlags.motor4_forward = false;
    stateFlags.motor5_forward = false;
    stateFlags.motor6_forward = false;
    stateFlags.motor1_lastPos = 0;
    stateFlags.motor1_lastPosChange = 0;
    stateFlags.motor3_lastPos = 0;
    stateFlags.motor3_lastPosChange = 0;
    stateFlags.motor4_lastPos = 0;
    stateFlags.motor4_lastPosChange = 0;
    stateFlags.motor5_lastPos = getMotorPosition(5);
    stateFlags.motor5_lastPosChange = millis();
    stateFlags.motor6_lastPos = getMotorPosition(6);
    stateFlags.motor6_lastPosChange = millis();

    // 启动阶段1: M5+M6→0（同步）
    Serial.println("=> Stage 1: Moving Motor5 and Motor6 to position 0...");
    stateFlags.motor5_done = !startMotorToPosition(5, 0, &stateFlags.motor5_forward);
    stateFlags.motor6_done = !startMotorToPosition(6, 0, &stateFlags.motor6_forward);

    if (stateFlags.motor5_done) {
      Serial.println("=> Motor5 already at position 0");
    }

    if (stateFlags.motor6_done) {
      Serial.println("=> Motor6 already at position 0");
    }

    currentState = STATE_END_TOILET_STAGE1;
    stateStartTime = millis();
    return;
  }

  // 未知指令
  Serial.println("=> Unknown command");
}

// ================= 串口中断服务函数 =================

// USART1 (Serial1) 中断服务函数
// Arduino框架会自动调用这个函数（如果定义了）
void serialEvent1() {
  while (SERIAL_SCREEN.available()) {
    byte inByte = SERIAL_SCREEN.read();
    unsigned long currentTime = millis();

    // 超时检测：如果距离上次接收超过50ms，重置接收缓冲区
    if (cmdIndex > 0 && (currentTime - lastRxTime > 50)) {
      cmdIndex = 0;
    }
    lastRxTime = currentTime;

    // 检测帧头 0xEE
    if (inByte == 0xEE) {
      cmdIndex = 0;  // 重置索引
      cmdBuffer[cmdIndex++] = inByte;
    }
    // 接收后续字节
    else if (cmdIndex > 0 && cmdIndex < 8) {
      cmdBuffer[cmdIndex++] = inByte;

      // 接收完整8字节命令
      if (cmdIndex == 8) {
        cmdReady = true;  // 标记命令就绪
        cmdIndex = 0;     // 重置索引，准备接收下一条命令
      }
    }
    // 其他情况：未收到帧头的数据，忽略
  }
}

// 主循环中处理接收到的命令（非中断环境）
void handleScreenCommands() {
  if (cmdReady) {
    // 原子操作：复制命令并清除标志
    noInterrupts();
    byte cmd[8];
    for (int i = 0; i < 8; i++) {
      cmd[i] = cmdBuffer[i];
    }
    cmdReady = false;
    interrupts();

    // 处理命令
    processCommand(cmd);

    // 回传命令确认
    SERIAL_SCREEN.write(cmd, 8);
    SERIAL_SCREEN.flush();
  }
}

void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);

  // 显示所有7个电机的精简信息
  for (int i = 0; i < 7; i++) {
    display.setCursor(0, i * 9);

    // 电机号
    display.print("M");
    display.print(i);
    display.print(":");

    // 位置（6字符宽度）- 原子读取
    int16_t pos;
    noInterrupts();
    pos = motors[i].position;
    interrupts();

    display.setCursor(18, i * 9);
    if (pos >= 0) display.print(" ");
    display.print(pos);

    // 电流（6字符宽度）
    display.setCursor(60, i * 9);
    display.print(motors[i].current_amps, 2);
    display.print("A");

    // 保存状态
    if (pos == motors[i].lastSavedPosition) {
      display.setCursor(110, i * 9);
      display.print("S");
    }
  }

  display.display();
}

// ================= 主程序 =================

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // 禁用 JTAG
  Serial.begin(9600);
  SERIAL_SCREEN.begin(9600);

  // 初始化串口中断接收变量
  cmdIndex = 0;
  cmdReady = false;
  lastRxTime = 0;

  // --- 1. 初始化所有引脚 ---
  pinMode(PIN_4051_S0, OUTPUT);
  pinMode(PIN_4051_S1, OUTPUT);
  pinMode(PIN_4051_S2, OUTPUT);

  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_OE,    OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_ADC_CURRENT, INPUT_ANALOG);

  // 初始化所有电机的霍尔信号引脚
  for (int i = 0; i < 7; i++) {
    pinMode(motors[i].hallPinA, INPUT_PULLUP);
    pinMode(motors[i].hallPinB, INPUT_PULLUP);
  }

  // --- 读取并初始化所有电机的霍尔编码器初始状态 ---
  for (int i = 0; i < 7; i++) {
    uint8_t A = digitalRead(motors[i].hallPinA);
    uint8_t B = digitalRead(motors[i].hallPinB);
    motors[i].lastState = (A << 1) | B;
  }
  Serial.println("Hall encoder initial states loaded.");

  digitalWrite(PIN_OE, HIGH);
  shiftOut16(0x00, 0x00);
  digitalWrite(PIN_OE, LOW);

  // --- 2. 初始化 I2C 总线 ---
  Wire.begin();

  // --- 3. 初始化 OLED ---
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("System Init...");
  display.display();

  // --- 4. 初始化 PCA9685 ---
  pca9685.begin();
  pca9685.setPWMFreq(PCA9685_FREQ);
  pca9685.setOutputMode(true);

  // --- 5. 读取 EEPROM ---
  Wire.beginTransmission(EEPROM_ADDR);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("EEPROM detected.");
    display.println("Memory: OK");
    display.display();

    // 初始化检测
    initEEPROM();

    // 读取所有电机的位置数据
    for (int i = 0; i < 7; i++) {
      motors[i].position = readPositionFromEEPROM(i);
      motors[i].lastSavedPosition = motors[i].position;
      motors[i].lastLoopPosition = motors[i].position;
      motors[i].lastPosChangeTime = millis();
      Serial.print("Load Motor");
      Serial.print(i);
      Serial.print(" Pos: ");
      Serial.println(motors[i].position);
    }
    display.println("Positions loaded");
  } else {
    Serial.print("EEPROM Error: ");
    Serial.println(error);
    display.println("Memory: ERROR!");
    display.println("Check Wiring!");
    // 初始化所有电机位置为0
    for (int i = 0; i < 7; i++) {
      motors[i].position = 0;
      motors[i].lastSavedPosition = 0;
      motors[i].lastLoopPosition = 0;
      motors[i].lastPosChangeTime = millis();
    }
  }

  display.display();
  delay(1500);

  // --- 6. 为所有7个电机开启中断（4倍频正交解码）---
  Serial.println("Attaching interrupts for all motors (4x quadrature decoding)...");

  // Motor0 (PA0, PA1) - A和B都绑定到同一个中断函数
  attachInterrupt(digitalPinToInterrupt(motors[0].hallPinA), ISR_motor0_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[0].hallPinB), ISR_motor0_AB, CHANGE);

  // Motor1 (PA2, PA3)
  attachInterrupt(digitalPinToInterrupt(motors[1].hallPinA), ISR_motor1_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].hallPinB), ISR_motor1_AB, CHANGE);

  // Motor2 (PA4, PA5) - A相和B相都监听CHANGE（4倍频），方向由motor2_dir标志决定
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinA), ISR_motor2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinB), ISR_motor2_B, CHANGE);

  // Motor3 (PA6, PA7)
  attachInterrupt(digitalPinToInterrupt(motors[3].hallPinA), ISR_motor3_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[3].hallPinB), ISR_motor3_AB, CHANGE);

  // Motor4 (PB8, PB9)
  attachInterrupt(digitalPinToInterrupt(motors[4].hallPinA), ISR_motor4_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[4].hallPinB), ISR_motor4_AB, CHANGE);

  // Motor5 (PB10, PB11)
  attachInterrupt(digitalPinToInterrupt(motors[5].hallPinA), ISR_motor5_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[5].hallPinB), ISR_motor5_AB, CHANGE);

  // Motor6 (PB12, PB15)
  attachInterrupt(digitalPinToInterrupt(motors[6].hallPinA), ISR_motor6_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[6].hallPinB), ISR_motor6_AB, CHANGE);

  Serial.println("All interrupts attached successfully.");

  last_ms = millis();
}

void loop() {
  // 串口中断接收（STM32需要手动调用）
  if (SERIAL_SCREEN.available()) {
    serialEvent1();
  }

  // 状态机更新（优先级最高，非阻塞）
  updateStateMachine();

  handleScreenCommands();

  // 频率统计（所有电机）
  uint32_t now_ms = millis();
  if (now_ms - last_ms >= 1000) {
    uint32_t dt = now_ms - last_ms;
    for (int i = 0; i < 7; i++) {
      motors[i].freq_hallA = (motors[i].hallA_count - motors[i].last_hallA_cnt) * 1000.0f / dt;
      motors[i].freq_hallB = (motors[i].hallB_count - motors[i].last_hallB_cnt) * 1000.0f / dt;
      motors[i].last_hallA_cnt = motors[i].hallA_count;
      motors[i].last_hallB_cnt = motors[i].hallB_count;
    }
    last_ms = now_ms;
  }

  // 电流采样（轮询所有电机，每5ms采样一个电机）
  static unsigned long lastCurrentUpdate = 0;
  static uint8_t currentMotorIndex = 0;
  if (millis() - lastCurrentUpdate >= 5) {
    updateCurrentReading(currentMotorIndex);

    // 过流保护检测
    if (motors[currentMotorIndex].current_amps > motors[currentMotorIndex].maxCurrent) {
      motorStop(currentMotorIndex);
      setPCA9685PWM(motors[currentMotorIndex].pwmChannel, 0);
      Serial.print("!!! OVERCURRENT Motor");
      Serial.print(currentMotorIndex);
      Serial.print(": ");
      Serial.print(motors[currentMotorIndex].current_amps, 2);
      Serial.print("A > ");
      Serial.print(motors[currentMotorIndex].maxCurrent, 2);
      Serial.println("A - STOPPED");

      // ⚠️ 重要：过流时中止状态机，防止卡死
      if (currentState != STATE_IDLE) {
        Serial.println("!!! State machine aborted due to overcurrent");
        currentState = STATE_IDLE;
      }
    }

    currentMotorIndex++;
    if (currentMotorIndex >= 7) currentMotorIndex = 0;
    lastCurrentUpdate = millis();
  }

  // OLED 刷新
  if (millis() - lastOLEDUpdate >= 100) {
    drawOLED();
    lastOLEDUpdate = millis();
  }

  // 自动保存逻辑（所有电机）
  for (int i = 0; i < 7; i++) {
    // 监测位置变化
    if (motors[i].position != motors[i].lastLoopPosition) {
      motors[i].lastPosChangeTime = millis();
      motors[i].lastLoopPosition = motors[i].position;
    }

    // 位置静止超过1秒且未保存，则保存
    if ((motors[i].position != motors[i].lastSavedPosition) &&
        (millis() - motors[i].lastPosChangeTime > 1000)) {
      savePositionToEEPROM(i, motors[i].position);
      motors[i].lastSavedPosition = motors[i].position;
    }
  }
}