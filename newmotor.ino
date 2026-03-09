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
    3.0                     // maxCurrent (A)
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
    3.0
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
    3.0
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
    3.0
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
    3.0
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
    3.0
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
    3.0
  }
};

// 74HC595 全局状态变量
uint16_t hc595_state = 0x0000;

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

byte cmdBuffer[8];  // 8字节命令缓冲区
int cmdIndex = 0;

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

  shiftOut16((hc595_state >> 8) & 0xFF, hc595_state & 0xFF);
}

// 电机后退：只设置该电机的后退位，清除前进位
void motorReverse(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  uint16_t forwardMask = ((uint16_t)motors[motorIndex].forwardMaskHigh << 8) | motors[motorIndex].forwardMaskLow;
  uint16_t reverseMask = ((uint16_t)motors[motorIndex].reverseMaskHigh << 8) | motors[motorIndex].reverseMaskLow;

  hc595_state &= ~forwardMask;  // 清除前进位
  hc595_state |= reverseMask;   // 设置后退位

  shiftOut16((hc595_state >> 8) & 0xFF, hc595_state & 0xFF);
}

// 电机停止：清除该电机的前进和后退位
void motorStop(uint8_t motorIndex) {
  if (motorIndex >= 7) return;

  uint16_t forwardMask = ((uint16_t)motors[motorIndex].forwardMaskHigh << 8) | motors[motorIndex].forwardMaskLow;
  uint16_t reverseMask = ((uint16_t)motors[motorIndex].reverseMaskHigh << 8) | motors[motorIndex].reverseMaskLow;

  hc595_state &= ~forwardMask;  // 清除前进位
  hc595_state &= ~reverseMask;  // 清除后退位

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

// Motor2 (PA4, PA5)
void ISR_motor2_AB() {
  uint8_t A = digitalRead(motors[2].hallPinA);
  uint8_t B = digitalRead(motors[2].hallPinB);
  uint8_t currentState = (A << 1) | B;
  uint8_t lastState = motors[2].lastState;

  if ((lastState == 0b00 && currentState == 0b01) ||
      (lastState == 0b01 && currentState == 0b11) ||
      (lastState == 0b11 && currentState == 0b10) ||
      (lastState == 0b10 && currentState == 0b00)) {
    motors[2].position--;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[2].position++;
  }

  motors[2].lastState = currentState;
  if (A != (lastState >> 1)) motors[2].hallA_count++;
  if (B != (lastState & 0x01)) motors[2].hallB_count++;
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
    motors[6].position++;
  }
  else if ((lastState == 0b00 && currentState == 0b10) ||
           (lastState == 0b10 && currentState == 0b11) ||
           (lastState == 0b11 && currentState == 0b01) ||
           (lastState == 0b01 && currentState == 0b00)) {
    motors[6].position--;
  }

  motors[6].lastState = currentState;
  if (A != (lastState >> 1)) motors[6].hallA_count++;
  if (B != (lastState & 0x01)) motors[6].hallB_count++;
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
    return;
  }
  else if (memcmp(cmd, cmdMotor0Forward, 8) == 0) {
    Serial.println("=> Motor0 FORWARD");
    motorForward(0);
    setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor0Reverse, 8) == 0) {
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
    return;
  }
  else if (memcmp(cmd, cmdMotor1Forward, 8) == 0) {
    Serial.println("=> Motor1 FORWARD");
    motorForward(1);
    setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor1Reverse, 8) == 0) {
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
    return;
  }
  else if (memcmp(cmd, cmdMotor2Forward, 8) == 0) {
    Serial.println("=> Motor2 FORWARD");
    motorForward(2);
    setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor2Reverse, 8) == 0) {
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
    Serial.println("=> Motor3 STOP");
    motorStop(3);
    setPCA9685PWM(motors[3].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor3Forward, 8) == 0) {
    Serial.println("=> Motor3 FORWARD");
    motorForward(3);
    setPCA9685PWM(motors[3].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor3Reverse, 8) == 0) {
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
    Serial.println("=> Motor4 STOP");
    motorStop(4);
    setPCA9685PWM(motors[4].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor4Forward, 8) == 0) {
    Serial.println("=> Motor4 FORWARD");
    motorForward(4);
    setPCA9685PWM(motors[4].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor4Reverse, 8) == 0) {
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

  // ---- 电机5指令 ----
  else if (memcmp(cmd, cmdMotor5Stop, 8) == 0) {
    Serial.println("=> Motor5 STOP");
    motorStop(5);
    setPCA9685PWM(motors[5].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor5Forward, 8) == 0) {
    Serial.println("=> Motor5 FORWARD");
    motorForward(5);
    setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor5Reverse, 8) == 0) {
    Serial.println("=> Motor5 REVERSE");
    motorReverse(5);
    setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdResetMotor5, 8) == 0) {
    Serial.println("=> Motor5 RESET POSITION TO 0");
    noInterrupts();
    motors[5].position = 0;
    motors[5].hallA_count = 0;
    motors[5].hallB_count = 0;
    interrupts();
    savePositionToEEPROM(5, 0);
    motors[5].lastSavedPosition = 0;
    motors[5].lastLoopPosition = 0;
    motors[5].last_hallA_cnt = 0;
    motors[5].last_hallB_cnt = 0;
    return;
  }

  // ---- 电机6指令 ----
  else if (memcmp(cmd, cmdMotor6Stop, 8) == 0) {
    Serial.println("=> Motor6 STOP");
    motorStop(6);
    setPCA9685PWM(motors[6].pwmChannel, 0);
    return;
  }
  else if (memcmp(cmd, cmdMotor6Forward, 8) == 0) {
    Serial.println("=> Motor6 FORWARD");
    motorForward(6);
    setPCA9685PWM(motors[6].pwmChannel, pctToDuty(SPEED_PERCENT));
    return;
  }
  else if (memcmp(cmd, cmdMotor6Reverse, 8) == 0) {
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

  // 左翻身
  else if (memcmp(cmd, cmdLeftTurn, 8) == 0) {
    Serial.println("=> LEFT TURN: Checking Motor1 position...");

    // 检查电机1位置是否在6500-6900范围内
    if (motors[1].position < 6500 || motors[1].position > 6900) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(motors[1].position);
      Serial.println(" (required: 6500-6900)");
      Serial.println("=> LEFT TURN: Command ignored");
      return;
    }

    Serial.print("=> Motor1 position OK (");
    Serial.print(motors[1].position);
    Serial.println("), executing LEFT TURN...");

    motorReverse(0);
    setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));

    // 等待电机到达极限位置（2秒内位置不变则认为到达极限）
    unsigned long lastPosChange = millis();
    int16_t lastPos = motors[0].position;

    while (millis() - lastPosChange < 2000) {
      delay(50);
      updateCurrentReading(0);
      drawOLED();  // 刷新显示

      if (motors[0].position != lastPos) {
        lastPos = motors[0].position;
        lastPosChange = millis();
      }
    }

    // 停止电机
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
    return;
  }

  // 右翻身
  else if (memcmp(cmd, cmdRightTurn, 8) == 0) {
    Serial.println("=> RIGHT TURN: Checking Motor1 position...");

    // 检查电机1位置是否在6500-6900范围内
    if (motors[1].position < 6500 || motors[1].position > 6900) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(motors[1].position);
      Serial.println(" (required: 6500-6900)");
      Serial.println("=> RIGHT TURN: Command ignored");
      return;
    }

    Serial.print("=> Motor1 position OK (");
    Serial.print(motors[1].position);
    Serial.println("), executing RIGHT TURN...");

    if (motors[0].position < 6400) {
      motorForward(0);
      setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 等待到达目标位置
      while (motors[0].position < 6400) {
        delay(50);
        updateCurrentReading(0);
        drawOLED();  // 刷新显示
      }

      // 停止电机
      motorStop(0);
      setPCA9685PWM(motors[0].pwmChannel, 0);
      Serial.println("=> RIGHT TURN completed, Motor0 reached position 6400");
    } else {
      Serial.println("=> Motor0 already at or beyond position 6400");
    }
    return;
  }

  // 平躺
  else if (memcmp(cmd, cmdFlat, 8) == 0) {
    Serial.println("=> FLAT: Checking bed state...");

    bool motor0_in_range = (motors[0].position >= 4200 && motors[0].position <= 4800);
    bool motor1_in_range = (motors[1].position >= 6500 && motors[1].position <= 6900);

    // 情况1：两个条件都满足 - 只需要移动电机2
    if (motor0_in_range && motor1_in_range) {
      Serial.println("=> State: Both turn and raise are homed, only moving Motor2...");

      bool motor2_done = (motors[2].position == 5000);
      bool motor2_forward = false;

      if (!motor2_done) {
        if (motors[2].position < 5000) {
          motorForward(2);
          motor2_forward = true;
        } else {
          motorReverse(2);
          motor2_forward = false;
        }
        setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
      }

      // 等待电机2到达目标位置
      while (!motor2_done) {
        delay(50);
        updateCurrentReading(2);
        drawOLED();

        if ((motor2_forward && motors[2].position >= 5000) ||
            (!motor2_forward && motors[2].position <= 5000)) {
          motorStop(2);
          setPCA9685PWM(motors[2].pwmChannel, 0);
          motor2_done = true;
          Serial.println("=> Motor2 reached position 5000");
        }
      }

      Serial.println("=> FLAT completed");
      return;
    }

    // 情况2：状态1 - 翻身后归位（电机1在范围内）
    if (motor1_in_range) {
      Serial.println("=> State 1: After turn, moving Motor0 and Motor2...");

      bool motor0_done = (motors[0].position == 4500);
      bool motor2_done = (motors[2].position == 5000);
      bool motor0_forward = false;
      bool motor2_forward = false;

      if (!motor0_done) {
        if (motors[0].position < 4500) {
          motorForward(0);
          motor0_forward = true;
        } else {
          motorReverse(0);
          motor0_forward = false;
        }
        setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));
      }

      if (!motor2_done) {
        if (motors[2].position < 5000) {
          motorForward(2);
          motor2_forward = true;
        } else {
          motorReverse(2);
          motor2_forward = false;
        }
        setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
      }

      // 等待两个电机都到达目标位置
      while (!motor0_done || !motor2_done) {
        delay(50);
        updateCurrentReading(0);
        updateCurrentReading(2);
        drawOLED();

        // 检查电机0是否到达
        if (!motor0_done) {
          if ((motor0_forward && motors[0].position >= 4500) ||
              (!motor0_forward && motors[0].position <= 4500)) {
            motorStop(0);
            setPCA9685PWM(motors[0].pwmChannel, 0);
            motor0_done = true;
            Serial.println("=> Motor0 reached position 4500");
          }
        }

        // 检查电机2是否到达
        if (!motor2_done) {
          if ((motor2_forward && motors[2].position >= 5000) ||
              (!motor2_forward && motors[2].position <= 5000)) {
            motorStop(2);
            setPCA9685PWM(motors[2].pwmChannel, 0);
            motor2_done = true;
            Serial.println("=> Motor2 reached position 5000");
          }
        }
      }

      Serial.println("=> FLAT completed");
      return;
    }

    // 情况3：状态2 - 起背后归位（电机0在范围内）
    if (motor0_in_range) {
      Serial.println("=> State 2: After raise, moving Motor1 to limit and Motor2...");

      // 启动电机1前进到极限位置
      motorForward(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 启动电机2移动到5000
      bool motor2_done = (motors[2].position == 5000);
      bool motor2_forward = false;

      if (!motor2_done) {
        if (motors[2].position < 5000) {
          motorForward(2);
          motor2_forward = true;
        } else {
          motorReverse(2);
          motor2_forward = false;
        }
        setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
      }

      // 等待电机1到达极限位置（2秒内位置不变则认为到达极限）
      unsigned long lastPosChange = millis();
      int16_t lastPos = motors[1].position;
      bool motor1_done = false;

      // 等待两个电机都到达目标位置
      while (!motor1_done || !motor2_done) {
        delay(50);
        updateCurrentReading(1);
        updateCurrentReading(2);
        drawOLED();

        // 检查电机1是否到达极限位置
        if (!motor1_done) {
          if (motors[1].position != lastPos) {
            lastPos = motors[1].position;
            lastPosChange = millis();
          }

          if (millis() - lastPosChange >= 2000) {
            motorStop(1);
            setPCA9685PWM(motors[1].pwmChannel, 0);
            motor1_done = true;
            Serial.println("=> Motor1 reached limit position");
          }
        }

        // 检查电机2是否到达目标位置
        if (!motor2_done) {
          if ((motor2_forward && motors[2].position >= 5000) ||
              (!motor2_forward && motors[2].position <= 5000)) {
            motorStop(2);
            setPCA9685PWM(motors[2].pwmChannel, 0);
            motor2_done = true;
            Serial.println("=> Motor2 reached position 5000");
          }
        }
      }

      Serial.println("=> FLAT completed");
      return;
    }

    // 情况4：两个条件都不满足 - 拒绝执行
    Serial.println("=> ERROR: Bed state invalid for FLAT command");
    Serial.print("=> Motor0 position: ");
    Serial.print(motors[0].position);
    Serial.println(" (required: 4200-4800 for State 2)");
    Serial.print("=> Motor1 position: ");
    Serial.print(motors[1].position);
    Serial.println(" (required: 6500-6900 for State 1)");
    Serial.println("=> FLAT command ignored");
    return;
  }

  // 起背
  else if (memcmp(cmd, cmdRaiseBack, 8) == 0) {
    Serial.println("=> RAISE BACK: Checking Motor0 position...");

    // 检查电机0位置是否在4200-4800范围内
    if (motors[0].position >= 4200 && motors[0].position <= 4800) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(motors[0].position);
      Serial.println("), Motor1 reversing...");

      motorReverse(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      Serial.println("=> RAISE BACK: Motor1 reversing (non-blocking)");
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(motors[0].position);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> RAISE BACK: Command ignored");
    }
    return;
  }

  // 抬腿
  else if (memcmp(cmd, cmdRaiseLeg, 8) == 0) {
    Serial.println("=> RAISE LEG: Moving Motor2 to position 6700...");

    // 检查是否已在目标范围内
    if (motors[2].position >= 6500 && motors[2].position <= 6900) {
      Serial.print("=> Motor2 already in target range: ");
      Serial.println(motors[2].position);
      Serial.println("=> RAISE LEG completed");
      return;
    }

    bool motor2_done = false;
    bool motor2_forward = false;

    if (motors[2].position < 6500) {
      motorForward(2);
      motor2_forward = true;
      Serial.println("=> Motor2 moving forward to 6700");
    } else {
      motorReverse(2);
      motor2_forward = false;
      Serial.println("=> Motor2 moving reverse to 6700");
    }
    setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));

    // 等待电机2到达目标范围
    while (!motor2_done) {
      delay(50);
      updateCurrentReading(2);
      drawOLED();

      if ((motor2_forward && motors[2].position >= 6700) ||
          (!motor2_forward && motors[2].position <= 6700)) {
        motorStop(2);
        setPCA9685PWM(motors[2].pwmChannel, 0);
        motor2_done = true;
        Serial.print("=> Motor2 reached position: ");
        Serial.println(motors[2].position);
      }
    }

    Serial.println("=> RAISE LEG completed");
    return;
  }

  // 坐立（起背+屈腿归零）
  else if (memcmp(cmd, cmdSit, 8) == 0) {
    Serial.println("=> RAISE BACK + LEG ZERO: Checking Motor0 position...");

    // 检查电机0位置是否在4200-4800范围内
    if (motors[0].position < 4200 || motors[0].position > 4800) {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(motors[0].position);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> RAISE BACK + LEG ZERO: Command ignored");
      return;
    }

    Serial.print("=> Motor0 position OK (");
    Serial.print(motors[0].position);
    Serial.println("), executing RAISE BACK + LEG ZERO...");

    // 启动电机1反转（起背）
    motorReverse(1);
    setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

    // 启动电机2移动到0
    bool motor2_done = (motors[2].position == 0);
    bool motor2_forward = false;

    if (!motor2_done) {
      if (motors[2].position < 0) {
        motorForward(2);
        motor2_forward = true;
        Serial.println("=> Motor2 moving forward to 0");
      } else {
        motorReverse(2);
        motor2_forward = false;
        Serial.println("=> Motor2 moving reverse to 0");
      }
      setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
    } else {
      Serial.println("=> Motor2 already at position 0");
    }

    // 等待电机1到达极限位置（2秒内位置不变则认为到达极限）
    unsigned long lastPosChange = millis();
    int16_t lastPos = motors[1].position;
    bool motor1_done = false;

    // 等待两个电机都到达目标位置
    while (!motor1_done || !motor2_done) {
      delay(50);
      updateCurrentReading(1);
      updateCurrentReading(2);
      drawOLED();

      // 检查电机1是否到达极限位置
      if (!motor1_done) {
        if (motors[1].position != lastPos) {
          lastPos = motors[1].position;
          lastPosChange = millis();
        }

        if (millis() - lastPosChange >= 2000) {
          motorStop(1);
          setPCA9685PWM(motors[1].pwmChannel, 0);
          motor1_done = true;
          Serial.println("=> Motor1 reached limit position");
        }
      }

      // 检查电机2是否到达目标位置
      if (!motor2_done) {
        if ((motor2_forward && motors[2].position >= 0) ||
            (!motor2_forward && motors[2].position <= 0)) {
          motorStop(2);
          setPCA9685PWM(motors[2].pwmChannel, 0);
          motor2_done = true;
          Serial.println("=> Motor2 reached position 0");
        }
      }
    }

    Serial.println("=> RAISE BACK + LEG ZERO completed");
    return;
  }

  // 如厕（三阶段顺序执行：M4→0, M3→0, M5→4500，同时并行执行坐起：M1反转+M2→0）
  else if (memcmp(cmd, cmdToilet, 8) == 0) {
    Serial.println("=> TOILET: Starting parallel execution...");

    // ===== 检查并启动坐起动作 =====
    bool sit_enabled = false;
    bool motor1_done = false;
    bool motor2_done = false;
    unsigned long motor1_lastPosChange = 0;
    int16_t motor1_lastPos = 0;
    bool motor2_forward = false;

    if (motors[0].position >= 4200 && motors[0].position <= 4800) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(motors[0].position);
      Serial.println("), starting SIT action (M1+M2)...");
      sit_enabled = true;

      // 启动M1反转（起背）
      motorReverse(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));
      motor1_lastPosChange = millis();
      motor1_lastPos = motors[1].position;

      // 启动M2→0
      motor2_done = (motors[2].position == 0);
      if (!motor2_done) {
        if (motors[2].position < 0) {
          motorForward(2);
          motor2_forward = true;
          Serial.println("=> Motor2 moving forward to 0");
        } else {
          motorReverse(2);
          motor2_forward = false;
          Serial.println("=> Motor2 moving reverse to 0");
        }
        setPCA9685PWM(motors[2].pwmChannel, pctToDuty(SPEED_PERCENT));
      } else {
        Serial.println("=> Motor2 already at position 0");
      }
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(motors[0].position);
      Serial.println(" (required: 4200-4800)");
      Serial.println("=> SIT action skipped");
    }

    // ===== 三阶段并行执行 =====
    int stage = 1;  // 1=M4, 2=M3, 3=M5, 4=完成
    bool motor4_done = false;
    bool motor3_done = false;
    bool motor5_done = false;
    bool motor4_forward = false;
    bool motor3_forward = false;
    bool motor5_forward = false;

    // 启动阶段1: M4→0
    Serial.println("=> Stage 1: Moving Motor4 to position 0...");
    motor4_done = (motors[4].position == 0);
    if (!motor4_done) {
      if (motors[4].position < 0) {
        motorForward(4);
        motor4_forward = true;
        Serial.println("=> Motor4 moving forward to 0");
      } else {
        motorReverse(4);
        motor4_forward = false;
        Serial.println("=> Motor4 moving reverse to 0");
      }
      setPCA9685PWM(motors[4].pwmChannel, pctToDuty(SPEED_PERCENT));
    } else {
      Serial.println("=> Motor4 already at position 0");
    }

    // ===== 主循环：同时监控所有电机 =====
    while (stage <= 3 || (sit_enabled && (!motor1_done || !motor2_done))) {
      delay(50);

      // 更新电流读数
      if (stage == 1 && !motor4_done) updateCurrentReading(4);
      if (stage == 2 && !motor3_done) updateCurrentReading(3);
      if (stage == 3 && !motor5_done) updateCurrentReading(5);
      if (sit_enabled && !motor1_done) updateCurrentReading(1);
      if (sit_enabled && !motor2_done) updateCurrentReading(2);
      drawOLED();

      // ===== 检查三阶段进度 =====
      if (stage == 1 && !motor4_done) {
        if ((motor4_forward && motors[4].position >= 0) ||
            (!motor4_forward && motors[4].position <= 0)) {
          motorStop(4);
          setPCA9685PWM(motors[4].pwmChannel, 0);
          motor4_done = true;
          Serial.println("=> Motor4 reached position 0");
          Serial.println("=> Stage 1 completed");

          // 启动阶段2: M3→0
          stage = 2;
          Serial.println("=> Stage 2: Moving Motor3 to position 0...");
          motor3_done = (motors[3].position == 0);
          if (!motor3_done) {
            if (motors[3].position < 0) {
              motorForward(3);
              motor3_forward = true;
              Serial.println("=> Motor3 moving forward to 0");
            } else {
              motorReverse(3);
              motor3_forward = false;
              Serial.println("=> Motor3 moving reverse to 0");
            }
            setPCA9685PWM(motors[3].pwmChannel, pctToDuty(SPEED_PERCENT));
          } else {
            Serial.println("=> Motor3 already at position 0");
          }
        }
      }
      else if (stage == 2 && !motor3_done) {
        if ((motor3_forward && motors[3].position >= 0) ||
            (!motor3_forward && motors[3].position <= 0)) {
          motorStop(3);
          setPCA9685PWM(motors[3].pwmChannel, 0);
          motor3_done = true;
          Serial.println("=> Motor3 reached position 0");
          Serial.println("=> Stage 2 completed");

          // 启动阶段3: M5→4500
          stage = 3;
          Serial.println("=> Stage 3: Moving Motor5 to position 4500...");
          motor5_done = (motors[5].position == 4500);
          if (!motor5_done) {
            if (motors[5].position < 4500) {
              motorForward(5);
              motor5_forward = true;
              Serial.println("=> Motor5 moving forward to 4500");
            } else {
              motorReverse(5);
              motor5_forward = false;
              Serial.println("=> Motor5 moving reverse to 4500");
            }
            setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
          } else {
            Serial.println("=> Motor5 already at position 4500");
          }
        }
      }
      else if (stage == 3 && !motor5_done) {
        if ((motor5_forward && motors[5].position >= 4500) ||
            (!motor5_forward && motors[5].position <= 4500)) {
          motorStop(5);
          setPCA9685PWM(motors[5].pwmChannel, 0);
          motor5_done = true;
          Serial.println("=> Motor5 reached position 4500");
          Serial.println("=> Stage 3 completed");
          stage = 4;  // 三阶段完成
        }
      }

      // ===== 检查坐起进度 =====
      if (sit_enabled) {
        // 检查M1（极限位置检测）
        if (!motor1_done) {
          if (motors[1].position != motor1_lastPos) {
            motor1_lastPos = motors[1].position;
            motor1_lastPosChange = millis();
          }
          if (millis() - motor1_lastPosChange >= 2000) {
            motorStop(1);
            setPCA9685PWM(motors[1].pwmChannel, 0);
            motor1_done = true;
            Serial.println("=> Motor1 reached limit position (SIT)");
          }
        }

        // 检查M2（目标位置检测）
        if (!motor2_done) {
          if ((motor2_forward && motors[2].position >= 0) ||
              (!motor2_forward && motors[2].position <= 0)) {
            motorStop(2);
            setPCA9685PWM(motors[2].pwmChannel, 0);
            motor2_done = true;
            Serial.println("=> Motor2 reached position 0 (SIT)");
          }
        }
      }
    }

    Serial.println("=> TOILET sequence completed successfully (all actions finished)");
    return;
  }

  // 结束如厕（三阶段顺序执行：M5→0, M3→11800, M4→7300）
  else if (memcmp(cmd, cmdEndToilet, 8) == 0) {
    Serial.println("=> END TOILET: Starting 3-stage sequence...");

    // ===== 阶段1：M5移动到位置0 =====
    Serial.println("=> Stage 1: Moving Motor5 to position 0...");

    bool motor5_done = (motors[5].position == 0);
    bool motor5_forward = false;

    if (!motor5_done) {
      if (motors[5].position < 0) {
        motorForward(5);
        motor5_forward = true;
        Serial.println("=> Motor5 moving forward to 0");
      } else {
        motorReverse(5);
        motor5_forward = false;
        Serial.println("=> Motor5 moving reverse to 0");
      }
      setPCA9685PWM(motors[5].pwmChannel, pctToDuty(SPEED_PERCENT));
    } else {
      Serial.println("=> Motor5 already at position 0");
    }

    // 等待M5到达位置0
    while (!motor5_done) {
      delay(50);
      updateCurrentReading(5);
      drawOLED();

      if ((motor5_forward && motors[5].position >= 0) ||
          (!motor5_forward && motors[5].position <= 0)) {
        motorStop(5);
        setPCA9685PWM(motors[5].pwmChannel, 0);
        motor5_done = true;
        Serial.println("=> Motor5 reached position 0");
      }
    }

    Serial.println("=> Stage 1 completed");

    // ===== 阶段2：M3移动到位置11800 =====
    Serial.println("=> Stage 2: Moving Motor3 to position 11800...");

    bool motor3_done = (motors[3].position == 11800);
    bool motor3_forward = false;

    if (!motor3_done) {
      if (motors[3].position < 11800) {
        motorForward(3);
        motor3_forward = true;
        Serial.println("=> Motor3 moving forward to 11800");
      } else {
        motorReverse(3);
        motor3_forward = false;
        Serial.println("=> Motor3 moving reverse to 11800");
      }
      setPCA9685PWM(motors[3].pwmChannel, pctToDuty(SPEED_PERCENT));
    } else {
      Serial.println("=> Motor3 already at position 11800");
    }

    // 等待M3到达位置11800
    while (!motor3_done) {
      delay(50);
      updateCurrentReading(3);
      drawOLED();

      if ((motor3_forward && motors[3].position >= 11800) ||
          (!motor3_forward && motors[3].position <= 11800)) {
        motorStop(3);
        setPCA9685PWM(motors[3].pwmChannel, 0);
        motor3_done = true;
        Serial.println("=> Motor3 reached position 11800");
      }
    }

    Serial.println("=> Stage 2 completed");

    // ===== 阶段3：M4移动到位置7300 =====
    Serial.println("=> Stage 3: Moving Motor4 to position 7300...");

    bool motor4_done = (motors[4].position == 7300);
    bool motor4_forward = false;

    if (!motor4_done) {
      if (motors[4].position < 7300) {
        motorForward(4);
        motor4_forward = true;
        Serial.println("=> Motor4 moving forward to 7300");
      } else {
        motorReverse(4);
        motor4_forward = false;
        Serial.println("=> Motor4 moving reverse to 7300");
      }
      setPCA9685PWM(motors[4].pwmChannel, pctToDuty(SPEED_PERCENT));
    } else {
      Serial.println("=> Motor4 already at position 7300");
    }

    // 等待M4到达位置7300
    while (!motor4_done) {
      delay(50);
      updateCurrentReading(4);
      drawOLED();

      if ((motor4_forward && motors[4].position >= 7300) ||
          (!motor4_forward && motors[4].position <= 7300)) {
        motorStop(4);
        setPCA9685PWM(motors[4].pwmChannel, 0);
        motor4_done = true;
        Serial.println("=> Motor4 reached position 7300");
      }
    }

    Serial.println("=> Stage 3 completed");
    Serial.println("=> END TOILET sequence completed successfully");
    return;
  }

  // 未知指令
  Serial.println("=> Unknown command");
}

void handleScreenCommands() {
  while (SERIAL_SCREEN.available()) {
    byte receivedByte = SERIAL_SCREEN.read();

    // 检测帧头 0xEE
    if (receivedByte == 0xEE) {
      cmdBuffer[0] = receivedByte;
      cmdIndex = 1;
      unsigned long startTime = millis();

      // 读取剩余7字节，带20ms超时保护
      while (cmdIndex < 8) {
        if (SERIAL_SCREEN.available()) {
          cmdBuffer[cmdIndex++] = SERIAL_SCREEN.read();
        }
        if (millis() - startTime > 20) {
          cmdIndex = 0;
          return;
        }
      }

      // 收到完整8字节后处理
      if (cmdIndex == 8) {
        processCommand(cmdBuffer);

        // 回传命令确认
        SERIAL_SCREEN.write(cmdBuffer, 8);
        SERIAL_SCREEN.flush();
      }
      cmdIndex = 0;
    }
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

  // Motor2 (PA4, PA5)
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinA), ISR_motor2_AB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinB), ISR_motor2_AB, CHANGE);

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