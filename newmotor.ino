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
    0, 0, 0,                // position, hallA_count, hallB_count
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
    0, 0, 0,
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
    0, 0, 0,
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
    0, 0, 0,
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
    0, 0, 0,
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
    0, 0, 0,
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
    0, 0, 0,
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

byte cmdBuffer[2];  // 2字节命令缓冲区：[电机号, 动作]
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

// Motor0 (PA0, PA1)
void ISR_motor0_A() {
  motors[0].hallA_count++;
  if (digitalRead(motors[0].hallPinA) == digitalRead(motors[0].hallPinB)) {
    motors[0].position--;
  } else {
    motors[0].position++;
  }
}
void ISR_motor0_B() {
  motors[0].hallB_count++;
}

// Motor1 (PA2, PA3)
void ISR_motor1_A() {
  motors[1].hallA_count++;
  if (digitalRead(motors[1].hallPinA) == digitalRead(motors[1].hallPinB)) {
    motors[1].position--;
  } else {
    motors[1].position++;
  }
}
void ISR_motor1_B() {
  motors[1].hallB_count++;
}

// Motor2 (PA4, PA5)
void ISR_motor2_A() {
  motors[2].hallA_count++;
  if (digitalRead(motors[2].hallPinA) == digitalRead(motors[2].hallPinB)) {
    motors[2].position--;
  } else {
    motors[2].position++;
  }
}
void ISR_motor2_B() {
  motors[2].hallB_count++;
}

// Motor3 (PA6, PA7)
void ISR_motor3_A() {
  motors[3].hallA_count++;
  if (digitalRead(motors[3].hallPinA) == digitalRead(motors[3].hallPinB)) {
    motors[3].position++;
  } else {
    motors[3].position--;
  }
}
void ISR_motor3_B() {
  motors[3].hallB_count++;
}

// Motor4 (PB8, PB9)
void ISR_motor4_A() {
  motors[4].hallA_count++;
  if (digitalRead(motors[4].hallPinA) == digitalRead(motors[4].hallPinB)) {
    motors[4].position++;
  } else {
    motors[4].position--;
  }
}
void ISR_motor4_B() {
  motors[4].hallB_count++;
}

// Motor5 (PB10, PB11)
void ISR_motor5_A() {
  motors[5].hallA_count++;
  if (digitalRead(motors[5].hallPinA) == digitalRead(motors[5].hallPinB)) {
    motors[5].position++;
  } else {
    motors[5].position--;
  }
}
void ISR_motor5_B() {
  motors[5].hallB_count++;
}

// Motor6 (PB12, PB15)
void ISR_motor6_A() {
  motors[6].hallA_count++;
  if (digitalRead(motors[6].hallPinA) == digitalRead(motors[6].hallPinB)) {
    motors[6].position++;
  } else {
    motors[6].position--;
  }
}
void ISR_motor6_B() {
  motors[6].hallB_count++;
}

// ================= 指令解析与执行 =================

void processCommand(byte motorIndex, byte action) {
  // 特殊指令：左翻身 0x08 0x00
  if (motorIndex == 0x08 && action == 0x00) {
    Serial.println("=> LEFT TURN: Checking Motor1 position...");

    // 检查电机1位置是否在3250-3450范围内
    if (motors[1].position < 3250 || motors[1].position > 3450) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(motors[1].position);
      Serial.println(" (required: 3250-3450)");
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

  // 特殊指令：右翻身 0x08 0x01
  if (motorIndex == 0x08 && action == 0x01) {
    Serial.println("=> RIGHT TURN: Checking Motor1 position...");

    // 检查电机1位置是否在3250-3450范围内
    if (motors[1].position < 3250 || motors[1].position > 3450) {
      Serial.print("=> Motor1 position out of range: ");
      Serial.print(motors[1].position);
      Serial.println(" (required: 3250-3450)");
      Serial.println("=> RIGHT TURN: Command ignored");
      return;
    }

    Serial.print("=> Motor1 position OK (");
    Serial.print(motors[1].position);
    Serial.println("), executing RIGHT TURN...");

    if (motors[0].position < 3100) {
      motorForward(0);
      setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));

      // 等待到达目标位置
      while (motors[0].position < 3100) {
        delay(50);
        updateCurrentReading(0);
        drawOLED();  // 刷新显示
      }

      // 停止电机
      motorStop(0);
      setPCA9685PWM(motors[0].pwmChannel, 0);
      Serial.println("=> RIGHT TURN completed, Motor0 reached position 3100");
    } else {
      Serial.println("=> Motor0 already at or beyond position 3100");
    }
    return;
  }

  // 特殊指令：平躺 0x08 0x02
  if (motorIndex == 0x08 && action == 0x02) {
    Serial.println("=> FLAT: Checking bed state...");

    bool motor0_in_range = (motors[0].position >= 2000 && motors[0].position <= 2200);
    bool motor1_in_range = (motors[1].position >= 3250 && motors[1].position <= 3450);

    // 情况1：两个条件都满足 - 只需要移动电机2
    if (motor0_in_range && motor1_in_range) {
      Serial.println("=> State: Both turn and raise are homed, only moving Motor2...");

      bool motor2_done = (motors[2].position == 2700);
      bool motor2_forward = false;

      if (!motor2_done) {
        if (motors[2].position < 2700) {
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

        if ((motor2_forward && motors[2].position >= 2700) ||
            (!motor2_forward && motors[2].position <= 2700)) {
          motorStop(2);
          setPCA9685PWM(motors[2].pwmChannel, 0);
          motor2_done = true;
          Serial.println("=> Motor2 reached position 2700");
        }
      }

      Serial.println("=> FLAT completed");
      return;
    }

    // 情况2：状态1 - 翻身后归位（电机1在范围内）
    if (motor1_in_range) {
      Serial.println("=> State 1: After turn, moving Motor0 and Motor2...");

      bool motor0_done = (motors[0].position == 2100);
      bool motor2_done = (motors[2].position == 2700);
      bool motor0_forward = false;
      bool motor2_forward = false;

      if (!motor0_done) {
        if (motors[0].position < 2100) {
          motorForward(0);
          motor0_forward = true;
        } else {
          motorReverse(0);
          motor0_forward = false;
        }
        setPCA9685PWM(motors[0].pwmChannel, pctToDuty(SPEED_PERCENT));
      }

      if (!motor2_done) {
        if (motors[2].position < 2700) {
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
          if ((motor0_forward && motors[0].position >= 2100) ||
              (!motor0_forward && motors[0].position <= 2100)) {
            motorStop(0);
            setPCA9685PWM(motors[0].pwmChannel, 0);
            motor0_done = true;
            Serial.println("=> Motor0 reached position 2100");
          }
        }

        // 检查电机2是否到达
        if (!motor2_done) {
          if ((motor2_forward && motors[2].position >= 2700) ||
              (!motor2_forward && motors[2].position <= 2700)) {
            motorStop(2);
            setPCA9685PWM(motors[2].pwmChannel, 0);
            motor2_done = true;
            Serial.println("=> Motor2 reached position 2700");
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

      // 启动电机2移动到2700
      bool motor2_done = (motors[2].position == 2700);
      bool motor2_forward = false;

      if (!motor2_done) {
        if (motors[2].position < 2700) {
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
          if ((motor2_forward && motors[2].position >= 2700) ||
              (!motor2_forward && motors[2].position <= 2700)) {
            motorStop(2);
            setPCA9685PWM(motors[2].pwmChannel, 0);
            motor2_done = true;
            Serial.println("=> Motor2 reached position 2700");
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
    Serial.println(" (required: 2000-2200 for State 2)");
    Serial.print("=> Motor1 position: ");
    Serial.print(motors[1].position);
    Serial.println(" (required: 3250-3450 for State 1)");
    Serial.println("=> FLAT command ignored");
    return;
  }

  // 特殊指令：起背 0x08 0x03
  if (motorIndex == 0x08 && action == 0x03) {
    Serial.println("=> RAISE BACK: Checking Motor0 position...");

    // 检查电机0位置是否在2000-2200范围内
    if (motors[0].position >= 2000 && motors[0].position <= 2200) {
      Serial.print("=> Motor0 position OK (");
      Serial.print(motors[0].position);
      Serial.println("), Motor1 reversing...");

      motorReverse(1);
      setPCA9685PWM(motors[1].pwmChannel, pctToDuty(SPEED_PERCENT));

      Serial.println("=> RAISE BACK: Motor1 reversing (non-blocking)");
    } else {
      Serial.print("=> Motor0 position out of range: ");
      Serial.print(motors[0].position);
      Serial.println(" (required: 2000-2200)");
      Serial.println("=> RAISE BACK: Command ignored");
    }
    return;
  }

  // 检查电机编号有效性
  if (motorIndex >= 7) {
    Serial.print("Invalid motor index: ");
    Serial.println(motorIndex);
    return;
  }

  // 执行对应动作
  switch (action) {
    case CMD_FORWARD:
      Serial.print("=> Motor");
      Serial.print(motorIndex);
      Serial.println(" FORWARD");
      motorForward(motorIndex);
      setPCA9685PWM(motors[motorIndex].pwmChannel, pctToDuty(SPEED_PERCENT));
      break;

    case CMD_REVERSE:
      Serial.print("=> Motor");
      Serial.print(motorIndex);
      Serial.println(" REVERSE");
      motorReverse(motorIndex);
      setPCA9685PWM(motors[motorIndex].pwmChannel, pctToDuty(SPEED_PERCENT));
      break;

    case CMD_STOP:
      Serial.print("=> Motor");
      Serial.print(motorIndex);
      Serial.println(" STOP");
      motorStop(motorIndex);
      setPCA9685PWM(motors[motorIndex].pwmChannel, 0);
      break;

    case CMD_RESET_POS:
      Serial.print("=> Motor");
      Serial.print(motorIndex);
      Serial.println(" RESET POSITION TO 0");
      noInterrupts();  // 关闭中断
      motors[motorIndex].position = 0;
      motors[motorIndex].hallA_count = 0;
      motors[motorIndex].hallB_count = 0;
      interrupts();    // 恢复中断
      savePositionToEEPROM(motorIndex, 0);
      motors[motorIndex].lastSavedPosition = 0;
      motors[motorIndex].lastLoopPosition = 0;
      motors[motorIndex].last_hallA_cnt = 0;
      motors[motorIndex].last_hallB_cnt = 0;
      break;

    default:
      Serial.print("Invalid action: ");
      Serial.println(action);
      break;
  }
}

void handleScreenCommands() {
  while (SERIAL_SCREEN.available()) {
    byte receivedByte = SERIAL_SCREEN.read();

    cmdBuffer[cmdIndex++] = receivedByte;

    // 收到2字节后处理命令
    if (cmdIndex >= 2) {
      processCommand(cmdBuffer[0], cmdBuffer[1]);

      // 回传命令确认
      SERIAL_SCREEN.write(cmdBuffer, 2);
      SERIAL_SCREEN.flush();

      cmdIndex = 0;  // 重置索引
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

  // --- 6. 为所有7个电机开启中断 ---
  Serial.println("Attaching interrupts for all motors...");

  // Motor0 (PA0, PA1)
  attachInterrupt(digitalPinToInterrupt(motors[0].hallPinA), ISR_motor0_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[0].hallPinB), ISR_motor0_B, CHANGE);

  // Motor1 (PA2, PA3)
  attachInterrupt(digitalPinToInterrupt(motors[1].hallPinA), ISR_motor1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].hallPinB), ISR_motor1_B, CHANGE);

  // Motor2 (PA4, PA5)
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinA), ISR_motor2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].hallPinB), ISR_motor2_B, CHANGE);

  // Motor3 (PA6, PA7)
  attachInterrupt(digitalPinToInterrupt(motors[3].hallPinA), ISR_motor3_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[3].hallPinB), ISR_motor3_B, CHANGE);

  // Motor4 (PB8, PB9)
  attachInterrupt(digitalPinToInterrupt(motors[4].hallPinA), ISR_motor4_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[4].hallPinB), ISR_motor4_B, CHANGE);

  // Motor5 (PB10, PB11)
  attachInterrupt(digitalPinToInterrupt(motors[5].hallPinA), ISR_motor5_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[5].hallPinB), ISR_motor5_B, CHANGE);

  // Motor6 (PB12, PB15)
  attachInterrupt(digitalPinToInterrupt(motors[6].hallPinA), ISR_motor6_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[6].hallPinB), ISR_motor6_B, CHANGE);

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