/* STM32F103C8T6 + 74HC595 x2 + 74HC165(H,G,F)
   165:  PA4=QH, PA5=CLK, PA6=SH/LD#
   595:  PA0 -> SER, PA1 -> OE#(低有效), PA2 -> RCLK, PA3 -> SRCLK
   U34.QH' -> U33.SER
   PB0 -> PWM
   这里把 PB10 / PB11 当成来自电机的霍尔/脉冲输入
*/

#define PIN_SER    PA0
#define PIN_OE     PA1
#define PIN_RCLK   PA2
#define PIN_SRCLK  PA3
#define PIN_PWM    PB0
#define PIN_PB10   PB10
#define PIN_PB11   PB11

// ---- 74HC165 ----
#define PIN_165_QH     PA4
#define PIN_165_CLK    PA5
#define PIN_165_SHLD   PA6

// ---- ADC电流采样 ----
#define PIN_ADC_CURRENT PA7       // 电流采样引脚
#define SHUNT_RESISTANCE 0.05     // 采样电阻 R126 = 50mΩ = 0.05Ω
#define CURRENT_SAMPLE_COUNT 99    // 中值滤波采样次数（建议奇数）
#define CURRENT_UPDATE_INTERVAL 1000  // 电流更新间隔(ms)如果是500的话，可能会有零点误差（与单片机本身有关系）

// ---- 串口屏通讯(RS485) ----
#define SERIAL_SCREEN Serial1  // PA9(TX), PA10(RX)

// 8字节十六进制电机控制命令定义
const byte motorForwardCmd[8] = {0xEE, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
const byte motorStopCmd[8]    = {0xEE, 0x02, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const byte motorReverseCmd[8] = {0xEE, 0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00};

// RS485接收缓冲区
byte rs485Buffer[8];
int rs485Index = 0;

// OLED显示
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>  // PCA9685库

#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
const uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// PCA9685 PWM控制器
#define PCA9685_ADDR 0x40        // PCA9685默认I2C地址
#define PCA9685_FREQ 1600        // PWM频率 1600Hz（PCA9685最大频率）
#define PCA9685_MOTOR_CHANNEL 0  // 电机PWM使用第0通道
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ===== 595 脉冲 =====
inline void pulseSRCLK() { digitalWrite(PIN_SRCLK, HIGH); digitalWrite(PIN_SRCLK, LOW); }
inline void pulseRCLK()  { digitalWrite(PIN_RCLK,  HIGH); digitalWrite(PIN_RCLK,  LOW); }

// ===== 发送 16 位到 74HC595（高字节先发）=====
void shiftOut16(uint8_t highByte, uint8_t lowByte) {
  digitalWrite(PIN_OE, LOW); // 使能输出
  for (int8_t i = 7; i >= 0; --i) { digitalWrite(PIN_SER, (highByte >> i) & 0x01); pulseSRCLK(); }
  for (int8_t i = 7; i >= 0; --i) { digitalWrite(PIN_SER, (lowByte  >> i) & 0x01); pulseSRCLK(); }
  pulseRCLK();
}
inline void motorForward() { shiftOut16(0x00, 0x01); } // 示例
inline void motorReverse() { shiftOut16(0x01, 0x00); }
inline void motorStop()    { shiftOut16(0x00, 0x00);  }

const uint8_t SPEED_PERCENT = 100;  // 100%占空比
inline uint8_t pctToDuty(uint8_t p){ return (uint8_t)((p * 255UL) / 100UL); }

// ===== PCA9685 PWM 控制函数 =====
// 设置PCA9685指定通道的PWM占空比
// channel: 通道号 (0-15)
// dutyCycle: 占空比 (0-255)，0=关闭，255=全开
void setPCA9685PWM(uint8_t channel, uint8_t dutyCycle) {
  if (channel > 15) return;  // 通道范围检查

  // PCA9685使用12位分辨率 (0-4095)
  // 将8位占空比 (0-255) 转换为12位 (0-4095)
  uint16_t pwmValue = (uint16_t)((dutyCycle * 4095UL) / 255UL);

  // setPWM(通道, 开始时间, 结束时间)
  // 开始时间=0，结束时间=pwmValue，实现占空比控制
  pca9685.setPWM(channel, 0, pwmValue);
}

// ===== 中值+均值滤波采样电流 =====
float sampleCurrentFiltered() {
  float samples[CURRENT_SAMPLE_COUNT];

  // 采集多个样本
  for (int i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    int adc_value = analogRead(PIN_ADC_CURRENT);
    float voltage = (adc_value / 4095.0) * 3.3;
    samples[i] = voltage / SHUNT_RESISTANCE;
    delayMicroseconds(100);  // 样本间隔100us
  }

  // 冒泡排序
  for (int i = 0; i < CURRENT_SAMPLE_COUNT - 1; i++) {
    for (int j = 0; j < CURRENT_SAMPLE_COUNT - i - 1; j++) {
      if (samples[j] > samples[j + 1]) {
        float temp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = temp;
      }
    }
  }

  // 去掉最大最小值，计算中间值的平均
  float sum = 0;
  int start = 1;  // 去掉最小值
  int end = CURRENT_SAMPLE_COUNT - 1;  // 去掉最大值
  for (int i = start; i < end; i++) {
    sum += samples[i];
  }

  return sum / (end - start);
}

// ===== 读取 74HC165 的 H、G、F 三位 =====
uint8_t read165_HGF() {
  digitalWrite(PIN_165_SHLD, LOW);            // 并行装载
  delayMicroseconds(1);
  digitalWrite(PIN_165_SHLD, HIGH);           // 进入移位
  uint8_t H = digitalRead(PIN_165_QH);        // 此时为 H
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  uint8_t G = digitalRead(PIN_165_QH);        // 移到 G
  digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  uint8_t F = digitalRead(PIN_165_QH);        // 移到 F
  return (uint8_t)((H<<2)|(G<<1)|(F<<0));     // bit2=H, bit1=G, bit0=F
}

// ====== 霍尔/脉冲统计（PB10 / PB11）- 正交解码 ======
volatile int32_t position = 0;        // 相对位置（可正可负）
volatile uint32_t pb10_count = 0;     // A相原始计数
volatile uint32_t pb11_count = 0;     // B相原始计数

// 正交解码中断函数
void ISR_pb10() {
  pb10_count++;
  // A相变化时，读取B相电平判断方向
  if (digitalRead(PIN_PB10) == digitalRead(PIN_PB11)) {
    position++;  // 正转
  } else {
    position--;  // 反转
  }
}

void ISR_pb11() {
  pb11_count++;
}

// ====== 165 信号变化检测（H/G/F = 限位开关）======
uint8_t last_hgf = 0xFF;  // 上次的 HGF 值
uint32_t hgf_change_count = 0;  // HGF 变化次数
uint32_t hgf_history[10];  // 记录最近10次变化的时间戳
uint8_t hgf_values[10];    // 记录最近10次的 HGF 值
uint8_t history_index = 0;

// 频率统计用
uint32_t last_ms = 0;
uint32_t last_pb10_cnt = 0, last_pb11_cnt = 0;
float    freq_pb10 = 0.0f, freq_pb11 = 0.0f;

// 电流采样变量
float current_amps = 0.0f;  // 当前电流值（A）
unsigned long lastCurrentUpdate = 0;  // 电流采样计时器

// OLED 刷新时间控制（非阻塞）
unsigned long lastOLEDUpdate = 0;

void setup() {
  // 调试串口初始化
  Serial.begin(9600);
  delay(1000);
  Serial.println("Motor Control System Starting...");

  // 串口屏通讯初始化 (RS485)
  SERIAL_SCREEN.begin(9600);     // Serial1默认使用PA9(TX)/PA10(RX)

  // 清空串口缓冲区，防止残留数据干扰
  delay(100);
  while(SERIAL_SCREEN.available()) {
    SERIAL_SCREEN.read();
  }

  Serial.println("RS485 Initialized (Serial1)");

  // 595
  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_OE,    OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_PWM,   OUTPUT);

  // 165
  pinMode(PIN_165_QH,   INPUT);
  pinMode(PIN_165_CLK,  OUTPUT);
  pinMode(PIN_165_SHLD, OUTPUT);
  digitalWrite(PIN_165_CLK,  LOW);
  digitalWrite(PIN_165_SHLD, HIGH);

  // ADC电流采样引脚初始化
  pinMode(PIN_ADC_CURRENT, INPUT_ANALOG);

  // 霍尔/脉冲输入（启用上拉，兼容开漏输出）
  pinMode(PIN_PB10, INPUT_PULLUP);
  pinMode(PIN_PB11, INPUT_PULLUP);

  // 595 初始化
  digitalWrite(PIN_SER,   LOW);
  digitalWrite(PIN_RCLK,  LOW);
  digitalWrite(PIN_SRCLK, LOW);
  digitalWrite(PIN_OE, HIGH);
  shiftOut16(0x00, 0x00);
  digitalWrite(PIN_OE, LOW);

  // I2C和OLED初始化（必须先初始化I2C）
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Hall Monitor");
  display.display();

  // PCA9685 初始化（必须在使用之前初始化）
  pca9685.begin();
  pca9685.setPWMFreq(PCA9685_FREQ);  // 设置PWM频率为1600Hz
  delay(10);
  Serial.println("PCA9685 Initialized at 1600Hz");

  // 上电默认停止电机
  // motorStop();
  // analogWrite(PIN_PWM, 0);

  motorReverse();
 // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));  // 测试：注释PWM输出
  setPCA9685PWM(PCA9685_MOTOR_CHANNEL, pctToDuty(SPEED_PERCENT));  // PCA9685输出90%占空比PWM

  // motorForward();
  // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));

  // 中断计数：检测所有跳变（CHANGE），兼容推挽和开漏
  attachInterrupt(digitalPinToInterrupt(PIN_PB10), ISR_pb10, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_PB11), ISR_pb11, CHANGE);

  last_ms = millis();
}

// OLED显示函数 - 显示霍尔脉冲和位置
void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);  // 使用小字体以显示6行内容

  // 第一行：相对位置
  display.setCursor(0, 0);
  display.print("Pos: ");
  display.println(position);

  // 第二行：PB10计数
  display.setCursor(0, 10);
  display.print("PB10: ");
  display.println(pb10_count);

  // 第三行：PB11计数
  display.setCursor(0, 20);
  display.print("PB11: ");
  display.println(pb11_count);

  // 第四行：PB10频率
  display.setCursor(0, 30);
  display.print("F10: ");
  display.print(freq_pb10, 1);
  display.println("Hz");

  // 第五行：PB11频率
  display.setCursor(0, 40);
  display.print("F11: ");
  display.print(freq_pb11, 1);
  display.println("Hz");

  // 第六行：电流显示
  display.setCursor(0, 50);
  display.print("I: ");
  display.print(current_amps, 2);  // 显示2位小数
  display.println("A");

  display.display();
}

// ========== 串口屏通讯功能 ==========

// RS485发送16进制数据函数（参照 store22.txt）
void sendHex485(byte data[8]) {
  SERIAL_SCREEN.write(data, 8);    // 发送8字节数据
  SERIAL_SCREEN.flush();           // 等待发送完成

  Serial.print("Sent: ");
  for (int i = 0; i < 8; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// ===== 串口屏指令处理（完全参照 store22.txt 的 handleRS485Commands）=====
void handleScreenCommands() {
  while (SERIAL_SCREEN.available()) {
    byte receivedByte = SERIAL_SCREEN.read();

    // 查找帧头 0xEE
    if (receivedByte == 0xEE) {
      Serial.println("Frame header 0xEE detected");
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;

      // 尝试接收剩余7字节，带超时机制
      unsigned long startTime = millis();
      while (rs485Index < 8) {
        if (SERIAL_SCREEN.available()) {
          rs485Buffer[rs485Index] = SERIAL_SCREEN.read();
          rs485Index++;
        }
        // 超时检测（50ms）
        if (millis() - startTime > 50) {
          Serial.print("TIMEOUT at byte ");
          Serial.println(rs485Index);
          rs485Index = 0;
          return;
        }
        delayMicroseconds(100); // 给数据传输时间
      }

      // 如果成功接收到8字节，处理命令
      if (rs485Index == 8) {
        // 打印接收到的十六进制数据
        Serial.print("Recv: ");
        for (int i = 0; i < 8; i++) {
          if (rs485Buffer[i] < 16) Serial.print("0");
          Serial.print(rs485Buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // 处理8字节命令
        processHexCommand(rs485Buffer);
      }

      // 重置缓冲区
      rs485Index = 0;
    }
    // 如果不是帧头，打印并继续查找
    else {
      Serial.print("Skip byte: 0x");
      if (receivedByte < 16) Serial.print("0");
      Serial.println(receivedByte, HEX);
    }
  }
}

// 8字节十六进制指令解析与执行（完全参照 store22.txt 的 processHexCommand）
void processHexCommand(byte cmd[8]) {
  // 检查帧头是否为0xEE
  if (cmd[0] != 0xEE) {
    Serial.println("ERROR: Invalid header");
    return;
  }

  // 电机正转控制
  if (memcmp(cmd, motorForwardCmd, 8) == 0) {
    Serial.println("=> Motor FORWARD");
    motorForward();
    // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));  // 测试：注释PWM输出
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, pctToDuty(SPEED_PERCENT));  // PCA9685输出90%占空比PWM
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 电机停止控制
  else if (memcmp(cmd, motorStopCmd, 8) == 0) {
    Serial.println("=> Motor STOP");
    motorStop();
    // analogWrite(PIN_PWM, 0);  // 测试：注释PWM输出
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, 0);  // PCA9685输出0%占空比（停止）
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 电机反转控制
  else if (memcmp(cmd, motorReverseCmd, 8) == 0) {
    Serial.println("=> Motor REVERSE");
    motorReverse();
    // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));  // 测试：注释PWM输出
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, pctToDuty(SPEED_PERCENT));  // PCA9685输出90%占空比PWM
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 未知指令不响应
  else {
    Serial.println("ERROR: Unknown command");
  }
}

void loop() {
  // 处理串口屏指令（最高优先级，无阻塞）
  handleScreenCommands();

  // ==== 高频读取 165（检测限位开关变化）====
  uint8_t hgf = read165_HGF();

  // 检测 HGF 变化
  if (hgf != last_hgf && last_hgf != 0xFF) {
    hgf_change_count++;
    // 记录到历史
    hgf_history[history_index] = millis();
    hgf_values[history_index] = hgf;
    history_index = (history_index + 1) % 10;
  }
  last_hgf = hgf;

  // 计算霍尔脉冲频率
  uint32_t now_ms = millis();
  if (now_ms - last_ms >= 1000) {  // 每秒更新一次频率
    uint32_t dt = now_ms - last_ms;
    freq_pb10 = (pb10_count - last_pb10_cnt) * 1000.0f / dt;
    freq_pb11 = (pb11_count - last_pb11_cnt) * 1000.0f / dt;
    last_pb10_cnt = pb10_count;
    last_pb11_cnt = pb11_count;
    last_ms = now_ms;
  }

  // 定时采样电流（每300ms采样一次，使用中值+均值滤波）
  if (millis() - lastCurrentUpdate >= CURRENT_UPDATE_INTERVAL) {
    current_amps = sampleCurrentFiltered();
    lastCurrentUpdate = millis();
  }

  // OLED 显示更新（每100ms刷新一次）
  unsigned long currentTime = millis();
  if (currentTime - lastOLEDUpdate >= 100) {
    drawOLED();
    lastOLEDUpdate = currentTime;
  }

  // 移除阻塞延时，改为非阻塞方式，提高串口响应速度
}