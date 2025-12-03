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
#define PIN_165_QH     PA4  // QH作为中断引脚
#define PIN_165_CLK    PA5
#define PIN_165_SHLD   PA6

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

#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
const uint8_t OLED_ADDR = 0x3C;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

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

const uint8_t SPEED_PERCENT = 90;
inline uint8_t pctToDuty(uint8_t p){ return (uint8_t)((p * 255UL) / 100UL); }

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

// ====== 霍尔/脉冲统计（165的G/H口）- 正交解码 ======
volatile int32_t position = 0;        // 相对位置（可正可负）
volatile uint32_t g_count = 0;        // G相(A相)原始计数
volatile uint32_t h_count = 0;        // H相(B相)原始计数
uint8_t last_g = 0;                   // 上次G相状态
uint8_t last_h = 0;                   // 上次H相状态
volatile uint32_t interrupt_count = 0; // 中断触发计数（调试用）

// 165中断服务函数 - 只递增计数器
void ISR_165() {
  interrupt_count++;  // 记录中断次数
}

// ====== 165 信号变化检测（H/G/F = 限位开关）======
uint8_t last_hgf = 0xFF;  // 上次的 HGF 值
uint32_t hgf_change_count = 0;  // HGF 变化次数
uint32_t hgf_history[10];  // 记录最近10次变化的时间戳
uint8_t hgf_values[10];    // 记录最近10次的 HGF 值
uint8_t history_index = 0;

// 频率统计用
uint32_t last_ms = 0;
uint32_t last_g_cnt = 0, last_h_cnt = 0;
float    freq_pb10 = 0.0f, freq_pb11 = 0.0f;  // 保留变量名用于显示

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

  // PA4配置为输入（用于接收165的QH中断信号）
  pinMode(PIN_165_QH, INPUT);

  // 595 初始化
  digitalWrite(PIN_SER,   LOW);
  digitalWrite(PIN_RCLK,  LOW);
  digitalWrite(PIN_SRCLK, LOW);
  digitalWrite(PIN_OE, HIGH);
  shiftOut16(0x00, 0x00);
  digitalWrite(PIN_OE, LOW);

  // 上电默认停止电机
  // motorStop();
  // analogWrite(PIN_PWM, 0);

  motorReverse();
  analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));  // 测试：注释PWM输出

  // motorForward();
  // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));

  // OLED 初始化
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Hall Monitor");
  display.display();

  // 初始化last_g和last_h状态
  uint8_t initial_hgf = read165_HGF();
  last_g = (initial_hgf >> 1) & 0x01;
  last_h = (initial_hgf >> 2) & 0x01;

  // 不使用中断，改用高频轮询
  // attachInterrupt(digitalPinToInterrupt(PIN_165_QH), ISR_165, CHANGE);

  last_ms = millis();
}

// OLED显示函数 - 显示霍尔脉冲和位置
void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);

  // 第一行：相对位置
  display.setCursor(0, 0);
  display.print("Position: ");
  display.println(position);

  // 第二行：G相计数
  display.setCursor(0, 12);
  display.print("G: ");
  display.println(g_count);

  // 第三行：H相计数
  display.setCursor(0, 24);
  display.print("H: ");
  display.println(h_count);

  // 第四行：G相频率
  display.setCursor(0, 36);
  display.print("FreqG: ");
  display.print(freq_pb10, 1);
  display.println(" Hz");

  // 第五行：H相频率
  display.setCursor(0, 48);
  display.print("FreqH: ");
  display.print(freq_pb11, 1);
  display.println(" Hz");

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
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 电机停止控制
  else if (memcmp(cmd, motorStopCmd, 8) == 0) {
    Serial.println("=> Motor STOP");
    motorStop();
    // analogWrite(PIN_PWM, 0);  // 测试：注释PWM输出
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 电机反转控制
  else if (memcmp(cmd, motorReverseCmd, 8) == 0) {
    Serial.println("=> Motor REVERSE");
    motorReverse();
    // analogWrite(PIN_PWM, pctToDuty(SPEED_PERCENT));  // 测试：注释PWM输出
    sendHex485(cmd);  // 返回接收到的原指令
  }
  // 未知指令不响应
  else {
    Serial.println("ERROR: Unknown command");
  }
}

// 调试变量
uint32_t debug_loop_count = 0;
uint32_t last_debug_time = 0;

void loop() {
  // 处理串口屏指令（最高优先级，无阻塞）
  handleScreenCommands();

  // ==== 高频轮询读取165（霍尔编码+限位检测）====
  uint8_t hgf = read165_HGF();

  uint8_t current_g = (hgf >> 1) & 0x01;  // 提取G位 (bit1)
  uint8_t current_h = (hgf >> 2) & 0x01;  // 提取H位 (bit2)

  // 调试输出（每秒一次）
  debug_loop_count++;
  if (millis() - last_debug_time >= 1000) {
    Serial.print("HGF=0b");
    Serial.print(hgf, BIN);
    Serial.print(" G=");
    Serial.print(current_g);
    Serial.print(" H=");
    Serial.print(current_h);
    Serial.print(" last_g=");
    Serial.print(last_g);
    Serial.print(" last_h=");
    Serial.print(last_h);
    Serial.print(" g_count=");
    Serial.print(g_count);
    Serial.print(" h_count=");
    Serial.print(h_count);
    Serial.print(" loops/s=");
    Serial.println(debug_loop_count);
    debug_loop_count = 0;
    last_debug_time = millis();
  }

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

  // 限位检测
  if (hgf != last_hgf && last_hgf != 0xFF) {
    hgf_change_count++;
    hgf_history[history_index] = millis();
    hgf_values[history_index] = hgf;
    history_index = (history_index + 1) % 10;
  }
  last_hgf = hgf;

  // 计算霍尔脉冲频率
  uint32_t now_ms = millis();
  if (now_ms - last_ms >= 1000) {  // 每秒更新一次频率
    uint32_t dt = now_ms - last_ms;
    freq_pb10 = (g_count - last_g_cnt) * 1000.0f / dt;
    freq_pb11 = (h_count - last_h_cnt) * 1000.0f / dt;
    last_g_cnt = g_count;
    last_h_cnt = h_count;
    last_ms = now_ms;
  }

  // OLED 显示更新（每100ms刷新一次）
  unsigned long currentTime = millis();
  if (currentTime - lastOLEDUpdate >= 100) {
    drawOLED();
    lastOLEDUpdate = currentTime;
  }

  // 移除阻塞延时，改为非阻塞方式，提高串口响应速度
}
