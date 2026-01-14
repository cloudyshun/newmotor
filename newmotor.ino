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
#define PIN_PB10    PB10
#define PIN_PB11    PB11

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
float current_buffer[FILTER_N]; // 环形缓冲区
int buffer_index = 0;
float sum_current = 0.0;
bool buffer_filled = false;

// ---- 电机控制命令 ----
const byte motorForwardCmd[8] = {0xEE, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
const byte motorStopCmd[8]    = {0xEE, 0x02, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
const byte motorReverseCmd[8] = {0xEE, 0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00};

byte rs485Buffer[8];
int rs485Index = 0;

// ---- 外设对象 ----
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_RESET   -1
const uint8_t OLED_ADDR = 0x3C;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 600
#define PCA9685_MOTOR_CHANNEL 5
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ---- 全局变量 ----
volatile int16_t position = 0; // 相对位置（掉电会从 EEPROM 恢复）
volatile uint32_t pb10_count = 0;
volatile uint32_t pb11_count = 0;

// 自动保存相关变量
int16_t lastSavedPosition = 0;       // 上次写入 EEPROM 的值
int16_t lastLoopPosition = 0;        // 上次循环检测到的位置值
unsigned long lastPosChangeTime = 0; // 位置最后一次变化的时间戳

float current_amps = 0.0f;
unsigned long lastCurrentUpdate = 0;
unsigned long lastOLEDUpdate = 0;
uint32_t last_ms = 0;
uint32_t last_pb10_cnt = 0, last_pb11_cnt = 0;
float freq_pb10 = 0.0f, freq_pb11 = 0.0f;

const uint8_t SPEED_PERCENT = 100;

// ================= AT24C02 存储逻辑 =================

// 将 16 位 position 写入 EEPROM (2字节补码)
void savePositionToEEPROM(int16_t pos) {
  uint8_t data[2];
  data[0] = (pos >> 8) & 0xFF;  // 高字节
  data[1] = pos & 0xFF;          // 低字节

  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(ADDR_MOTOR1_POS);
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.endTransmission();
  delay(5); // AT24C02 写周期延时（必须！）
  Serial.print("Auto-Save Position: ");
  Serial.println(pos);
}

// 从 EEPROM 读取 16 位 position (2字节补码)
int16_t readPositionFromEEPROM() {
  uint8_t data[2] = {0, 0};
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(ADDR_MOTOR1_POS);
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

inline void motorForward() { shiftOut16(0x20, 0x00); }
inline void motorReverse() { shiftOut16(0x00, 0x20); }
inline void motorStop()    { shiftOut16(0x00, 0x00); }
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

float sampleCurrentFiltered() {
  float samples[CURRENT_SAMPLE_COUNT];
  for (int i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    int adc_value = analogRead(PIN_ADC_CURRENT);
    float voltage = (adc_value / 4095.0) * 3.3;
    samples[i] = voltage / SHUNT_RESISTANCE;
    delayMicroseconds(100);
  }
  for (int i = 0; i < CURRENT_SAMPLE_COUNT - 1; i++) {
    for (int j = 0; j < CURRENT_SAMPLE_COUNT - i - 1; j++) {
      if (samples[j] > samples[j + 1]) {
        float temp = samples[j]; samples[j] = samples[j + 1]; samples[j + 1] = temp;
      }
    }
  }
  float sum = 0;
  for (int i = 1; i < CURRENT_SAMPLE_COUNT - 1; i++) sum += samples[i];
  return sum / (CURRENT_SAMPLE_COUNT - 2);
}

// 替换掉原来的 sampleCurrentFiltered
void updateCurrentReading() {
    // 1. 读取一次 ADC (不延时)
    int adc_value = analogRead(PIN_ADC_CURRENT);
    float voltage = (adc_value / 4095.0) * 3.3;
    float new_amp = voltage / SHUNT_RESISTANCE;

    // 2. 递推平均算法 (减去最旧的，加上最新的)
    sum_current -= current_buffer[buffer_index]; // 减去旧值
    current_buffer[buffer_index] = new_amp;      // 存入新值
    sum_current += new_amp;                      // 加上新值

    // 3. 更新索引
    buffer_index++;
    if (buffer_index >= FILTER_N) {
        buffer_index = 0;
        buffer_filled = true;
    }

    // 4. 计算平均值并赋值给全局变量 current_amps
    if (buffer_filled) {
        current_amps = sum_current / FILTER_N;
    } else {
        current_amps = sum_current / buffer_index; // 还没填满时除以当前数量
    }
}

// ================= 中断处理 =================

void ISR_pb10() {
  pb10_count++;
  if (digitalRead(PIN_PB10) == digitalRead(PIN_PB11)) {
    position++;
  } else {
    position--;
  }
}

void ISR_pb11() {
  pb11_count++;
}

// ================= 指令解析与执行 =================

void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;

  if (memcmp(cmd, motorForwardCmd, 8) == 0) {
    Serial.println("=> Motor FORWARD");
    motorForward();
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, pctToDuty(SPEED_PERCENT));
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motorStopCmd, 8) == 0) {
    Serial.println("=> Motor STOP");
    motorStop();
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, 0);
    
    // 【修改】这里不再直接保存，改为在 Loop 中自动检测静止保存
    
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motorReverseCmd, 8) == 0) {
    Serial.println("=> Motor REVERSE");
    motorReverse();
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, pctToDuty(SPEED_PERCENT));
    sendHex485(cmd);
  }
}

void handleScreenCommands() {
  while (SERIAL_SCREEN.available()) {
    byte receivedByte = SERIAL_SCREEN.read();
    if (receivedByte == 0xEE) {
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      while (rs485Index < 8) {
        if (SERIAL_SCREEN.available()) {
          rs485Buffer[rs485Index++] = SERIAL_SCREEN.read();
        }
        if (millis() - startTime > 50) { rs485Index = 0; return; }
      }
      if (rs485Index == 8) processHexCommand(rs485Buffer);
      rs485Index = 0;
    }
  }
}

void sendHex485(byte data[8]) {
  SERIAL_SCREEN.write(data, 8);
  SERIAL_SCREEN.flush();
}

void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);   display.print("Pos: "); display.println(position);
  display.setCursor(0, 10);  display.print("PB10: "); display.println(pb10_count);
  display.setCursor(0, 20);  display.print("PB11: "); display.println(pb11_count);
  display.setCursor(0, 30);  display.print("F10: "); display.print(freq_pb10, 1); display.println("Hz");
  display.setCursor(0, 40);  display.print("F11: "); display.print(freq_pb11, 1); display.println("Hz");
  display.setCursor(0, 50);  display.print("I: "); display.print(current_amps, 2); display.println("A");
  
  // 可以在屏幕上显示一个状态，告诉用户数据已保存
  if (position == lastSavedPosition) {
    display.setCursor(100, 0); 
    display.print("SV"); // Saved
  }
  
  display.display();
}

// ================= 主程序 =================

void setup() {
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // 禁用 JTAG
  Serial.begin(9600);
  SERIAL_SCREEN.begin(9600);

  // --- 1. 先初始化所有引脚 ---
  pinMode(PIN_4051_S0, OUTPUT);
  pinMode(PIN_4051_S1, OUTPUT);
  pinMode(PIN_4051_S2, OUTPUT);
  select4051Channel(5);

  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_OE,    OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_ADC_CURRENT, INPUT_ANALOG);
  pinMode(PIN_PB10, INPUT_PULLUP);
  pinMode(PIN_PB11, INPUT_PULLUP);

  digitalWrite(PIN_OE, HIGH);
  shiftOut16(0x00, 0x00);
  digitalWrite(PIN_OE, LOW);

  // --- 2. 初始化 I2C 总线 ---
  Wire.begin(); 

  // --- 3. 初始化 OLED (先亮屏) ---
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

    // 初始化检测（首次上电会写入 MagicNumber 和默认值）
    initEEPROM();

    // 读取位置数据
    position = readPositionFromEEPROM();
    Serial.print("Load Position: ");
    Serial.println(position);
    display.print("Load Pos: ");
    display.println(position);
  } else {
    Serial.print("EEPROM Error: ");
    Serial.println(error);
    display.println("Memory: ERROR!");
    display.println("Check Wiring!");
    position = 0; // 读不到则重置为0
  }
  
  // 【关键】初始化自动保存相关的变量
  lastSavedPosition = position; 
  lastLoopPosition = position;
  lastPosChangeTime = millis();

  display.display(); 
  delay(1500);

  // --- 6. 开启中断 ---
  attachInterrupt(digitalPinToInterrupt(PIN_PB10), ISR_pb10, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_PB11), ISR_pb11, CHANGE);

  last_ms = millis();
}

void loop() {
  handleScreenCommands();

  // 频率统计
  uint32_t now_ms = millis();
  if (now_ms - last_ms >= 1000) {
    uint32_t dt = now_ms - last_ms;
    freq_pb10 = (pb10_count - last_pb10_cnt) * 1000.0f / dt;
    freq_pb11 = (pb11_count - last_pb11_cnt) * 1000.0f / dt;
    last_pb10_cnt = pb10_count;
    last_pb11_cnt = pb11_count;
    last_ms = now_ms;
  }

  // 电流采样
  // if (millis() - lastCurrentUpdate >= CURRENT_UPDATE_INTERVAL) {
  //   current_amps = sampleCurrentFiltered();
  //   lastCurrentUpdate = millis();
  // }
  // 新的写法：每次 Loop 都采一个样，积少成多，完全不卡顿
    // 只要控制采样间隔别太快就行，例如每 5ms 采一个点
  if (millis() - lastCurrentUpdate >= 5) { 
    updateCurrentReading();
    lastCurrentUpdate = millis();
  }

  // OLED 刷新
  if (millis() - lastOLEDUpdate >= 100) {
    drawOLED();
    lastOLEDUpdate = millis();
  }

  // ================== 新增：自动保存逻辑 ==================
  
  // 1. 监测位置变化
  if (position != lastLoopPosition) {
    lastPosChangeTime = millis(); // 只要位置在变，就更新时间戳
    lastLoopPosition = position;  // 更新缓存
  }

  // 2. 只有当位置和上次保存的不一样，且静止时间超过 1000ms 时，才执行保存
  if ((position != lastSavedPosition) && (millis() - lastPosChangeTime > 1000)) {
    
    // 执行保存
    savePositionToEEPROM(position);
    
    // 更新“已保存”标记，避免重复写
    lastSavedPosition = position; 
  }
}