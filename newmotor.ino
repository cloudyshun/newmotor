/* STM32F103C8T6 + 74HC595 x2 + 74HC165 + AT24C02 + PCA9685 + OLED
   功能：电机控制、正交解码、电流监测、位置记忆（EEPROM）
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
#define EEPROM_ADDR 0x50      // AT24C02 器件地址
#define ADDR_POS_STORAGE 0x00 // position 数据在 EEPROM 中的起始存储地址

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
volatile int32_t position = 0; // 相对位置（掉电会从 EEPROM 恢复）
volatile uint32_t pb10_count = 0;
volatile uint32_t pb11_count = 0;

float current_amps = 0.0f;
unsigned long lastCurrentUpdate = 0;
unsigned long lastOLEDUpdate = 0;
uint32_t last_ms = 0;
uint32_t last_pb10_cnt = 0, last_pb11_cnt = 0;
float freq_pb10 = 0.0f, freq_pb11 = 0.0f;

const uint8_t SPEED_PERCENT = 100;

// ================= AT24C02 存储逻辑 =================

// 将 32 位 position 写入 EEPROM
void savePositionToEEPROM(int32_t pos) {
  uint8_t data[4];
  data[0] = (pos >> 24) & 0xFF;
  data[1] = (pos >> 16) & 0xFF;
  data[2] = (pos >> 8) & 0xFF;
  data[3] = pos & 0xFF;

  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(ADDR_POS_STORAGE); 
  for (int i = 0; i < 4; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  delay(5); // AT24C02 写周期延时
  Serial.println("Position saved to EEPROM.");
}

// 从 EEPROM 读取 32 位 position
int32_t readPositionFromEEPROM() {
  uint8_t data[4] = {0, 0, 0, 0};
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write(ADDR_POS_STORAGE);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDR, (uint8_t)4);
  for (int i = 0; i < 4 && Wire.available(); i++) {
    data[i] = Wire.read();
  }

  return ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) | 
         ((int32_t)data[2] << 8)  | (int32_t)data[3];
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
    Serial.println("=> Motor STOP & Saving Pos...");
    motorStop();
    setPCA9685PWM(PCA9685_MOTOR_CHANNEL, 0);
    
    // 【关键】电机停止时保存位置到 EEPROM
    savePositionToEEPROM(position);
    
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

  // --- 3. 【关键修改】先初始化 OLED，保证屏幕能亮 ---
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("System Init...");
  display.display(); // 先显示这一行，证明 OLED 没坏
  
  // --- 4. 初始化 PCA9685 ---
  pca9685.begin();
  pca9685.setPWMFreq(PCA9685_FREQ);
  pca9685.setOutputMode(true);

  // --- 5. 【带检测的】读取 EEPROM ---
  // 先检查设备是否在线，防止卡死
  Wire.beginTransmission(EEPROM_ADDR);
  byte error = Wire.endTransmission();

  if (error == 0) {
    // 设备存在，读取数据
    position = readPositionFromEEPROM();
    Serial.print("EEPROM OK. Pos: ");
    Serial.println(position);
    
    display.println("Memory: OK");
    display.print("Load Pos: ");
    display.println(position);
  } else {
    // 设备不存在或接线错误
    Serial.print("EEPROM Error: ");
    Serial.println(error);
    
    display.println("Memory: ERROR!");
    display.println("Check Wiring!");
    // position 保持为 0
  }
  display.display(); // 刷新屏幕显示存储器状态
  delay(1500);       // 延时一下让你看清屏幕上的提示

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
  if (millis() - lastCurrentUpdate >= CURRENT_UPDATE_INTERVAL) {
    current_amps = sampleCurrentFiltered();
    lastCurrentUpdate = millis();
  }

  // OLED 刷新
  if (millis() - lastOLEDUpdate >= 100) {
    drawOLED();
    lastOLEDUpdate = millis();
  }
}