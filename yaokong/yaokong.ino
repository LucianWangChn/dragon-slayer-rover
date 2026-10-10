/*
 * 蓝牙摇杆协议拆包与实车控制终极版 (接收端/小车)
 * 协议: 11字节版 (0xA5 + 状态压缩 + SHOOT + X + Y + O + 校验和 + 0x5A)
 * 特性: 双轴指数曲线(Expo)、动态微调(Trim)、静止防转保护、电机死区补偿、倒车逻辑修正
 */
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(11, 2); // RX 接蓝牙 TX, TX 接蓝牙 RX

// --- 电机引脚定义 ---
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

// 🚀 电机死区阈值：低于此 PWM 无法克服静摩擦力，按需微调
const int MIN_PWM = 50; 

// --- 协议与状态全局变量 ---
const byte HEADER = 0xA5;
const byte FOOTER = 0x5A;
const int PACKET_LEN = 11;

byte rxBuf[PACKET_LEN];
int rxIdx = 0;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600); 
  
  // 初始化电机引脚
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  
  Serial.println("==================================================");
  Serial.println("🏎️ 11字节双轴 Expo + 完美死区/微调控制系统 已就绪！");
  Serial.println("==================================================\n");
}

void loop() {
  // --- 1. 严格对齐的高效抓包状态机 ---
  while (bluetooth.available() > 0) {
    byte inByte = bluetooth.read();
    
    // 找包头
    if (rxIdx == 0 && inByte != HEADER) continue;
    
    rxBuf[rxIdx++] = inByte;
    
    // 收满一帧
    if (rxIdx == PACKET_LEN) {
      rxIdx = 0; // 重置指针
      
      // 校验包尾
      if (rxBuf[10] != FOOTER) continue; 
      
      // 校验和 (START~O 的所有 8 个字节累加)
      byte calcSum = 0;
      for (int i = 1; i <= 8; i++) {
        calcSum += rxBuf[i];
      }
      if (calcSum != rxBuf[9]) continue; // 校验失败丢弃
      
      // 校验通过，移交控制核心
      parseAndDrive(rxBuf);
    }
  }
}

// ==========================================================
// 核心逻辑：解析、滤波与驱动解算
// ==========================================================
void parseAndDrive(byte* packet) {
  // 1. 状态位解码 
  byte stateByte = packet[1];
  bool isStart   = stateByte & 0x01;         
  bool isCarMode = (stateByte & 0x02) >> 1;  
  // byte shoot = packet[2]; 
  
  // 2. 提取数据 (小端拼接)
  int16_t rawX = packet[3] | (packet[4] << 8);
  int16_t rawY = packet[5] | (packet[6] << 8);
  int16_t rawO = packet[7] | (packet[8] << 8); // 提取动态修正值 O

  // 🚨 硬件方向修复补丁：向右打摇杆确保是向右转
  rawX = -rawX; 

  // 3. 摇杆死区保护 
  if (abs(rawX) < 150) rawX = 0; 
  if (abs(rawY) < 150) rawY = 0;

  // 4. 双轴指数优化 (Expo)
  float normX = rawX / 10000.0; 
  float normY = rawY / 10000.0;
  
  float expoX = normX * normX * normX; 
  float expoY = normY * normY * normY; 
  
  int turnPower = expoX * 255; 
  int forwardPower = expoY * 255; 

  // 🚨 倒车方向直觉修复补丁
  if (forwardPower < 0) {
    turnPower = -turnPower;
  }

  // 5. 基础混控解算
  int leftPower = forwardPower + turnPower;
  int rightPower = forwardPower - turnPower;
  
  // 🚨 核心修复：静止状态下禁止微调值介入！
  // 只有在摇杆被实质性推动（期望动力或转向不为 0）时，才叠加 O 值，防止原地空转
  if (forwardPower != 0 || turnPower != 0) {
    leftPower += rawO;
    rightPower -= rawO;
  }

  if (isCarMode) {
    // 【真车阿克曼模式】：强制接管，绝对禁止内侧轮反转
    if (forwardPower >= 0) { 
      if (leftPower < 0) leftPower = 0;
      if (rightPower < 0) rightPower = 0;
    } else { 
      if (leftPower > 0) leftPower = 0;
      if (rightPower > 0) rightPower = 0;
    }
  }
  
  // 6. 强制极值限幅，防止计算溢出
  leftPower = constrain(leftPower, -255, 255);
  rightPower = constrain(rightPower, -255, 255);

  // 7. 传递给底层驱动
  driveJoyBase(leftPower, rightPower);
}

// ==========================================================
// 底层物理驱动 (带死区补偿)
// ==========================================================
void driveJoyBase(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;

  int finalSpeedL = abs(powerL);
  int finalSpeedR = abs(powerR);

  // 🚀 核心修复：电机死区补偿 (Deadzone Compensation)
  // 只要期望动力大于 0，就把 1~255 的控制范围映射到 50~255 的真实起步范围
  if (finalSpeedL > 0) {
    finalSpeedL = map(finalSpeedL, 1, 255, MIN_PWM, 255);
  }
  if (finalSpeedR > 0) {
    finalSpeedR = map(finalSpeedR, 1, 255, MIN_PWM, 255);
  }

  digitalWrite(AIN1, dirL ? LOW : HIGH);
  digitalWrite(AIN2, dirL ? HIGH : LOW);
  digitalWrite(BIN1, dirR ? LOW : HIGH);
  digitalWrite(BIN2, dirR ? HIGH : LOW);

  analogWrite(PWMA, finalSpeedL);
  analogWrite(PWMB, finalSpeedR);
}