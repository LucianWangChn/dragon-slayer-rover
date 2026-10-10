/*
 * 蓝牙摇杆协议拆包与实车控制终极版 (接收端/小车)
 * 协议: 11字节版 (0xA5 + 状态压缩 + SHOOT + X + Y + O + 校验和 + 0x5A)
 * 特性: Expo、微调、死区补偿、倒车修正、🔥 双 360°舵机非阻塞时间控制与物理拔管
 */
#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial bluetooth(11, 2); // RX 接蓝牙 TX, TX 接蓝牙 RX

// --- 电机与舵机引脚定义 ---
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

#define PIN_SERVO_SG90 A4 
#define PIN_SERVO_MG995 13

// --- 360° 舵机 (MG995) 专属配置 ---
const int MG995_STOP = 90;      
const int MG995_SPIN = 0;       
const int STRIKE_TIME = 160;    // 旋转持续时间 (毫秒)

// --- 360° 舵机 (SG90) 专属配置 ---
const int SG90_STOP = 90;       // SG90 物理停止位
const int SG90_POS_A = 0;       // 往 A 位置转的速度/方向 (可改为180反向)
const int SG90_POS_B = 180;     // 往 B 位置转的速度/方向 (可改为0反向)
const int SG90_MOVE_TIME = 150; // 💥 从位置 A 切到位置 B 需要旋转的时间 (毫秒)

// --- 对象实例化 ---
Servo servoSG90;
Servo servoMG995;

// 🚀 电机死区阈值：低于此 PWM 无法克服静摩擦力，按需微调
const int MIN_PWM = 50; 

// --- 协议与状态全局变量 ---
const byte HEADER = 0xA5;
const byte FOOTER = 0x5A;
const int PACKET_LEN = 11;

byte rxBuf[PACKET_LEN];
int rxIdx = 0;

// --- MG995 非阻塞时间状态机变量 ---
bool lastShootState = false; 
unsigned long mg995Time = 0; 
bool isMg995Moving = false;      

// --- SG90 非阻塞时间状态机变量 ---
bool lastStartState = false; 
unsigned long sg90Time = 0; 
bool isSg90Moving = false;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600); 
  
  // 初始化电机引脚
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // 🚨 双 360° 舵机开机时不执行 attach()，默认全部物理断电防蠕动
  
  Serial.println("==================================================");
  Serial.println("🏎️ 11字节全功能版 (含双 360° 舵机时间管理) 已就绪！");
  Serial.println("==================================================\n");
}

void loop() {
  // 🚀 MG995 后台回正拔管逻辑
  if (isMg995Moving && (millis() - mg995Time >= STRIKE_TIME)) { 
    servoMG995.write(MG995_STOP); 
    servoMG995.detach();          
    isMg995Moving = false;
  }

  // 🚀 SG90 后台回正拔管逻辑
  if (isSg90Moving && (millis() - sg90Time >= SG90_MOVE_TIME)) { 
    servoSG90.write(SG90_STOP); 
    servoSG90.detach();          
    isSg90Moving = false;
  }

  // --- 1. 严格对齐的高效抓包状态机 ---
  while (bluetooth.available() > 0) {
    byte inByte = bluetooth.read();
    
    if (rxIdx == 0 && inByte != HEADER) continue;
    
    rxBuf[rxIdx++] = inByte;
    
    if (rxIdx == PACKET_LEN) {
      rxIdx = 0; 
      if (rxBuf[10] != FOOTER) continue; 
      
      byte calcSum = 0;
      for (int i = 1; i <= 8; i++) {
        calcSum += rxBuf[i];
      }
      if (calcSum != rxBuf[9]) continue; 
      
      parseAndDrive(rxBuf);
    }
  }
}

// ==========================================================
// 核心逻辑：解析、滤波与驱动解算
// ==========================================================
void parseAndDrive(byte* packet) {
  // 1. 状态位与执行器解码
  byte stateByte = packet[1];
  bool isStart   = stateByte & 0x01;         
  bool isCarMode = (stateByte & 0x02) >> 1;  
  byte shootVal  = packet[2]; 

  // =====================================
  // 🎯 机械机构执行控制
  // =====================================
  
  // A. SG90 360°舵机 模拟两点切换 (边缘触发)
  if (isStart != lastStartState) {
    servoSG90.attach(PIN_SERVO_SG90); // 瞬间通电
    if (isStart) {
      servoSG90.write(SG90_POS_B);    // 往 B 位置转
    } else {
      servoSG90.write(SG90_POS_A);    // 往 A 位置转
    }
    sg90Time = millis();
    isSg90Moving = true;
    lastStartState = isStart;         // 更新状态，防止重复触发
  }

  // B. MG995 360°舵机 边缘时间触发
  bool currentShoot = (shootVal == 1);
  if (currentShoot && !lastShootState) {
    servoMG995.attach(PIN_SERVO_MG995); 
    servoMG995.write(MG995_SPIN);       
    mg995Time = millis();               
    isMg995Moving = true;                   
  }
  lastShootState = currentShoot;

  // =====================================
  // 🎯 底盘动力解算
  // =====================================
  
  int16_t rawX = packet[3] | (packet[4] << 8);
  int16_t rawY = packet[5] | (packet[6] << 8);
  int16_t rawO = packet[7] | (packet[8] << 8); 

  rawX = -rawX; 

  if (abs(rawX) < 150) rawX = 0; 
  if (abs(rawY) < 150) rawY = 0;

  float normX = rawX / 10000.0; 
  float normY = rawY / 10000.0;
  
  float expoX = normX * normX * normX; 
  float expoY = normY * normY * normY; 
  
  int turnPower = expoX * 255; 
  int forwardPower = expoY * 255; 

  if (forwardPower < 0) {
    turnPower = -turnPower;
  }

  int leftPower = forwardPower + turnPower;
  int rightPower = forwardPower - turnPower;
  
  if (forwardPower != 0 || turnPower != 0) {
    leftPower += rawO;
    rightPower -= rawO;
  }

  if (isCarMode) {
    if (forwardPower >= 0) { 
      if (leftPower < 0) leftPower = 0;
      if (rightPower < 0) rightPower = 0;
    } else { 
      if (leftPower > 0) leftPower = 0;
      if (rightPower > 0) rightPower = 0;
    }
  }
  
  leftPower = constrain(leftPower, -255, 255);
  rightPower = constrain(rightPower, -255, 255);

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