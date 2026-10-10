#include <Arduino.h>
#include <SoftwareSerial.h>

// ==================== 引脚分配表 ====================
// 电机驱动
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

// 传感器组
const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5}; // 红外模块
#define TRIG_L 2
#define ECHO_L 3
#define TRIG_C 4
#define ECHO_C 12          // 现已改为普通轮询，不再占用中断
#define TRIG_R A4          // (复用) 阶段二使用
#define ECHO_R A5          // (复用) 阶段二使用

// 蓝牙软串口 (仅接收: RX=11, TX=13悬空不用)
SoftwareSerial bt(11, 13);

// ==================== 系统全局状态 ====================
// 工作模式：0=待机停止, 1=红外循迹, 2=超声波迷宫
int currentMode = 0; 

// ==================== 阶段一：循迹参数 ====================
float Kp = 25, Ki = 0.0, Kd = 100000;
int irBaseSpeed = 70;
int irOffset = -10;
float irLastError = 0, irLastValidError = 0, irIntegral = 0;
uint8_t irLastSensorState = 0b001100;

// ==================== 阶段二：超声波参数 ====================
const int   US_BASE_SPEED   = 60;
const int   US_TURN_SPEED   = 60;
const int   US_OFFSET_L     = 5;
const int   US_OFFSET_R     = -5;
const float TARGET_DIST     = 9.0;
const float WALL_LOST       = 15.0;
const float Kp_s = 4.5, Kd_s = 10.0;
const float FRONT_DANGER    = 9.0;
const int   BACKUP_t        = 300;
const int   TURN_45_t       = 300;
float       usLastError     = 0;
const float STOP_DISTANCE   = 30.0;
const int   STOP_DURATION   = 1000;

// ==================== 初始化 ====================
void setup() {
  bt.begin(9600);
  bt.setTimeout(10); 
  
  // 初始化固定引脚
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  
  // 初始化红外 0-3 (4和5在模式切换时动态分配)
  for (int i = 0; i < 4; i++) pinMode(IR_PINS[i], INPUT);
  
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);

  stopMotors();
  delay(1000);
}

// ==================== 主循环 ====================
void loop() {
  checkBluetooth(); // 全局监听手机指令

  switch (currentMode) {
    case 0:
      stopMotors();
      break;
    case 1:
      runLineTrackingMode();
      break;
    case 2:
      runUltrasonicMazeMode();
      break;
    case 3:                 // 🟢 新增：模式 3
      runStraightMode();    // 专门用于调校 Offset 的直行模式
      break;
  }
}

// ==================== 蓝牙控制台 (14字节二进制状态机解析) ====================
void checkBluetooth() {
  static uint8_t rx_buf[14]; // 🟢 数组容量扩大为 14 字节
  static uint8_t rx_idx = 0;

  while (bt.available() > 0) {
    uint8_t c = bt.read();

    if (rx_idx == 0) {
      if (c == 0xA5) rx_buf[rx_idx++] = c; // 捕捉包头
    } 
    else if (rx_idx < 13) {
      rx_buf[rx_idx++] = c; // 接收数据体和校验和
    } 
    else if (rx_idx == 13) {
      rx_buf[rx_idx] = c; // 接收包尾
      
      // 验证包尾
      if (c == 0x5A) {
        // 🟢 计算校验和：累加 Byte 1 到 Byte 11
        uint8_t sum = 0;
        for (int i = 1; i <= 11; i++) {
          sum += rx_buf[i];
        }
        
        // 如果校验和正确 (存放在 Byte 12)，应用参数
        if (sum == rx_buf[12]) {
          int newMode = rx_buf[1];
          
          // 只有模式发生改变时，才执行刹车和引脚重置
          if (newMode != currentMode) {
            currentMode = newMode;
            stopMotors();
            if (currentMode == 1) {
              pinMode(A4, INPUT); pinMode(A5, INPUT);
            } else if (currentMode == 2) {
              pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
            }
          }
          
          // 🟢 按照新顺序 (P, S, O, D) 与小端模式解析
          Kp = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
          irBaseSpeed = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);
          irOffset = (int16_t)(((uint16_t)rx_buf[7] << 8) | rx_buf[6]);
          
          // 4字节的 D 解析
          Kd = (int32_t)(
            ((uint32_t)rx_buf[11] << 24) | 
            ((uint32_t)rx_buf[10] << 16) | 
            ((uint32_t)rx_buf[9]  << 8)  | 
             (uint32_t)rx_buf[8]
          );
        }
      }
      // 无论成功失败，重置索引，准备接收下一帧
      rx_idx = 0; 
    }
  }
}

// ==================== 第一阶段：红外循迹逻辑 ====================
void runLineTrackingMode() {
  float error = getIrError();
  
  if (error == 100.0) { // 全白脱轨
    if (irLastSensorState & 0b111000) setMotorSpeeds(-60, 60);
    else setMotorSpeeds(60, -60);
  } 
  else if (error == 200.0) { // 终点停车
    stopMotors();
    currentMode = 0; // 自动切回待机模式
  } 
  else {
    float P = Kp * error;
    irIntegral += error;
    float I = Ki * irIntegral;
    float D = Kd * (error - irLastError);
    float pidOutput = P + I + D;
    irLastError = error;

    int powerL = irBaseSpeed + pidOutput;
    int powerR = irBaseSpeed - pidOutput;

    if (powerL >= 0 && powerR >= 0 && abs(powerL - powerR) <= 10) {
      powerL = constrain(powerL + irOffset, -255, 255);
      powerR = constrain(powerR - irOffset, -255, 255);
    }
    setMotorSpeeds(powerL, powerR);
  }
}

float getIrError() {
  float sum = 0; int count = 0; uint8_t currentSensorState = 0;
  float weights[6] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};
  
  for (int i = 0; i < 6; i++) {
    if (digitalRead(IR_PINS[i]) == 1) {
      sum += weights[i];
      count++;
      currentSensorState |= (1 << (5 - i));
    }
  }

  if (count == 0) return 100.0;
  if (count >= 5) return 200.0;

  float currentError = sum / count;
  if (count == 1) currentError = (currentError + irLastValidError) / 2.0;

  irLastValidError = currentError;
  irLastSensorState = currentSensorState;
  return currentError;
}

// ==================== 第二阶段：超声波避障逻辑 ====================
void runUltrasonicMazeMode() {
  // 统一使用 pulseIn 获取三路超声波距离
  float distL = get_distance(TRIG_L, ECHO_L);
  float distR = get_distance(TRIG_R, ECHO_R);
  float distC = get_distance(TRIG_C, ECHO_C);

  // 限制异常值
  if (distL <= 0 || distL > 40) distL = 40;
  if (distR <= 0 || distR > 40) distR = 40;
  if (distC <= 0 || distC > 40) distC = 40; 

  static unsigned long lost_since = 0;

  // 脱离迷宫检测 (三路全部超出阈值)
  if (distL > STOP_DISTANCE && distC > STOP_DISTANCE && distR > STOP_DISTANCE) {
    if (lost_since == 0) lost_since = millis();
    if (millis() - lost_since >= STOP_DURATION) {
      stopMotors();
      currentMode = 0; // 自动切回待机模式
      return;
    }
  } else {
    lost_since = 0;
  }

  // 核心逻辑：前方有障碍优先避障，否则判断两侧贴墙
  if (distC < FRONT_DANGER) {
    evade_front(distL, distR);
    usLastError = 0;
  }
  else if (distL <= WALL_LOST && distR <= WALL_LOST) state_follow_wall(distR, true);
  else if (distL <= WALL_LOST) state_follow_wall(distL, false);
  else if (distR <= WALL_LOST) state_follow_wall(distR, true);
  else {
    setMotorSpeeds(US_BASE_SPEED + US_OFFSET_L, US_BASE_SPEED + US_OFFSET_R);
  }
  delay(20);
}

void state_follow_wall(float dist, bool isRight) {
  float error = isRight ? (TARGET_DIST - dist) : (dist - TARGET_DIST);
  float adjust = Kp_s * error + Kd_s * (error - usLastError);
  usLastError = error;
  setMotorSpeeds(US_BASE_SPEED + US_OFFSET_L + adjust, 
                 US_BASE_SPEED + US_OFFSET_R - adjust);
}

void evade_front(float distL, float distR) {
  stopMotors(); delay(100);
  
  // 倒退一段
  setMotorSpeeds(-US_BASE_SPEED, -US_BASE_SPEED); delay(BACKUP_t);
  stopMotors(); delay(100);
  
  // 🧭 【核心修改】：对调了满足条件时的底盘输出动作
  if (distL > distR) {
    // 左边开阔本该左转，既然原来反了，这里换成右转指令 (左轮正转，右轮反转)
    setMotorSpeeds(US_TURN_SPEED, -US_TURN_SPEED); 
  } else {
    // 右边开阔本该右转，换成左转指令 (左轮反转，右轮正转)
    setMotorSpeeds(-US_TURN_SPEED, US_TURN_SPEED);               
  }
  
  delay(TURN_45_t);
  stopMotors(); delay(50);
}

float get_distance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float dur = pulseIn(echo, HIGH, 4000); // 4000us 超时，约4ms，非常快，不会卡死主循环
  return (dur == 0) ? 999 : dur * 0.017;
}

// ==================== 统一底层电机驱动 ====================
void setMotorSpeeds(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;
  
  analogWrite(PWMA, constrain(abs(powerL), 0, 255));
  analogWrite(PWMB, constrain(abs(powerR), 0, 255));
  
  digitalWrite(AIN1, dirL ? LOW : HIGH);
  digitalWrite(AIN2, dirL ? HIGH : LOW);
  digitalWrite(BIN1, dirR ? LOW : HIGH);
  digitalWrite(BIN2, dirR ? HIGH : LOW);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// ==================== 调校模式：纯开环直行 ====================
void runStraightMode() {
  // 屏蔽所有传感器，只受基础速度 (irBaseSpeed) 和 偏置 (irOffset) 控制
  int powerL = constrain(irBaseSpeed + irOffset, -255, 255);
  int powerR = constrain(irBaseSpeed - irOffset, -255, 255);
  
  setMotorSpeeds(powerL, powerR);
  delay(20);
}