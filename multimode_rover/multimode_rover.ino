/*
 * ================================================================
 *  双模式差速小车遥控器 — 接收端 (Arduino)
 * ================================================================
 *
 *  协议: 11字节版
 *        0xA5 | SHOOT | MOOD | X_L | X_H | Y_L | Y_H | O_L | O_H | checksum | 0x5A
 *
 *  SHOOT:  射击触发 (0=无, 1=开火)
 *  MOOD:   控制模式 (0=曲率, 1=坦克掉头)
 *
 *  模式:
 *    0 = Curvature  曲率映射    摇杆X→转弯曲率, 转弯半径恒定, 高速稳
 *    1 = TankTurn   坦克掉头    直接混控, 允许原地差速旋转, 灵活激进
 *
 *  完整信号管线:
 *    摇杆 → 死区 → Expo曲线 → 模式解算 → 斜坡限制 → 等比缩放 → 启动补偿 → PWM输出
 */

#include <SoftwareSerial.h>
#include <Servo.h>

// ============================================================
// 引脚定义
// ============================================================
SoftwareSerial bluetooth(11, 2);

#define PWMA  5
#define PWMB  6
#define AIN1  7
#define AIN2  8
#define BIN1  9
#define BIN2  10

#define PIN_SERVO_MG995  13

// ============================================================
// 车体物理参数
// ============================================================
const float WHEEL_BASE   = 0.15f;
const float WHEEL_RADIUS = 0.032f;

const float MAX_LINEAR_V  = 0.5f;
const float MAX_ANGULAR_W = 3.14f;

// ============================================================
// 曲率模式参数
// ============================================================
const float MAX_CURVATURE = 6.67f;

// ============================================================
// Expo & 死区
// ============================================================
const float EXPO         = 2.5f;
const int   DEADZONE_RAW = 150;

// ============================================================
// 斜坡限制
// ============================================================
const float SLEW_RISE = 0.04f;
const float SLEW_FALL = 0.12f;

// ============================================================
// 电机
// ============================================================
const int MIN_PWM = 50;
const int MAX_PWM = 255;

// ============================================================
// MG995 射击舵机
// ============================================================
const int MG995_STOP  = 90;
const int MG995_SPIN  = 0;
const int STRIKE_TIME = 180;

// ============================================================
// 协议
// ============================================================
const byte HEADER     = 0xA5;
const byte FOOTER     = 0x5A;
const int  PACKET_LEN = 11;

// ============================================================
// 全局状态
// ============================================================
Servo servoMG995;

byte rxBuf[PACKET_LEN];
int  rxIdx = 0;

// 舵机
bool lastShootState = false;
unsigned long mg995Time = 0;
bool isMg995Moving = false;

// 模式
uint8_t currentMode = 0;

// 斜坡
float smoothLeft  = 0.0f;
float smoothRight = 0.0f;

unsigned long lastLoopTime  = 0;
unsigned long lastDebugPrint = 0;

// ============================================================
// 初始化
// ============================================================
void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  lastLoopTime = millis();

  Serial.println(F("=================================================="));
  Serial.println(F(" Dual-Mode Rover: 0=Curvature  1=TankTurn"));
  Serial.println(F("=================================================="));
}

// ============================================================
// 主循环
// ============================================================
void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  if (dt > 0.1f)  dt = 0.02f;
  if (dt < 0.001f) dt = 0.02f;
  lastLoopTime = now;

  // MG995 回正拔管
  if (isMg995Moving && (now - mg995Time >= STRIKE_TIME)) {
    servoMG995.write(MG995_STOP);
    servoMG995.detach();
    isMg995Moving = false;
  }

  // 蓝牙抓包
  while (bluetooth.available() > 0) {
    byte inByte = bluetooth.read();
    if (rxIdx == 0 && inByte != HEADER) continue;
    rxBuf[rxIdx++] = inByte;
    if (rxIdx == PACKET_LEN) {
      rxIdx = 0;
      if (rxBuf[10] != FOOTER) continue;

      byte calcSum = 0;
      for (int i = 1; i <= 8; i++) calcSum += rxBuf[i];
      if (calcSum != rxBuf[9]) continue;

      parseAndDrive(rxBuf, dt);
    }
  }

  // 调试
  if (now - lastDebugPrint > 2000) {
    lastDebugPrint = now;
    Serial.print(F("Mode: "));
    Serial.println(currentMode == 0 ? F("Curvature") : F("TankTurn"));
  }
}

// ============================================================
// 协议解析 & 全管线驱动
// ============================================================
void parseAndDrive(byte* packet, float dt) {
  // --- 1. 解码 ---
  byte shootVal  = packet[1];
  uint8_t mode   = packet[2];

  if (mode != currentMode) {
    smoothLeft = 0; smoothRight = 0;
    currentMode = mode;
  }

  // --- 2. MG995 射击 (边缘触发) ---
  bool currentShoot = (shootVal == 1);
  if (currentShoot && !lastShootState) {
    servoMG995.attach(PIN_SERVO_MG995);
    servoMG995.write(MG995_SPIN);
    mg995Time = millis();
    isMg995Moving = true;
  }
  lastShootState = currentShoot;

  // --- 3. 摇杆解析 ---
  int16_t rawX = packet[3] | (packet[4] << 8);
  int16_t rawY = packet[5] | (packet[6] << 8);
  int16_t rawO = packet[7] | (packet[8] << 8);

  if (abs(rawX) < DEADZONE_RAW) rawX = 0;
  if (abs(rawY) < DEADZONE_RAW) rawY = 0;

  float joyX = rawX / 10000.0f;
  float joyY = rawY / 10000.0f;

  // --- 4. Expo ---
  joyX = applyExpo(joyX, EXPO);
  joyY = applyExpo(joyY, EXPO);

  // --- 5. 模式解算 → 轮速 ---
  float targetL, targetR;

  if (mode == 0) {
    // 曲率模式: v,ω → 逆运动学 → 轮速
    float v, w;
    curvatureMode(joyX, joyY, v, w);
    inverseKinematics(v, w, targetL, targetR);
  } else {
    // 坦克掉头模式: 直接混控, 允许差速反转
    tankTurnMode(joyX, joyY, targetL, targetR);
  }

  // --- 6. Balance 比例校正 (O通道) ---
  // O=100 → 1:1 平衡, O=110 → 左+10%/右-10%, O=90 → 反向
  // 合理范围 70~130, 极端 clamp 50~150
  float rawBalance = constrain(rawO / 100.0f, 0.5f, 1.5f);
  targetL *= rawBalance;
  targetR *= (2.0f - rawBalance);

  // --- 7. 斜坡 ---
  smoothLeft  = slewLimit(targetL, smoothLeft, dt);
  smoothRight = slewLimit(targetR, smoothRight, dt);

  // --- 8. 等比缩放 ---
  proportionalScale(smoothLeft, smoothRight);

  // --- 9. PWM 输出 ---
  motorOutput(smoothLeft, smoothRight);
}

// ============================================================
// 模式0: 曲率映射
//
// 摇杆X → 转弯曲率 κ
// 摇杆Y → 线速度 v
// ω = v × κ (转弯半径恒定, 高速自动钝化)
// ============================================================
void curvatureMode(float jx, float jy, float &v, float &w) {
  v = jy * MAX_LINEAR_V;

  // 原地旋转: 无油门 + 有方向
  if (abs(jy) < 0.03f && abs(jx) > 0.03f) {
    v = 0;
    w = jx * MAX_ANGULAR_W;
    return;
  }

  // 正常行驶: ω = v × κ
  float curvature = jx * MAX_CURVATURE;
  w = v * curvature;
}

// ============================================================
// 模式1: 坦克掉头 (原版直接混控)
//
// 允许两轮反向旋转, 实现原地 360° 旋转
//   left  = joyY + joyX    (归一化)
//   right = joyY - joyX
//
// 无前进时推方向 = 纯差速原地转 (坦克掉头)
// 有前进时推方向 = 单侧加速/减速 (战斗转弯)
// ============================================================
void tankTurnMode(float jx, float jy, float &left, float &right) {
  // 倒车时转向反向 (保持操作直觉)
  float steer = (jy < 0) ? -jx : jx;

  left  = jy + steer;
  right = jy - steer;
}

// ============================================================
// 逆运动学: (v, ω) → 左右轮速 (归一化)
// ============================================================
void inverseKinematics(float v, float w, float &left, float &right) {
  float halfBase = WHEEL_BASE * 0.5f;
  float v_left   = v - w * halfBase;
  float v_right  = v + w * halfBase;
  left  = v_left  / MAX_LINEAR_V;
  right = v_right / MAX_LINEAR_V;
}

// ============================================================
// Expo 曲线
// ============================================================
float applyExpo(float input, float expo) {
  float sign = (input > 0) ? 1.0f : -1.0f;
  return sign * pow(abs(input), expo);
}

// ============================================================
// 斜坡限制 (加速慢, 刹车快)
// ============================================================
float slewLimit(float target, float current, float dt) {
  float diff    = target - current;
  float maxStep = (diff > 0) ? SLEW_RISE : SLEW_FALL;
  float dtScale = dt / 0.02f;
  float step    = maxStep * dtScale;

  if (abs(diff) > step)
    return current + ((diff > 0) ? step : -step);
  return target;
}

// ============================================================
// 等比缩放 (保持转弯轨迹)
// ============================================================
void proportionalScale(float &left, float &right) {
  float maxAbs = max(abs(left), abs(right));
  if (maxAbs > 1.0f) {
    left  /= maxAbs;
    right /= maxAbs;
  }
}

// ============================================================
// 启动补偿
// ============================================================
float startupCompensate(float input) {
  if (abs(input) < 0.001f) return 0;
  float sign    = (input > 0) ? 1.0f : -1.0f;
  float minNorm = (float)MIN_PWM / MAX_PWM;
  return sign * (minNorm + abs(input) * (1.0f - minNorm));
}

// ============================================================
// 电机 PWM 输出
// ============================================================
void motorOutput(float left, float right) {
  left  = startupCompensate(left);
  right = startupCompensate(right);

  int dirL = (left  >= 0) ? 1 : 0;
  int dirR = (right >= 0) ? 1 : 0;

  int pL = (int)(abs(left)  * MAX_PWM);
  int pR = (int)(abs(right) * MAX_PWM);

  if (pL > 0 && pL < MIN_PWM) pL = MIN_PWM;
  if (pR > 0 && pR < MIN_PWM) pR = MIN_PWM;

  pL = constrain(pL, 0, MAX_PWM);
  pR = constrain(pR, 0, MAX_PWM);

  digitalWrite(AIN1, dirL ? LOW  : HIGH);
  digitalWrite(AIN2, dirL ? HIGH : LOW);
  digitalWrite(BIN1, dirR ? LOW  : HIGH);
  digitalWrite(BIN2, dirR ? HIGH : LOW);

  analogWrite(PWMA, pL);
  analogWrite(PWMB, pR);
}
