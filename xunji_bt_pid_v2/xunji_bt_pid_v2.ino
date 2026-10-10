/*
 * 智能小车 - 蓝牙动态调参 PID 版本 (xunji_bt_pid_v2.ino - 修正重构版)
 * 特点：结合 v1 稳定的 PID 算法与 v2 的动态调参及开环测试功能
 * 
 * 指令格式 (英文开头 + 数字)：
 * P15.5  -> 将 Kp 设为 15.5
 * I0.5   -> 将 Ki 设为 0.5 (同时清零积分)
 * D10.2  -> 将 Kd 设为 10.2
 * S80    -> 将基础速度 baseSpeed 设为 80
 * L0.95  -> 将左轮比例 ratioL 设为 0.95
 * R0.85  -> 将右轮比例 ratioR 设为 0.85
 * T      -> 切换开环测试模式（两轮同速直行，不看传感器，用于调比例）
 * ?      -> 查询当前所有参数
 */

#include <SoftwareSerial.h>

SoftwareSerial bluetooth(11, 2); 

#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// 核心参数 (默认值与 v1 保持一致)
float Kp = 25.0;
float Ki = 0.0;
float Kd = 10000000.0;
int baseSpeed = 70;
float ratioL = 1.0;   // 左轮动力系数
float ratioR = 1.0;   // 右轮动力系数 (偏快的一侧设<1.0)

float lastError = 0;
float integral = 0;
uint8_t lastSensorState = 0b001100;

// 终点停止：连续检测到大面积黑线的次数
int blackLineCount = 0;
const int BLACK_THRESHOLD = 5;    // 检测到 >= 5个黑线传感器时视为遇到横向终点线
const int BLACK_HOLD = 2;         // 连续2次确认才停车，防摆头误判

// 开环测试模式
bool testMode = false;

// 启动保护：前N个循环不检测终点，防止起步在黑线上误停车
int startProtect = 100;

void setup() {
  bluetooth.begin(9600);
  bluetooth.setTimeout(50);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  for (int i = 0; i < 6; i++) pinMode(IR_PINS[i], INPUT);
  
  stopMotors();
  delay(2000);
  printCurrentSettings();
  bluetooth.println(">>> 调参系统就绪，请输入指令 (如 P20) <<<");
}

void loop() {
  // 1. 检查蓝牙指令
  checkBluetooth();

  // 2. 开环测试模式：不看传感器，两轮同速
  if (testMode) {
    driveMotor(baseSpeed, baseSpeed);
    return; // 在测试模式下不执行后续循迹逻辑
  }

  // 3. 循迹逻辑
  float error = getError();
  
  if (error == 200.0) {
    // 终点停车
    stopMotors();
    bluetooth.println(">>> 检测到终点线，已停车 <<<");
    while (true) {} // 永久停止，复位重启
  } else if (error == 100.0) {
    // 脱轨找线
    handleOffTrack();
  } else {
    // 核心 PID 计算 (完全对标 v1)
    float P = Kp * error;
    integral += error;
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    
    float pidOutput = P + I + D;
    lastError = error;

    int leftSpeed = baseSpeed + pidOutput;
    int rightSpeed = baseSpeed - pidOutput;

    driveMotor(leftSpeed, rightSpeed);
  }
}

// ---------------- 蓝牙指令解析 ----------------
void checkBluetooth() {
  if (bluetooth.available() > 0) {
    char cmd = bluetooth.read();
    
    if (cmd == '?') {
      printCurrentSettings();
      return;
    }

    // 开环测试切换
    if (cmd == 'T' || cmd == 't') {
      testMode = !testMode;
      if (testMode) {
        integral = 0;
        lastError = 0;
        bluetooth.println(">>> 开环测试模式 ON：两轮同速直行 (再次发 T 退出) <<<");
      } else {
        stopMotors();
        bluetooth.println(">>> 开环测试模式 OFF：恢复循迹 <<<");
      }
      return;
    }

    float value = bluetooth.parseFloat();

    switch (cmd) {
      case 'P': case 'p':
        Kp = value;
        bluetooth.print("OK! Kp = "); bluetooth.println(Kp);
        break;
      case 'I': case 'i':
        Ki = value;
        integral = 0;
        bluetooth.print("OK! Ki = "); bluetooth.println(Ki);
        break;
      case 'D': case 'd':
        Kd = value;
        bluetooth.print("OK! Kd = "); bluetooth.println(Kd);
        break;
      case 'S': case 's':
        baseSpeed = (int)value;
        bluetooth.print("OK! Speed = "); bluetooth.println(baseSpeed);
        break;
      case 'L': case 'l':
        ratioL = value;
        bluetooth.print("OK! ratioL = "); bluetooth.println(ratioL);
        break;
      case 'R': case 'r':
        ratioR = value;
        bluetooth.print("OK! ratioR = "); bluetooth.println(ratioR);
        break;
      default:
        break;
    }
  }
}

void printCurrentSettings() {
  bluetooth.println("\n--- 当前参数状态 ---");
  bluetooth.print("P (Kp): "); bluetooth.println(Kp);
  bluetooth.print("I (Ki): "); bluetooth.println(Ki);
  bluetooth.print("D (Kd): "); bluetooth.println(Kd);
  bluetooth.print("S (Speed): "); bluetooth.println(baseSpeed);
  bluetooth.print("L (ratioL): "); bluetooth.println(ratioL);
  bluetooth.print("R (ratioR): "); bluetooth.println(ratioR);
  bluetooth.print("Test: "); bluetooth.println(testMode ? "ON" : "OFF");
  bluetooth.println("--------------------\n");
}

// ---------------- 循迹算法 ----------------
float getError() {
  float sum = 0;
  int count = 0;
  uint8_t currentSensorState = 0;
  
  // 权重与 v1 完全对齐
  float weights[6] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};

  for (int i = 0; i < 6; i++) {
    if (digitalRead(IR_PINS[i]) == 1) { // 假设 1 代表识别到黑线
      sum += weights[i];
      count++;
      currentSensorState |= (1 << (5 - i));
    }
  }

  // 全白脱轨处理（优先级最高）
  if (count == 0) return 100.0;

  // 启动保护期内跳过终点检测
  if (startProtect > 0) startProtect--;

  // 终点检测：黑线数量大于等于 5 视为压到终点横线
  if (startProtect == 0 && count >= BLACK_THRESHOLD) {
    blackLineCount++;
    if (blackLineCount >= BLACK_HOLD) {
      return 200.0;
    }
  } else {
    blackLineCount = 0;
  }

  lastSensorState = currentSensorState;
  return sum / count; // 返回平均位置偏差 (与 v1 完全一致)
}

void handleOffTrack() {
  // 根据最后记忆旋转找线 (与 v1 完全一致)
  if (lastSensorState & 0b111000) {
    driveMotor(-60, 60); 
  } else {
    driveMotor(60, -60); 
  }
}

// ---------------- 电机驱动 + 比例补偿 ----------------
void driveMotor(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;

  int speedL = abs(powerL);
  int speedR = abs(powerR);

  // 比例补偿：替换了 v1 生硬的 offset，提供更平滑的两轮平衡
  speedL = (int)(speedL * ratioL);
  speedR = (int)(speedR * ratioR);

  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  digitalWrite(AIN1, dirL ? LOW : HIGH);
  digitalWrite(AIN2, dirL ? HIGH : LOW);
  digitalWrite(BIN1, dirR ? LOW : HIGH);
  digitalWrite(BIN2, dirR ? HIGH : LOW);
  analogWrite(PWMA, speedL);
  analogWrite(PWMB, speedR);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
