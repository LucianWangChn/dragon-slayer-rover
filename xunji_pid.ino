/*
 * 智能小车 - PID 循迹算法版本 (xunji_pid.ino)
 * 目标：通过 PID 连续调节解决“摆头/画龙”问题
 * 
 * 调参指南：
 * 1. Kp (比例): 基础修正力度。从小往大调，直到小车能跟着线走，但会有明显晃动。
 * 2. Kd (微分): 抑制晃动力度。增加 Kd 可以抵消 Kp 带来的惯性冲过头，让小车变稳。
 * 3. Ki (积分): 消除静态误差。通常循迹不需要，保持 0 即可。
 */

#include <SoftwareSerial.h>

// 蓝牙引脚：D11接TX，D2接RX
SoftwareSerial bluetooth(11, 2); 

// 电机引脚
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// ================= PID 参数区 (调试重点) =================
float Kp = 25.0;  // 比例：决定反应快慢
float Ki = 0.0;   // 积分：决定纠偏彻底度 (通常设为 0)
float Kd = 15.0;  // 微分：抑制摆头，数值越大越“粘”在线上
// ========================================================

int baseSpeed = 70;    // 基础巡航速度 (0-255)
int offset = -10;      // 电机物理偏置修正
float lastError = 0;   // 上次误差
float integral = 0;    // 误差累积
uint8_t lastSensorState = 0b001100;

void setup() {
  bluetooth.begin(9600);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  for (int i = 0; i < 6; i++) pinMode(IR_PINS[i], INPUT);
  
  stopMotors();
  delay(2000);
  bluetooth.println(">>> PID 循迹系统已就绪 <<<");
}

void loop() {
  float error = getError();
  
  if (error == 100.0) { // 触发全白脱轨
    handleOffTrack();
  } else {
    // 核心 PID 计算
    float P = Kp * error;
    integral += error;
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    
    float pidOutput = P + I + D;
    lastError = error;

    // 计算电机速度
    int leftSpeed = baseSpeed + pidOutput;
    int rightSpeed = baseSpeed - pidOutput;

    driveMotor(leftSpeed, rightSpeed);

    // 蓝牙打印调试信息 (建议调稳后注释掉，以提升运行频率)
    /*
    bluetooth.print("E:"); bluetooth.print(error);
    bluetooth.print(" -> Out:"); bluetooth.println(pidOutput);
    */
  }
}

// 获取加权偏差值：范围约 [-2.5, 2.5]
float getError() {
  float sum = 0;
  int count = 0;
  uint8_t currentSensorState = 0;

  // 权重分配：A0:-2.5, A1:-1.5, A2:-0.5, A3:0.5, A4:1.5, A5:2.5
  float weights[6] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};

  for (int i = 0; i < 6; i++) {
    if (digitalRead(IR_PINS[i]) == 1) {
      sum += weights[i];
      count++;
      currentSensorState |= (1 << (5 - i));
    }
  }

  if (count == 0) return 100.0; // 特殊值代表脱轨

  lastSensorState = currentSensorState;
  return sum / count; // 返回平均位置偏差
}

void handleOffTrack() {
  bluetooth.print("! 脱轨回复: ");
  // 依据最后记忆旋转找线
  if (lastSensorState & 0b111000) {
    bluetooth.println("左旋");
    driveMotor(-60, 60); 
  } else {
    bluetooth.println("右旋");
    driveMotor(60, -60); 
  }
}

void driveMotor(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;
  int speedL = abs(powerL);
  int speedR = abs(powerR);

  // 物理偏置补丁
  if (dirL == 1 && dirR == 1 && abs(powerL - powerR) <= 10) {
    speedL = constrain(speedL + offset, 0, 255);
    speedR = constrain(speedR - offset, 0, 255);
  } else {
    speedL = constrain(speedL, 0, 255);
    speedR = constrain(speedR, 0, 255);
  }

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
