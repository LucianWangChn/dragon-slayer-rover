/*
 * 智能小车 - 6路红外循迹无线蓝牙监视程序（防摆头平滑优化版）
 * 特点：开机即循迹，所有 Debug 日志通过蓝牙实时打包发送至手机
 * 优化：大幅度缓解“画龙摆头”现象，修复差速转弯时的跑偏补偿干扰
 * 物理接线：
 * 1. 红外排线 -> A0 ~ A5
 * 2. 蓝牙模块 TX -> Arduino D11 (主板接收)
 * 3. 🚀 蓝牙模块 RX -> Arduino D2  (主板发送，必须接！)
 * 4. 电机驱动板 -> D5 ~ D10
 */

#include <SoftwareSerial.h>

// 初始化软串口：Arduino的 D11接蓝牙TX，D2接蓝牙RX
SoftwareSerial bluetooth(11, 2); 

// 电机引脚定义
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

// 6路红外引脚数组 (A0 到 A5)
const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// 核心控制变量
int offset = -10;                  // 你的黄金跑偏修正值 (用于抵消电机物理差异)
uint8_t lastSensorState = 0b001100; // 历史轨迹大脑记忆

void setup() {
  // 启动蓝牙串口（波特率9600）
  bluetooth.begin(9600);
  
  // 初始化电机引脚
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  
  // 初始化6路红外引脚
  for (int i = 0; i < 6; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  
  stopMotors();
  
  // 间歇等待 2 秒，方便你把车放到跑道上
  delay(2000);
  bluetooth.println("========= 📡 无线遥测系统已连通 =========");
  bluetooth.println("🏎️ 低速平滑循迹已启动，正在发射实时数据流...");
}

void loop() {
  // 全速运行循迹算法
  runLineTracking();
}

// ==========================================
// 核心算法：带记忆的低速循迹状态机 (防摆头优化版)
// ==========================================
void runLineTracking() {
  uint8_t sensorState = 0;
  
  // 读取红外状态
  for (int i = 0; i < 6; i++) {
    if (digitalRead(IR_PINS[i]) == 1) { 
      sensorState |= (1 << (5 - i)); 
    }
  }

  // 刷新有效记忆
  if (sensorState != 0b000000) {
    lastSensorState = sensorState;
  }

  // 📡 【无线遥测】将当前传感器状态发送至手机
  bluetooth.print("状态: 0b");
  for (int i = 5; i >= 0; i--) {
    bluetooth.print((sensorState >> i) & 0x01);
  }
  bluetooth.print(" -> ");

  // 低速差速状态机匹配 (降低车速，减小差速比例)
  switch (sensorState) {
    
    // --- 1. 完美居中 ---
    case 0b001100: case 0b000100: case 0b001000:
      bluetooth.println("稳步直行");
      driveMotor(75, 75); // 基础速度降到 75，避免冲刺太快
      break;

    // --- 2. 车身偏右，向左微调 ---
    case 0b011000: case 0b010000:
      bluetooth.println("左微调");
      driveMotor(55, 80); // 柔和差速，防甩尾过度
      break;

    // --- 3. 车身偏左，向右微调 ---
    case 0b000110: case 0b000010:
      bluetooth.println("右微调");
      driveMotor(80, 55); 
      break;

    // --- 4. 严重偏右/直角弯 (安全慢速原地扭头) ---
    case 0b110000: case 0b100000: case 0b111000:
      bluetooth.println("🚨 左急转弯");
      driveMotor(-20, 85); // 降低暴力倒转，保留少许原地转向能力
      break;

    // --- 5. 严重偏左/直角弯 (安全慢速原地扭头) ---
    case 0b000011: case 0b000001: case 0b000111:
      bluetooth.println("🚨 右急转弯");
      driveMotor(85, -20); 
      break;

    // --- 6. 十字路口 ---
    case 0b111111:
      bluetooth.println("十字交叉：平稳通过");
      driveMotor(75, 75); 
      break;

    // --- 7. 全白脱轨：触发大脑记忆机制 ---
    case 0b000000:
      bluetooth.print("⚠️ 全白脱轨！依据记忆：");
      if (lastSensorState & 0b111000) {
        bluetooth.println("向左旋转找线");
        driveMotor(-60, 60); 
      } 
      else if (lastSensorState & 0b000111) {
        bluetooth.println("向右旋转找线");
        driveMotor(60, -60); 
      } 
      else {
        bluetooth.println("方向不明：安全后退");
        driveMotor(-60, -60);
      }
      break;

    default:
      bluetooth.println("散杂状态：惯性滑行");
      driveMotor(70, 70);
      break;
  }
}

// ==========================================
// 统一底层驱动 (带-10差速补丁与反转修正)
// ==========================================
void driveMotor(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;

  int speedL = abs(powerL);
  int speedR = abs(powerR);

  // 【核心修复】直线补偿逻辑修复
  // 确保只有在“真正打算跑绝对直线”的时候（速度差<=10），才介入 offset 跑偏补丁
  // 防止在微调转弯时错误叠加补丁导致转弯变形
  if (dirL == 1 && dirR == 1 && abs(powerL - powerR) <= 10) {
    speedL = constrain(speedL + offset, 0, 255);
    speedR = constrain(speedR - offset, 0, 255);
  } else {
    speedL = constrain(speedL, 0, 255);
    speedR = constrain(speedR, 0, 255);
  }

  // 硬件电平逻辑反转修正
  digitalWrite(AIN1, dirL ? LOW : HIGH);
  digitalWrite(AIN2, dirL ? HIGH : LOW);
  digitalWrite(BIN1, dirR ? LOW : HIGH);
  digitalWrite(BIN2, dirR ? HIGH : LOW);

  // 物理输出
  analogWrite(PWMA, speedL);
  analogWrite(PWMB, speedR);
}

void stopMotors() {
  analogWrite(PWMA, 0); analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}