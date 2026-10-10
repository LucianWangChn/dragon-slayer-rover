/*
 * 智能小车 - 蓝牙动态调参 PID 终极版 (xunji_bt_pid.ino)
 * 特点：
 * 1. 无需反复烧录，直接通过手机蓝牙助手发送指令实时修改参数
 * 2. 细线平滑补间优化，杜绝单灯跳变画龙
 * 3. 终点线识别（5路或6路全黑自动停车保护）
 * * 指令格式 (英文开头 + 数字)：
 * P15.5  -> 将 Kp 设为 15.5
 * D10.2  -> 将 Kd 设为 10.2
 * S80    -> 将基础速度 baseSpeed 设为 80
 * O-15   -> 将物理偏置 offset 设为 -15
 * ?      -> 查询当前所有参数
 */

#include <SoftwareSerial.h>

// 初始化软串口：Arduino的 D11接蓝牙TX，D2接蓝牙RX
SoftwareSerial bluetooth(11, 2); 

// === 硬件引脚定义 ===
#define PWMA 5
#define PWMB 6
#define AIN1 7
#define AIN2 8
#define BIN1 9
#define BIN2 10

const int IR_PINS[6] = {A0, A1, A2, A3, A4, A5};

// === 核心参数 (初始值) ===
float Kp = 25.0;
float Ki = 0.0;
float Kd = 100000; // 这里的初始值保留你之前的设定，可通过蓝牙修改
int baseSpeed = 70;
int offset = -10;

// === 系统记忆变量 ===
float lastError = 0;
float lastValidError = 0; // 用于平滑细线跳变
float integral = 0;
uint8_t lastSensorState = 0b001100;

void setup() {
  bluetooth.begin(9600);
  // 【关键设置】设置串口读取超时，防止阻塞主循环
  bluetooth.setTimeout(10);
  
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  
  for (int i = 0; i < 6; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
  
  stopMotors();
  delay(2000);
  printCurrentSettings();
  bluetooth.println(">>> 🚀 终极调参循迹系统就绪，请输入指令 (如 P20) <<<");
}

void loop() {
  // 1. 检查是否有手机发来的蓝牙指令
  checkBluetooth();
  
  // 2. 核心循迹状态获取
  float error = getError();
  
  // 3. 行为决策分支
  if (error == 100.0) {
    // 状态 A: 全白脱轨，进入找线逻辑
    handleOffTrack();
  } 
  else if (error == 200.0) {
    // 状态 B: 终点线识别（5路或6路全黑），直接紧急停车
    stopMotors();
    bluetooth.println("🛑 检测到终点(5路以上全黑)，已停车！");
    // 给一点延时防抖，防止车辆由于惯性滑出黑线立刻重新起步
    while(1); 
  } 
  else {
    // 状态 C: 正常赛道，执行 PID 丝滑循迹
    float P = Kp * error;
    integral += error;
    float I = Ki * integral;
    float D = Kd * (error - lastError);
    
    float pidOutput = P + I + D;
    lastError = error;

    // 计算左右轮速度
    int leftSpeed = baseSpeed + pidOutput;
    int rightSpeed = baseSpeed - pidOutput;

    // 将计算结果交由底层物理驱动执行
    driveMotor(leftSpeed, rightSpeed);
  }
}

// ---------------- 蓝牙指令解析 ----------------
void checkBluetooth() {
  if (bluetooth.available() > 0) {
    char cmd = bluetooth.read(); // 读取首字母
    
    // 如果是查询指令
    if (cmd == '?') {
      printCurrentSettings();
      return;
    }

    // 解析后面的数字
    float value = bluetooth.parseFloat();
    
    switch (cmd) {
      case 'P': case 'p':
        Kp = value;
        bluetooth.print("✅ OK! Kp 修改为: "); bluetooth.println(Kp);
        break;
      case 'D': case 'd':
        Kd = value;
        bluetooth.print("✅ OK! Kd 修改为: "); bluetooth.println(Kd);
        break;
      case 'I': case 'i': // 顺手帮你加上了 I 的调节，以防未来想用
        Ki = value;
        bluetooth.print("✅ OK! Ki 修改为: "); bluetooth.println(Ki);
        break;
      case 'S': case 's':
        baseSpeed = (int)value;
        bluetooth.print("✅ OK! Speed 修改为: "); bluetooth.println(baseSpeed);
        break;
      case 'O': case 'o':
        offset = (int)value;
        bluetooth.print("✅ OK! Offset 修改为: "); bluetooth.println(offset);
        break;
      default:
        // 忽略无效字符（如回车换行）
        break;
    }
  }
}

void printCurrentSettings() {
  bluetooth.println("\n--- 📊 当前参数状态 ---");
  bluetooth.print("P (Kp): "); bluetooth.println(Kp);
  bluetooth.print("I (Ki): "); bluetooth.println(Ki);
  bluetooth.print("D (Kd): "); bluetooth.println(Kd);
  bluetooth.print("S (Speed): "); bluetooth.println(baseSpeed);
  bluetooth.print("O (Offset): "); bluetooth.println(offset);
  bluetooth.println("----------------------\n");
}

// ---------------- 循迹算法核心 ----------------
float getError() {
  float sum = 0;
  int count = 0;
  uint8_t currentSensorState = 0;
  
  // 6路传感器的空间权重分配
  float weights[6] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};
  
  for (int i = 0; i < 6; i++) {
    if (digitalRead(IR_PINS[i]) == 1) {
      sum += weights[i];
      count++;
      currentSensorState |= (1 << (5 - i));
    }
  }

  // 1. 异常状态拦截：全白脱轨 (0个灯亮) -> 返回 100.0 触发找线
  if (count == 0) return 100.0;
  
  // 2. 异常状态拦截：终点停车 (5个或6个灯亮) -> 返回 200.0 触发停车
  if (count >= 5) return 200.0;

  // 3. 正常状态：计算质心偏移误差
  float currentError = sum / count;

  // 4. 【核心优化】针对细线的平滑补间逻辑
  // 如果当前只有 1 路触发，说明线卡在了传感器正下方，
  // 我们把它和上一次的误差做个平均，相当于手动拉高刷新率，防止 PID 反应过激
  if (count == 1) {
    currentError = (currentError + lastValidError) / 2.0;
  }

  lastValidError = currentError;
  lastSensorState = currentSensorState;
  
  return currentError;
}

// ---------------- 底层行为控制 ----------------
void handleOffTrack() {
  // 脱轨后，依靠大脑记忆最后一次的严重偏移方向原地转圈找线
  if (lastSensorState & 0b111000) {
    driveMotor(-60, 60);
  } else {
    driveMotor(60, -60); 
  }
}

void driveMotor(int powerL, int powerR) {
  int dirL = (powerL >= 0) ? 1 : 0;
  int dirR = (powerR >= 0) ? 1 : 0;
  
  int speedL = abs(powerL);
  int speedR = abs(powerR);
  
  // 【直线物理跑偏补偿】
  // 当双轮都向前，且计算出的速度差极小时，认为系统意图是跑直线，介入 Offset
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
  
  // 物理 PWM 占空比输出
  analogWrite(PWMA, speedL);
  analogWrite(PWMB, speedR);
}

void stopMotors() {
  analogWrite(PWMA, 0); 
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); 
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); 
  digitalWrite(BIN2, LOW);
}