#include <Arduino.h>

// ==================== 引脚定义 ====================
// 底盘驱动 (左轮)
#define PWMA 5
#define AIN1 7
#define AIN2 8

// 底盘驱动 (右轮)
#define PWMB 6
#define BIN1 9
#define BIN2 10

// 超声波传感器
#define TRIG_L 2
#define ECHO_L 3
#define TRIG_C 4
#define ECHO_C 12          // 接 PCINT4 (PB4)，由引脚变化中断接管测距
#define TRIG_R A4
#define ECHO_R A5

#define LEFT  0
#define RIGHT 1

// ==================== 贴墙 PD 参数 ====================
const int   BASE_SPEED      = 60;
const int   TURN_SPEED      = 60;
const int   MOTOR_OFFSET_L  = 5;
const int   MOTOR_OFFSET_R  = -5;

const float TARGET_DIST     = 9.0;   // 贴墙目标距离 (cm)
const float WALL_LOST       = 15.0;  // 墙壁消失判定阈值
const float Kp_s            = 4.5;   // 贴墙 PD 比例系数
const float Kd_s            = 10.0;  // 贴墙 PD 微分系数

// ==================== 前方避障参数 ====================
const float FRONT_DANGER    = 9.0;  // 前方遇障触发距离 (cm)
const int   BACKUP_t       = 300;   // 倒退时长
const int   TURN_45_t      = 300;   // 原地转 45° 时长

// ==================== 全局状态 ====================
float        last_error_s = 0;        // 贴墙 PD 上次误差

// ---- 前方超声波中断相关 ----
volatile unsigned long echo_rise_us = 0;    // Echo 上升沿时刻
volatile float         center_dist  = 999;  // 前方距离 (ISR 更新)
volatile bool          front_danger = false;

const float STOP_DISTANCE  = 30.0;  // 三路超声波均超此距离，认为脱离迷宫
const int   STOP_DURATION  = 1000;  // 持续超距超过此时间 (ms)，CPU 空闲停转

// ==================== 函数声明 ====================
void   state_follow_right(float dR);
void   state_follow_left(float dL);
void   evade_front(float distL, float distR);
void   set_motors(int spdL, int spdR);
void   reverse_motors(int spd);
void   spin_left();
void   spin_right();
void   stop_motors();
float  get_distance(int trig, int echo);

// ==================== 初始化 ====================
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // ---- 配置 D12 引脚变化中断 (PCINT4, PCI0 组) ----
  PCICR  |= (1 << PCIE0);   // 使能 PCI0 组 (D8～D13)
  PCMSK0 |= (1 << PCINT4);  // 只响应 D12 (PB4) 的电平变化

  delay(2000);
}

// ==================== D12 引脚变化中断 (前方超声波 Echo) ====================
ISR(PCINT0_vect) {
  static uint8_t prev = 0;
  uint8_t cur   = PINB;
  uint8_t diff  = cur ^ prev;
  prev = cur;

  if (!(diff & (1 << PB4))) return;       // 不是 D12 的变化，放行电机引脚

  if (cur & (1 << PB4)) {                 // 上升沿 → 开始计时
    echo_rise_us = micros();
  } else {                                // 下降沿 → 计算距离
    if (echo_rise_us == 0) return;
    unsigned long dur = micros() - echo_rise_us;
    echo_rise_us = 0;
    if (dur == 0 || dur > 30000) return;  // 异常值丢弃

    float dist = dur * 0.017;
    center_dist = dist;
    if (dist > 0 && dist < FRONT_DANGER) {
      front_danger = true;
    }
  }
}

// ==================== 主循环 ====================
void loop() {
  // 1. 读取左右超声波 (轮询)
  float distL = get_distance(TRIG_L, ECHO_L);
  float distR = get_distance(TRIG_R, ECHO_R);
  if (distL <= 0 || distL > 40) distL = 40;
  if (distR <= 0 || distR > 40) distR = 40;

  // 2. 脱离迷宫检测：三路全超 STOP_DISTANCE 并持续 STOP_DURATION → 空闲停转
  float distC;
  noInterrupts();
  distC = center_dist;
  interrupts();

  static unsigned long lost_since = 0;

  if (distL > STOP_DISTANCE && distC > STOP_DISTANCE && distR > STOP_DISTANCE) {
    if (lost_since == 0) lost_since = millis();
    if (millis() - lost_since >= STOP_DURATION) {
      stop_motors();
      while (true) { delay(1000); }
    }
  } else {
    lost_since = 0;
  }

  // 3. 定期触发前方超声波 (中断会异步设置 front_danger)
  static unsigned long last_trig_ms = 0;
  if (millis() - last_trig_ms >= 50) {
    digitalWrite(TRIG_C, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_C, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_C, LOW);
    last_trig_ms = millis();
  }

  // 3. 前方遇障 → 倒车 + 转向避障
  if (front_danger) {
    evade_front(distL, distR);
    front_danger = false;
    last_error_s = 0;
  }
  // 4. 贴墙巡线: 两侧都在 → 跟右墙; 单侧在 → 跟该侧; 都无 → 直行
  else if (distL <= WALL_LOST && distR <= WALL_LOST) {
    state_follow_right(distR);
  } else if (distL <= WALL_LOST) {
    state_follow_left(distL);
  } else if (distR <= WALL_LOST) {
    state_follow_right(distR);
  } else {
    set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  }

  delay(20);
}

// ==================== 贴墙 PD 算法 ====================

// 右侧贴墙 (右侧超声波反馈，误差 = 目标距离 - 右侧距离)
void state_follow_right(float dR) {
  float error  = TARGET_DIST - dR;
  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;
  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust,
             BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// 左侧贴墙 (左侧超声波反馈，误差 = 左侧距离 - 目标距离)
void state_follow_left(float dL) {
  float error  = dL - TARGET_DIST;
  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;
  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust,
             BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// ==================== 前方避障动作 ====================
void evade_front(float distL, float distR) {
  // 1. 刹停
  stop_motors();
  delay(100);

  // 2. 倒退一段
  reverse_motors(BASE_SPEED);
  delay(BACKUP_t);
  stop_motors();
  delay(100);

  // 3. 向开阔侧转 45°
  if (distL > distR) {
    spin_left();
  } else {
    spin_right();
  }
  delay(TURN_45_t);
  stop_motors();
  delay(50);
}

// ==================== 电机底层驱动 ====================

void set_motors(int spdL, int spdR) {
  analogWrite(PWMA, constrain(spdL, 0, 255));
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);  // 左轮正转
  analogWrite(PWMB, constrain(spdR, 0, 255));
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);  // 右轮正转
}

void reverse_motors(int spd) {
  analogWrite(PWMA, constrain(spd, 0, 255));
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);   // 左轮反转
  analogWrite(PWMB, constrain(spd, 0, 255));
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);   // 右轮反转
}

void spin_left() {
  analogWrite(PWMA, TURN_SPEED);
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);  // 左轮正转
  analogWrite(PWMB, TURN_SPEED);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);   // 右轮反转
}

void spin_right() {
  analogWrite(PWMA, TURN_SPEED);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);   // 左轮反转
  analogWrite(PWMB, TURN_SPEED);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);  // 右轮正转
}

void stop_motors() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// ==================== 超声波测距 (轮询，仅左右) ====================
float get_distance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  float dur = pulseIn(echo, HIGH, 4000);
  return (dur == 0) ? 999 : dur * 0.017;
}
