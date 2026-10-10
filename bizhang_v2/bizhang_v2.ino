// ==================== 引脚定义 (与接线表完全一致) ====================
// 底盘驱动引脚 (左轮)
#define PWMA 5
#define AIN1 7
#define AIN2 8

// 底盘驱动引脚 (右轮)
#define PWMB 6
#define BIN1 9
#define BIN2 10

// 超声波雷达引脚
#define TRIG_L 2
#define ECHO_L 3
#define TRIG_C 4
#define ECHO_C 12
#define TRIG_R A4
#define ECHO_R A5

// 状态机宏定义
#define LEFT 0
#define RIGHT 1

// ==================== 🏆 核心调参区 (重点关注这里) ====================

// 1. 基础运动参数
const int BASE_SPEED = 70;         // 直行基础PWM (建议70-90)
const int TURN_SPEED = 80;         // 原地转弯PWM (需足以克服静摩擦力)
const int MOTOR_OFFSET_L = -10;    // 左电机固有偏差修正 (根据实测，左电机给-10跑直线)
const int MOTOR_OFFSET_R = 0;      // 右电机固有偏差修正

// 2. 空间与尺寸阈值 (单位: cm)
const float TARGET_DIST = 7.5;     // 理想单边距墙距离 [(30-15)/2]
const float WALL_LOST = 15.0;      // 墙壁丢失阈值 (超过此值判定为路口/出口)
const float FRONT_STOP = 13.0;     // 正前遇障刹车阈值 (考虑到车长25cm，中心旋转需要的安全距离)
const float CRITICAL_STOP = 7.0;   // 极限防撞刹车阈值 (死胡同兜底，保证U型弯不被打断)
const float ROUNDABOUT_ENTRY = 15.0; // 判定环岛中间柱子的距离阈值

// 3. PD 控制器参数 (比例Kp，微分Kd)
const float Kp_c = 3;            // 居中巡线-比例系数 (纠偏力度)
const float Kd_c = 15;           // 居中巡线-微分系数 (抑制震荡，预测趋势)
const float Kp_s = 5.0;          // 单边贴墙-比例系数 (贴墙通常需要比居中更强的纠偏)
const float Kd_s = 1.5;          // 单边贴墙-微分系数

// 4. 盲转时间参数 (单位: ms)
const int TIME_90_DEGREE = 360;    // 原地转90度所需时间 (核心调参项)
const int TIME_45_DEGREE = 180;    // 原地转45度所需时间
const int SHIELD_TIME = 1000;      // 环岛入环后屏蔽右侧超声波的时间 (防早退)

// ==================== 全局变量 ====================
float last_error_c = 0;
float last_error_s = 0;
int obstacle_debounce_count = 0; // 危险防抖计数器

// 墙壁丢失确认计时器与动作锁
unsigned long wall_lost_time_L = 0;
unsigned long wall_lost_time_R = 0;
const unsigned long WALL_LOST_CONFIRM_MS = 100; // 确认墙壁真丢失需要的时间 (毫秒)
bool has_executed_45_L = false; // 防止死循环旋转的动作锁
bool has_executed_45_R = false;

// 当前状态记录
enum State { CENTERING, SINGLE_WALL_L, SINGLE_WALL_R };
State currentState = CENTERING;

// 函数声明
void handle_roundabout_right_entry();
void execute_turn_90(int dir);
void execute_turn_45(int dir);
void state_single_wall(int side, float dL, float dR);
void state_centering(float dL, float dR);
void set_motors(int spdL, int spdR);
void spinLeft();
void spinRight();
void stopMotors();
float getDistance(int trig, int echo);

// ==================== 初始化 ====================
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  delay(2000); // 开机缓冲，等待传感器和陀螺仪稳定
}

// ==================== 主循环 ====================
void loop() {
  float distL = getDistance(TRIG_L, ECHO_L);
  float distC = getDistance(TRIG_C, ECHO_C);
  float distR = getDistance(TRIG_R, ECHO_R);

  // --- 数据清洗 ---
  if (distL <= 0 || distL > 40) distL = 40;
  if (distR <= 0 || distR > 40) distR = 40;
  if (distC <= 0) distC = 999;

  // --- 状态机判断逻辑 ---

  // 【防抖滤波计算】
  bool is_front_dangerous = ((distC < FRONT_STOP && (distL > WALL_LOST || distR > WALL_LOST)) || distC < CRITICAL_STOP);
  if (is_front_dangerous) {
    obstacle_debounce_count++;
  } else {
    obstacle_debounce_count = 0;
  }

  // 优先级 1: 环岛入口判定 (前方有柱子，两侧同时变宽)
  if (distC < ROUNDABOUT_ENTRY && distL > WALL_LOST && distR > WALL_LOST) {
    handle_roundabout_right_entry();
    if (currentState != CENTERING) { last_error_c = 0; } // 状态切换清零误差
    currentState = CENTERING;
  }
  
  // 优先级 2: 正前方被堵 (直角路口 / 死胡同防撞) - 加入防抖
  else if (obstacle_debounce_count >= 2) {
    if (currentState == SINGLE_WALL_L) {
      execute_turn_90(RIGHT);
    } else if (currentState == SINGLE_WALL_R) {
      execute_turn_90(LEFT);
    } else {
      if (distL > distR) execute_turn_90(LEFT);
      else execute_turn_90(RIGHT);
    }
    obstacle_debounce_count = 0; // 清零防抖
    wall_lost_time_L = 0; wall_lost_time_R = 0; // 转弯后清理计时器
    has_executed_45_L = false; has_executed_45_R = false;
    if (currentState != CENTERING) { last_error_c = 0; }
    currentState = CENTERING; // 转弯后重置为居中
  }

  // 以下逻辑合并到同一个 else 链中，保持互斥性
  else {
    // 维护丢失计时器
    if (distL > WALL_LOST) {
      if (wall_lost_time_L == 0) wall_lost_time_L = millis();
    } else {
      wall_lost_time_L = 0; 
      has_executed_45_L = false; 
    }

    if (distR > WALL_LOST) {
      if (wall_lost_time_R == 0) wall_lost_time_R = millis();
    } else {
      wall_lost_time_R = 0; 
      has_executed_45_R = false; 
    }

    // 增加前方视野守卫：只有前方足够开阔（>31cm），才允许提前切弯。否则留给90度直角防撞处理。
    const float SAFE_CUT_DIST = 31.0; 
    bool is_real_lost_L = (wall_lost_time_L > 0 && millis() - wall_lost_time_L > WALL_LOST_CONFIRM_MS && distC > SAFE_CUT_DIST);
    bool is_real_lost_R = (wall_lost_time_R > 0 && millis() - wall_lost_time_R > WALL_LOST_CONFIRM_MS && distC > SAFE_CUT_DIST);

    // 优先级 3: 主动预测切弯
    if (is_real_lost_L && !has_executed_45_L && !is_real_lost_R) {
      execute_turn_45(LEFT);
      has_executed_45_L = true; 
      if (currentState != SINGLE_WALL_R) { last_error_s = 0; }
      currentState = SINGLE_WALL_R; 
    }
    else if (is_real_lost_R && !has_executed_45_R && !is_real_lost_L) {
      execute_turn_45(RIGHT);
      has_executed_45_R = true; 
      if (currentState != SINGLE_WALL_L) { last_error_s = 0; }
      currentState = SINGLE_WALL_L;
    }
    
    // 优先级 4: 单侧变宽期间的过渡
    else if (distL > WALL_LOST && distR <= WALL_LOST) {
      if (currentState != SINGLE_WALL_R) { last_error_s = 0; }
      currentState = SINGLE_WALL_R;
      state_single_wall(RIGHT, distL, distR);
    }
    else if (distR > WALL_LOST && distL <= WALL_LOST) {
      if (currentState != SINGLE_WALL_L) { last_error_s = 0; }
      currentState = SINGLE_WALL_L;
      state_single_wall(LEFT, distL, distR);
    }
    
    // 优先级 5: 默认状态 (居中直行) 或 完全无墙保护
    else {
          if (currentState != CENTERING) { last_error_c = 0; }
          currentState = CENTERING;
          state_centering(distL, distR);
    }
  }

  delay(20); 
}

// ==================== 执行动作库 ====================

// 居中巡线 PD 控制
void state_centering(float dL, float dR) {
  float error = dL - dR; 
  float adjust = Kp_c * error + Kd_c * (error - last_error_c);
  last_error_c = error;

  // 【修复控制方向】
  // 如果 error > 0 (dL > dR)，说明车偏右，需要向左转 (左减速，右加速)
  int speedL = BASE_SPEED + MOTOR_OFFSET_L + adjust; 
  int speedR = BASE_SPEED + MOTOR_OFFSET_R - adjust; 

  set_motors(speedL, speedR);
}

// 单边贴墙 PD 控制
void state_single_wall(int side, float dL, float dR) {
  float error;
  if (side == RIGHT) {
    error = TARGET_DIST - dR; // dR变大，error变小(负数)
  } else {
    error = dL - TARGET_DIST; // dL变大，error变大(正数)
  }

  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;

  // 如果贴右墙，车离右墙远了 (dR > TARGET)，error < 0。我们需要向右转 (左加速，右减速)。
  // 如果 adjust = Kp_s * error (负数)，那么 speedL 应该减去 adjust(变成加上绝对值)，speedR 应该加上 adjust(变成减去绝对值)。
  int speedL = BASE_SPEED + MOTOR_OFFSET_L + adjust;
  int speedR = BASE_SPEED + MOTOR_OFFSET_R - adjust;

  set_motors(speedL, speedR);
}

// 执行 90 度直角转弯 (盲转)
void execute_turn_90(int dir) {
  stopMotors();
  delay(100); 

  if (dir == RIGHT) {
    spinRight();
  } else {
    spinLeft();
  }

  delay(TIME_90_DEGREE); 
  stopMotors();
  delay(50);

  last_error_c = 0; 
  last_error_s = 0;
}

// 执行 45 度切弯 (主动预测)
void execute_turn_45(int dir) {
  if (dir == RIGHT) {
    spinRight();
  } else {
    spinLeft();
  }

  delay(TIME_45_DEGREE);

  last_error_c = 0;
  last_error_s = 0;
}

void handle_roundabout_right_entry() {
  stopMotors();
  delay(100);

  spinRight();
  delay(TIME_45_DEGREE);

  stopMotors();
  delay(50);

  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(300); 

  last_error_s = 0;
  unsigned long enter_time = millis();

  while (true) {
    float dR = getDistance(TRIG_R, ECHO_R);
    if (dR <= 0 || dR > 40) dR = 40;

    if (millis() - enter_time > SHIELD_TIME && dR > WALL_LOST) {
      break;
    }

    state_single_wall(RIGHT, 0, dR);
    delay(20);
  }

  stopMotors(); delay(100);
  spinRight(); delay(TIME_90_DEGREE);

  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(400);
}

// ==================== 底层驱动 ====================

void set_motors(int spdL, int spdR) {
  spdL = constrain(spdL, 0, 255);
  spdR = constrain(spdR, 0, 255);

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spdL);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spdR);
}

// ==================== 底层转向驱动 ====================

void spinLeft() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, TURN_SPEED);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, TURN_SPEED);
}

void spinRight() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, TURN_SPEED);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, TURN_SPEED);
}

void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float duration = pulseIn(echo, HIGH, 4000); 
  if (duration == 0) return 999; 
  return duration * 0.017;
}