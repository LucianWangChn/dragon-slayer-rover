// ==================== 引脚定义 ====================
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

// 状态机方向宏定义
#define LEFT 0
#define RIGHT 1

// ==================== 🏆 你的环岛核心参数 (最新) ====================
const int BASE_SPEED = 60;         
const int TURN_SPEED = 60;         
const int MOTOR_OFFSET_L = 5;    
const int MOTOR_OFFSET_R = -5;      

const float TARGET_DIST = 9;         // 贴右侧柱子/墙壁的目标距离
const float WALL_LOST = 15.0;        // 判定墙壁/柱子消失的阈值
const float ROUNDABOUT_ENTRY = 21.0; // 判定环岛中间柱子的前方距离阈值

// 右侧贴墙 PD 控制参数
const float Kp_s = 4.0; 
const float Kd_s = 10.0;

const int TIME_90_DEGREE = 500;      // 原地转90度时间
const int TIME_45_DEGREE = 300;      // 原地转45度时间(用于切入与切出)
const int SHIELD_TIME = 1000;        // 入环后屏蔽时间，防止把刚才路口的空缺当成出口

// ==================== 🛠️ 居中与直角弯参数 ====================
const float FRONT_STOP = 10.0;       // 正前遇障刹车阈值 (直角弯判定)
const float Kp_c = 3.0;              // 居中巡线-比例系数 (更新为 2.0)
const float Kd_c = 15.0;             // 居中巡线-微分系数 (更新为 10.0)

float last_error_s = 0;
float last_error_c = 0;              // 居中误差记录

// ⭐ 新增：待转状态记忆锁
int pending_turn = -1;               // -1: 无待转, LEFT: 准备左转, RIGHT: 准备右转

// ==================== 初始化 ====================
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  delay(2000); 
}

// ==================== 主循环 ====================
void loop() {
  float distL = getDistance(TRIG_L, ECHO_L);
  float distC = getDistance(TRIG_C, ECHO_C);
  float distR = getDistance(TRIG_R, ECHO_R);

  // 基础数据清洗
  if (distL <= 0 || distL > 40) distL = 40;
  if (distR <= 0 || distR > 40) distR = 40;
  if (distC <= 0) distC = 999;

  // 【优先级 1：检测到环岛】
  if (distC < ROUNDABOUT_ENTRY && distL > WALL_LOST && distR > WALL_LOST) {
    handle_roundabout(); 
    pending_turn = -1; // 执行完环岛，清空待转记忆
  } 
  // 【优先级 2：正前方到头了，执行直角弯】
  else if (distC < FRONT_STOP) {
    if (pending_turn == LEFT) {
      execute_turn_90(LEFT);
    } else if (pending_turn == RIGHT) {
      execute_turn_90(RIGHT);
    } else {
      // 兜底逻辑：如果两边墙同时没了，谁宽往哪转
      if (distL > distR) execute_turn_90(LEFT);
      else execute_turn_90(RIGHT);
    }
    pending_turn = -1; // 转弯完成，清空待转记忆
  } 
  // 【优先级 3：单边墙消失，进入“待转状态”并贴另一侧墙直行】
  else if (distL > WALL_LOST && distR <= WALL_LOST) {
    pending_turn = LEFT;             // 记住：准备左转
    state_single_wall_right(distR);  // 靠着右墙继续往前开，防止跑偏
  } 
  else if (distR > WALL_LOST && distL <= WALL_LOST) {
    pending_turn = RIGHT;            // 记住：准备右转
    state_single_wall_left(distL);   // 靠着左墙继续往前开，防止跑偏
  } 
  // 【优先级 4：两边都有墙，正常居中巡线】
  else if (distL <= WALL_LOST && distR <= WALL_LOST) {
    pending_turn = -1;               // 正常路段，取消任何误判的待转
    state_centering(distL, distR);
  }
  // 【优先级 5：大空旷地带兜底直行】
  else {
    set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  }

  delay(20); 
}

// ==================== 动作：居中与直角转弯 ====================

// 居中巡线 PD 控制
void state_centering(float dL, float dR) {
  float error = dL - dR; 
  float adjust = Kp_c * error + Kd_c * (error - last_error_c);
  last_error_c = error;

  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust, BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// 执行 90 度直角转弯
void execute_turn_90(int dir) {
  stopMotors(); delay(100); 
  
  if (dir == RIGHT) spinRight(); 
  else spinLeft();
  
  delay(TIME_90_DEGREE); 
  stopMotors(); delay(50);
  
  last_error_c = 0; 
  last_error_s = 0;
}


// ==================== 环岛处理全流程 (最新配置) ====================
void handle_roundabout() {
  // 1. 右转切弯入环
  stopMotors(); delay(100);
  spinRight(); delay(TIME_45_DEGREE);
  stopMotors(); delay(50);

  // 盲前开一小段，让右侧超声波对准中间柱子
  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(100); 

  // 2. 沿右侧直行 (贴中间柱子巡迹)
  last_error_s = 0;
  unsigned long enter_time = millis();
  
  while (true) {
    float dR = getDistance(TRIG_R, ECHO_R);
    if (dR <= 0 || dR > 40) dR = 40;

    // 判断切出时机：度过屏蔽期(防误判)，并且右侧柱子消失
    if (millis() - enter_time > SHIELD_TIME && dR > WALL_LOST) {
      break; 
    }

    // 执行右侧单边贴墙 PD 控制
    state_single_wall_right(dR);
    delay(20);
  }

  // 3. 切出环岛 (出环切弯已修改为 TIME_45_DEGREE)
  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(500);
  stopMotors(); delay(100);
  spinRight(); delay(TIME_45_DEGREE);
  
  // 切出后盲前开脱离环岛区域
  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(300);
}

// 右侧贴墙PD算法 (环岛及左墙消失时复用)
void state_single_wall_right(float dR) {
  float error = TARGET_DIST - dR;
  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;

  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust, BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// 左侧贴墙PD算法 (右墙消失时，贴着左墙直行防撞)
void state_single_wall_left(float dL) {
  float error = dL - TARGET_DIST;
  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;

  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust, BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// ==================== 底层驱动 ====================

void set_motors(int spdL, int spdR) {
  analogWrite(PWMA, constrain(spdL, 0, 255)); digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMB, constrain(spdR, 0, 255)); digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

void spinLeft() {
  analogWrite(PWMA, TURN_SPEED); digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMB, TURN_SPEED); digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}

void spinRight() {
  analogWrite(PWMA, TURN_SPEED); digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  analogWrite(PWMB, TURN_SPEED); digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
}

void stopMotors() {
  analogWrite(PWMA, 0); digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  analogWrite(PWMB, 0); digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  float duration = pulseIn(echo, HIGH, 4000); 
  return (duration == 0) ? 999 : duration * 0.017;
}