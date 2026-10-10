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

// ==================== 环岛核心参数 ====================
const int BASE_SPEED = 60;         
const int TURN_SPEED = 60;         
const int MOTOR_OFFSET_L = 5;    
const int MOTOR_OFFSET_R = -5;      

const float TARGET_DIST = 9;       // 贴右侧柱子/墙壁的目标距离
const float WALL_LOST = 15.0;        // 判定墙壁/柱子消失的阈值
const float ROUNDABOUT_ENTRY = 21.0; // 判定环岛中间柱子的前方距离阈值

// 右侧贴墙 PD 控制参数
const float Kp_s = 4.0; 
const float Kd_s = 10;

const int TIME_90_DEGREE = 600;      // 原地转90度时间(用于切出)
const int TIME_45_DEGREE = 300;      // 原地转45度时间(用于切入)
const int SHIELD_TIME = 1000;        // 入环后屏蔽时间，防止把刚才路口的空缺当成出口

float last_error_s = 0;

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

  // 【核心逻辑：检测到环岛】(前方遇柱子，且左右均无墙)
  if (distC < ROUNDABOUT_ENTRY && distL > WALL_LOST && distR > WALL_LOST) {
    handle_roundabout(); // 进入环岛完整动作链
  } else {
    // 平时默认直行，寻找环岛
    set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  }

  delay(20); 
}

// ==================== 环岛处理全流程 ====================
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

  // 3. 切出环岛
  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(500);
  stopMotors(); delay(100);
  spinRight(); delay(TIME_90_DEGREE);
  
  // 切出后盲前开脱离环岛区域
  set_motors(BASE_SPEED + MOTOR_OFFSET_L, BASE_SPEED + MOTOR_OFFSET_R);
  delay(300);
}

// 仅用于环岛内的右侧贴墙PD算法
void state_single_wall_right(float dR) {
  float error = TARGET_DIST - dR;
  float adjust = Kp_s * error + Kd_s * (error - last_error_s);
  last_error_s = error;

  set_motors(BASE_SPEED + MOTOR_OFFSET_L + adjust, BASE_SPEED + MOTOR_OFFSET_R - adjust);
}

// ==================== 底层驱动 ====================

void set_motors(int spdL, int spdR) {
  analogWrite(PWMA, constrain(spdL, 0, 255)); digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  analogWrite(PWMB, constrain(spdR, 0, 255)); digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
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