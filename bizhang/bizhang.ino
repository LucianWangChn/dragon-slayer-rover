
// ==================== 引脚定义 (基于接线表状态B) ====================

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



// ==================== 运动参数配置 ====================

const int BASE_SPEED = 70;         // 基础前行速度 (建议不要太快，赛道窄)

const int TURN_SPEED = 70;        // 原地转向速度 (克服静摩擦力)

const int MOTOR_OFFSET = -10;      // 底盘电机固有偏差修正值，确保基准直线能力



const float Kp = 3.3;              // 居中比例系数 (需根据实车反应速度微调)



const int FRONT_TURN_DIST = 12;    // 前方遇到障碍物的转弯阈值 (考虑到车长25cm，需提早转)

const int SIDE_MAX_VALID = 30;     // 侧边超声波最大有效距离，防止赛道开口处误判



void setup() {

  Serial.begin(9600); // 预留硬件串口打印调试数据

 

  // 电机引脚初始化

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

 

  // 超声波引脚初始化

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);

  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);

  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

 

  delay(2000); // 开机缓冲，等待传感器稳定

}



void loop() {

  // 获取三路超声波距离 (单位: cm)

  float distL = getDistance(TRIG_L, ECHO_L);

  float distC = getDistance(TRIG_C, ECHO_C);

  float distR = getDistance(TRIG_R, ECHO_R);

 

  // 核心清洗逻辑：处理测不到 (-1) 或 距离过大的情况

  // 侧边超声波：测不到或太远，都视为达到了有效追踪的上限

  if (distL <= 0 || distL > SIDE_MAX_VALID) distL = SIDE_MAX_VALID;

  if (distR <= 0 || distR > SIDE_MAX_VALID) distR = SIDE_MAX_VALID;



  // 正前超声波：测不到说明正前方无限远，非常安全

  if (distC <= 0) distC = 999;



  // 1. 障碍物避让逻辑 (优先级最高)

  if (distC > 0 && distC < FRONT_TURN_DIST) {

    stopMotors();

    delay(100); // 刹车缓冲

   

    // 判断左右哪边宽敞，执行原地 45° 转弯

    if (distL > distR) {

      spinLeft();

      delay(250); // 【极客备注】此时间需根据TURN_SPEED和电池电压实测，刚好转45度为止

    } else {

      spinRight();

      delay(250); // 同上，调参重点

    }

    stopMotors();

    delay(100);

  }

  // 2. 直行与动态居中逻辑 (前方安全)

  else {

    // 简易P控制器：差值 = 左边距 - 右边距

    // 如果左边更近(distL变小)，差值为负，修正值 correction 变负

    // 左轮需要减速，右轮需要加速，促使车体向右微调

    float error = distL - distR;

    int correction = error * Kp;

   

    // 限制修正幅度，防止矫枉过正导致撞墙

    correction = constrain(correction, -40, 40);



    // 叠加偏差修正与居中修正

    int speedL = BASE_SPEED + correction;

    int speedR = BASE_SPEED - correction + MOTOR_OFFSET;



    // 限幅保护

    speedL = constrain(speedL, 0, 255);

    speedR = constrain(speedR, 0, 255);



    moveForward(speedL, speedR);

  }

 

  delay(20); // 循环周期，防止超声波发波冲突

}



// ==================== 传感器驱动 ====================

float getDistance(int trig, int echo) {

  digitalWrite(trig, LOW); delayMicroseconds(2);

  digitalWrite(trig, HIGH); delayMicroseconds(10);

  digitalWrite(trig, LOW);

  float duration = pulseIn(echo, HIGH, 30000); // 超时设置为30ms(约5米)，防卡死

  if (duration == 0) return 999; // 未测到返回大值

  return duration * 0.017;

}



// ==================== 底盘运动学 (已修正反向问题) ====================

void moveForward(int spdL, int spdR) {

  // 状态反转：LOW在前，HIGH在后

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, spdL);

  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, spdR);

}



void spinLeft() { // 原地左转

  // 左轮倒转，右轮正转

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, TURN_SPEED);

  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, TURN_SPEED);

}



void spinRight() { // 原地右转

  // 左轮正转，右轮倒转

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, TURN_SPEED);

  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, TURN_SPEED);

}



void stopMotors() { // 紧急制动 (保持不变)

  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);

}