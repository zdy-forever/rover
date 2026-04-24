#include <Servo.h>

/*
  =========================================================
  Rover maze navigation
  Version: ultrasonic + servo only
  Strategy: simplified topological DFS

  核心思想：
  1. 只在“路口”建立节点。
  2. 每个节点记录 left / front / right 哪些方向可走。
  3. 每个节点记录哪些方向已经尝试过。
  4. 优先直走，其次左转，其次右转。
  5. 如果当前节点所有方向都试过，就掉头回退。
  6. 没有 MPU6050 / 编码器，所以这是弱化版拓扑探索，不是精确建图。
  =========================================================
*/


// =========================================================
// 电机引脚定义
// =========================================================
const uint8_t LEFT_EN  = 6;
const uint8_t LEFT_IN1 = 13;
const uint8_t LEFT_IN2 = 12;

const uint8_t RIGHT_EN  = 11;
const uint8_t RIGHT_IN1 = 8;
const uint8_t RIGHT_IN2 = 7;


// =========================================================
// 舵机 + 超声波引脚定义
// =========================================================
const uint8_t SERVO_PIN = 3;
const uint8_t TRIG_PIN  = 5;
const uint8_t ECHO_PIN  = 4;


// =========================================================
// 舵机扫描角度
// =========================================================
const int SERVO_LEFT_ANGLE  = 180;
const int SERVO_FRONT_ANGLE = 90;
const int SERVO_RIGHT_ANGLE = 10;

Servo sonarServo;


// =========================================================
// 电机速度参数
// =========================================================
const int FORWARD_SPEED = 165;
const int CAREFUL_SPEED = 110;
const int TURN_SPEED    = 165;


// =========================================================
// 左右轮补偿
// 如果车直行偏右/偏左，调这里
// =========================================================
const int LEFT_TRIM  = 20;
const int RIGHT_TRIM = 0;


// =========================================================
// 时间参数
// =========================================================
const unsigned long CENTER_MOVE_STEP_MS = 150;
const unsigned long CAREFUL_MOVE_MS     = 130;
const unsigned long TURN_45_MS          = 650;
const unsigned long PAUSE_TIME_MS       = 100;
const unsigned long STOP_BEFORE_SCAN_MS = 80;


// =========================================================
// 超声波稳定测距参数
// =========================================================
const int SERVO_SETTLE_MS = 220;
const int SAMPLES = 11;
const int SAMPLE_GAP_MS = 45;
const float MEDIAN_WINDOW_CM = 6.0;


// =========================================================
// 行为判断参数
// =========================================================

// 前方小于这个距离，认为快撞墙 / 死路
const float FRONT_NEAR_CM = 7.0;

// 前方大于这个距离，认为前方可走
const float FRONT_OPEN_CM = 15.0;

// 左右距离大于这个值，认为侧边可走
const float SIDE_OPEN_CM = 11.0;

// 前方小于这个距离时，禁止居中
const float FRONT_CENTER_STOP_CM = 16.0;


// =========================================================
// 居中参数
// =========================================================
const float CENTER_TOLERANCE_CM = 1.5;
const float KP = 2.8;
const int MAX_CORRECTION = 35;

const float MIN_NORMAL_WIDTH_CM = 10.0;
const float MAX_NORMAL_WIDTH_CM = 24.0;
const float MAX_CENTER_DIFF_CM  = 8.0;


// =========================================================
// 简化 DFS 参数
// =========================================================
const int MAX_NODES = 20;

// 刚转弯后忽略路口检测几轮，避免重复识别同一个路口
int ignoreJunctionCounter = 0;
const int IGNORE_JUNCTION_STEPS_AFTER_TURN = 3;


// =========================================================
// 相对方向定义
// =========================================================
enum RelDir {
  DIR_LEFT,
  DIR_FRONT,
  DIR_RIGHT,
  DIR_BACK,
  DIR_NONE
};


// =========================================================
// 探索状态
// =========================================================
enum ExploreState {
  STATE_EXPLORE,
  STATE_RETURNING
};

ExploreState exploreState = STATE_EXPLORE;


// =========================================================
// 节点结构
// =========================================================
struct Node {
  bool active;

  bool leftAvailable;
  bool frontAvailable;
  bool rightAvailable;

  bool leftTried;
  bool frontTried;
  bool rightTried;
};

Node nodeStack[MAX_NODES];
int stackTop = -1;


// =========================================================
// 工具函数
// =========================================================
int clampSpeed(int val) {
  if (val < 0) return 0;
  if (val > 255) return 255;
  return val;
}


// =========================================================
// 电机控制函数
// =========================================================
void setSingleMotor(int enPin, int in1Pin, int in2Pin, int speedVal, int trimVal) {
  int speed = speedVal + trimVal;

  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } 
  else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, -speed);
  } 
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}


void setMotorSpeeds(int rightSpeed, int leftSpeed) {
  setSingleMotor(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, rightSpeed, RIGHT_TRIM);
  setSingleMotor(LEFT_EN,  LEFT_IN1,  LEFT_IN2,  leftSpeed,  LEFT_TRIM);
}


void stopMotors() {
  setMotorSpeeds(0, 0);
}


void moveForwardRaw(int speedVal) {
  setMotorSpeeds(speedVal, speedVal);
}


void turnLeftInPlaceRaw(int speedVal) {
  setMotorSpeeds(-speedVal, speedVal);
}


void turnRightInPlaceRaw(int speedVal) {
  setMotorSpeeds(speedVal, -speedVal);
}


// =========================================================
// 高层动作函数
// =========================================================
void driveStraightStep(int speedVal, unsigned long durationMs) {
  moveForwardRaw(speedVal);
  delay(durationMs);
  stopMotors();
  delay(PAUSE_TIME_MS);
}


void turnLeft45() {
  Serial.println("Turn 45 left");

  turnLeftInPlaceRaw(TURN_SPEED);
  delay(TURN_45_MS);

  stopMotors();
  delay(PAUSE_TIME_MS);
}


void turnRight45() {
  Serial.println("Turn 45 right");

  turnRightInPlaceRaw(TURN_SPEED);
  delay(TURN_45_MS);

  stopMotors();
  delay(PAUSE_TIME_MS);
}


void turnLeft90() {
  Serial.println("Turn 90 left");

  turnLeft45();
  turnLeft45();

  ignoreJunctionCounter = IGNORE_JUNCTION_STEPS_AFTER_TURN;

  // 转完后小步前进，离开墙角
  driveStraightStep(CAREFUL_SPEED, 120);
}


void turnRight90() {
  Serial.println("Turn 90 right");

  turnRight45();
  turnRight45();

  ignoreJunctionCounter = IGNORE_JUNCTION_STEPS_AFTER_TURN;

  driveStraightStep(CAREFUL_SPEED, 120);
}


void turnAround180() {
  Serial.println("Turn 180");

  turnRight90();
  turnRight90();

  ignoreJunctionCounter = IGNORE_JUNCTION_STEPS_AFTER_TURN;
}


// =========================================================
// 超声波辅助函数
// =========================================================
void swapFloat(float &a, float &b) {
  float temp = a;
  a = b;
  b = temp;
}


void sortArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[j] < arr[i]) {
        swapFloat(arr[i], arr[j]);
      }
    }
  }
}


float getMedian(float arr[], int n) {
  sortArray(arr, n);

  if (n % 2 == 1) {
    return arr[n / 2];
  } 
  else {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  }
}


float readDistanceCMOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }

  float distance = duration * 0.0343 / 2.0;

  if (distance < 2.0 || distance > 400.0) {
    return -1.0;
  }

  return distance;
}


float readDistanceStableAtCurrentAngle() {
  float validReadings[SAMPLES];
  int validCount = 0;

  delay(SERVO_SETTLE_MS);

  for (int i = 0; i < 2; i++) {
    readDistanceCMOnce();
    delay(SAMPLE_GAP_MS);
  }

  for (int i = 0; i < SAMPLES; i++) {
    float d = readDistanceCMOnce();

    if (d > 0) {
      validReadings[validCount] = d;
      validCount++;
    }

    delay(SAMPLE_GAP_MS);
  }

  if (validCount == 0) {
    return -1.0;
  }

  float tempForMedian[SAMPLES];

  for (int i = 0; i < validCount; i++) {
    tempForMedian[i] = validReadings[i];
  }

  float median = getMedian(tempForMedian, validCount);

  float filtered[SAMPLES];
  int filteredCount = 0;

  for (int i = 0; i < validCount; i++) {
    if (abs(validReadings[i] - median) <= MEDIAN_WINDOW_CM) {
      filtered[filteredCount] = validReadings[i];
      filteredCount++;
    }
  }

  if (filteredCount == 0) {
    return median;
  }

  sortArray(filtered, filteredCount);

  if (filteredCount >= 3) {
    float sum = 0.0;
    int count = 0;

    for (int i = 1; i < filteredCount - 1; i++) {
      sum += filtered[i];
      count++;
    }

    if (count > 0) {
      return sum / count;
    }
  }

  float sum = 0.0;

  for (int i = 0; i < filteredCount; i++) {
    sum += filtered[i];
  }

  return sum / filteredCount;
}


float readDistanceAtAngleStable(int angle) {
  sonarServo.write(angle);
  return readDistanceStableAtCurrentAngle();
}


void scanThreeDirections(float &leftDist, float &frontDist, float &rightDist) {
  stopMotors();
  delay(STOP_BEFORE_SCAN_MS);

  leftDist  = readDistanceAtAngleStable(SERVO_LEFT_ANGLE);
  frontDist = readDistanceAtAngleStable(SERVO_FRONT_ANGLE);
  rightDist = readDistanceAtAngleStable(SERVO_RIGHT_ANGLE);

  sonarServo.write(SERVO_FRONT_ANGLE);
  delay(SERVO_SETTLE_MS);
}


// =========================================================
// 开口判断
// =========================================================
bool isSideOpen(float dist) {
  if (dist < 0) return false;
  return dist > SIDE_OPEN_CM;
}


bool isFrontOpen(float dist) {
  if (dist < 0) return false;
  return dist > FRONT_OPEN_CM;
}


int countOpenDirections(float leftDist, float frontDist, float rightDist) {
  int count = 0;

  if (isSideOpen(leftDist)) count++;
  if (isFrontOpen(frontDist)) count++;
  if (isSideOpen(rightDist)) count++;

  return count;
}


bool isJunction(float leftDist, float frontDist, float rightDist) {
  if (ignoreJunctionCounter > 0) {
    return false;
  }

  int openCount = countOpenDirections(leftDist, frontDist, rightDist);

  // 两个或三个方向可走，认为是路口
  return openCount >= 2;
}


bool isDeadEnd(float leftDist, float frontDist, float rightDist) {
  bool leftOpen  = isSideOpen(leftDist);
  bool frontOpen = isFrontOpen(frontDist);
  bool rightOpen = isSideOpen(rightDist);

  return !leftOpen && !frontOpen && !rightOpen;
}


// =========================================================
// 是否允许居中
// 只在正常直走廊里居中，避免斜墙误导
// =========================================================
bool canUseCentering(float leftDist, float frontDist, float rightDist) {
  if (frontDist < 0) return false;

  if (frontDist < FRONT_CENTER_STOP_CM) return false;

  if (leftDist < 0 || rightDist < 0) return false;

  float totalWidth = leftDist + rightDist;
  float diff = abs(leftDist - rightDist);

  if (totalWidth > MAX_NORMAL_WIDTH_CM) return false;
  if (totalWidth < MIN_NORMAL_WIDTH_CM) return false;
  if (diff > MAX_CENTER_DIFF_CM) return false;

  return true;
}


// =========================================================
// 道路居中
// =========================================================
void keepLaneCenteredAndGo(float leftDist, float rightDist) {
  if (leftDist < 0 || rightDist < 0) {
    Serial.println("Center fallback: invalid side reading");
    driveStraightStep(CAREFUL_SPEED, CENTER_MOVE_STEP_MS);
    return;
  }

  float error = leftDist - rightDist;

  int correction = 0;

  if (abs(error) > CENTER_TOLERANCE_CM) {
    correction = (int)(KP * error);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  }

  // correction > 0：往左修
  // correction < 0：往右修
  int leftSpeed  = FORWARD_SPEED - correction;
  int rightSpeed = FORWARD_SPEED + correction;

  leftSpeed  = clampSpeed(leftSpeed);
  rightSpeed = clampSpeed(rightSpeed);

  Serial.print("Center | left=");
  Serial.print(leftDist);
  Serial.print(" right=");
  Serial.print(rightDist);
  Serial.print(" correction=");
  Serial.println(correction);

  setMotorSpeeds(rightSpeed, leftSpeed);
  delay(CENTER_MOVE_STEP_MS);

  stopMotors();
  delay(PAUSE_TIME_MS);
}


// =========================================================
// DFS 节点栈函数
// =========================================================
void pushNewNode(float leftDist, float frontDist, float rightDist) {
  if (stackTop >= MAX_NODES - 1) {
    Serial.println("Node stack full. Cannot push new node.");
    return;
  }

  stackTop++;

  nodeStack[stackTop].active = true;

  nodeStack[stackTop].leftAvailable  = isSideOpen(leftDist);
  nodeStack[stackTop].frontAvailable = isFrontOpen(frontDist);
  nodeStack[stackTop].rightAvailable = isSideOpen(rightDist);

  nodeStack[stackTop].leftTried  = false;
  nodeStack[stackTop].frontTried = false;
  nodeStack[stackTop].rightTried = false;

  Serial.print("Push node ");
  Serial.println(stackTop);

  Serial.print("Available | L=");
  Serial.print(nodeStack[stackTop].leftAvailable);
  Serial.print(" F=");
  Serial.print(nodeStack[stackTop].frontAvailable);
  Serial.print(" R=");
  Serial.println(nodeStack[stackTop].rightAvailable);
}


RelDir chooseUntriedDirection(Node &n) {
  /*
    终点可能在迷宫中间，所以优先直走。
    如果想更像左手法则，把顺序改成 left -> front -> right。
  */

  if (n.frontAvailable && !n.frontTried) {
    n.frontTried = true;
    return DIR_FRONT;
  }

  if (n.leftAvailable && !n.leftTried) {
    n.leftTried = true;
    return DIR_LEFT;
  }

  if (n.rightAvailable && !n.rightTried) {
    n.rightTried = true;
    return DIR_RIGHT;
  }

  return DIR_BACK;
}


void popNode() {
  if (stackTop >= 0) {
    Serial.print("Pop node ");
    Serial.println(stackTop);

    nodeStack[stackTop].active = false;
    stackTop--;
  }
}


// =========================================================
// 执行方向
// =========================================================
void executeDirection(RelDir dir) {
  if (dir == DIR_FRONT) {
    Serial.println("Execute: FRONT");
    driveStraightStep(CAREFUL_SPEED, 180);
  } 
  else if (dir == DIR_LEFT) {
    Serial.println("Execute: LEFT");
    turnLeft90();
  } 
  else if (dir == DIR_RIGHT) {
    Serial.println("Execute: RIGHT");
    turnRight90();
  } 
  else if (dir == DIR_BACK) {
    Serial.println("Execute: BACK");
    turnAround180();
  } 
  else {
    Serial.println("Execute: NONE");
    stopMotors();
  }
}


// =========================================================
// 当前节点决策
// =========================================================
void decideAtCurrentNode() {
  if (stackTop < 0) {
    Serial.println("No node in stack.");
    return;
  }

  RelDir dir = chooseUntriedDirection(nodeStack[stackTop]);

  if (dir == DIR_BACK) {
    Serial.println("All directions tried at this node. Backtracking.");

    popNode();

    executeDirection(DIR_BACK);
    exploreState = STATE_RETURNING;
    return;
  }

  executeDirection(dir);
  exploreState = STATE_EXPLORE;
}


// =========================================================
// 返回状态处理
// =========================================================
void handleReturning(float leftDist, float frontDist, float rightDist) {
  Serial.println("State: RETURNING");

  // 返回时如果再次看到路口，认为回到了上一个节点附近
  if (isJunction(leftDist, frontDist, rightDist)) {
    Serial.println("Returned to a junction.");

    exploreState = STATE_EXPLORE;

    // 如果还有上一个节点，继续在上一个节点选择未尝试方向
    if (stackTop >= 0) {
      decideAtCurrentNode();
    } 
    else {
      // 如果栈空了，就把这个路口当新起点
      pushNewNode(leftDist, frontDist, rightDist);
      decideAtCurrentNode();
    }

    return;
  }

  // 返回途中遇到前方墙，说明回退失败或方向偏了
  if (frontDist > 0 && frontDist < FRONT_NEAR_CM) {
    Serial.println("Returning blocked. Turn around and continue exploring.");
    turnAround180();
    exploreState = STATE_EXPLORE;
    return;
  }

  // 还没回到路口，继续往前走
  if (canUseCentering(leftDist, frontDist, rightDist)) {
    keepLaneCenteredAndGo(leftDist, rightDist);
  } 
  else {
    driveStraightStep(CAREFUL_SPEED, CAREFUL_MOVE_MS);
  }
}


// =========================================================
// 普通探索状态处理
// =========================================================
void handleExplore(float leftDist, float frontDist, float rightDist) {
  Serial.println("State: EXPLORE");

  // 如果看到路口，建立节点并做 DFS 决策
  if (isJunction(leftDist, frontDist, rightDist)) {
    Serial.println("Junction detected.");

    pushNewNode(leftDist, frontDist, rightDist);
    decideAtCurrentNode();
    return;
  }

  // 如果是死路
  if (frontDist > 0 && frontDist < FRONT_NEAR_CM) {
    Serial.println("Dead end or wall detected.");

    // 有节点历史：回退
    if (stackTop >= 0) {
      popNode();
      executeDirection(DIR_BACK);
      exploreState = STATE_RETURNING;
    } 
    else {
      // 没节点历史：普通转向
      turnAround180();
      exploreState = STATE_EXPLORE;
    }

    return;
  }

  // 普通直走廊：可以居中
  if (canUseCentering(leftDist, frontDist, rightDist)) {
    Serial.println("Safe corridor -> centering");
    keepLaneCenteredAndGo(leftDist, rightDist);
    return;
  }

  // 不确定区域：小步慢速前进
  Serial.println("Unsafe centering -> careful straight");
  driveStraightStep(CAREFUL_SPEED, CAREFUL_MOVE_MS);
}


// =========================================================
// 主决策函数
// =========================================================
void roverStep() {
  float leftDist, frontDist, rightDist;

  scanThreeDirections(leftDist, frontDist, rightDist);

  if (ignoreJunctionCounter > 0) {
    ignoreJunctionCounter--;
  }

  Serial.println("======================================");
  Serial.print("StackTop: ");
  Serial.print(stackTop);
  Serial.print(" | State: ");
  Serial.println(exploreState == STATE_EXPLORE ? "EXPLORE" : "RETURNING");

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | Front: ");
  Serial.print(frontDist);
  Serial.print(" cm | Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  if (exploreState == STATE_RETURNING) {
    handleReturning(leftDist, frontDist, rightDist);
  } 
  else {
    handleExplore(leftDist, frontDist, rightDist);
  }
}


// =========================================================
// setup
// =========================================================
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  sonarServo.attach(SERVO_PIN);
  sonarServo.write(SERVO_FRONT_ANGLE);

  digitalWrite(TRIG_PIN, LOW);

  stopMotors();

  for (int i = 0; i < MAX_NODES; i++) {
    nodeStack[i].active = false;
  }

  Serial.println("======================================");
  Serial.println("Rover started");
  Serial.println("Mode: ultrasonic + servo simplified topological DFS");
  Serial.println("======================================");
}


// =========================================================
// loop
// =========================================================
void loop() {
  roverStep();
}