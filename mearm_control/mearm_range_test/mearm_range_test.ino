/*
 * ============================================================
 *  meArm 机械臂 · 关节活动范围测试程序
 *  以奇果派官方示例（code.ino）为基准：底座=D9、大臂=D7、小臂=D8、夹爪=D6
 * ============================================================
 *
 * ────────────────────────────────────────────────────────────
 * 【这个程序是干嘛的？（为什么要先跑它）】
 * ────────────────────────────────────────────────────────────
 * 舵机标称能转 0°~180°，但装到机械臂上后，结构会挡住它：
 *   可能转到 150° 就顶到连杆了，可能转到 10° 就卡住了。
 * 如果不清楚实际范围就乱写角度，舵机会一直憋着劲"卡卡"响，
 * 轻则发抖，重则烧毁舵机、撞坏结构。
 *
 * 所以先跑本程序，把每个关节的【实际安全范围】测出来，
 * 再填进正式代码（比如 mearm_basic_control.ino）的角度常量里。
 *
 * ────────────────────────────────────────────────────────────
 * 【编程思路】
 * ────────────────────────────────────────────────────────────
 * 本质就是一个"串口遥控器"：
 *   - 接收两个信息：① 哪个关节（a/b/c/d 字母）② 目标角度（数字）
 *   - 解析出来 → 让对应关节一格一格（每步 1°）慢慢转过去
 *   - 每走一步都打印当前角度，方便你盯着看"从几度开始卡"
 *
 * 指令格式：
 *   关节字母 + 角度  → 转到指定角度  例：a90  b120  c45  d30
 *   只发字母         → 自动扫描 0→180→0  例：a    b    c    d
 *
 * 字母含义（和官方基准一致）：
 *   a = 底座（base → D9）
 *   b = 大臂（D7）
 *   c = 小臂（D8）
 *   d = 夹爪（claw → D6）
 *
 * ────────────────────────────────────────────────────────────
 * 【怎么判断活动范围】（测试方法）
 * ────────────────────────────────────────────────────────────
 *   1. 发 "a" 让底座自动扫描，眼睛盯着底座；
 *   2. 观察到"卡卡"抖动、吃力、异响、或顶到别的零件 → 记下当前角度；
 *   3. 这个角度就是机械极限，安全范围要往里收几度；
 *   4. 四个关节都测完，把范围填回正式代码。
 * ============================================================
 */

#include <Servo.h>        // 引入舵机库

/* ============================================================
 * 引脚定义（官方基准）
 * ============================================================ */
const int PIN_BASE     = 9;    // 底座舵机信号线（base → D9）
const int PIN_SHOULDER = 7;    // 大臂舵机信号线（D7）
const int PIN_ELBOW    = 8;    // 小臂舵机信号线（D8）
const int PIN_GRIPPER  = 6;    // 夹爪舵机信号线（claw → D6）

const int STEP_DELAY   = 20;   // 每走 1° 的间隔（毫秒）。
                               // 调大（如 50）走得慢，更好观察；调小更快。

/* ============================================================
 * 全局变量
 * ============================================================ */
Servo servoBase;        // 底座舵机对象
Servo servoShoulder;    // 大臂舵机对象
Servo servoElbow;       // 小臂舵机对象
Servo servoGripper;     // 夹爪舵机对象

int curBase     = 90;   // 底座当前角度
int curShoulder = 90;   // 大臂当前角度
int curElbow    = 90;   // 小臂当前角度
int curGripper  = 90;   // 夹爪当前角度

String serialBuffer = "";   // 串口接收缓冲（攒成一行再处理）

/* ============================================================
 * setup()：开机执行一次
 * ============================================================ */
void setup() {
  // 绑定四个舵机
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoGripper.attach(PIN_GRIPPER);

  // 全部回到 90°（中间位置）
  servoBase.write(90);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoGripper.write(90);

  // 串口初始化
  Serial.begin(9600);

  // 打印使用说明
  Serial.println("=============================================");
  Serial.println(" 关节活动范围测试 | 波特率 9600");
  Serial.println(" 指令格式: 关节字母+角度  (a90 / b120 / c45 / d30)");
  Serial.println(" 自动扫描: 只发关节字母   (a / b / c / d)");
  Serial.println(" 字母含义: a=底座  b=大臂  c=小臂  d=夹爪");
  Serial.println("=============================================");
  Serial.println("当前角度: a=90 b=90 c=90 d=90");
}

/* ============================================================
 * loop()：主循环 —— 逐字符接收，攒成一行后解析执行
 * ============================================================ */
void loop() {
  while (Serial.available() > 0) {       // 缓冲区有字符就取
    char c = Serial.read();

    if (c == '\n' || c == '\r') {        // 收到换行 → 一行输入完毕
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);     // 解析这一行
        serialBuffer = "";               // 清空缓冲
      }
    } else {
      serialBuffer += c;                 // 普通字符攒进缓冲
    }
  }
}

/* ============================================================
 * handleCommand()：解析一条指令
 * ------------------------------------------------------------
 * 例子：
 *   "a90"  → joint='a'，numPart="90"  → 底座转到 90°
 *   "b"    → joint='b'，numPart=""    → 大臂自动扫描
 * 解析技巧：
 *   cmd.charAt(0)       → 取第一个字符（关节字母）
 *   cmd.substring(1)    → 取第 1 个字符往后的所有内容（数字部分）
 *   numPart.length()==0 → 说明没带数字 → 自动扫描
 * ============================================================ */
void handleCommand(String cmd) {
  cmd.trim();                    // trim()：去掉首尾空格（防手误）
  if (cmd.length() == 0) return; // 空指令直接退出

  char joint = cmd.charAt(0);        // 关节字母（a/b/c/d）
  String numPart = cmd.substring(1); // 剩余数字部分

  if (numPart.length() == 0) {
    scanJoint(joint);                // 只有字母 → 自动扫描
  } else {
    int angle = numPart.toInt();     // 字符串 → 整数
    angle = constrain(angle, 0, 180);   // 限制在 0~180（安全）
    moveJointTo(joint, angle);       // 转到指定角度
  }
}

/* ============================================================
 * moveJointTo()：把指定关节一格一格移动到目标角度
 * ------------------------------------------------------------
 * switch-case 按字母分发到四个关节之一；
 * 每个分支做的事都一样：while 里每次 ±1°，write()，delay()。
 * ============================================================ */
void moveJointTo(char joint, int target) {
  switch (joint) {
    case 'a':                          // 底座
      Serial.print("底座 → "); Serial.println(target);
      while (curBase != target) {      // 没到目标就继续
        if (curBase < target) curBase++; else curBase--;
        servoBase.write(curBase);      // 每走 1° 输出一次
        delay(STEP_DELAY);
      }
      break;

    case 'b':                          // 大臂
      Serial.print("大臂 → "); Serial.println(target);
      while (curShoulder != target) {
        if (curShoulder < target) curShoulder++; else curShoulder--;
        servoShoulder.write(curShoulder);
        delay(STEP_DELAY);
      }
      break;

    case 'c':                          // 小臂
      Serial.print("小臂 → "); Serial.println(target);
      while (curElbow != target) {
        if (curElbow < target) curElbow++; else curElbow--;
        servoElbow.write(curElbow);
        delay(STEP_DELAY);
      }
      break;

    case 'd':                          // 夹爪
      Serial.print("夹爪 → "); Serial.println(target);
      while (curGripper != target) {
        if (curGripper < target) curGripper++; else curGripper--;
        servoGripper.write(curGripper);
        delay(STEP_DELAY);
      }
      break;

    default:
      Serial.println("ERR: 关节字母应为 a/b/c/d");
      return;                          // 非法字母直接退出
  }
  printCurrent();   // 移动完成，打印四个关节当前角度
}

/* ============================================================
 * scanJoint()：自动扫描指定关节 0°→180°→0°
 * ------------------------------------------------------------
 * 思路：先把它转到 0°，然后 for 循环 0、1、2、...、180
 * 每走 1° 都调用 moveJointTo（会打印角度），你盯着看：
 * 打印到哪个数时开始"卡卡"，那个数附近就是机械极限。
 * 扫到 180 后再反向扫回 0。
 * ============================================================ */
void scanJoint(char joint) {
  Serial.print("自动扫描关节 "); Serial.println(joint);

  moveJointTo(joint, 0);   // 先到 0° 起点

  // 从 0° 加到 180°
  for (int angle = 0; angle <= 180; angle++) {
    moveJointTo(joint, angle);
  }
  // 从 180° 减回 0°
  for (int angle = 180; angle >= 0; angle--) {
    moveJointTo(joint, angle);
  }

  Serial.print("关节 "); Serial.print(joint);
  Serial.println(" 扫描完成。请记录它在哪个角度开始卡顿/抖动。");
}

/* ============================================================
 * printCurrent()：打印四个关节的当前角度
 *   目的：每动一下都能看到实时数值，方便对照记录
 * ============================================================ */
void printCurrent() {
  Serial.print("当前: a="); Serial.print(curBase);
  Serial.print(" b="); Serial.print(curShoulder);
  Serial.print(" c="); Serial.print(curElbow);
  Serial.print(" d="); Serial.println(curGripper);
}
