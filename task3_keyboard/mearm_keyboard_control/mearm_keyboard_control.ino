/*
 * ============================================================
 *  meArm 机械臂 · 任务三：按键遥控板控制
 *  考核方向：算法（控制、视觉）方向 · 二面考核
 *  说明：任务三联动电路方向（电路方向制作按键遥控板），
 *       本程序是控制侧的代码：按下按键 → 对应关节转动。
 * ============================================================
 *
 * ────────────────────────────────────────────────────────────
 * 【这个程序要解决的问题】
 * ────────────────────────────────────────────────────────────
 * 不做任何编程操作（不开串口），直接用【实体按键】遥控机械臂：
 *   按下"底座键" → 底座转动
 *   按下"大臂键" → 大臂转动
 *   按住不放     → 持续转动，松开就停
 * 这才是"遥控"的感觉 —— 不需要电脑，插上电就能玩。
 *
 * ────────────────────────────────────────────────────────────
 * 【编程思路（两个知识点）】
 * ────────────────────────────────────────────────────────────
 * 知识点一：按键怎么接、怎么读？
 *   - 按键一端接 Arduino 引脚，另一端接 GND（地线）；
 *   - 引脚用 pinMode(引脚, INPUT_PULLUP) 开启【内部上拉】，
 *     内部上拉 = 引脚内部自动接到 +5V，不按时读到高电平 HIGH(1)，
 *     按下去（连通到 GND）读到低电平 LOW(0)；
 *   - 这样按键电路不需要外接任何电阻，一根线就搞定。
 *   - 判断按键：digitalRead(引脚) == LOW 就是"被按着"。
 *
 * 知识点二：怎么实现"按住持续转、松开即停"？
 *   - loop() 每圈都检查 4 个按键；
 *   - 哪个按键是 LOW（按着），就让对应关节当前角度 +1 或 -1，
 *     然后 write() 输出，再 delay(10) 休息 10 毫秒；
 *   - loop 无限循环 → 按键按多久，加角度就持续多久；
 *   - 松开 → 读到 HIGH → 不执行转动 → 舵机停在原地。
 *
 * ────────────────────────────────────────────────────────────
 * 【方向与边界】
 * ────────────────────────────────────────────────────────────
 * - 每个按键的转动方向由 DIR_xxx 决定：1=角度增大，-1=角度减小，
 *   想换方向只改这一个数；
 * - constrain() 把角度限制在安全范围，防止转过头撞坏结构。
 * ============================================================
 * 【硬件接线】（默认接法，按你的按键板修改）
 * ============================================================
 *   舵机：底座 D9 / 大臂 D7 / 小臂 D8 / 夹爪 D6
 *   按键（一端接引脚、一端接 GND）：
 *     B1 → D2  控制底座
 *     B2 → D3  控制大臂
 *     B3 → D10 控制小臂
 *     B4 → D11 控制夹爪
 *   【注意】D4/D5 是官方摇杆的按钮引脚（摇杆模块已占用），
 *           D6~D9 是舵机，所以按键选 D2/D3/D10/D11 避开冲突。
 * ============================================================
 */

#include <Servo.h>        // 引入舵机库

/* ============================================================
 * 舵机引脚定义（官方基准）
 * ============================================================ */
const int PIN_BASE     = 9;    // 底座舵机信号线（base → D9）
const int PIN_SHOULDER = 7;    // 大臂舵机信号线（D7）
const int PIN_ELBOW    = 8;    // 小臂舵机信号线（D8）
const int PIN_GRIPPER  = 6;    // 夹爪舵机信号线（claw → D6）

/* ============================================================
 * 按键引脚定义（默认接法，按你的按键板接线修改）
 * ============================================================ */
const int KEY_BASE     = 2;    // 底座按键 → D2
const int KEY_SHOULDER = 3;    // 大臂按键 → D3
const int KEY_ELBOW    = 10;   // 小臂按键 → D10
const int KEY_GRIPPER  = 11;   // 夹爪按键 → D11

/* ============================================================
 * 按键方向设置
 * ------------------------------------------------------------
 * DIR = 1   → 按下时角度 +1°/步（比如大臂"抬起"）
 * DIR = -1  → 按下时角度 -1°/步（比如底座"向左转"）
 * 如果按下后方向反了，改这里的 1 和 -1 即可。
 * ============================================================ */
const int DIR_BASE     = -1;   // 底座：按下左转（角度减小）
const int DIR_SHOULDER = 1;    // 大臂：按下抬起（角度增大）
const int DIR_ELBOW    = 1;    // 小臂：按下抬起（角度增大）
const int DIR_GRIPPER  = 1;    // 夹爪：按下闭合（角度增大）

/* 关节角度边界（用 mearm_range_test 实测后收紧，防止转过头） */
const int BASE_MIN     = 0;
const int BASE_MAX     = 180;
const int SHOULDER_MIN = 0;
const int SHOULDER_MAX = 180;
const int ELBOW_MIN    = 0;
const int ELBOW_MAX    = 180;
const int GRIPPER_MIN  = 0;
const int GRIPPER_MAX  = 180;

const int STEP_DELAY = 10;   // 连续转动时每步间隔（毫秒）。
                             // 调小 → 转得快；调大 → 转得慢（建议先 20 试手感）。

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

/* ============================================================
 * setup()：开机执行一次
 * ============================================================ */
void setup() {
  // 绑定四个舵机
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoGripper.attach(PIN_GRIPPER);

  // 上电回 90°（中间位置）
  servoBase.write(90);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoGripper.write(90);

  // 四个按键引脚全部开启内部上拉
  pinMode(KEY_BASE, INPUT_PULLUP);
  pinMode(KEY_SHOULDER, INPUT_PULLUP);
  pinMode(KEY_ELBOW, INPUT_PULLUP);
  pinMode(KEY_GRIPPER, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("READY | 按键遥控板：按住按键转动对应关节");
}

/* ============================================================
 * loop()：主循环 —— 逐个检查按键，按住的关节持续转动
 * ------------------------------------------------------------
 * 四个 if 块结构完全一样，以底座为例解释：
 *   digitalRead(KEY_BASE) == LOW
 *     → 按键被按下（上拉模式下按下=低电平）
 *   curBase += DIR_BASE
 *     → 当前角度朝指定方向走 1°
 *   constrain(curBase, BASE_MIN, BASE_MAX)
 *     → 把角度夹在安全范围内
 *   servoBase.write(curBase)
 *     → 舵机转到新角度
 *   delay(STEP_DELAY)
 *     → 休息 10ms，再回到循环开头检查下一次
 * 因为 loop 是无限循环，按着不放 = 每秒约 100 次"走 1°"，
 * 就形成了连续转动。松开 = 跳过 if = 角度不变 = 舵机停住。
 * ============================================================ */
void loop() {
  /* ---- 底座按键 ---- */
  if (digitalRead(KEY_BASE) == LOW) {
    curBase += DIR_BASE;                          // 按方向走 1°
    curBase = constrain(curBase, BASE_MIN, BASE_MAX);   // 边界保护
    servoBase.write(curBase);                     // 输出
    delay(STEP_DELAY);                            // 控制速度
  }

  /* ---- 大臂按键 ---- */
  if (digitalRead(KEY_SHOULDER) == LOW) {
    curShoulder += DIR_SHOULDER;
    curShoulder = constrain(curShoulder, SHOULDER_MIN, SHOULDER_MAX);
    servoShoulder.write(curShoulder);
    delay(STEP_DELAY);
  }

  /* ---- 小臂按键 ---- */
  if (digitalRead(KEY_ELBOW) == LOW) {
    curElbow += DIR_ELBOW;
    curElbow = constrain(curElbow, ELBOW_MIN, ELBOW_MAX);
    servoElbow.write(curElbow);
    delay(STEP_DELAY);
  }

  /* ---- 夹爪按键 ---- */
  if (digitalRead(KEY_GRIPPER) == LOW) {
    curGripper += DIR_GRIPPER;
    curGripper = constrain(curGripper, GRIPPER_MIN, GRIPPER_MAX);
    servoGripper.write(curGripper);
    delay(STEP_DELAY);
  }

  /* 所有按键都松开时：四个 if 都不执行，舵机保持当前角度不动 */
}

/* ============================================================
 * 【扩展说明】
 *   如果按键板是 8 键（每个关节"正转+反转"两个键）：
 *     复制任意一个 if 块，把 KEY_xx 改成新引脚、
 *     DIR_xx 取反（1 变 -1），就是一个反向控制键。
 *   如果想加"速度档"，仿照任务一固定指令程序里的
 *     速度档变量 + H/L 键即可。
 * ============================================================ */
