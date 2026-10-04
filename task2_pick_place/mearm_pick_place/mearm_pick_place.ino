/*
 * ============================================================
 *  meArm 机械臂 · 任务二：A/B/C 物体夹取与放置
 *  考核方向：算法（控制、视觉）方向 · 二面考核
 * ============================================================
 *
 * ────────────────────────────────────────────────────────────
 * 【这个程序要解决的问题】
 * ────────────────────────────────────────────────────────────
 * 机械臂前方摆着 A、B、C 三个物体和一个放置区，
 * 要求机械臂【自动】依次把 A、B、C 抓起来，放到放置区。
 * 这就是"取放任务"（pick & place），工业机器人最常见的动作之一。
 *
 * ────────────────────────────────────────────────────────────
 * 【编程思路（核心思想：点位 + 动作序列）】
 * ────────────────────────────────────────────────────────────
 * 机器人动作编程最经典的方法 ——
 *   第一步：把空间里的"关键位置"定成【点位】。
 *           一个点位 = 4 个舵机角度（底座/大臂/小臂/夹爪），
 *           只要四个角度确定，机械臂姿态就确定了。
 *   第二步：把任务拆成【动作序列】（一串点位顺序移动）。
 *           夹取一个物体 = 6 步：
 *             ① 到抬升位（安全高度，防撞）
 *             ② 下降到取物位（对准物体）
 *             ③ 合爪（抓住）
 *             ④ 回到抬升位（带起物体）
 *             ⑤ 转到放置位（对准放置区）
 *             ⑥ 开爪（放下）+ 回抬升位
 *   第三步：写一个 moveTo() 函数，专门负责"四轴同步走到某点位"，
 *           再把 6 步串起来，任务就完成了。
 *
 * 在代码里用什么表示"点位"？用 struct（结构体）：
 *   struct Point { int base; int shoulder; int elbow; int gripper; };
 *   一个 Point 变量就能装下 4 个角度，像一摞四个抽屉的盒子。
 *
 * ────────────────────────────────────────────────────────────
 * 【为什么点位角度要实测后填？】
 * ────────────────────────────────────────────────────────────
 * 每个物体在什么角度、放置区在什么角度，取决于你的桌面摆放，
 * 只有你的机械臂自己知道。先用 mearm_range_test 测出关节范围，
 * 再手动把机械臂拨到"对准物体"的角度，记下来填进 POS_A 等常量。
 * 下面给出的是占位值（都能编译，但物理上不一定对准）。
 * ============================================================
 * 【硬件接线】（官方基准）
 * ============================================================
 *   底座 D9 / 大臂 D7 / 小臂 D8 / 夹爪 D6
 * ============================================================
 * 【测试方法】（串口监视器 9600）
 * ============================================================
 *   A    → 夹取物体 A，放到 A 的放置位
 *   B    → 夹取物体 B，放到 B 的放置位
 *   C    → 夹取物体 C，放到 C 的放置位
 *   run  → 依次执行 A、B、C
 *   home → 回到初始姿态（全部 90°）
 *   注：大小写都认（程序内部统一转成小写再比较）
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
 * 点位参数（用 range_test 实测后填入！）
 * ------------------------------------------------------------
 * struct：自定义"数据类型"。定义一个叫 Point 的类型，
 * 里面装 4 个 int：base(底座)、shoulder(大臂)、elbow(小臂)、gripper(夹爪)。
 * 有了它，一个变量就能完整描述"机械臂的一个姿态"。
 * ============================================================ */
struct Point { int base; int shoulder; int elbow; int gripper; };

/* 夹爪开合角度：0°=张开，180°=闭合（方向反了就对调） */
const int GRIPPER_OPEN  = 0;
const int GRIPPER_CLOSE = 180;

/* 关键点位（都是占位值，按你的实际摆放修改）：
 *   POS_A/B/C ：三个物体的取物位（现在分在底座 30° / 90° / 150° 三个方向上）
 *   POS_PLACE_A/B/C：三个【互不相同】的放置位。
 *       考核要求「放置到另一位置，三个明显不同的位置，且与物体在 x 与 y
 *       方向上都有明显位移」。用底座角拉开 x 向位移、用大臂/小臂角拉开
 *       y 向位移，三个放置位彼此也分得开。
 *   POS_LIFT  ：通用的"抬升位"—— 夹取/放置前的安全高度，
 *               角度设置成让机械臂抬起来、不会撞到物体的姿态。 */
const Point POS_A    = { 30,  60,  90,  GRIPPER_OPEN };  // 物体 A 位置
const Point POS_B    = { 90,  60,  90,  GRIPPER_OPEN };  // 物体 B 位置
const Point POS_C    = { 150, 60,  90,  GRIPPER_OPEN };  // 物体 C 位置
const Point POS_PLACE_A = { 55,  95,  115, GRIPPER_OPEN };// A 的放置位（偏右后）
const Point POS_PLACE_B = { 90,  110, 130, GRIPPER_OPEN };// B 的放置位（正前更远）
const Point POS_PLACE_C = { 125, 95,  115, GRIPPER_OPEN };// C 的放置位（偏左后）
const Point POS_LIFT = { 90,  45,  120, GRIPPER_OPEN };  // 通用抬升（安全高度）

const int STEP_DELAY = 15;   // 每走 1° 的间隔（毫秒），越小动作越快
const int GRIP_WAIT  = 300;  // 夹爪动作后的停顿（毫秒）。
                             // 给夹爪 0.3 秒夹稳/放稳，防止物体滑落。

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

String serialBuffer = "";   // 串口接收缓冲

/* ============================================================
 * setup()：开机执行一次
 * ============================================================ */
void setup() {
  // 【顺序很重要】串口必须先初始化，否则后面 goHome() 里的打印会全部丢失
  Serial.begin(9600);

  // 绑定四个舵机
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoGripper.attach(PIN_GRIPPER);

  goHome();   // 上电先回初始姿态（全部 90°），保证动作从已知状态开始

  Serial.println("READY | 指令: A/B/C=夹取对应物体  run=依次夹取三个  home=回初始位");
}

/* ============================================================
 * loop()：主循环 —— 接收串口指令
 * ============================================================ */
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        String cmd = serialBuffer;
        serialBuffer = "";

        cmd.trim();               // 去首尾空格
        cmd.toLowerCase();        // 统一转小写：RUN 和 run 都能识别

        // 考核要求：上位机发送 "A" / "B" / "C" 分别夹取对应物体
        if (cmd == "a") {
          pickAndPlace(POS_A, POS_PLACE_A, "A");
        } else if (cmd == "b") {
          pickAndPlace(POS_B, POS_PLACE_B, "B");
        } else if (cmd == "c") {
          pickAndPlace(POS_C, POS_PLACE_C, "C");
        } else if (cmd == "run") {
          pickAndPlaceAll();      // 依次夹取 A、B、C（连做三次）
        } else if (cmd == "home") {
          goHome();               // 回初始姿态
        } else if (cmd.length() > 0) {
          Serial.println("未知指令: 支持 A / B / C / run / home");
        }
      }
    } else {
      serialBuffer += c;
    }
  }
}

/* ============================================================
 * pickAndPlaceAll()：任务主流程 —— 依次夹取 A、B、C 并放置
 * ------------------------------------------------------------
 * 这里体现"函数复用"：pickAndPlace() 处理"一个物体"的完整流程，
 * 对 A、B、C 各调用一次就完成任务。代码只写一遍，用三次。
 * ============================================================ */
void pickAndPlaceAll() {
  Serial.println("== 任务二开始：依次夹取 A、B、C ==");

  pickAndPlace(POS_A, POS_PLACE_A, "A");   // 夹取 A 放到 A 的放置位
  pickAndPlace(POS_B, POS_PLACE_B, "B");   // 夹取 B 放到 B 的放置位
  pickAndPlace(POS_C, POS_PLACE_C, "C");   // 夹取 C 放到 C 的放置位

  goHome();                   // 全部完成后回初始姿态
  Serial.println("== 任务二完成 ==");
}

/* ============================================================
 * pickAndPlace()：单个物体的完整取放流程
 * ------------------------------------------------------------
 * 参数：
 *   pickPos ：这个物体的取物点位
 *   name    ：物体名字（"A"/"B"/"C"，仅用于打印）
 *
 * 六步流程（前面思路里讲过）：
 *   抬升位 → 取物位 → 合爪 → 抬升 → 放置位 → 开爪 → 抬升
 * 为什么取物前先经过抬升位？
 *   直接从一个物体飞到另一个物体容易撞到，先统一升到安全高度，
 *   再水平移动，最后才下降 —— 这是工业机器人取放的标准安全路径。
 * ============================================================ */
void pickAndPlace(Point pickPos, Point placePos, const char* name) {
  Serial.print("-- 夹取物体 "); Serial.println(name);

  // ① 先到抬升位（安全高度）
  moveTo(POS_LIFT);

  // ② 夹爪张开，然后下降到取物位（对准物体）
  moveGripper(GRIPPER_OPEN);       // 先张开，避免合着爪子撞到物体
  moveTo(pickPos);                 // 四轴同步走到取物点位

  // ③ 合爪抓住物体，停顿一下让爪夹稳
  moveGripper(GRIPPER_CLOSE);
  delay(GRIP_WAIT);

  // ④ 回到抬升位（带着物体升起来，避免拖拽刮到桌面）
  moveTo(POS_LIFT);

  // ⑤ 转到【该物体专属的】放置位，下降
  moveTo(placePos);

  // ⑥ 开爪放下物体，停顿让物体落稳
  moveGripper(GRIPPER_OPEN);
  delay(GRIP_WAIT);

  // ⑦ 回抬升位，一个物体处理完毕
  moveTo(POS_LIFT);

  Serial.print("-- 物体 "); Serial.print(name); Serial.println(" 放置完成");
}

/* ============================================================
 * moveTo()：四关节【同步】平滑移动到目标点位
 * ------------------------------------------------------------
 * 和任务一多舵机程序里的 moveSync 同一套思路：
 * 每个舵机每轮走 1°，四个 write() 连着执行 + 统一 delay = 同步。
 * 参数 p 是一个 Point 结构体，里面装着 4 个目标角度。
 * ============================================================ */
void moveTo(Point p) {
  // 只要还有任何一个关节没到位就继续
  while (curBase != p.base || curShoulder != p.shoulder ||
         curElbow != p.elbow || curGripper != p.gripper) {

    // 底座朝目标走 1°
    if (curBase < p.base)         curBase++;
    else if (curBase > p.base)    curBase--;

    // 大臂朝目标走 1°
    if (curShoulder < p.shoulder) curShoulder++;
    else if (curShoulder > p.shoulder) curShoulder--;

    // 小臂朝目标走 1°
    if (curElbow < p.elbow)       curElbow++;
    else if (curElbow > p.elbow)  curElbow--;

    // 夹爪朝目标走 1°
    if (curGripper < p.gripper)   curGripper++;
    else if (curGripper > p.gripper) curGripper--;

    // 四个 write 连着执行 → 同步输出
    servoBase.write(curBase);
    servoShoulder.write(curShoulder);
    servoElbow.write(curElbow);
    servoGripper.write(curGripper);

    delay(STEP_DELAY);   // 统一延时 → 速度一致
  }
  delay(200);   // 到位后额外停 0.2 秒，让机械臂稳定下来（防止晃动）
}

/* ============================================================
 * moveGripper()：只移动夹爪（开/合）
 *   单独写一个函数，因为夹爪经常要单独动作（开、关、开、关）。
 * ============================================================ */
void moveGripper(int target) {
  while (curGripper != target) {
    if (curGripper < target) curGripper++;
    else                     curGripper--;
    servoGripper.write(curGripper);
    delay(STEP_DELAY);
  }
}

/* ============================================================
 * goHome()：回到初始姿态（全部 90°）
 * ============================================================ */
void goHome() {
  Point home = { 90, 90, 90, 90 };   // 组装一个"全 90°"的点位
  moveTo(home);
  Serial.println("已回初始位 (90,90,90,90)");
}
