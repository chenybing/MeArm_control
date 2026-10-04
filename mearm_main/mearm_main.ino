/* ============================================================================
 *  meArm 机械臂 · 主控固件（算法/控制方向 二面考核 · 整合工程）
 * ----------------------------------------------------------------------------
 *  本工程把「任务一 ~ 任务五」的控制代码整合进一个可直接编译、烧录、演示的
 *  Arduino 工程（对应提交要求：完整源代码 · 一份可复现视频效果的工程）。
 *
 *  上位机（串口监视器）或电路组的按键遥控板，通过串口 9600 发送指令，
 *  本程序解析指令后驱动机器人完成动作。
 *
 *  ── 指令表 ─────────────────────────────────────────────────────────────────
 *   【任务一】基础控制      M              进入 / 退出「摇杆实时操控」模式
 *   【任务一】固定指令      O / S          夹爪张开 / 闭合
 *                          H / L          速度档 +1 / -1（1~5 档，全局生效）
 *   【任务一】多舵机协同    x10,y30,z20    x=底座 y=大臂 z=小臂，同步转动
 *   【任务二】夹取放置      A / B / C      夹取对应物体，放到各自的放置位
 *                          RUN            依次夹取 A、B、C（连做三次）
 *   【任务三】按键遥控板    1              循环夹取：每按一次换下一个物体
 *                          2              录制开始 / 结束（摇杆操控，>10 秒）
 *                          3              回放上一次录制的动作
 *                          4              回中（回到初始姿态）
 *   【任务五】绘图          LINE           直线
 *                          N/TRI/Z/V      字母 N / 三角形 / 字母 Z / 字母 V
 *                          TEACH          进入五点示教模式（摇杆移动笔尖）
 *                          SET            记录当前笔尖位置为一个示教点
 *                          CLR            清空示教点
 *                          DRAW           按示教点画折线
 *                          CURVE          按示教点画平滑曲线（Catmull-Rom）
 *                          DEMO           用内置五点演示
 *                          PAUSE/RESUME/CANCEL   暂停 / 继续 / 取消
 *                          GO             回待机位
 *   【调试】                CAL            打印当前笔尖坐标（标定用）
 *                          JOG 大臂角 肘角  直接用连杆角驱动（标定用）
 *                          HELP           打印指令表
 *
 *  ── 接线（官方基准）────────────────────────────────────────────────────────
 *   舵机：底座 D9   大臂 D7   小臂 D8   夹爪 D6
 *   摇杆：X1=A0  Y1=A1   X2=A2  Y2=A3     摇杆按键：D5(摇杆1) / D4(摇杆2)
 *   本地按键（另一端接 GND，内部上拉）：
 *         暂停 D2   继续 D3   取消 D10   示教记录 D11
 *   ⚠ 舵机请用 5~6V/2A 外接电源并共地，只靠 USB 供电会抖动。
 *
 *  ── 标定步骤（必做，否则画不准）─────────────────────────────────────────────
 *   ① 量尺寸：ARM_L1(大臂)、ARM_L2(肘→笔尖)、H_SHOULDER(肩关节轴离桌面高度)
 *   ② 校方向：用 M 模式摇杆把大臂抬到「水平前伸」，执行 CAL 看打印的坐标。
 *             方向反了就翻 SHOULDER_INVERT / ELBOW_INVERT / BASE_INVERT；
 *             笔尖整体偏一个固定角度就调 SHOULDER_TRIM / ELBOW_TRIM。
 *   ③ 校落点：执行 JOG 0 0，此时笔尖应落在正前方水平线上，记录实际位置微调。
 *
 *  ── 坐标约定 ───────────────────────────────────────────────────────────────
 *   桌面坐标 (u, v, h)：u = 左右（右为正），v = 前后（远离底座为正），
 *   h = 笔尖/夹爪离桌面的高度（cm）。绘图时 h=0 表示笔尖压在纸面上。
 *   底座中位 BASE_ANG 对应的正前方为 v 轴。
 * ==========================================================================*/

#include <Servo.h>
#include <math.h>
#include <ctype.h>

/* ── 按键遥控板接入方式（任务三）──────────────────────────────────────────
 *  0 = 用硬件串口 D0/D1（推荐，最稳）
 *      接线：遥控板 TX → 机械臂 D0(RX)，遥控板 GND → 机械臂 GND
 *      ⚠ 演示任务三时必须【拔掉机械臂的 USB 线】，改由外接 5V 供电。
 *        否则板载 CH340 的 TX 和遥控板的 TX 会一起驱动 D0，互相打架。
 *  1 = 用软串口 D12(RX)/D13(TX)
 *      接线：遥控板 TX → D12，遥控板 RX → D13（可选，回状态给 OLED）
 *      好处：USB 可以一直连着电脑，任务一/二/五 的串口指令随时能发。
 */
#define REMOTE_SOFT_SERIAL 0

#if REMOTE_SOFT_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial remoteSerial(12, 13);   // RX, TX
#endif

/* =========================== 1. 引脚定义 ================================= */
const uint8_t PIN_BASE     = 9;    // 底座舵机
const uint8_t PIN_SHOULDER = 7;    // 大臂舵机
const uint8_t PIN_ELBOW    = 8;    // 小臂舵机
const uint8_t PIN_GRIPPER  = 6;    // 夹爪舵机

const uint8_t PIN_J1X = A0;        // 摇杆1 X
const uint8_t PIN_J1Y = A1;        // 摇杆1 Y
const uint8_t PIN_J2X = A2;        // 摇杆2 X
const uint8_t PIN_J2Y = A3;        // 摇杆2 Y
const uint8_t PIN_BTN1 = 5;        // 摇杆1 自带按键
const uint8_t PIN_BTN2 = 4;        // 摇杆2 自带按键

const uint8_t BTN_PAUSE  = 2;      // 任务五：暂停
const uint8_t BTN_GO     = 3;      // 任务五：继续
const uint8_t BTN_CANCEL = 10;     // 任务五：取消
const uint8_t BTN_TEACH  = 11;     // 任务五：记录示教点

/* =========================== 2. 机械参数（需标定）======================== */
const float ARM_L1 = 8.0;          // 大臂长度 cm：肩关节轴 → 肘关节轴
const float ARM_L2 = 8.0;          // 小臂长度 cm：肘关节轴 → 笔尖（不是腕中心！）
const float H_SHOULDER = 6.0;      // 肩关节轴离桌面的高度 cm（拿尺子量）

/* 舵机标定开关 —— 方向反了就翻 true/false，整体偏一个角度就调 TRIM */
const int  SHOULDER_TRIM   = 90;   // 大臂水平前伸时，舵机角应接近这个值
const bool SHOULDER_INVERT = true; // true → 舵机角 = TRIM - 连杆角
const int  ELBOW_TRIM      = 0;    // 肘部舵机零位
const bool ELBOW_INVERT    = true; // true → 舵机角 = TRIM - 肘角
const bool BASE_INVERT     = false;// 底座左右方向反了就改 true
const int  ELBOW_SIGN      = -1;   // 肘部解支：-1 = 小臂下垂解（桌面画图推荐）

const int BASE_ANG = 90;           // 底座中位（正前方）
const int BASE_SWEEP = 60;         // 底座左右最大摆幅（±度）

/* 桌面可达范围（防止指令把机械臂甩到够不着的地方） */
const float R_MIN = 3.0;           // 笔尖离底座轴最小半径 cm
const float R_MAX = 14.0;          // 最大半径 cm（需 ≤ ARM_L1+ARM_L2）

/* =========================== 3. 运行参数 ================================= */
const int   JOY_CENTER = 512;      // 摇杆中位读数
const int   JOY_DEADZONE = 40;     // 摇杆死区
const float TEACH_SPEED = 0.05;    // 示教模式：摇杆满偏时每步移动多少 cm（≈5cm/s）
const float PEN_GRIP = 60;         // 夹持铅笔的夹爪角度
const float PEN_LIFT = 2.0;        // 绘图时抬笔高度 cm
const float TEACH_H  = 0.3;        // 示教时笔尖离纸面的高度 cm
const float HOME_U = 0.0, HOME_V = 8.0, HOME_H = 4.0;   // 待机位
const int   GRIP_OPEN = 20, GRIP_CLOSE = 110;          // 夹爪张开 / 闭合角度

/* 速度档（任务一 H/L）：档位越高，每步延时越短，整体动作越快 */
int speedLevel = 3;                                    // 1~5
inline int stepDelay() { return 44 - speedLevel * 7; } // 档3 → 23ms

/* =========================== 4. 全局状态 ================================= */
Servo svBase, svShoulder, svElbow, svGripper;
int curBase = 90, curShoulder = 90, curElbow = 90, curGripper = 90;

enum Mode { MODE_IDLE, MODE_MANUAL, MODE_TEACH, MODE_REC };
Mode mode = MODE_IDLE;

bool stPause = false;              // 暂停标志
bool stAbort = false;              // 取消标志

/* ---- 串口行缓冲（不使用 String，避免内存碎片）----
 * 上位机 / 遥控板各用一个独立缓冲，互不干扰。                      */
const uint8_t LINE_MAX = 48;
const uint16_t LINE_GAP_MS = 40;   // 超过这么久没收到新字符就当作一条完整指令
struct LineCh {
  char     buf[LINE_MAX];
  uint8_t  len;
  uint32_t lastMs;
};
LineCh lcPC = { {0}, 0, 0 };       // 上位机（硬件串口）
LineCh lcRC = { {0}, 0, 0 };       // 按键遥控板（REMOTE_SOFT_SERIAL=1 时用软串口）

/* ---- 任务三：动作录制 ---- */
const uint16_t REC_MAX = 200;      // 最多 200 帧（SRAM 受限，勿盲目调大）
const uint16_t REC_MS  = 60;       // 每帧 60ms → 最长 12 秒（要求 >10 秒）
uint8_t  recBuf[REC_MAX][4];       // 每帧存 4 个舵机角度（各 1 字节）
uint16_t recCount = 0;             // 已录帧数
bool     recOn = false;
uint32_t recNextMs = 0;
uint8_t  cycleIdx = 0;             // 任务三按键1 的循环计数

/* ---- 任务五：五点示教 ---- */
const uint8_t TEACH_N = 5;
float teachU[TEACH_N], teachV[TEACH_N];
uint8_t teachCount = 0;
float tgU = 0, tgV = 0;            // 示教时笔尖目标的浮点累加器（见 teachStep）

/* ---- 按键扫描 ---- */
const uint8_t BTN_COUNT = 6;
const uint8_t BTN_PINS[BTN_COUNT] = {
  BTN_PAUSE, BTN_GO, BTN_CANCEL, BTN_TEACH, PIN_BTN2, PIN_BTN1
};
enum { B_PAUSE = 0, B_GO, B_CANCEL, B_TEACH, B_REC, B_HOME };
bool     btnEdge[BTN_COUNT];
bool     btnLast[BTN_COUNT];
uint32_t btnLock[BTN_COUNT];

/* ---- 运行状态名称 / 状态上报（遥控板可据此在 OLED 上显示）---- */
const char* modeName() {
  switch (mode) {
    case MODE_MANUAL: return recOn ? "REC" : "MANUAL";
    case MODE_TEACH:  return "TEACH";
    case MODE_REC:    return "REC";
    default:          return "IDLE";
  }
}
void printStat(const char *s) { Serial.print(F("STAT:")); Serial.println(s); }

/* =========================== 5. setup / loop ============================= */
void setup() {
  svBase.attach(PIN_BASE);
  svShoulder.attach(PIN_SHOULDER);
  svElbow.attach(PIN_ELBOW);
  svGripper.attach(PIN_GRIPPER);

  // 先摆到中间安全姿态，避免上电瞬间四个舵机乱甩
  curBase = curShoulder = curElbow = curGripper = 90;
  writeServos();

  for (uint8_t i = 0; i < BTN_COUNT; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("READY | meArm 主控 | 输入 HELP 查看指令表"));
  printStat("IDLE");
}

void loop() {
  pollSerial(false);     // 解析上位机 / 遥控板指令
  scanButtons();         // 刷新按键边沿

  switch (mode) {
    case MODE_MANUAL: joystickAll();       break;  // 任务一：摇杆实时操控
    case MODE_TEACH:  teachStep();         break;  // 任务五：摇杆示教
    case MODE_REC:    recordStep();        break;  // 任务三：摇杆 + 录制
    default:          handleLocalButtons(); break; // 空闲：处理本地按键
  }
}

/* =========================== 6. 串口接收 ================================= */
/* 逐字符攒成一行；收到换行、或间隔超时，即认为一条指令结束。
 * 这样既能接标准上位机发的 "A\r\n"，也能接遥控板发的、不带换行的 "1"。 */
void pollOne(Stream &s, LineCh &lc, bool inMotion) {
  while (s.available() > 0) {
    char c = s.read();
    lc.lastMs = millis();
    if (c == '\n' || c == '\r') {
      if (lc.len > 0) { lc.buf[lc.len] = 0; lc.len = 0; deliver(lc.buf, inMotion); }
    } else if (lc.len < LINE_MAX - 1) {
      lc.buf[lc.len++] = c;
    } else {
      lc.len = 0;
      Serial.println(F("ERR: 指令过长"));
    }
  }
  if (lc.len > 0 && millis() - lc.lastMs > LINE_GAP_MS) {
    lc.buf[lc.len] = 0; lc.len = 0; deliver(lc.buf, inMotion);
  }
}

void deliver(char *cmd, bool inMotion) {
  if (inMotion) dispatchInMotion(cmd);
  else          dispatch(cmd);
}

void pollSerial(bool inMotion) {
  pollOne(Serial, lcPC, inMotion);
#if REMOTE_SOFT_SERIAL
  pollOne(remoteSerial, lcRC, inMotion);
#endif
}

/* 运动过程中只受理「暂停 / 继续 / 取消 / 状态查询」，其余指令回 BUSY，
 * 避免在动作执行中途递归触发新动作。                               */
void dispatchInMotion(const char *cmd) {
  if      (eq(cmd, "PAUSE"))                    { stPause = true;  printStat("PAUSED"); }
  else if (eq(cmd, "RESUME") || eq(cmd, "GO"))  { stPause = false; printStat("DRAWING"); }
  else if (eq(cmd, "CANCEL"))                   { stAbort = true;  stPause = false; Serial.println(F("CANCEL...")); }
  else if (eq(cmd, "STATUS"))                   { printStat(modeName()); }
  else if (cmd[0])                              { Serial.println(F("BUSY: 动作执行中，仅支持 PAUSE/RESUME/CANCEL")); }
}

/* =========================== 7. 指令派发 ================================= */
void dispatch(char *cmd) {
  // 去掉首尾空白，并统一转大写（便于比较）
  char *p = cmd;
  while (*p == ' ' || *p == '\t') p++;
  char *e = p + strlen(p);
  while (e > p && (e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
  if (!*p) return;
  for (char *q = p; *q; q++) *q = toupper(*q);

  /* ---- 任务一：多舵机协同 x..,y..,z..（先判，避免与字母 Z 冲突）---- */
  if (p[0] == 'X' && strchr(p, ',')) { cmdMultiServo(p); return; }

  /* ---- 任务一：单字符指令 ---- */
  if (strlen(p) == 1) {
    switch (p[0]) {
      case 'M': mode = (mode == MODE_MANUAL) ? MODE_IDLE : MODE_MANUAL;
                stPause = stAbort = false;
                Serial.println(mode == MODE_MANUAL ? F("STAT:MANUAL 摇杆操控中")
                                                   : F("STAT:IDLE"));
                return;
      case 'O': cmdGripper(GRIP_OPEN,  "开爪"); return;
      case 'S': cmdGripper(GRIP_CLOSE, "合爪"); return;
      case 'H': if (speedLevel < 5) speedLevel++;
                Serial.print(F("速度档 = ")); Serial.println(speedLevel); return;
      case 'L': if (speedLevel > 1) speedLevel--;
                Serial.print(F("速度档 = ")); Serial.println(speedLevel); return;
      case 'A': pickAndPlace(0, "A"); return;      // 任务二
      case 'B': pickAndPlace(1, "B"); return;
      case 'C': pickAndPlace(2, "C"); return;
      case 'N': drawFigure('N'); return;           // 任务五：字母 N
      case 'Z': drawFigure('Z'); return;           // 任务五：字母 Z
      case 'V': drawFigure('V'); return;           // 任务五：字母 V
      case '1': cyclePick(); return;               // 任务三：按键1
      case '2': toggleRecord(); return;            // 任务三：按键2
      case '3': playRecord(); return;              // 任务三：按键3
      case '4': goHome(); return;                  // 任务三：按键4
      default: break;
    }
  }

  /* ---- 多字符指令 ---- */
  if (eq(p, "RUN"))        { pickAndPlace(0, "A"); pickAndPlace(1, "B"); pickAndPlace(2, "C"); }
  else if (eq(p, "LINE"))  drawLineDemo();
  else if (eq(p, "TRI"))   drawFigure('T');
  else if (eq(p, "TEACH")) enterTeach();
  else if (eq(p, "SET"))   recordTeachPoint();
  else if (eq(p, "CLR"))   { teachCount = 0; Serial.println(F("示教点已清空")); }
  else if (eq(p, "DRAW"))  drawByTeach(false);
  else if (eq(p, "CURVE")) drawByTeach(true);
  else if (eq(p, "DEMO"))  { loadDemoPoints(); drawByTeach(false); }
  else if (eq(p, "PAUSE")) { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  else if (eq(p, "RESUME")){ stPause = false; Serial.println(F("STAT:IDLE")); }
  else if (eq(p, "CANCEL")){ stAbort = true;  stPause = false; }
  else if (eq(p, "GO"))    goHome();
  else if (eq(p, "CAL"))   printCalib();
  else if (p[0] == 'J' && p[1] == 'O' && p[2] == 'G') cmdJog(p);
  else if (eq(p, "HELP"))  printHelp();
  else if (eq(p, "STATUS"))printStat(modeName());
  else { Serial.print(F("未知指令: ")); Serial.println(p); Serial.println(F("输入 HELP 查看指令表")); }
}


/* =========================== 8. 按键扫描 ================================= */
void scanButtons() {
  for (uint8_t i = 0; i < BTN_COUNT; i++) {
    bool now = (digitalRead(BTN_PINS[i]) == LOW);   // 上拉输入：按下 = LOW
    btnEdge[i] = false;
    if (now && !btnLast[i] && (millis() - btnLock[i] > 180)) {  // 180ms 消抖
      btnEdge[i] = true;
      btnLock[i] = millis();
    }
    btnLast[i] = now;
  }
}

/* 空闲状态下的本地按键（任务三 / 任务五的操作键） */
void handleLocalButtons() {
  if (btnEdge[B_REC])    toggleRecord();
  if (btnEdge[B_HOME])   goHome();
  if (btnEdge[B_TEACH] && mode == MODE_IDLE) Serial.println(F("提示：先发送 TEACH 进入示教模式"));
}

/* 运动过程中的按键：暂停 / 继续 / 取消 */
void handleMotionButtons() {
  if (btnEdge[B_PAUSE])  { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  if (btnEdge[B_GO])     { stPause = false; }
  if (btnEdge[B_CANCEL]) { stAbort = true;  stPause = false; }
}

/* =========================== 9. 运动基元 ================================= */
void writeServos() {
  svBase.write(curBase);
  svShoulder.write(curShoulder);
  svElbow.write(curElbow);
  svGripper.write(curGripper);
}

/* 每个运动步都调用：处理暂停 / 取消。
 * 返回 true 表示「已被取消」，调用者应立即中止当前动作。          */
bool stepGate() {
  scanButtons();
  handleMotionButtons();
  pollSerial(true);
  while (stPause && !stAbort) {          // 暂停：原地保持，直到继续或取消
    scanButtons();
    handleMotionButtons();
    pollSerial(true);
    delay(5);
  }
  return stAbort;
}

/* 四轴同步移动到目标角度；返回 false 表示被取消 */
bool moveJoints(int tB, int tS, int tE, int tG) {
  tB = constrain(tB, 0, 180); tS = constrain(tS, 0, 180);
  tE = constrain(tE, 0, 180); tG = constrain(tG, 0, 180);
  while (curBase != tB || curShoulder != tS || curElbow != tE || curGripper != tG) {
    if (curBase     < tB) curBase++;     else if (curBase     > tB) curBase--;
    if (curShoulder < tS) curShoulder++; else if (curShoulder > tS) curShoulder--;
    if (curElbow    < tE) curElbow++;    else if (curElbow    > tE) curElbow--;
    if (curGripper  < tG) curGripper++;  else if (curGripper  > tG) curGripper--;
    writeServos();
    if (stepGate()) return false;
    delay(stepDelay());
  }
  return true;
}

/* 末端移动到桌面坐标 (u,v)，高度 h；返回 false 表示被取消 */
bool moveToTable(float u, float v, float h) {
  int tB, tS, tE;
  if (!tableToJoints(u, v, h, tB, tS, tE)) return true;   // 不可达：已告警，跳过
  return moveJoints(tB, tS, tE, curGripper);
}

/* =========================== 10. 运动学 ================================= */
/* 桌面坐标 (u,v,h) → 关节角 (底座/大臂/小臂)，同时做可达性钳制。
 *   u = 左右（右正）  v = 前后（远离底座正）  h = 离桌面高度
 * 原理：
 *   ① 底座角 θ = atan2(u, v)  → 把机械臂转到目标所在竖直平面
 *   ② 该平面内臂展半径 r = √(u²+v²)，笔尖高度相对肩轴 z = h - H_SHOULDER
 *   ③ 二连杆逆解（余弦定理 + atan2）解出 大臂角 s 和 肘角 e
 * 返回 false 表示目标被钳到了可达边界（已打印提示）。            */
bool tableToJoints(float u, float v, float h, int &tB, int &tS, int &tE) {
  bool clipped = false;

  // ---- ① 底座 ----
  float theta = degrees(atan2(u, v));                 // 目标相对正前方的偏角
  int base = BASE_ANG + (int)round(BASE_INVERT ? -theta : theta);
  if (base < BASE_ANG - BASE_SWEEP) { base = BASE_ANG - BASE_SWEEP; clipped = true; }
  if (base > BASE_ANG + BASE_SWEEP) { base = BASE_ANG + BASE_SWEEP; clipped = true; }
  tB = constrain(base, 0, 180);

  // ---- ② 半径钳制 ----
  float r0 = sqrt(u * u + v * v);
  float r  = constrain(r0, R_MIN, R_MAX);
  if (r != r0) clipped = true;

  // ---- ③ 二连杆逆解 ----
  float z = h - H_SHOULDER;                           // 笔尖相对肩关节轴的高度
  float d = sqrt(r * r + z * z);
  if (d > ARM_L1 + ARM_L2 || d < fabs(ARM_L1 - ARM_L2)) {
    // 够不到：把目标点沿原方向缩到可达球面，保证机械臂朝目标方向走到极限
    float limit = (d > ARM_L1 + ARM_L2) ? (ARM_L1 + ARM_L2) * 0.995
                                        : fabs(ARM_L1 - ARM_L2) * 1.005;
    float k = limit / d;
    r *= k; z *= k; d = limit;
    clipped = true;
  }
  float cosE = (d * d - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) / (2.0 * ARM_L1 * ARM_L2);
  cosE = constrain(cosE, -1.0f, 1.0f);
  float e = ELBOW_SIGN * degrees(acos(cosE));         // 肘角（可正可负）
  float s = degrees(atan2(z, r)) - degrees(atan2(ARM_L2 * sin(radians(e)),
                                                ARM_L1 + ARM_L2 * cos(radians(e))));
  // ---- ④ 连杆角 → 舵机角 ----
  tS = constrain(SHOULDER_TRIM + (int)round(SHOULDER_INVERT ? -s : s), 0, 180);
  tE = constrain(ELBOW_TRIM    + (int)round(ELBOW_INVERT    ? -e : e), 0, 180);

  if (clipped) {
    Serial.print(F("WARN 目标超出可达范围，已就近处理: u=")); Serial.print(u);
    Serial.print(F(" v=")); Serial.print(v);
    Serial.print(F(" h=")); Serial.println(h);
  }
  return true;
}

/* 正解：关节角 → 桌面坐标（标定 / 示教时显示用） */
void jointsToTable(int b, int s, int e, float &u, float &v, float &h) {
  float cs = SHOULDER_INVERT ? -(s - SHOULDER_TRIM) : (s - SHOULDER_TRIM);
  float ce = ELBOW_INVERT    ? -(e - ELBOW_TRIM)    : (e - ELBOW_TRIM);
  float x = ARM_L1 * cos(radians(cs)) + ARM_L2 * cos(radians(cs + ce));
  float z = ARM_L1 * sin(radians(cs)) + ARM_L2 * sin(radians(cs + ce));
  float th = radians(BASE_INVERT ? -(b - BASE_ANG) : (b - BASE_ANG));
  h = z + H_SHOULDER;
  u = x * sin(th);
  v = x * cos(th);
}

/* =========================== 11. 任务一 ================================= */
/* 基础控制：摇杆实时操控四个关节（死区 + 平滑）
 * 轴与关节的对应关系沿用官方 code.ino：
 *   A0→底座  A1→小臂  A2→夹爪  A3→大臂                       */
void joystickAll() {
  int jBase     = analogRead(PIN_J1X);
  int jElbow    = analogRead(PIN_J1Y);
  int jGrip     = analogRead(PIN_J2X);
  int jShoulder = analogRead(PIN_J2Y);

  smoothJoint(curBase,     joyTarget(jBase,     curBase));
  smoothJoint(curShoulder, joyTarget(jShoulder, curShoulder));
  smoothJoint(curElbow,    joyTarget(jElbow,    curElbow));
  smoothJoint(curGripper,  joyTarget(jGrip,     curGripper));
  writeServos();
  delay(10);
}

/* 摇杆读数 → 目标角度：死区内不动，否则按满量程映射到 0~180 */
int joyTarget(int joy, int cur) {
  int off = joy - JOY_CENTER;
  if (off > -JOY_DEADZONE && off < JOY_DEADZONE) return cur;   // 死区：保持
  int ang = (int)((long)joy * 180L / 1023L);
  return constrain(ang, 0, 180);
}
/* 平滑逼近：每次只走差值的 25%，动作柔和 */
void smoothJoint(int &cur, int target) {
  cur += (int)((target - cur) * 0.25f);
  cur = constrain(cur, 0, 180);
}

/* 固定指令 O / S：夹爪开合 */
void cmdGripper(int target, const char *name) {
  Serial.print(F("CMD ")); Serial.println(name);
  moveJoints(curBase, curShoulder, curElbow, target);
}

/* 多舵机协同：x10,y30,z20 */
void cmdMultiServo(const char *p) {
  int vx = numAfter(p, 'x'), vy = numAfter(p, 'y'), vz = numAfter(p, 'z');
  if (vx < 0 || vy < 0 || vz < 0) {
    Serial.println(F("ERR: 格式应为 x角度,y角度,z角度  例: x10,y30,z20"));
    return;
  }
  vx = constrain(vx, 0, 180); vy = constrain(vy, 0, 180); vz = constrain(vz, 0, 180);
  Serial.print(F("CMD x=")); Serial.print(vx);
  Serial.print(F(" y="));    Serial.print(vy);
  Serial.print(F(" z="));    Serial.println(vz);
  if (moveJoints(vx, vy, vz, curGripper)) Serial.println(F("DONE"));
}

/* =========================== 12. 任务二：夹取与放置 ===================== */
/* 三个物体的取物位 / 放置位（桌面坐标 cm）。
 * 放置位与取物位在 u、v 两个方向上都有明显位移，且三个放置位互不相同。 */
const float PICK_U[3]  = { -5.0,  0.0,  5.0 };
const float PICK_V[3]  = {  8.5,  9.0,  8.5 };
const float PLACE_U[3] = { -2.5,  2.5, -5.0 };
const float PLACE_V[3] = { 12.5, 13.0, 12.0 };
const float GRIP_H     = 0.8;     // 夹取 / 放置时夹爪离桌面的高度
const float CARRY_H    = 3.5;     // 搬运时的抬升高度

void pickAndPlace(uint8_t idx, const char *name) {
  stAbort = false; stPause = false;
  Serial.print(F("-- 夹取物体 ")); Serial.println(name);

  moveToTable(HOME_U, HOME_V, HOME_H);                              // ① 抬到安全高度
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);            // ② 张开夹爪
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);                   // ③ 到取物位上方
  moveToTable(PICK_U[idx], PICK_V[idx], GRIP_H);                    // ④ 下降对准物体
  moveJoints(curBase, curShoulder, curElbow, GRIP_CLOSE);           // ⑤ 合爪抓取
  delay(300);                                                       //    等夹稳
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);                   // ⑥ 抬起
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);                 // ⑦ 转到放置位上方
  moveToTable(PLACE_U[idx], PLACE_V[idx], GRIP_H);                  // ⑧ 下降
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);            // ⑨ 开爪放下
  delay(300);                                                       //    等落稳
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);                 // ⑩ 抬起离开

  Serial.print(F("-- 物体 ")); Serial.print(name); Serial.println(F(" 放置完成"));
}

/* 任务三 · 按键1：每次按下循环切换 A → B → C → A … */
void cyclePick() {
  const char *names[3] = { "A", "B", "C" };
  Serial.print(F("== 按键1 循环夹取，本次: ")); Serial.print(names[cycleIdx]);
  Serial.println(F(" =="));
  pickAndPlace(cycleIdx, names[cycleIdx]);
  cycleIdx = (cycleIdx + 1) % 3;      // 下一次换下一个物体
}

/* =========================== 13. 任务三：录制 / 回放 / 回中 ============= */
void toggleRecord() {
  if (mode == MODE_REC) {                       // 第二次按下：结束并保存
    mode = MODE_IDLE; recOn = false;
    Serial.print(F("录制结束，共 ")); Serial.print(recCount);
    Serial.print(F(" 帧 ≈ ")); Serial.print(recCount * REC_MS / 1000.0, 1);
    Serial.println(F(" 秒"));
    if (recCount * REC_MS < 10000) {
      Serial.println(F("WARN 录制时长不足 10 秒，建议重新录制"));
    }
    printStat("IDLE");
  } else {                                      // 首次按下：开始录制
    if (mode != MODE_IDLE) { Serial.println(F("BUSY: 请先结束当前模式")); return; }
    recCount = 0; recOn = true; mode = MODE_REC;
    recNextMs = millis();
    Serial.println(F("== 开始录制：请用摇杆操控机械臂（>10 秒），再按一次结束 =="));
    printStat("REC");
  }
}

/* 录制态每个循环：摇杆驱动四关节 + 定时采样 */
void recordStep() {
  joystickAll();
  if (millis() >= recNextMs) {
    recNextMs += REC_MS;
    if (recCount < REC_MAX) {
      recBuf[recCount][0] = (uint8_t)curBase;
      recBuf[recCount][1] = (uint8_t)curShoulder;
      recBuf[recCount][2] = (uint8_t)curElbow;
      recBuf[recCount][3] = (uint8_t)curGripper;
      recCount++;
    } else {                                    // 缓冲满：自动结束，保住已录数据
      Serial.println(F("WARN 录制缓冲已满（约 12 秒），自动结束"));
      toggleRecord();
    }
  }
}

void playRecord() {
  if (recCount < 2) { Serial.println(F("ERR: 还没有录制数据，请先按 2 录制")); return; }
  if (mode != MODE_IDLE) { Serial.println(F("BUSY: 请先结束当前模式")); return; }

  stAbort = false; stPause = false;
  Serial.print(F("== 回放录制动作，")); Serial.print(recCount);
  Serial.print(F(" 帧 ≈ ")); Serial.print(recCount * REC_MS / 1000.0, 1);
  Serial.println(F(" 秒 =="));
  printStat("PLAY");

  for (uint16_t i = 0; i < recCount; i++) {
    curBase     = recBuf[i][0];
    curShoulder = recBuf[i][1];
    curElbow    = recBuf[i][2];
    curGripper  = recBuf[i][3];
    writeServos();
    if (stepGate()) { Serial.println(F("回放被取消")); printStat("IDLE"); return; }
    delay(REC_MS);
  }
  Serial.println(F("回放完成"));
  printStat("IDLE");
}

/* 按键4 / 回中：回到初始姿态 */
void goHome() {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.println(F("== 回中 =="));
  moveJoints(90, 90, 90, 90);
  printStat("IDLE");
}

/* =========================== 14. 任务五：绘图引擎 ======================= */
/* 统一绘图入口：笔尖从当前位置 → 目标点，可被暂停 / 取消打断。
 * 返回 false 表示被取消。                                        */
bool penTo(float u, float v, float h) { return moveToTable(u, v, h); }

/* 抬笔 / 落笔 */
bool penUp(float u, float v)   { return penTo(u, v, PEN_LIFT); }
bool penDown(float u, float v) { return penTo(u, v, 0.0); }

/* 待机位 */
bool goStandby() { return penTo(HOME_U, HOME_V, HOME_H); }

void beginDrawTask(const char *title) {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.print(F("== ")); Serial.print(title); Serial.println(F(" =="));
  printStat("DRAWING");
  moveJoints(curBase, curShoulder, curElbow, (int)PEN_GRIP);   // 先夹好笔
}

void endDrawTask(const char *title, bool cancelled) {
  if (cancelled) {
    Serial.println(F(">> 已取消当前绘图，返回待机位"));
    stAbort = false;                       // 清除标志，否则回程也会被立刻取消
  }
  goStandby();
  Serial.print(F("== ")); Serial.print(title);
  Serial.println(cancelled ? F(" 已取消 ==") : F(" 完成 =="));
  printStat("IDLE");
}

/* ---- 子任务1：直线 ---- */
const float LINE_U1 = -6.0, LINE_V1 = 10.5;
const float LINE_U2 =  6.0, LINE_V2 = 10.5;

void drawLineDemo() {
  beginDrawTask("子任务1 直线");
  bool cc = false;
  if (!penUp(LINE_U1, LINE_V1)) cc = true;
  if (!cc && !penDown(LINE_U1, LINE_V1)) cc = true;
  if (!cc) {
    const int N = 40;                        // 离散成 40 段
    for (int i = 1; i <= N && !cc; i++) {
      float t = (float)i / N;
      if (!penTo(LINE_U1 + (LINE_U2 - LINE_U1) * t,
                 LINE_V1 + (LINE_V2 - LINE_V1) * t, 0.0)) cc = true;
    }
  }
  endDrawTask("直线", cc);
}

/* ---- 子任务2：字母 N / 三角形 / 字母 Z / 字母 V ---- */
/* 图形都画在以 (FIG_CU, FIG_CV) 为中心、边长约 4cm 的区域内 */
const float FIG_CU = 0.0, FIG_CV = 11.5, FIG_S = 4.0;

void drawFigure(char kind) {
  const char *name = (kind == 'N') ? "字母 N" : (kind == 'T') ? "三角形"
                   : (kind == 'Z') ? "字母 Z" : "字母 V";
  beginDrawTask(name);

  const float c = FIG_CU, s = FIG_S / 2.0;
  const float vB = FIG_CV - s, vT = FIG_CV + s;    // 底边 / 顶边
  const float uL = c - s, uR = c + s;              // 左边 / 右边
  float px[6], py[6]; int n = 0;                   // 折线顶点序列

  if (kind == 'N') {                               // 左竖 → 斜线 → 右竖
    px[0] = uL; py[0] = vB; px[1] = uL; py[1] = vT;
    px[2] = uR; py[2] = vB; px[3] = uR; py[3] = vT; n = 4;
  } else if (kind == 'T') {                        // 等边三角形（一笔闭合）
    float hgt = FIG_S * 0.866;
    px[0] = c;  py[0] = vB + hgt; px[1] = uL; py[1] = vB;
    px[2] = uR; py[2] = vB;       px[3] = c;  py[3] = vB + hgt; n = 4;
  } else if (kind == 'Z') {                        // 上横 → 斜线 → 下横
    px[0] = uL; py[0] = vT; px[1] = uR; py[1] = vT;
    px[2] = uL; py[2] = vB; px[3] = uR; py[3] = vB; n = 4;
  } else {                                         // 'V'
    px[0] = uL; py[0] = vT; px[1] = c;  py[1] = vB;
    px[2] = uR; py[2] = vT; n = 3;
  }

  bool cc = false;
  if (!penUp(px[0], py[0])) cc = true;
  if (!cc && !penDown(px[0], py[0])) cc = true;
  for (int i = 1; i < n && !cc; i++) {             // 逐段插值，保证直线度
    const int M = 25;
    for (int k = 1; k <= M && !cc; k++) {
      float t = (float)k / M;
      if (!penTo(px[i - 1] + (px[i] - px[i - 1]) * t,
                 py[i - 1] + (py[i] - py[i - 1]) * t, 0.0)) cc = true;
    }
  }
  endDrawTask(name, cc);
}

/* ---- 子任务3/4/5：示教模式 ---- */
void enterTeach() {
  if (mode == MODE_TEACH) { mode = MODE_IDLE; Serial.println(F("退出示教")); printStat("IDLE"); return; }
  stAbort = false; stPause = false;
  mode = MODE_TEACH;
  moveJoints(curBase, curShoulder, curElbow, (int)PEN_GRIP);
  // 用当前实际姿态初始化浮点累加器，保证摇杆一推就朝正确方向走
  float h;
  jointsToTable(curBase, curShoulder, curElbow, tgU, tgV, h);
  Serial.println(F("== 示教模式：摇杆1 移动笔尖(X=左右 Y=前后)，按 D11 或发送 SET 记录 =="));
  Serial.print(F("   当前笔尖 u=")); Serial.print(tgU, 1);
  Serial.print(F(" v="));           Serial.println(tgV, 1);
  printStat("TEACH");
}

/* 示教：摇杆控制笔尖在 (u,v) 平面移动
 * 关键点：目标位置用「浮点累加器」保存。
 *   舵机角是整数（1° ≈ 1.4mm），若每次都从正解结果重新累加，
 *   小幅推杆的位移会被取整吃掉，笔尖就"推不动"。
 *   所以这里维护一个浮点目标 (tgU, tgV)，摇杆的微小增量能累积起来，
 *   越过 1° 的分辨率后笔尖才会动 —— 手感是"一格一格"的，但一定能到。 */
void teachStep() {
  int jx = analogRead(PIN_J1X) - JOY_CENTER;      // 左右 → u
  int jy = analogRead(PIN_J1Y) - JOY_CENTER;      // 前后 → v
  bool moved = false;
  if (abs(jx) > JOY_DEADZONE) { tgU += (jx / 512.0f) * TEACH_SPEED; moved = true; }
  if (abs(jy) > JOY_DEADZONE) { tgV += (jy / 512.0f) * TEACH_SPEED; moved = true; }

  if (moved) {
    // 把累加器钳在可达范围内，避免一直往够不着的方向累加
    float rr = sqrt(tgU * tgU + tgV * tgV);
    if (rr > R_MAX) { tgU *= R_MAX / rr; tgV *= R_MAX / rr; }
    if (rr < R_MIN && rr > 0.01f) { tgU *= R_MIN / rr; tgV *= R_MIN / rr; }

    int tB, tS, tE;
    tableToJoints(tgU, tgV, TEACH_H, tB, tS, tE);
    curBase = tB; curShoulder = tS; curElbow = tE;   // 直接赋值：目标只差 1° 左右
    writeServos();
  }

  if (btnEdge[B_TEACH])  recordTeachPoint();
  if (btnEdge[B_HOME])   { mode = MODE_IDLE; goHome(); }
  if (btnEdge[B_CANCEL]) { mode = MODE_IDLE; Serial.println(F("退出示教")); printStat("IDLE"); }
  delay(10);
}

/* 记录当前笔尖位置为一个示教点 */
void recordTeachPoint() {
  if (mode != MODE_TEACH) { Serial.println(F("ERR: 请先发送 TEACH 进入示教模式")); return; }
  if (teachCount >= TEACH_N) {
    Serial.println(F("示教点已满 5 个，如需重录请发送 CLR 清空"));
    return;
  }
  float u, v, h;
  jointsToTable(curBase, curShoulder, curElbow, u, v, h);
  teachU[teachCount] = u;
  teachV[teachCount] = v;
  teachCount++;
  Serial.print(F("已记录示教点 ")); Serial.print(teachCount);
  Serial.print(F("/5 : (u=")); Serial.print(u, 1);
  Serial.print(F(", v="));     Serial.print(v, 1);
  Serial.println(F(")"));
}

void loadDemoPoints() {
  const float du[5] = { -6.0, -3.0, 0.0, 3.0, 6.0 };
  const float dv[5] = {  9.0, 12.0, 10.0, 12.0, 9.0 };
  for (uint8_t i = 0; i < TEACH_N; i++) { teachU[i] = du[i]; teachV[i] = dv[i]; }
  teachCount = TEACH_N;
  Serial.println(F("已载入内置五点"));
}

/* Catmull-Rom 样条单点求值：
 *   x(t) = 0.5*( 2P1 + (-P0+P2)t + (2P0-5P1+4P2-P3)t² + (-P0+3P1-3P2+P3)t³ )
 * t=0 在 P1，t=1 在 P2 —— 数学上保证曲线严格经过每个给定点。 */
void catmullRom(float p0u, float p0v, float p1u, float p1v,
                float p2u, float p2v, float p3u, float p3v,
                float t, float &u, float &v) {
  float t2 = t * t, t3 = t2 * t;
  u = 0.5f * (2.0f * p1u + (-p0u + p2u) * t
            + (2.0f * p0u - 5.0f * p1u + 4.0f * p2u - p3u) * t2
            + (-p0u + 3.0f * p1u - 3.0f * p2u + p3u) * t3);
  v = 0.5f * (2.0f * p1v + (-p0v + p2v) * t
            + (2.0f * p0v - 5.0f * p1v + 4.0f * p2v - p3v) * t2
            + (-p0v + 3.0f * p1v - 3.0f * p2v + p3v) * t3);
}

/* 子任务4 / 子任务5：按示教点自动绘制
 *   smooth=false → 折线（四点四段直线）
 *   smooth=true  → Catmull-Rom 平滑曲线，依次经过五个点
 * 流程：先回待机位 → 抬笔到第 1 点 → 落笔 → 依次过点 → 抬笔回待机 */
void drawByTeach(bool smooth) {
  if (teachCount < 2) { Serial.println(F("ERR: 示教点不足 2 个，请先 TEACH + SET")); return; }
  beginDrawTask(smooth ? "子任务5 五点示教 + 平滑曲线" : "子任务4 五点示教 + 折线");

  bool cc = false;
  if (!goStandby()) cc = true;                                  // 先回待机位
  if (!cc && !penUp(teachU[0], teachV[0])) cc = true;           // 抬笔移到第 1 点上方
  if (!cc && !penDown(teachU[0], teachV[0])) cc = true;         // 落笔

  int segN = teachCount - 1;
  for (int i = 0; i < segN && !cc; i++) {
    const int M = smooth ? 30 : 25;                             // 每段离散点数
    for (int k = 1; k <= M && !cc; k++) {
      float t = (float)k / M, u, v;
      if (smooth) {
        // Catmull-Rom：用前后邻居决定切线，保证段与段之间方向连续（C1 平滑）
        float p0u = (i == 0)         ? teachU[0]         : teachU[i - 1];
        float p0v = (i == 0)         ? teachV[0]         : teachV[i - 1];
        float p1u = teachU[i],     p1v = teachV[i];
        float p2u = teachU[i + 1], p2v = teachV[i + 1];
        float p3u = (i == segN - 1)  ? teachU[teachCount - 1] : teachU[i + 2];
        float p3v = (i == segN - 1)  ? teachV[teachCount - 1] : teachV[i + 2];
        catmullRom(p0u, p0v, p1u, p1v, p2u, p2v, p3u, p3v, t, u, v);
      } else {
        u = teachU[i]     + (teachU[i + 1] - teachU[i])     * t;
        v = teachV[i]     + (teachV[i + 1] - teachV[i])     * t;
      }
      if (!penTo(u, v, 0.0)) cc = true;
    }
  }
  if (!cc) penUp(teachU[teachCount - 1], teachV[teachCount - 1]);
  endDrawTask(smooth ? "平滑曲线" : "折线", cc);
}


/* =========================== 15. 调试 / 标定工具 ======================== */
void printCalib() {
  float u, v, h;
  jointsToTable(curBase, curShoulder, curElbow, u, v, h);
  Serial.print(F("CAL 舵机角 B=")); Serial.print(curBase);
  Serial.print(F(" S=")); Serial.print(curShoulder);
  Serial.print(F(" E=")); Serial.print(curElbow);
  Serial.print(F("  → 笔尖 u=")); Serial.print(u, 2);
  Serial.print(F(" v="));        Serial.print(v, 2);
  Serial.print(F(" h="));        Serial.println(h, 2);
}

/* JOG 大臂连杆角 肘连杆角：直接给理论连杆角驱动，标定用
 *   大臂角 0 = 水平前伸，+ = 抬起
 *   肘角   0 = 完全伸直，- = 小臂下垂（画图常用解支） */
void cmdJog(const char *p) {
  const char *q = p + 3;
  float s = atof(q);
  const char *r = strchr(q, ' ');
  float e = r ? atof(r + 1) : 0;
  int ts = constrain(SHOULDER_TRIM + (int)round(SHOULDER_INVERT ? -s : s), 0, 180);
  int te = constrain(ELBOW_TRIM    + (int)round(ELBOW_INVERT    ? -e : e), 0, 180);
  int tb = constrain(BASE_ANG, 0, 180);
  Serial.print(F("JOG3D s=")); Serial.print(s);
  Serial.print(F(" e="));      Serial.print(e);
  Serial.print(F(" → 舵机 S=")); Serial.print(ts);
  Serial.print(F(" E="));        Serial.println(te);
  moveJoints(tb, ts, te, curGripper);
  printCalib();
}

void printHelp() {
  Serial.println(F("---- meArm 指令表 ----"));
  Serial.println(F("任务一: M=摇杆操控  O/S=开爪/合爪  H/L=加速/减速  x10,y30,z20=多舵机"));
  Serial.println(F("任务二: A/B/C=夹取对应物体  RUN=依次夹取三个"));
  Serial.println(F("任务三: 1=循环夹取  2=录制  3=回放  4=回中"));
  Serial.println(F("任务五: LINE=直线  N/TRI/Z/V=图形  TEACH=示教  SET=记录点  CLR=清空"));
  Serial.println(F("        DRAW=折线  CURVE=平滑曲线  DEMO=内置五点  GO=待机位"));
  Serial.println(F("        PAUSE/RESUME/CANCEL=暂停/继续/取消"));
  Serial.println(F("调试  : CAL=打印笔尖坐标  JOG s e=连杆角驱动  STATUS=状态"));
}

/* =========================== 16. 工具函数 =============================== */
/* 忽略大小写比较（cmd 已在 dispatch 中转大写，这里把常量也转大写比较） */
bool eq(const char *a, const char *b) {
  while (*a && *b) {
    if (toupper(*a) != toupper(*b)) return false;
    a++; b++;
  }
  return *a == 0 && *b == 0;
}

/* 取出 key 字母后面的数字，如 numAfter("x10,y30,z20", 'y') → 30
 * 找不到该字母返回 -1；顺序无关，x/y/z 谁前谁后都能解析。 */
int numAfter(const char *s, char key) {
  char k = toupper(key);
  for (const char *p = s; *p; p++) {
    if (toupper(*p) == k) {
      const char *q = p + 1;
      while (*q && !isdigit((unsigned char)*q) && *q != '-' && *q != '+') q++;
      if (!*q) return -1;
      return atoi(q);
    }
  }
  return -1;
}
