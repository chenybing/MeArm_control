/* ====
========================================================================
 *  task1_basic.ino · 任务一：机械臂的基本功能
 *  摇杆实时操控（各关节独立移动） / 固定指令 O,S,H,L / 多舵机协同 x,y,z
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 * ============================================================================*/

/* ---- 任务一 ---- */

/* ── 摇杆手感（2026-10-06 整体调慢）──────────────────────────────────
 * 老代码两个毛病，合起来就是"又快又猛"：
 *   ① 步长 = 误差 × 比例。杆推到底误差就是 90°，乘 0.25 一步就是 22.5°，
 *      而循环只有 10ms → 约 2000°/秒，机械臂像被抽了一鞭子。
 *   ② 差值小的时候取整成 0，只能强制走 1° → 不管推多轻，一律 100°/秒。
 * 现在：比例整体调小 + 加一道"每循环最多走几度"的硬闸 +
 *       用【小数位置】累积（这样轻轻推才真的能慢慢走）。 */

/* ★★ 2026-10-16 新增：M 档（摇杆实时操控）整体【降一档】。
 *
 * 用户反馈 M 档偏快、不好对准。用一个系数把 M 档速度整体缩下来。
 *
 * ⚠ 为什么要同时缩【比例】和【上限】两张表：
 *     实际每轮步长 = clamp(误差 × 比例, ±上限)。
 *     满推时误差 ≈ 90°，90 × 0.055 ≈ 5°，**早就撞到上限了**
 *     ⇒ **满推的速度其实由【上限】决定，光缩比例一点用都没有。**
 *     只缩比例的话，只有"轻轻推"（误差 ≲ 23°）会变慢，满推完全不变。
 *     所以两张表一起缩 ⇒ 全推 / 轻推都真的慢下来。
 *
 * 系数取 0.65 —— 约等于速度档表里相邻两档的比值（0.035/0.055 ≈ 0.64），
 * 所以这个改动就叫"M 档降一档"。
 *
 * ⚠ **只影响 M 档**！绘图 / 取放 / 回中 / 任务三回放用的 stepDelay() 一个字没动。
 * ⚠ 还想再慢 → 调小这个系数；若小臂抬不动了 → 单独把 JOY_STEP_ELBOW 放大回来。 */
#define JOY_M_SCALE 0.65f

/* 每轮走差值的这个比例 —— 随 H/L 档位变化（档 1 最柔，档 5 最快）。 */
const float JOY_RATE_TAB[5] = { 0.020f, 0.035f, 0.055f, 0.080f, 0.110f };
inline float joyFollowRate() { return JOY_RATE_TAB[speedLevel - 1] * JOY_M_SCALE; }

/* 每个轴各自的摇杆速度上限（度/循环）—— 每循环 10ms，×100 就是 °/秒。
 *
 * ⚠ 小臂（肘）特意给得比别的轴大。
 *   原因（2026-10-06 用户上机实测）：小臂要抬着整条小臂 + 手 + 笔，负载最重，
 *   而【位置舵机的输出力矩跟"指令减当前"的差值有关】——
 *   差值大 → 使劲推 → 抬得起来；差值小 → 使不上劲 → 抬不动（得用手帮它）。
 *   所以小臂给大一点的步长，既快，也能让舵机真的使上劲。
 *   如果哪天换了更结实的舵机/供电，可以把这个数降回来跟上别的轴。 */
const float JOY_STEP_BASE     = 1.3f * JOY_M_SCALE;
const float JOY_STEP_SHOULDER = 1.3f * JOY_M_SCALE;
const float JOY_STEP_ELBOW    = 3.0f * JOY_M_SCALE;   // ← 小臂：给大（理由见上）
const float JOY_STEP_GRIPPER  = 1.3f * JOY_M_SCALE;

/* 摇杆模式的【小数位置】—— 每个关节一个。
 * ⚠ 刚进 M 模式时这几个值可能跟当前姿态对不上，
 *   smoothJoint 里有个"差太多就拉回来"的兜底。 */
float joyPosBase = 90, joyPosShoulder = 90, joyPosElbow = 90, joyPosGrip = 90;

/* 基础控制：摇杆实时操控四个关节（死区 + 平滑）
 * 轴与关节的对应关系沿用官方 code.ino：
 *   A0→底座  A1→小臂  A2→夹爪  A3→大臂                       */
void joystickAll() {
  int jBase     = analogRead(PIN_J1X);
  int jElbow    = analogRead(PIN_J1Y);
  int jGrip     = analogRead(PIN_J2X);
  int jShoulder = analogRead(PIN_J2Y);

  smoothJoint(curBase,     joyPosBase,
              joyTarget(jBase,     curBase,     JOY_CENTER[0], JOY_INV_J1X),
              BASE_MIN, BASE_MAX, JOY_STEP_BASE);
  smoothJoint(curShoulder, joyPosShoulder,
              joyTarget(jShoulder, curShoulder, JOY_CENTER[3], JOY_INV_J2Y),
              SH_MIN, SH_MAX, JOY_STEP_SHOULDER);
  smoothJoint(curElbow,    joyPosElbow,
              joyTarget(jElbow,    curElbow,    JOY_CENTER[1], JOY_INV_J1Y),
              EL_MIN, EL_MAX, JOY_STEP_ELBOW);
  smoothJoint(curGripper,  joyPosGrip,
              joyTarget(jGrip,     curGripper,  JOY_CENTER[2], JOY_INV_J2X),
              GR_MIN, GR_MAX, JOY_STEP_GRIPPER);
  writeServos();
  delay(10);
}

/* 摇杆读数 → 目标角度。
 * 以该轴【自己实测的中位】center 为 0 点：中位对应舵机 90°，
 * 往一侧满偏对应 0°、往另一侧满偏对应 180°，两侧各按自己的半量程缩放
 * （摇杆电位器两端不一定对称，所以上下半量程分开算）。
 * invert = true 把偏转量取反，用于「推杆方向和机械臂实际方向相反」，
 * 开关见 mearm_main.ino 顶部的 JOY_INV_J1X ~ JOY_INV_J2Y。 */
int joyTarget(int joy, int cur, int center, bool invert) {
  int off = joy - center;                                      // 偏离中位多少
  if (off > -JOY_DEADZONE && off < JOY_DEADZONE) return cur;   // 死区：保持不动
  if (invert) off = -off;                                      // 方向取反
  int half = (off > 0) ? (1023 - center) : center;             // 该侧的满偏行程
  return constrain(90 + (int)((long)off * 90L / half), 0, 180);
}
/* 平滑逼近：每轮只走差值的一个比例（再有硬上限），动作柔和。
 *   cur —— 整数角度，真正下发给舵机的那个（writeServos 用）
 *   pos —— 该关节在摇杆模式下的【小数位置】
 *   lo/hi —— 该轴的物理安全行程（和 writeServos 用的是同一组常量）
 *   maxStep —— 该轴每循环最多走几度（各轴可以不一样，见上面 JOY_STEP_*）
 *
 * ⚠ pos 必须按同一组 lo/hi 钳制。
 *   不钳的话：cur 会被 writeServos 钳在 lo/hi，而 pos 继续往上爬，
 *   两个值一分开，"拉回来"的兜底就会把手臂钉在限位上 —— 推回来也不动。 */
void smoothJoint(int &cur, float &pos, int target, int lo, int hi,
                 float maxStep) {
  /* 兜底：pos 跟 cur 差得离谱（刚进 M 模式、或中间被别的指令改过），
   * 就把 pos 拉回 cur —— 免得机械臂"从 90° 慢慢爬过去"。 */
  if (pos < (float)cur - 12.0f || pos > (float)cur + 12.0f) pos = (float)cur;

  float d = (float)target - pos;
  if (d > -0.05f && d < 0.05f) { pos = (float)target; cur = target; return; }

  float step = d * joyFollowRate();
  if (step >  maxStep) step =  maxStep;
  if (step < -maxStep) step = -maxStep;

  pos += step;
  if (pos < (float)lo) pos = (float)lo;
  else if (pos > (float)hi) pos = (float)hi;

  int v = (int)(pos + 0.5f);
  if (v != cur) cur = v;
}

/* ---- ⚠ 诊断：F<逻辑角> —— 把夹爪【直接】设到该角度（66~167，大 = 张开）----
 * 为什么要它：夹爪平时只能通过 G<0~100>（张开程度）命令，而那个只在
 *   GRIP_OPEN~GRIP_CLOSE 之间插值。要标定某个具体开口（硬币 / 物体 C / 笔），
 *   就得能直接下角度。
 *
 * 入参是【逻辑角】（大 = 张开），跟别处一样走镜像；回显里同时打印对应的
 * 舵机原始角，方便和 range test 工具的读数对照（逻辑 = 220 − 原始角）。
 * ⚠ 正式提交前要么删掉这条，要么在说明文档里注明它是标定用的诊断指令。 */
void cmdGripperRaw(const char *p) {
  int a = atoi(p + 1);
  a = constrain(a, GR_MIN, GR_MAX);          // 逻辑角也限位，别顶到机械止点
  Serial.print(F("[诊断] 夹爪 逻辑角 → ")); Serial.print(a);
  Serial.print(F("（舵机原始角 ")); Serial.print(GR_RAW_SUM - a); Serial.println(F("）"));
  moveJoints(curBase, curShoulder, curElbow, a);
  Serial.println(F("[诊断] 已下发。舵机是开环的，实际停在哪得用眼睛看"));
}

/* 固定指令 O / S：夹爪开合 */
void cmdGripper(int target, const char *name) {
  Serial.print(F("CMD ")); Serial.println(name);
  moveJoints(curBase, curShoulder, curElbow, target);
}

/* 原来这里有个「G<角度> 直接把夹爪设到舵机角」的标定指令。
 * 已删除：G 这个字母让给 task2_teach.ino 的 cmdAxisAbs() 了，那边用 0~100 的
 * 「夹紧程度」（G0=合死 / G100=全开），比舵机角直观。功能没丢 —— 0~100 共 101
 * 档，比舵机自己的 64 档还细。 */

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
