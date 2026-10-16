/* ============================================================================
 *  task5_drawing.ino · 任务五：轨迹绘制与画图
 *  直线 / 基础图形 / 绘图过程控制 / 五点示教折线 / 五点示教平滑曲线
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 * ============================================================================*/

/* ---- 任务五状态：五点示教 ---- */
const uint8_t TEACH_N = 5;                 // 示教点个数（考核固定 5 点）
float   teachU[TEACH_N], teachV[TEACH_N];  // 已记录的示教点（桌面坐标 cm）
uint8_t teachCount = 0;                    // 已记录个数
float   tgU = 0, tgV = 0;                  // 示教时笔尖目标的浮点累加器（见 teachStep）

/* 清空示教点。主文件 dispatch() 处理 CLR 时调用这里 ——
 * 之所以包成函数而不是让主文件直接改 teachCount：本文件在合并顺序里排在
 * 主文件之后，主文件的代码看不到这里定义的变量。 */
void clearTeachPoints() {
  teachCount = 0;
  Serial.println(F("示教点已清空"));
}

/* ---- 任务五：绘图引擎 ---- */
/* 统一绘图入口：笔尖从当前位置 → 目标点，可被暂停 / 取消打断。
 * 返回 false 表示被取消。
 * ⚠ 这里统一叠加 penHTrim（诊断用笔尖高度偏移，见 mearm_main.ino 里 penHTrim 处）——
 *   绘图 / 抬笔 / 落笔 / 待机位 / 示教全走这个函数，所以一个偏移就能整体上下挪。 */
bool penTo(float u, float v, float h) { return moveToTable(u, v, h + penHTrim); }

/* ⚠ 诊断命令：D<毫米> —— 设置笔尖高度偏移（正=抬高，负=往下压）。见 penHTrim 处说明。 */
void cmdPenTrim(const char *p) {
  penHTrim = atof(p + 1) / 10.0;                 // 入参是 mm，内部按 cm 存
  Serial.print(F("[诊断] 笔尖高度偏移 = "));
  Serial.print(penHTrim * 10.0, 1);
  Serial.println(F(" mm  （负 = 往下压）"));
}

/* 抬笔 / 落笔 / 回待机位 —— 这三个都是【空中的整段移动】，必须走【空间直线】。
 * ★ 2026-10-16 改：原来走 `penTo`（= `moveToTable`，关节空间一把走完），笔尖在
 *   空间里走的是【弧线】——模拟实测：落笔那 2cm，笔尖横向甩出去【8.1mm】再收回来。
 *   配合 S/E 那 ~7mm 的机械回差（臂滞后命令一拍）⇒ 笔尖是【一边横着摆、
 *   一边落到纸上】的 ⇒ 起笔/收笔两端各拖出一个小钩（现场照片确认）。
 *   改用 `moveTipLinear`（离散 20 段、每段重解）⇒ 横向摆动 8.1mm → 1.0mm。
 *   ⚠ 绘图循环里的 `penTo` 保持不动 —— 那里本来就是"一点一点走"的，不需要再套一层
 *     （再套一层会把步数乘 20 倍，慢到跑不动）。 */
bool penTravel(float u, float v, float h) { return moveTipLinear(u, v, h + penHTrim); }
bool penUp(float u, float v)   { return penTravel(u, v, PEN_LIFT); }
bool penDown(float u, float v) { return penTravel(u, v, 0.0); }

/* 待机位 */
bool goStandby() { return penTravel(HOME_U, HOME_V, HOME_H); }

void beginDrawTask(const char *title) {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.print(F("== ")); Serial.print(title); Serial.println(F(" =="));
  printStat("DRAWING");
  moveJoints(curBase, curShoulder, curElbow, (int)PEN_GRIP);   // 先夹好笔
  gDrawSteady = true;      // ★ 2026-10-16：整条绘图路径走匀速（见 stepDelayFor）
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
  gDrawSteady = false;     // ★ 2026-10-16：离开绘图任务，恢复加减速（手动/示教/取放不受影响）
}

/* ---- 子任务1：直线 ----
 * ⚠ 2026-10-10 晚【v 从 10.5 挪到 13.0】：
 *   换平行夹爪后 L3B 从 1.5 变成 8.5 ⇒ L3A+L3B = 10cm ⇒ 笔尖【到不了离底座 10cm 以内】。
 *   老坐标 v=10.5 时中间的 r 只有 10.5（贴着 R_MIN=10 的边），其实已经在范围外一点点。 */
const float LINE_U1 = -6.0, LINE_V1 = 13.0;
const float LINE_U2 =  6.0, LINE_V2 = 13.0;

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
/* 图形都画在以 (FIG_CU, FIG_CV) 为中心、边长约 4cm 的区域内
 * ⚠ 2026-10-10 晚【中心 v 从 11.5 挪到 14.0】：
 *   老坐标底边在 v = 9.5，r = 9.5 < R_MIN = 10 ⇒【底边整段被钳掉、图形跑形】。
 *   现在：底边 v=12（r=12.0）、顶角 v=16（r=16.1），整块都落在 10~26 的可达环带里。 */
const float FIG_CU = 0.0, FIG_CV = 14.0, FIG_S = 4.0;

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
  /* ★★ 2026-10-16：摇杆方向【现场实测】标定 —— 这四个方向是量出来的，别再凭感觉改。
   *
   * 现场约定（用户确认）：**向前 = +v，向右 = +u**。
   *
   * 实测（改之前，用户逐方向推 + CAL 读笔尖坐标）：
   *     推【前】 → 笔尖 u 减小   ⇒ 前后轴当时在驱动 u   ❌
   *     推【右】 → 笔尖 v 增大   ⇒ 左右轴当时在驱动 v   ❌
   * 也就是【两根轴管错方向了】，不是单纯反号，换轴才对得上。
   *
   * 现在按实测重接（lr = J1X 通道、ff = J1Y 通道，引脚名和实机是【对】的）：
   *     推【前】 ⇒ ff > 0  ⇒  v = +ff  ⇒ v 增大 = 往前   ✅
   *     推【右】 ⇒ lr < 0  ⇒  u = −lr  ⇒ u 增大 = 往右   ✅
   *
   * ⚠ 没有动 JOY_INV_J1X / JOY_INV_J1Y —— 那组开关是【任务一摇杆操控】和
   *   【任务三录制】共用的，改了会把那两个模式一起弄乱。如果那两个模式也
   *   觉得方向不对，说一声，我照同一套实测方法一起改。
   * ⚠ 以后再有哪个方向反了：只调 `du` / `dv` 两行的正负号，
   *   并且【先推一下 + CAL 读数字】，别凭描述改。 */
  int lr = analogRead(PIN_J1X) - JOY_CENTER[0];   // 左右轴（J1X）
  int ff = analogRead(PIN_J1Y) - JOY_CENTER[1];   // 前后轴（J1Y）
  if (JOY_INV_J1X) lr = -lr;
  if (JOY_INV_J1Y) ff = -ff;
  int du = -lr;                                   // 左右 → u（推右 = u 增大）
  int dv =  ff;                                   // 前后 → v（推前 = v 增大）
  bool moved = false;
  if (abs(du) > JOY_DEADZONE) { tgU += (du / 512.0f) * TEACH_SPEED; moved = true; }
  if (abs(dv) > JOY_DEADZONE) { tgV += (dv / 512.0f) * TEACH_SPEED; moved = true; }

  if (moved) {
    // 把累加器钳在可达范围内，避免一直往够不着的方向累加
    float rr = sqrt(tgU * tgU + tgV * tgV);
    if (rr > R_MAX) { tgU *= R_MAX / rr; tgV *= R_MAX / rr; }
    if (rr < R_MIN && rr > 0.01f) { tgU *= R_MIN / rr; tgV *= R_MIN / rr; }

    int tB, tS, tE;
    /* ⚠ 2026-10-16 修：这里原来【没叠加 penHTrim】—— 而绘图路径（penTo）叠了。
     *   后果：用 D 标出来的笔尖高度偏移对示教【不生效】，示教时笔尖停在
     *   模型高度上，实机看就是"悬在半空中"（D 差多少就悬多少），很难对准纸上的目标点。
     *   现在两边口径一致：示教高度 = TEACH_H + penHTrim。 */
    tableToJoints(tgU, tgV, TEACH_H + penHTrim, tB, tS, tE);
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
