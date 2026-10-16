/* ============================================================================
 *  kinematics.ino · 公共模块
 *  按键扫描 / 舵机驱动与安全限位 / 运动基元 / 正逆运动学 / 标定工具 / 工具函数
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 * ============================================================================*/

/* ---- 按键扫描 ---- */
void scanButtons() {
#if LOCAL_BUTTONS
  for (uint8_t i = 0; i < BTN_COUNT; i++) {
    bool now = (digitalRead(BTN_PINS[i]) == LOW);   // 上拉输入：按下 = LOW
    btnEdge[i] = false;
    if (now && !btnLast[i] && (millis() - btnLock[i] > 180)) {  // 180ms 消抖
      btnEdge[i] = true;
      btnLock[i] = millis();
    }
    btnLast[i] = now;
  }
#else
  /* 没接按键：不读引脚，每轮把边沿全部清掉，
   * 让 handleLocalButtons() / handleMotionButtons() 永远读到"没按"。
   * 这样悬空引脚上的干扰就不会再误触发暂停/继续/取消/示教记录了。 */
  for (uint8_t i = 0; i < BTN_COUNT; i++) btnEdge[i] = false;
#endif
}

/* 空闲状态下的本地按键（任务三 / 任务五的操作键） */
/* 空闲状态下的本地按键，引脚分配见 mearm_main.ino 顶部。
 * 任务三的四个功能（循环夹取 / 录制 / 回放 / 回中）都在这儿；
 * 任务五的记录示教点也在（要先在示教模式里，否则会提示）。 */
void handleLocalButtons() {
  if (btnEdge[B_CYCLE])  cyclePick();        // D12  循环夹取
  if (btnEdge[B_REC])    toggleRecord();     // D4   摇杆2自带的按键：录制
  if (btnEdge[B_PLAY])   playRecord();       // D13  回放
  if (btnEdge[B_HOME])   goHome();           // D5   摇杆1自带的按键：回中
  if (btnEdge[B_TEACH])  recordTeachPoint(); // D11  记录示教点
}

/* 运动过程中的按键：暂停 / 继续 / 取消 */
void handleMotionButtons() {
  if (btnEdge[B_PAUSE])  { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  if (btnEdge[B_GO])     { stPause = false; }
  if (btnEdge[B_CANCEL]) { stAbort = true;  stPause = false; }
}


/* ---- 运动基元 ---- */
/* 所有舵机输出的唯一出口 —— 安全行程在这里统一生效。
 * 顺便把 cur* 自身也钳回合法值，否则摇杆/逆解把内部状态推到行程外以后，
 * 「当前角 vs 目标角」的比较会一直对不上（moveJoints 会死循环）。 */
/* ★★ 2026-10-16：舵机输出改走 writeMicroseconds() —— 专治"线走成波浪"。
 *
 * 原来用的是 Servo 库的 write(int)：它只吃【整数度】（544~2400µs 摊成 180 档，
 * 约 10.31µs 每度），**分辨率就是 1°**。
 * 而画线时大臂/小臂的 1° ≈ 笔尖半径走 2.3mm（dr/ds = 16·sin(a)·π/180，a≈56° 时）。
 * IK 算出该 56.3°，round() 成 56 ⇒ 半径偏 0.7mm；下一段变 57 ⇒ 又偏回去。
 * 一路走下来就是 ±0.8mm 的波浪（10-14 桌面实测，量级正好对上）。
 *
 * 改法：整数角照旧走，另外给大臂/小臂各留一个【小数残差】trim*，一起写进 µs。
 *   µs = 544 + 角度 × (2400−544)/180   ← Servo 库 write() 内部就是这么映射的，
 *   所以【同一个整数角，输出和以前逐 µs 相同】 —— 原有行为一点不变，
 *   只是多了小数部分：1µs ≈ 0.098°，分辨率提高约 10 倍。
 *
 * ⚠ trim* 只在 moveJoints() 开头清零；只有走 applyFine() 的路径（绘图用）
 *   才会把它设回去 ⇒ 手动 / 摇杆 / 示教那些直接改 cur* 再 writeServos() 的
 *   路径，行为与旧代码完全一样。
 * ⚠ 只给【大臂 / 小臂】留残差：底座那个 1° 是【沿笔道方向】的（1° ≈ 3.3mm），
 *   在线上看不出来；大臂/小臂的 1° 才是【径向】的 —— 波浪就是它。 */
float trimS = 0, trimE = 0;
/* 逆解算出来的【小数残差】：tS/tE 是取整后的角度，fineS/fineE 是被舍掉的那一截。
 * 由 tableToJoints() 填，由 applyFine() 用。 */
float fineS = 0, fineE = 0;

float usOf(float deg) { return 544.0f + deg * ((2400.0f - 544.0f) / 180.0f); }

void writeServos() {
  curBase     = constrain(curBase,     BASE_MIN, BASE_MAX);
  curShoulder = constrain(curShoulder, SH_MIN,   SH_MAX);
  curElbow    = constrain(curElbow,    EL_MIN,   EL_MAX);
  curGripper  = constrain(curGripper,  GR_MIN,   GR_MAX);
  svBase.writeMicroseconds((int)(usOf(curBase) + 0.5f));
  svShoulder.writeMicroseconds((int)(usOf(curShoulder + trimS) + 0.5f));
  svElbow.writeMicroseconds((int)(usOf(curElbow + trimE) + 0.5f));
  /* 夹爪镜像 —— 全程序里 curGripper 都是【逻辑角】（大 = 张开），
   * 但硬件方向是反的（舵机原始角小 = 开、大 = 闭），在这一处扳回来。
   * 这是夹爪舵机的唯一写出点，所以别的文件一个都不用改。
   *  逻辑 167 → 原始 50（张开）   逻辑 53 → 原始 170（闭合）
   * 【和 range test 工具对照：逻辑 = 220 − 工具读数】 */
  svGripper.writeMicroseconds((int)(usOf(GR_RAW_SUM - curGripper) + 0.5f));
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
inline int iabs(int v) { return v < 0 ? -v : v; }
inline int imax4(int a, int b, int c, int d) {
  int m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  if (d > m) m = d;
  return m;
}

/* 这一步该停多久 —— 两头慢、中段全速的加减速曲线。
 *
 * 原来是全程恒速（每步都 delay(stepDelay())），于是每段运动的开头结尾都是
 * "猛地一顿"。一条取放有 9 段运动，就顿 9 次 —— 现场看就是"一卡一卡"。
 *
 * 曲线形状：两头各占全长 40%，间隔按【二次曲线】从 2 倍平滑降到 1 倍。
 *   x = 离端点的比例(0~1)，间隔 = base × (1 + x²)
 * 用二次而不是线性，是因为线性曲线在"加速段接上全速段"那个接缝处,
 * 斜率是突变的 —— 听感上就是轻轻"咯"一下。二次曲线在接缝处斜率为 0，
 * 平滑地并进全速段，所以更顺。
 *
 * remain：这一步之前，四轴里还差得最多的那根还剩多少度
 * total ：这一整段要走的度数（用来决定加速/减速各占多长）
 *
 * 顺带一提：两头占 40%（原来是 1/3）反而整体更快 —— 两端平均只慢 1.33 倍，
 * 而中段不变速的那一段更长了。所以既更顺、又不牺牲速度。 */
/* ★ 2026-10-16 新增：绘图路径【匀速】开关（在 stepDelayFor 里生效）。
 *   beginDrawTask() 打开、endDrawTask() 关掉 —— 只在绘图任务里生效。 */
bool gDrawSteady = false;

int stepDelayFor(int remain, int total) {
  const int base = stepDelay();
  /* ★★ 2026-10-16：绘图路径走【匀速】，不套加减速曲线。
   *
   * 为什么：绘图把一整条线切成几十个小段（直线 40 段、图形每段 25 点、
   *   moveTipLinear 20 段），每段只走 1~2°。而加减速曲线两头各占 40% ——
   *   这么短的段【根本走不完曲线】，于是【每一步都被按在 2 倍延时上】：
   *   档 3 实际跑的是 28ms/° 而不是 14ms ⇒ **实际速度只有设定的一半**；
   *   而且段与段之间要重新"起步‑刹车"一次 ⇒ **画出来一顿一顿的**。
   *
   * 匀速之后：每步就是 base（档 3 = 14ms），段间不再起停。
   *   ⇒ 比原来【快一倍】，而且不再顿。
   *
   * ⚠ 只在绘图任务里开（`beginDrawTask` ~ `endDrawTask`）——
   *   手动 / 示教 / 取放 / 回中那些路径照旧带加减速，手感一点不变。
   * ⚠ 想再快就得动"每步走几度"（现在是 1°），不是继续压这个延时 ——
   *   档 3/4/5 的延时（14/9/6ms）都已经短于舵机库 20ms 的刷新周期了。 */
  if (gDrawSteady) return base;
  int ramp = total * 2 / 5;              // 两端各占 40%
  if (ramp < 1) ramp = 1;
  int d = remain;
  if (total - d < d) d = total - d;      // 取离两端更近的那一段
  if (d >= ramp) return base;            // 中段：全速
  int x = (ramp - d) * 100 / ramp;       // 0(刚进入中段) ~ 100(在端点)
  return base + base * x * x / 10000;    // 二次：端点是 2 倍，接缝处斜率为 0
}

bool moveJoints(int tB, int tS, int tE, int tG) {
  /* 目标角也要按实测行程钳制：只靠 writeServos() 钳制当前角的话，
   * 一个越界的 tB 会永远等不到 curBase == tB，下面这个 while 就死循环了。 */
  tB = constrain(tB, BASE_MIN, BASE_MAX); tS = constrain(tS, SH_MIN, SH_MAX);
  tE = constrain(tE, EL_MIN,   EL_MAX);   tG = constrain(tG, GR_MIN, GR_MAX);
  /* ★ 2026-10-16：粗定位一律把小数残差清零 ——
   *   摇杆 / 示教 / 指令 / 取放这些路径的输出与旧代码【逐 µs 相同】。
   *   只有走 applyFine() 的那几条（绘图用）才会把它设回去。 */
  trimS = trimE = 0.0f;
  const int total = imax4(iabs(curBase - tB), iabs(curShoulder - tS),
                          iabs(curElbow - tE), iabs(curGripper - tG));
  while (true) {
    int remain = imax4(iabs(curBase - tB), iabs(curShoulder - tS),
                       iabs(curElbow - tE), iabs(curGripper - tG));
    if (remain == 0) break;
    if (curBase     < tB) curBase++;     else if (curBase     > tB) curBase--;
    if (curShoulder < tS) curShoulder++; else if (curShoulder > tS) curShoulder--;
    if (curElbow    < tE) curElbow++;    else if (curElbow    > tE) curElbow--;
    if (curGripper  < tG) curGripper++;  else if (curGripper  > tG) curGripper--;
    writeServos();
    if (stepGate()) return false;
    delay(stepDelayFor(remain, total));
  }
  return true;
}

/* ★ 2026-10-16 新增：把 tableToJoints() 里被 round() 舍掉的那一截补上。
 *
 * 为什么要有它（桌面实测）：Servo 库的 write(int) 只吃整数度，分辨率 1°。
 * 画线时大臂/小臂的 1° ≈ 笔尖半径走 **2.3mm**（dr/ds = 16·sin(a)·π/180）。
 * IK 算出该 56.3°，round() 成 56 ⇒ 半径偏 0.7mm；下一段变 57 ⇒ 又偏回去。
 * 一路走下来就是用户摸到的 **±0.8mm 波浪**（量级正好对上）。
 *
 * 做法：粗定位（整数度、加减速、限位）一个字不改，走完再把残差写进脉冲宽度。
 *       µs = 544 + 角度 × (2400−544)/180 —— 就是 Servo 库 write() 内部那条映射，
 *       所以【整数角下输出与旧代码逐 µs 相同】，白拿 1µs ≈ 0.098° 的精度。
 * ⚠ 只在大臂/小臂上做：底座那个 1° 是【沿笔道方向】的，在线条上看不出来。 */
void applyFine() {
  trimS = fineS;
  trimE = fineE;
  writeServos();
}

/* 末端移动到桌面坐标 (u,v)，高度 h；返回 false 表示被取消。
 * ⚠ 这是【关节空间】的一把走：只算【目标点】的关节角，几个关节各转各的
 *   ⇒ 笔尖在空间里走的是【弧线】，中途会往外/往下兜出去。
 *   实测（落笔那一段 2cm）：笔尖横向甩出去【8.1mm】再收回来。
 *   要"笔尖走直线"用下面的 `moveTipLinear`。 */
bool moveToTable(float u, float v, float h) {
  int tB, tS, tE;
  if (!tableToJoints(u, v, h, tB, tS, tE)) return true;   // 不可达：已告警，跳过
  if (!moveJoints(tB, tS, tE, curGripper)) return false;
  applyFine();                 // ★ 2026-10-16 精定位：补上不足 1° 的残差
  return true;
}

/* ★ 2026-10-16 新增：笔尖【空间直线】移动 —— 从当前笔尖位置直线走到 (u,v,h)。
 *
 * 为什么要有它（现场照片 + 模拟确认）：
 *   `moveToTable` 是【关节空间】的直线，笔尖实际走的是【弧线】——
 *   落笔那 2cm 里，笔尖横向甩出去 8.1mm 再收回来。配合 S/E 那 ~7mm 的
 *   机械回差（臂滞后命令一拍），笔尖就是【一边横着摆、一边落到纸上】的
 *   ⇒ 起笔/收笔两端各拖出一个小钩。
 *
 * 做法：把 (u,v,h) 从【当前笔尖位置】到目标离散成 N 段，每段单独逆解再走
 *   ⇒ 笔尖贴着实打实的【空间直线】走，高度全程受控。
 *   实测：横向摆动 8.1mm → 1.0mm。
 *
 * ⚠ 别拿它放进绘图循环（`drawLineDemo` 那种"一点一点走"的循环）——
 *   那里本来就是离散的，再套一层会把步数乘 N 倍，慢到跑不动。
 *   它只给【空中的整段移动】用：抬笔 / 落笔 / 回待机位（见 task5 的 `penTravel`）。
 * ⚠ N 别太小 —— 段间仍是关节空间的小弧线，N 越大越贴直线（代价是慢）。 */
bool moveTipLinear(float u, float v, float h) {
  float cu, cv, ch;
  jointsToTable(curBase, curShoulder, curElbow, cu, cv, ch);   // 当前笔尖位置

  const int N = 20;
  bool moved = false;
  for (int i = 1; i <= N; i++) {
    float t = (float)i / N;
    int tB, tS, tE;
    if (!tableToJoints(cu + (u - cu) * t, cv + (v - cv) * t,
                       ch + (h - ch) * t, tB, tS, tE)) continue;  // 中间点不可达：跳过这一段
    if (!moveJoints(tB, tS, tE, curGripper)) return false;         // 被取消
    applyFine();                                                   // 精定位
    moved = true;
  }
  /* 兜底：中间点一个都没解出来 ⇒ 退回老办法，至少朝目标点试一次
   * （否则目标点只擦一点可达边界，整条移动就会原地不动，很难查） */
  if (!moved) {
    int tB, tS, tE;
    if (!tableToJoints(u, v, h, tB, tS, tE)) return true;
    if (!moveJoints(tB, tS, tE, curGripper)) return false;
    applyFine();
    return true;
  }
  return true;
}


/* ---- 运动学 ---- */
/* 桌面坐标 (u,v,h) → 关节角 (底座/大臂/小臂)，同时做可达性钳制。
 *   u = 左右（右正）  v = 前后（远离底座正）  h = 笔尖离桌面高度
 *
 * 模型（平行四边形连杆：手始终水平、笔始终竖直）：
 *     笔尖半径 r = L3A + L1·cos(a1) + L2·cos(a2) + L3B
 *     笔尖高度 h = H_SHOULDER + L1·sin(a1) + L2·sin(a2) − PEN_DROP
 *   a1 = 大臂绝对角，a2 = 小臂绝对角
 *
 * ★★ 2026-10-10 晚【重大更正】：小臂角是【绝对角】a2 = E − ELBOW_TRIM，
 *    【不是】相对角 a2 = a1 + (E − ELBOW_TRIM)。
 *    原因：小臂舵机装在底座上，力经【平行四边形】传到小臂 ——
 *    平行四边形的性质就是"小臂的绝对方向由该舵机直接定死，与大臂转角无关"
 *    （O-A-B-E 成平行四边形 ⇒ B→E 恒等于 A→O ⇒ 小臂方向 = 曲柄方向，
 *     和大臂 E→O 没关系）。实测：S 从 130 转到 110（大臂转 20°）、E 不变，
 *    【腕轴−肘轴】几乎不动（−0.1 vs +0.15），而相对角模型预测它要变 +2.72cm。
 *    ⚠ 之前那次五点标定【全场固定 S130】（a1 = 40），而两种模型在 a1 = 40 时
 *      是同一个式子，所以单点 a1 根本分不出来 —— 别再被那个"拟合很好"骗到。
 *
 * 步骤：
 *   ① 底座角 θ = atan2(u, v)  → 把机械臂转到目标所在竖直平面
 *   ② 【先把偏移项扣掉】换成"从肩关节轴出发的腕关节坐标"：
 *        腕的水平半径 = 笔尖半径 − L3A − L3B
 *        腕的高度     = 笔尖高度 + PEN_DROP − H_SHOULDER
 *   ③ 对腕关节做标准二连杆逆解（余弦定理 + atan2）→ 大臂角 a1、二连杆内角 e
 *      ★ 注意 e 是【二连杆的内角】（相对角），绝对角 a2 = a1 + e，
 *        发给小臂舵机的是 a2（见下面第 ④ 步）。
 * 返回 false 表示目标被钳到了可达边界（已打印提示）。            */
bool tableToJoints(float u, float v, float h, int &tB, int &tS, int &tE) {
  bool clipped = false;

  // ---- ① 底座 ----
  float theta = degrees(atan2(u, v));                 // 目标相对正前方的偏角
  int base = BASE_ANG + (int)round(BASE_INVERT ? -theta : theta);
  if (base < BASE_ANG - BASE_SWEEP) { base = BASE_ANG - BASE_SWEEP; clipped = true; }
  if (base > BASE_ANG + BASE_SWEEP) { base = BASE_ANG + BASE_SWEEP; clipped = true; }
  tB = constrain(base, BASE_MIN, BASE_MAX);

  // ---- ② 半径钳制 + 换成「相对肩关节轴」的腕坐标 ----
  float r0 = sqrt(u * u + v * v);                     // 笔尖相对底座轴的半径
  float rt = constrain(r0, R_MIN, R_MAX);
  if (rt != r0) clipped = true;

  float r = rt - L3A - L3B;                           // 腕的水平半径（从肩轴算）
  float z = (h + PEN_DROP) - H_SHOULDER;              // 腕相对肩轴的高度

  // ---- ③ 二连杆逆解（肩 → 肘 → 腕）----
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
  float e = ELBOW_SIGN * degrees(acos(cosE));         // 二连杆内角（相对角，可正可负）
  float s = degrees(atan2(z, r)) - degrees(atan2(ARM_L2 * sin(radians(e)),
                                                ARM_L1 + ARM_L2 * cos(radians(e))));
  float a2 = s + e;                                   // ★ 小臂【绝对】角 = 大臂角 + 内角
  // ---- ④ 连杆角 → 舵机角 ----
  /* 落到实测行程内。若标定还没做（TRIM/INVERT 不准），这一步会把越界角
   * 钳到极限上而不是发出去顶止点；但那样画出来的轨迹是变形的，
   * 所以钳制只是"不伤舵机"的保险，几何仍必须靠标定校准。
   * ⚠ 小臂：发出去的是【绝对角 a2】，不是内角 e（见文件头那段 ★★）。 */
  /* ★ 2026-10-16：顺手把被 round() 舍掉的那一截存进 fineS / fineE，
   *   给 applyFine() 补精度用（见 writeServos 上面那段）。
   *   被钳过就没有"小数"可言 —— 直接置 0，别补一个到不了的角。 */
  {
    float fs = SHOULDER_TRIM + (SHOULDER_INVERT ? -s : s);
    const int ts = (int)round(fs);
    tS = constrain(ts, SH_MIN, SH_MAX);
    fineS = (tS == ts) ? (fs - tS) : 0.0f;
  }
  {
    float fe = ELBOW_TRIM + (ELBOW_INVERT ? -a2 : a2);
    const int te = (int)round(fe);
    tE = constrain(te, EL_MIN, EL_MAX);
    fineE = (tE == te) ? (fe - tE) : 0.0f;
  }

  /* ★ 小臂不得超过大臂（机械止点，见 mearm_main.ino 里 ELBOW_OVER_ARM_MAX 那段）。
   *   绝对角模型下 a1、a2 各自独立，这条必须显式钳制，否则会把肘顶到止点上。 */
  {
    float a2Max = s - ELBOW_OVER_ARM_MAX;
    int teLim = ELBOW_TRIM + (int)floor(ELBOW_INVERT ? -a2Max : a2Max);
    if (ELBOW_INVERT ? (tE < teLim) : (tE > teLim)) {
      tE = constrain(teLim, EL_MIN, EL_MAX);
      fineE = 0.0f;                 // ★ 被这条钳过 ⇒ 小数残差作废
      clipped = true;
    }
  }

  if (clipped) {
    Serial.print(F("WARN 目标超出可达范围，已就近处理: u=")); Serial.print(u);
    Serial.print(F(" v=")); Serial.print(v);
    Serial.print(F(" h=")); Serial.println(h);
  }
  return true;
}

/* 正解：关节角 → 桌面坐标（标定 / 示教时显示用）
 * 注意这里算的是【笔尖】的位置：先算到腕关节的二连杆，再加上
 * L3A/L3B 两个水平偏移和 PEN_DROP 这个竖直偏移。 */
void jointsToTable(int b, int s, int e, float &u, float &v, float &h) {
  float cs = SHOULDER_INVERT ? -(s - SHOULDER_TRIM) : (s - SHOULDER_TRIM);
  float ce = ELBOW_INVERT    ? -(e - ELBOW_TRIM)    : (e - ELBOW_TRIM);
  float a2 = ce;                                      // ★ 小臂绝对角 = ce（不是 cs + ce！）
                                                      //   见文件头 ★★ 那段。
  float r = L3A + ARM_L1 * cos(radians(cs))
                + ARM_L2 * cos(radians(a2)) + L3B;    // 笔尖半径
  float z = ARM_L1 * sin(radians(cs)) + ARM_L2 * sin(radians(a2))
            - PEN_DROP + H_SHOULDER;                  // 笔尖高度
  float th = radians(BASE_INVERT ? -(b - BASE_ANG) : (b - BASE_ANG));
  h = z;
  u = r * sin(th);
  v = r * cos(th);
}


/* ---- 调试 / 标定工具 ---- */
void printCalib() {
  float u, v, h;
  jointsToTable(curBase, curShoulder, curElbow, u, v, h);
  Serial.print(F("CAL 舵机角 B=")); Serial.print(curBase);
  Serial.print(F(" S=")); Serial.print(curShoulder);
  Serial.print(F(" E=")); Serial.print(curElbow);
  Serial.print(F(" G=")); Serial.print(curGripper);
  Serial.print(F("  → 笔尖 u=")); Serial.print(u, 2);
  Serial.print(F(" v="));        Serial.print(v, 2);
  Serial.print(F(" h="));        Serial.println(h, 2);
}

/* JOY：连打 3 秒的摇杆原始读数（0~1023），标定摇杆用。
 *   松手不碰 → 读到的就是该轴的中位，填进 mearm_main.ino 的 JOY_CENTER[4]。
 *              四个轴各不相同（本机实测 529/512/529/509），不能共用 512。
 *   推到底   → 应接近 0 或 1023；某一端差很多说明摇杆量程不够。
 *   读数的抖动幅度 → 决定 JOY_DEADZONE 要留多大。
 * 打 3 秒是为了让你能一边推杆一边看数（松手看中位、按住看两端）。 */
/* ★ 2026-10-16 改：原来是「连打 3 秒」，但 3 秒是个【窗口】—— 用户得一边掐摇杆
 *   一边掐时间，很难对齐（实测踩过：打完再去推，窗口已经关了，看到的全是静止读数）。
 *   现在改成【一直打】：
 *     · 敲 `JOY` 之后持续打印，**边推摇杆边看数字跳**，不用卡时间
 *     · 敲**任意一个字符**就停（那个字符会被吃掉，不会被当成指令执行）
 *     · 最多打 60 秒自动停（免得忘在那儿，串口缓冲堆满把主循环卡住）
 *   顺手把【相对中位的偏差】也打出来 —— 推杆成不生效一眼就能看出来。 */
void printJoystick() {
  Serial.println(F("== 摇杆实时读数：一直打，敲任意字符停（最多 60 秒）=="));
  Serial.println(F("   括号里是【相对中位的偏差】，推杆应该让它跳到 ±几百"));
  for (uint16_t i = 0; i < 300; i++) {          // 300 × 200ms = 60 秒
    int a0 = analogRead(PIN_J1X), a1 = analogRead(PIN_J1Y);
    int a2 = analogRead(PIN_J2X), a3 = analogRead(PIN_J2Y);
    Serial.print(F("J1X=")); Serial.print(a0); Serial.print('(');
    Serial.print(a0 - JOY_CENTER[0]); Serial.print(F(")  J1Y="));
    Serial.print(a1); Serial.print('(');      Serial.print(a1 - JOY_CENTER[1]);
    Serial.print(F(")  J2X=")); Serial.print(a2); Serial.print('(');
    Serial.print(a2 - JOY_CENTER[2]); Serial.print(F(")  J2Y="));
    Serial.print(a3); Serial.print('(');      Serial.print(a3 - JOY_CENTER[3]);
    Serial.println(')');
    delay(200);
    if (Serial.available()) break;              // 敲任意字符就停
  }
  while (Serial.available()) (void)Serial.read();   // 把停下来的那个字符吃掉，别当指令执行
  Serial.println(F("== 结束 =="));
}

/* JOG 大臂连杆角 肘连杆角：直接给理论连杆角驱动，标定用
 *   大臂角 0 = 水平前伸，+ = 抬起
 *   肘角   ⚠ 现在是【小臂绝对角】：0 = 小臂水平，- = 小臂下垂
 *          （2026-10-10 晚改成绝对角之前的语义是"相对大臂"，已作废）
 *   ⚠ JOG 0 0 = 大臂小臂都水平 = 共线 = 机械止点够不到，别发，会顶住。 */
void cmdJog(const char *p) {
  const char *q = p + 3;
  while (*q == ' ') q++;      // ⚠ 必须先跳前导空格，否则 "JOG 10 -5" 里的第一个空格
  float s = atof(q);          //   会被当成两个参数的分隔符，e 变成 10 而不是 -5
  const char *r = strchr(q, ' ');
  float e = r ? atof(r + 1) : 0;
  /* 这里必须和 tableToJoints() 用同一套行程钳制，否则打印出来的 S/E
   * 是"想去的角"，实际驱动的是被钳过的角 —— 标定时照着打印值填 TRIM 就全错了。 */
  int ts = constrain(SHOULDER_TRIM + (int)round(SHOULDER_INVERT ? -s : s), SH_MIN, SH_MAX);
  int te = constrain(ELBOW_TRIM    + (int)round(ELBOW_INVERT    ? -e : e), EL_MIN, EL_MAX);
  int tb = constrain(BASE_ANG, BASE_MIN, BASE_MAX);
  Serial.print(F("JOG3D s=")); Serial.print(s);
  Serial.print(F(" e="));      Serial.print(e);
  Serial.print(F(" → 舵机 S=")); Serial.print(ts);
  Serial.print(F(" E="));        Serial.println(te);
  moveJoints(tb, ts, te, curGripper);
  printCalib();
}

/* ---- 工具函数 ---- */
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
