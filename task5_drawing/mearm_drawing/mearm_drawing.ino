/*
 * ============================================================
 *  meArm 机械臂 · 任务五：轨迹绘制（画图）—— 早期版本
 *  考核方向：算法（控制、视觉）方向 · 二面考核（占 35%）
 * ============================================================
 *  ⚠ 说明：本文件是任务五的【早期独立版本】，绘图平面按「竖直纸面」建模
 *     （底座固定 90°，笔尖在 x-z 竖直平面内移动）。
 *     正式提交 / 现场演示请使用整合工程 mearm_main/mearm_main.ino：
 *       · 绘图平面改为「水平桌面」（底座扫左右 + 大臂小臂调半径）
 *       · 补齐了子任务 2 的 N/三角形/Z/V（本文件只有正方形/圆/五角星）
 *       · 补齐了子任务 3 的 暂停 / 继续 / 取消
 *       · 补齐了子任务 4 / 5 的五点示教 + 折线 / 平滑曲线
 *     本文件保留作为开发过程记录，其中的逆解函数仍可作参考。
 * ============================================================
 *
 * ────────────────────────────────────────────────────────────
 * 【这个程序要解决的问题】（任务五的目标）
 * ────────────────────────────────────────────────────────────
 * 让机械臂自己"画画"：笔固定在夹爪上，
 * 机械臂末端在纸面上画出 直线 / 正方形 / 圆 / 五角星。
 * 这个任务占 35%，是控制方向最核心的考核 ——
 * 因为它要求你把【运动学】和【轨迹规划】真正做出来，
 * 而不是只控制舵机角度。
 *
 * ────────────────────────────────────────────────────────────
 * 【三层思路：从"角度"到"图形"的跨越】
 * ────────────────────────────────────────────────────────────
 * 前面所有程序都在说"把舵机转到某个角度"。
 * 但画画时，我们关心的不是角度，而是【笔尖在纸上的位置 (x, z)】。
 * 所以需要三层转换：

 *   第一层 · 运动学（逆解）：末端位置 → 关节角度
 *     大臂+小臂像人的上臂+前臂，组成一个两连杆机构。
 *     已知笔尖位置 (x, z)，用几何学（余弦定理）反推出
 *     大臂、小臂应该各转多少度 —— 这叫"逆运动学"。
 *     函数：targetToAngles()。
 *
 *   第二层 · 轨迹规划：图形 → 一串离散的点
 *     直线：从起点到终点，按比例均匀取 N 个点（线性插值）。
 *     圆  ：用参数方程 (x = cx + r·cosθ, z = cz + r·sinθ)，
 *           让 θ 从 0° 转到 360°，每隔几度取一个点。
 *     正方形：4 条直线连起来。
 *     五角星：一笔画顺序，交替经过外顶点和内顶点。
 *     函数：drawLine() / drawRect() / drawCircle() / drawStar()。
 *
 *   第三层 · 执行：逐点逆解、逐点移动
 *     拿到每个点的 (x, z) → targetToAngles 算角度 → moveToXY 走过去。
 *     点取得越密，画出的线越平滑（INTERP_STEPS 控制密度）。
 *
 * ────────────────────────────────────────────────────────────
 * 【运动学推导（面试会问，建议看懂）】
 * ────────────────────────────────────────────────────────────
 * 设：大臂长度 L1，小臂长度 L2，末端位置 (x, z)。
 *
 * ① 末端到肩轴的距离：d = √(x² + z²)          （勾股定理）
 * ② 小臂夹角 e：三边已知（L1、L2、d），用余弦定理：
 *        cos e = (d² - L1² - L2²) / (2·L1·L2)
 *        e = acos(cos e)
 * ③ 大臂夹角 s：用 atan2 分两步——
 *        末端方向角     = atan2(z, x)
 *        小臂造成的偏角 = atan2(L2·sin e, L1 + L2·cos e)
 *        s = 末端方向角 - 小臂偏角
 * ④ 最后把弧度转成度数，再加"安装偏移"补偿舵机装歪的误差。
 * 如果 d 超出 L1+L2（够不着）或小于 |L1-L2|（完全折叠），
 * 该点不可达，程序会跳过并提示。
 *
 * 【校准说明（必须做，否则画歪）】
 *   ARM_L1 / ARM_L2：量你的大臂、小臂实际长度（单位 cm）；
 *   SHOULDER_OFFSET / ELBOW_OFFSET：舵机零位和理论角度之间的偏差，
 *     画出来是歪的/斜的，就调这两个数；
 *   DRAW_Z：笔尖刚好接触纸面的高度（让笔尖压在纸上）。
 * ============================================================
 * 【硬件接线】（官方基准）
 * ============================================================
 *   底座 D9 / 大臂 D7 / 小臂 D8 / 夹爪 D6
 *   笔用橡皮筋/胶带固定在夹爪上，夹爪角度固定为 PEN_GRIP（夹笔角度）。
 *   建议把纸平铺在机械臂正前方（底座 90° 方向）。
 * ============================================================
 * 【使用方法】（串口监视器 9600）
 * ============================================================
 *   line → 画直线    rect → 画正方形
 *   circ → 画圆      star → 画五角星
 *   home → 回初始姿态
 * ============================================================
 */

#include <Servo.h>        // 引入舵机库
#include <math.h>         // 引入数学库：sin/cos/acos/atan2/sqrt 等函数都在这里

/* ============================================================
 * 舵机引脚定义（官方基准）
 * ============================================================ */
const int PIN_BASE     = 9;    // 底座舵机信号线（base → D9）
const int PIN_SHOULDER = 7;    // 大臂舵机信号线（D7）
const int PIN_ELBOW    = 8;    // 小臂舵机信号线（D8）
const int PIN_GRIPPER  = 6;    // 夹爪舵机信号线（claw → D6）

/* ============================================================
 * 机械参数（必填！按你的实物量好再填）
 * ============================================================ */
const float ARM_L1 = 8.0;     // 大臂长度（cm）：底座转轴中心 → 肘关节中心
                              // 你实测大臂为 80mm，即 8.0cm（1cm = 10mm）
const float ARM_L2 = 8.0;     // 小臂长度（cm）：肘关节中心 → 笔尖/腕中心
                              // 你实测小臂也是 80mm，即 8.0cm

/* 舵机安装偏移（度）：逆解算出的是"理论连杆角度"，
 * 实际舵机装上后零位有偏差，用偏移修正。
 * 画图整体歪/斜 → 调这两个数（先调大臂，再调小臂）。 */
const int SHOULDER_OFFSET = 0;   // 大臂偏移
const int ELBOW_OFFSET    = 0;   // 小臂偏移

/* ============================================================
 * 绘制参数
 * ============================================================ */
const float DRAW_Z   = 6.0;    // 绘制平面高度（cm）：笔尖所在高度。
                               // 调成"笔尖刚接触纸面"最合适。
const int   PEN_GRIP = 60;     // 夹爪角度：夹住笔的固定角度（按实际夹笔姿势调）
const int   BASE_ANG = 90;     // 画图时底座固定角度：90° = 正前方
const int   STEP_DELAY = 25;   // 每走一步的延时（毫秒）：调大画得慢更稳

/* 关节角安全边界 */
const int SHOULDER_MIN = 0;
const int SHOULDER_MAX = 180;
const int ELBOW_MIN    = 0;
const int ELBOW_MAX    = 180;

/* ============================================================
 * 轨迹参数（想画不同图形就改这里）
 * ============================================================ */
const float LINE_X1 = 4.0;  const float LINE_Z1 = DRAW_Z;   // 直线起点 (x,z)
const float LINE_X2 = 12.0; const float LINE_Z2 = DRAW_Z;   // 直线终点 (x,z)

const float RECT_SIZE = 6.0;    // 正方形边长（cm）
const float RECT_CX   = 8.0;    // 正方形中心 x
const float RECT_CZ   = DRAW_Z; // 正方形中心 z（就是绘制面高度）

const float CIRC_R = 4.0;       // 圆半径（cm）
const float CIRC_CX = 8.0;      // 圆心 x
const float CIRC_CZ = DRAW_Z;   // 圆心 z

const float STAR_R = 5.0;       // 五角星外接圆半径（cm）
const float STAR_CX = 8.0;      // 五角星中心 x
const float STAR_CZ = DRAW_Z;   // 五角星中心 z

const int   INTERP_STEPS = 40;  // 每条轨迹离散成的点数。
                                // 40 = 把一段线切成 40 个小段；
                                // 调大到 100 更平滑（但画得更慢）。

/* ============================================================
 * 全局变量
 * ============================================================ */
Servo servoBase;        // 底座舵机对象
Servo servoShoulder;    // 大臂舵机对象
Servo servoElbow;       // 小臂舵机对象
Servo servoGripper;     // 夹爪舵机对象

int   curBase     = 90;   // 底座当前舵机角度
int   curShoulder = 90;   // 大臂当前舵机角度
int   curElbow    = 90;   // 小臂当前舵机角度
int   curGripper  = 90;   // 夹爪当前舵机角度

String serialBuffer = "";   // 串口接收缓冲

/* ============================================================
 * setup()：开机执行一次
 * ============================================================ */
void setup() {
  // 绑定四个舵机
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoGripper.attach(PIN_GRIPPER);

  // 上电姿态：底座正前方、大臂小臂 90°、夹爪夹笔
  servoBase.write(BASE_ANG);
  servoShoulder.write(90);
  servoElbow.write(90);
  servoGripper.write(PEN_GRIP);
  curBase = BASE_ANG;
  curShoulder = 90;
  curElbow = 90;
  curGripper = PEN_GRIP;

  Serial.begin(9600);
  Serial.println("READY | 指令: line 直线 / rect 正方形 / circ 圆 / star 五角星 / home 回中");
  Serial.println("提示: 请先完成机械参数校准（ARM_L1/L2/偏移）");
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

        cmd.trim();            // 去空格
        cmd.toLowerCase();     // 转小写

        if      (cmd == "line")  drawLine();    // 画直线
        else if (cmd == "rect")  drawRect();    // 画正方形
        else if (cmd == "circ")  drawCircle();  // 画圆
        else if (cmd == "star")  drawStar();    // 画五角星
        else if (cmd == "home")  goHome();      // 回初始
        else if (cmd.length() > 0) Serial.println("未知指令: line/rect/circ/star/home");
      }
    } else {
      serialBuffer += c;
    }
  }
}

/* ============================================================
 * 【第一层 · 运动学】targetToAngles()：末端位置 → 舵机角度
 * ------------------------------------------------------------
 * 输入：x（水平前伸距离 cm）、z（垂直高度 cm）
 * 输出：shoulderDeg（大臂舵机应转的角度）、elbowDeg（小臂舵机角度）
 *       通过 & 引用参数把结果"带出去"（函数内部改了，外面也能用）
 *
 * 公式回顾（见文件头推导）：
 *   d  = √(x²+z²)                          —— 末端到肩轴的距离
 *   e  = acos((d²-L1²-L2²)/(2·L1·L2))      —— 小臂夹角（肘上弯解）
 *   s  = atan2(z,x) - atan2(L2·sin e, L1+L2·cos e)  —— 大臂夹角
 * 注意：三角函数全是弧度制，最后用 degrees() 转成度。
 * ============================================================ */
bool targetToAngles(float x, float z, int &shoulderDeg, int &elbowDeg) {
  float d = sqrt(x * x + z * z);                // ① 末端到肩轴距离（勾股定理）

  // 可达性检查：够不着（超过两臂总长）或折叠死（小于两臂差）都不可达
  // 【注意】必须给输出参数赋值再返回！早期版本直接 return，
  //         调用方会拿到未初始化的栈垃圾，机械臂会突然乱甩。
  if (d > ARM_L1 + ARM_L2 || d < fabs(ARM_L1 - ARM_L2)) {
    Serial.print("目标不可达: x="); Serial.print(x);
    Serial.print(" z="); Serial.println(z);
    shoulderDeg = -1; elbowDeg = -1;   // -1 = 不可达标记
    return false;
  }

  // ② 余弦定理求小臂夹角（余弦值可能因浮点误差略超 [-1,1]，先夹断）
  float cosE = (d * d - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2)
               / (2.0 * ARM_L1 * ARM_L2);
  cosE = constrain(cosE, -1.0f, 1.0f);
  float eRad = acos(cosE);                      // 小臂夹角（弧度）

  // ③ 大臂夹角：末端方向角 - 小臂偏角
  float sRad = atan2(z, x)
             - atan2(ARM_L2 * sin(eRad), ARM_L1 + ARM_L2 * cos(eRad));

  // ④ 弧度转度 + 安装偏移 = 舵机输出角度
  shoulderDeg = (int)(degrees(sRad)) + SHOULDER_OFFSET;
  elbowDeg    = (int)(degrees(eRad)) + ELBOW_OFFSET;

  // 边界保护：超出舵机物理范围就钳制
  shoulderDeg = constrain(shoulderDeg, SHOULDER_MIN, SHOULDER_MAX);
  elbowDeg    = constrain(elbowDeg,    ELBOW_MIN,    ELBOW_MAX);
  return true;
}

/* ============================================================
 * 【第三层 · 执行】moveToXY()：末端移动到坐标 (x,z)
 * ------------------------------------------------------------
 * 流程：逆解算角度 → 四轴（含固定不动的底座/夹爪）同步走过去。
 * 底座和夹爪在画图中保持固定角度，但也放进同步循环，
 * 确保任何时刻四个舵机状态一致。
 * ============================================================ */
void moveToXY(float x, float z) {
  int targetShoulder, targetElbow;            // 先声明两个"接收结果的盒子"
  // 逆解（结果写进这两个变量）；失败说明该点不可达 → 原地不动，直接返回
  if (!targetToAngles(x, z, targetShoulder, targetElbow)) return;

  // 只要还有关节没到位就继续（和前面程序的 moveSync 一样的同步逻辑）
  while (curShoulder != targetShoulder || curElbow != targetElbow ||
         curBase != BASE_ANG || curGripper != PEN_GRIP) {

    if (curShoulder < targetShoulder) curShoulder++;
    else if (curShoulder > targetShoulder) curShoulder--;

    if (curElbow < targetElbow) curElbow++;
    else if (curElbow > targetElbow) curElbow--;

    if (curBase < BASE_ANG) curBase++;
    else if (curBase > BASE_ANG) curBase--;

    if (curGripper < PEN_GRIP) curGripper++;
    else if (curGripper > PEN_GRIP) curGripper--;

    servoBase.write(curBase);
    servoShoulder.write(curShoulder);
    servoElbow.write(curElbow);
    servoGripper.write(curGripper);

    delay(STEP_DELAY);
  }
}

/* ============================================================
 * 【第二层 · 轨迹】drawLine()：画直线（线性插值）
 * ------------------------------------------------------------
 * 线性插值原理：
 *   从起点到终点，参数 t 从 0 走到 1：
 *     x = 起点x + (终点x - 起点x) * t
 *     z = 起点z + (终点z - 起点z) * t
 *   t=0 在起点，t=1 在终点，中间均匀铺点 → 一条直线。
 * ============================================================ */
void drawLine() {
  Serial.println("== 画直线 ==");
  for (int i = 0; i <= INTERP_STEPS; i++) {
    float t = (float)i / INTERP_STEPS;                     // 0.0 → 1.0
    float x = LINE_X1 + (LINE_X2 - LINE_X1) * t;           // 插值出 x
    float z = LINE_Z1 + (LINE_Z2 - LINE_Z1) * t;           // 插值出 z
    moveToXY(x, z);                                        // 走到这个点
  }
  Serial.println("直线完成");
}

/* ============================================================
 * 【第二层 · 轨迹】drawRect()：画正方形
 * ------------------------------------------------------------
 * 正方形 = 4 条首尾相连的直线。
 * 先算出 4 个顶点的坐标（中心 ± 半边长），
 * 再按 左下→右下→右上→左上→左下 的顺序逐边绘制。
 * drawLineBetween() 是通用的"两点连线"函数，被多处复用。
 * ============================================================ */
void drawRect() {
  Serial.println("== 画正方形 ==");
  float h = RECT_SIZE / 2.0;                    // 半边长
  // 四个顶点坐标
  float x1 = RECT_CX - h, z1 = RECT_CZ - h;   // 左下
  float x2 = RECT_CX + h, z2 = RECT_CZ - h;   // 右下
  float x3 = RECT_CX + h, z3 = RECT_CZ + h;   // 右上
  float x4 = RECT_CX - h, z4 = RECT_CZ + h;   // 左上

  moveToXY(x1, z1);                            // 先走到起点
  drawLineBetween(x1, z1, x2, z2);   // 边 1（下边）
  drawLineBetween(x2, z2, x3, z3);   // 边 2（右边）
  drawLineBetween(x3, z3, x4, z4);   // 边 3（上边）
  drawLineBetween(x4, z4, x1, z1);   // 边 4（左边，闭合）
  Serial.println("正方形完成");
}

/* ============================================================
 * 【第二层 · 轨迹】drawCircle()：画圆
 * ------------------------------------------------------------
 * 圆的参数方程：
 *   x = cx + r·cos(θ)
 *   z = cz + r·sin(θ)
 * θ（西塔）从 0 转到 2π（即 360°），
 * 每转一点取一个坐标，连起来就是圆。
 * 采样点 = INTERP_STEPS * 2（比直线密一倍，圆需要更多点才圆滑）。
 * ============================================================ */
void drawCircle() {
  Serial.println("== 画圆 ==");
  moveToXY(CIRC_CX + CIRC_R, CIRC_CZ);   // 先到圆上 0° 位置（起点）

  for (int i = 1; i <= INTERP_STEPS * 2; i++) {
    float theta = 2.0 * PI * (float)i / (INTERP_STEPS * 2);  // θ: 0 → 2π
    float x = CIRC_CX + CIRC_R * cos(theta);                 // 参数方程算 x
    float z = CIRC_CZ + CIRC_R * sin(theta);                 // 参数方程算 z
    moveToXY(x, z);
  }
  Serial.println("圆完成");
}

/* ============================================================
 * 【第二层 · 轨迹】drawStar()：画五角星（一笔画）
 * ------------------------------------------------------------
 * 五角星技巧：一笔画，笔不离纸。
 * 它由 10 个顶点组成：5 个外顶点 + 5 个内顶点（凹陷处）。
 * 顶点位置：以外顶点为基准每隔 72° 一个，
 *           内外交替，内顶点半径 = 外半径 × 0.4。
 * 连线顺序：外顶点 → 内顶点 → 下一个外顶点 → 内顶点 → …
 * 用 order[] 数组指定"按哪个外顶点顺序跳着画"：
 *   0 → 4 → 8 → 2 → 6（这是外顶点在一笔画中的访问顺序）
 * 先算好 10 个顶点的坐标（用 cos/sin），再按顺序连线。
 * ============================================================ */
void drawStar() {
  Serial.println("== 画五角星 ==");
  const int N = 5;                       // 角数
  float outerR = STAR_R;                 // 外顶点半径
  float innerR = STAR_R * 0.4;           // 内顶点半径

  // 预计算 10 个顶点的极角（弧度）：
  //   偶数下标 = 外顶点，奇数下标 = 内顶点；从正上方（-90°）开始
  float angles[10];
  for (int i = 0; i < N; i++) {
    angles[i * 2]     = -PI / 2.0 + i * 2.0 * PI / N;         // 外顶点
    angles[i * 2 + 1] = -PI / 2.0 + (i + 0.5) * 2.0 * PI / N; // 内顶点
  }

  // 一笔画的"外顶点访问顺序"：0 → 4 → 8 → 2 → 6（跳角连线）
  int order[5] = {0, 4, 8, 2, 6};

  // 先走到第一个外顶点
  moveToXY(STAR_CX + outerR * cos(angles[0]),
           STAR_CZ + outerR * sin(angles[0]));

  // 依次连接：当前外顶点 → 内顶点 → 下一个外顶点
  for (int k = 0; k < N; k++) {
    int idx = order[k];                             // 当前外顶点下标
    int nextIdx = order[(k + 1) % N];               // 下一个外顶点下标
    int innerIdx = (idx + 1) % 10;                  // 中间的内顶点下标

    // 三个关键点的坐标
    float ox = STAR_CX + outerR * cos(angles[idx]);
    float oz = STAR_CZ + outerR * sin(angles[idx]);
    float ix = STAR_CX + innerR * cos(angles[innerIdx]);
    float iz = STAR_CZ + innerR * sin(angles[innerIdx]);
    float nx = STAR_CX + outerR * cos(angles[nextIdx]);
    float nz = STAR_CZ + outerR * sin(angles[nextIdx]);

    // 外顶点 → 内顶点 → 下一个外顶点（两段直线）
    drawLineBetween(ox, oz, ix, iz);
    drawLineBetween(ix, iz, nx, nz);
  }
  Serial.println("五角星完成");
}

/* ============================================================
 * drawLineBetween()：两点之间的直线插补（通用工具函数）
 *   被画正方形、画五角星复用：从 (x1,z1) 到 (x2,z2) 取 INTERP_STEPS 个点。
 * ============================================================ */
void drawLineBetween(float x1, float z1, float x2, float z2) {
  for (int i = 1; i <= INTERP_STEPS; i++) {
    float t = (float)i / INTERP_STEPS;      // 插值比例
    float x = x1 + (x2 - x1) * t;           // 插值 x
    float z = z1 + (z2 - z1) * t;           // 插值 z
    moveToXY(x, z);                         // 逐点移动
  }
}

/* ============================================================
 * goHome()：回到初始姿态
 *   先让末端走到"两臂伸直"的可达位置，再回到正前方初始位。
 * ============================================================ */
void goHome() {
  moveToXY(ARM_L1 + ARM_L2, DRAW_Z);    // 末端走到两臂总长处（可达最远点）
  Serial.println("已回初始位");
}
