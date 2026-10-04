/*
 * ============================================================
 *  meArm 机械臂 · 任务五（进阶）：多点平滑曲线 —— 早期版本
 *  考核方向：算法（控制、视觉）方向 · 二面考核
 * ============================================================
 *  ⚠ 说明：本文件用「串口输入点列」的方式演示 Catmull-Rom 平滑曲线，
 *     没有示教功能、没有暂停/继续/取消、绘图平面是竖直纸面。
 *     正式提交 / 现场演示请使用整合工程 mearm_main/mearm_main.ino，
 *     那里做了完整的子任务 4 / 5（摇杆五点示教 + 折线 / 平滑曲线 +
 *     按键暂停继续取消），本文件保留为开发过程记录与算法参考。
 * ============================================================
 *
 * ────────────────────────────────────────────────────────────
 * 【这个程序要解决的问题】
 * ────────────────────────────────────────────────────────────
 * 任务五还有一个要求：给出一串点（例如 (4,6) (6,7) (8,5) (10,8) ...），
 * 让机械臂用【平滑的曲线】依次经过这些点，而不是画成折线。
 *
 * 如果你只会"两点连直线"，那点多了画出来就是一条拐来拐去的
 * 折线（像心电图），不叫曲线。本程序用【Catmull-Rom 样条】
 * 把这些点"捋顺"成一条处处圆滑的曲线，且曲线保证经过
 * 你给的每一个点（这点很重要，见下面的原理）。
 *
 * ────────────────────────────────────────────────────────────
 * 【平滑的原理：Catmull-Rom 样条（建议看懂，面试加分）】
 * ────────────────────────────────────────────────────────────
 * 一、为什么两点连直线会"折"？
 *    折点处两条直线方向突变，机械臂走到折点时相当于"急转弯"，
 *    既难看又容易抖。想让折点变圆滑，关键是：
 *    进折点和出折点时要"提前拐弯"。
 *
 * 二、Catmull-Rom 怎么做到圆滑？
 *    它把点列分成一段一段：第 i 段连接 P[i] → P[i+1]。
 *    画这一小段时，它不光看自己的两个端点，还"偷看"旁边两个点：
 *      前一个点 P[i-1]、后一个点 P[i+2]。
 *    用这四个点决定这一小段的走向：
 *      · 起点 P[i] 处的切线方向 = P[i+1] - P[i-1]
 *        （即"从哪来、往哪去"，由前后点共同决定）
 *      · 终点 P[i+1] 处的切线方向 = P[i+2] - P[i]
 *    因为每一段的切线方向都和邻居共享同一个算法，
 *    段与段之间方向连续变化 → 整条曲线处处平滑（数学上叫 C1 连续）。
 *
 * 三、公式（每段只算 x 和 z 两个分量，写法一样）
 *    已知四点 P0=P[i-1]、P1=P[i]、P2=P[i+1]、P3=P[i+2]，
 *    参数 t 从 0（段起点）走到 1（段终点），任一点为：
 *
 *      x(t) = 0.5 * ( 2*P1.x
 *                   + (-P0.x + P2.x) * t
 *                   + (2*P0.x - 5*P1.x + 4*P2.x - P3.x) * t²
 *                   + (-P0.x + 3*P1.x - 3*P2.x + P3.x) * t³ )
 *
 *    t 的 1 次项决定"方向"、2 次项决定"曲率"、3 次项决定"变化的
 *    变化"，三个系数一起把一段弯成符合前后趋势的形状。
 *    曲线程序里每段取 SEG_POINTS 个点，t 均匀取 1/SEG_POINTS、2/SEG_POINTS...
 *    点越密，画出来越丝滑。
 *
 * 四、端点怎么办？
 *    第一段缺 P[-1]、最后一段缺 P[n]，直接"复制"端点：
 *    P[-1] 用 P[0]，P[n] 用 P[n-1]（相当于切线由自身唯一邻居决定）。
 *    效果：曲线从第一个点平滑地滑出去，滑进最后一个点。
 *
 * 五、验证"曲线经过每个给定点"
 *    把 t=0 代进公式：x(0) = 0.5*(2*P1.x) = P1.x，正好等于段起点；
 *    t=1：x(1) = 0.5*(2*P1.x + (-P0.x+P2.x) + (2P0.x-5P1.x+4P2.x-P3.x)
 *                  + (-P0.x+3P1.x-3P2.x+P3.x))
 *               逐项合并后 = 0.5*(2*P2.x) = P2.x，正好是段终点。
 *    所以每个给定点都是曲线上的点——"经过给定的点"是数学保证，
 *    不是近似。
 *
 * ────────────────────────────────────────────────────────────
 * 【运动学：末端位置 → 关节角度（与 mearm_drawing.ino 相同）】
 * ────────────────────────────────────────────────────────────
 * 逆解三步（几何法/余弦定理）：
 *   ① d = √(x²+z²)
 *   ② e = acos((d²-L1²-L2²)/(2·L1·L2))
 *   ③ s = atan2(z,x) - atan2(L2·sin e, L1+L2·cos e)
 * 弧度转度 + 安装偏移 → 舵机角度；不可达点自动跳过。
 *
 * 【校准说明（必须做）】
 *   ARM_L1 / ARM_L2：大臂、小臂实际长度（cm）。
 *     你的机械臂大臂、小臂都是 80mm = 8.0cm，已填好。
 *   SHOULDER_OFFSET / ELBOW_OFFSET：舵机零位偏差，画歪就调。
 *   DRAW_Z：笔尖刚好接触纸面的高度。
 * ============================================================
 * 【硬件接线】（官方基准 code.ino）
 * ============================================================
 *   底座 D9 / 大臂 D7 / 小臂 D8 / 夹爪 D6
 *   笔固定在夹爪上，夹爪角度固定为 PEN_GRIP。
 *   纸平铺在机械臂正前方（底座 90° 方向）。
 * ============================================================
 * 【使用方法】（串口监视器 9600 波特率）
 * ============================================================
 *   curve 4,6;6,7;8,5;10,8;12,7;14,9   ← 输入点列，开始画平滑曲线
 *       格式：curve x1,z1;x2,z2;x3,z3;...  （分号隔点，逗号隔 x/z）
 *       注意：点之间用英文分号 ; 和英文逗号 ,（不是中文标点）
 *   demo        ← 画一条内置示例曲线（看看效果再自己输点）
 *   home        ← 回初始姿态
 * ============================================================
 */

#include <Servo.h>        // 舵机库
#include <math.h>         // sin/cos/acos/atan2/sqrt 等数学函数

/* ============================================================
 * 舵机引脚定义（官方基准：D9 底座 / D7 大臂 / D8 小臂 / D6 夹爪）
 * ============================================================ */
const int PIN_BASE     = 9;    // 底座舵机信号线（base → D9）
const int PIN_SHOULDER = 7;    // 大臂舵机信号线（D7）
const int PIN_ELBOW    = 8;    // 小臂舵机信号线（D8）
const int PIN_GRIPPER  = 6;    // 夹爪舵机信号线（claw → D6）

/* ============================================================
 * 机械参数（按你实测填写）
 * ============================================================ */
const float ARM_L1 = 8.0;     // 大臂长度（cm）：底座转轴中心 → 肘关节中心（80mm）
const float ARM_L2 = 8.0;     // 小臂长度（cm）：肘关节中心 → 笔尖/腕中心（80mm）

/* 舵机安装偏移（度）：逆解算的是"理论连杆角度"，实际舵机零位有偏差。
 * 画出来整体歪/斜 → 调这两个数。 */
const int SHOULDER_OFFSET = 0;   // 大臂偏移
const int ELBOW_OFFSET    = 0;   // 小臂偏移

/* ============================================================
 * 绘制参数
 * ============================================================ */
const float DRAW_Z   = 6.0;    // 绘制平面高度（cm）：调成"笔尖刚接触纸面"
const int   PEN_GRIP = 60;     // 夹爪角度：夹住笔的固定角度
const int   BASE_ANG = 90;     // 画图时底座固定角度：90° = 正前方
const int   STEP_DELAY = 25;   // 每走一步延时（毫秒）：调大更稳更慢

/* 关节角安全边界 */
const int SHOULDER_MIN = 0;
const int SHOULDER_MAX = 180;
const int ELBOW_MIN    = 0;
const int ELBOW_MAX    = 180;

/* ============================================================
 * 曲线参数（想调效果就改这里）
 * ============================================================ */
const int MAX_POINTS = 30;     // 最多接收多少个点（数组要定大小，防越界）
const int SEG_POINTS = 20;     // 每两个给定点之间采样多少个插值点。
                               // 20 = 每小段 20 步；调大到 40 更丝滑但更慢。

/* ============================================================
 * 全局变量
 * ============================================================ */
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

int   curBase     = 90;    // 底座当前角度
int   curShoulder = 90;    // 大臂当前角度
int   curElbow    = 90;    // 小臂当前角度
int   curGripper  = 90;    // 夹爪当前角度

String serialBuffer = "";  // 串口接收缓冲

/* ============================================================
 * setup()：开机执行一次
 * ============================================================ */
void setup() {
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
  Serial.println("READY | 指令: curve x1,z1;x2,z2;... 画平滑曲线 / demo 示例曲线 / home 回中");
  Serial.println("例: curve 4,6;6,7;8,5;10,8;12,7;14,9");
}

/* ============================================================
 * loop()：主循环 —— 接收串口指令
 * ============================================================ */
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {          // 收到换行 = 一条指令结束
      if (serialBuffer.length() > 0) {
        String cmd = serialBuffer;
        serialBuffer = "";
        cmd.trim();                        // 去首尾空格

        // 指令分两类：纯命令（demo/home）和带参数的（curve ...）
        if (cmd == "demo") {
          demoCurve();
        } else if (cmd == "home") {
          goHome();
        } else if (cmd.startsWith("curve")) {
          // 从 "curve " 后面取出点列文本
          String body = cmd.substring(5);
          body.trim();
          drawSmoothCurve(body);           // 解析并绘制
        } else if (cmd.length() > 0) {
          Serial.println("未知指令: curve / demo / home");
        }
      }
    } else {
      serialBuffer += c;
    }
  }
}

/* ============================================================
 * 【运动学】targetToAngles()：末端位置 (x,z) → 舵机角度
 * （与 mearm_drawing.ino 完全一致，这里复用同一套逆解公式）
 * ============================================================ */
bool targetToAngles(float x, float z, int &shoulderDeg, int &elbowDeg) {
  float d = sqrt(x * x + z * z);                        // ① 末端到肩轴距离

  // 可达性检查：够不着或折叠死都不可达
  // 【注意】必须给输出参数赋值再返回，否则调用方会用到未初始化的栈垃圾。
  if (d > ARM_L1 + ARM_L2 || d < fabs(ARM_L1 - ARM_L2)) {
    Serial.print("目标不可达: x="); Serial.print(x);
    Serial.print(" z="); Serial.println(z);
    shoulderDeg = -1; elbowDeg = -1;                    // -1 = 不可达标记
    return false;
  }

  // ② 余弦定理求小臂夹角（余弦值可能略超 [-1,1]，先夹断）
  float cosE = (d * d - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2)
               / (2.0 * ARM_L1 * ARM_L2);
  cosE = constrain(cosE, -1.0f, 1.0f);
  float eRad = acos(cosE);

  // ③ 大臂夹角 = 末端方向角 - 小臂偏角
  float sRad = atan2(z, x)
             - atan2(ARM_L2 * sin(eRad), ARM_L1 + ARM_L2 * cos(eRad));

  // ④ 弧度转度 + 安装偏移 + 边界钳制
  shoulderDeg = (int)(degrees(sRad)) + SHOULDER_OFFSET;
  elbowDeg    = (int)(degrees(eRad)) + ELBOW_OFFSET;
  shoulderDeg = constrain(shoulderDeg, SHOULDER_MIN, SHOULDER_MAX);
  elbowDeg    = constrain(elbowDeg,    ELBOW_MIN,    ELBOW_MAX);
  return true;
}

/* ============================================================
 * 【执行】moveToXY()：末端移动到坐标 (x,z)
 * （和 mearm_drawing.ino 相同的同步走法：四轴一起逐步到位）
 * ============================================================ */
void moveToXY(float x, float z) {
  int targetShoulder, targetElbow;
  // 逆解失败（点不可达）→ 原地不动直接返回，避免用到未初始化的角度
  if (!targetToAngles(x, z, targetShoulder, targetElbow)) return;

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
 * 【核心】catmullPoint()：Catmull-Rom 样条单点求值
 * ------------------------------------------------------------
 * 输入：P0/P1/P2/P3 四个点（前后邻居 + 本段两个端点），
 *       t 在 0~1 之间（0=段起点 P1，1=段终点 P2）
 * 输出：曲线在这一小段上、参数 t 处的位置 (x, z)（通过引用带出）
 *
 * 公式（x、z 分量各自算一遍，写法完全一样）：
 *   value = 0.5 * ( 2*P1
 *                 + (-P0 + P2) * t
 *                 + (2*P0 - 5*P1 + 4*P2 - P3) * t²
 *                 + (-P0 + 3*P1 - 3*P2 + P3) * t³ )
 * ============================================================ */
void catmullPoint(float P0x, float P0z,
                  float P1x, float P1z,
                  float P2x, float P2z,
                  float P3x, float P3z,
                  float t, float &outX, float &outZ) {
  float t2 = t * t;          // t 的平方（t²）
  float t3 = t2 * t;         // t 的立方（t³）

  // x 分量：四个点的 x 坐标按公式组合
  outX = 0.5 * ( (2.0 * P1x)
               + (-P0x + P2x) * t
               + (2.0 * P0x - 5.0 * P1x + 4.0 * P2x - P3x) * t2
               + (-P0x + 3.0 * P1x - 3.0 * P2x + P3x) * t3 );

  // z 分量：四个点的 z 坐标按同样公式组合
  outZ = 0.5 * ( (2.0 * P1z)
               + (-P0z + P2z) * t
               + (2.0 * P0z - 5.0 * P1z + 4.0 * P2z - P3z) * t2
               + (-P0z + 3.0 * P1z - 3.0 * P2z + P3z) * t3 );
}

/* ============================================================
 * 【主流程】drawSmoothCurve()：解析点列 → 逐段画平滑曲线
 * ------------------------------------------------------------
 * 步骤：
 *   ① parsePoints() 把 "x1,z1;x2,z2;..." 文本解析成数组 pts[]
 *   ② 先把笔移到第一个给定点（起点）
 *   ③ 对每一段（i 从 0 到 n-2，连接 pts[i]→pts[i+1]）：
 *      取四个点：P0=pts[i-1]（i=0 时复制 pts[0]）
 *                P1=pts[i]
 *                P2=pts[i+1]
 *                P3=pts[i+2]（i=n-2 时复制 pts[n-1]）
 *      段内采样 SEG_POINTS 个 t，调用 catmullPoint 逐点 moveToXY
 *   ④ 全部段走完，曲线完成
 * ============================================================ */
void drawSmoothCurve(String body) {
  // ---------- ① 解析点列 ----------
  float ptsX[MAX_POINTS];
  float ptsZ[MAX_POINTS];
  int n = parsePoints(body, ptsX, ptsZ);   // n = 成功解析出的点数

  if (n < 2) {                              // 至少 2 个点才能连线
    Serial.println("点数不足：请至少输入 2 个点，例如 curve 4,6;10,8");
    return;
  }
  if (n > MAX_POINTS) n = MAX_POINTS;       // 超出上限就截断（防止数组越界）

  Serial.print("== 画平滑曲线，共 "); Serial.print(n);
  Serial.println(" 个给定点 ==");

  // ---------- ② 先走到第一个点（曲线从这里开始） ----------
  moveToXY(ptsX[0], ptsZ[0]);

  // ---------- ③ 逐段插值 ----------
  for (int i = 0; i < n - 1; i++) {
    // 取本段的四个参考点（注意端点复制）
    float P0x = (i == 0)     ? ptsX[0] : ptsX[i - 1];  // 前邻居；第一段用自己
    float P0z = (i == 0)     ? ptsZ[0] : ptsZ[i - 1];
    float P1x = ptsX[i];                                // 本段起点
    float P1z = ptsZ[i];
    float P2x = ptsX[i + 1];                            // 本段终点
    float P2z = ptsZ[i + 1];
    float P3x = (i == n - 2) ? ptsX[n - 1] : ptsX[i + 2]; // 后邻居；最后段用自己
    float P3z = (i == n - 2) ? ptsZ[n - 1] : ptsZ[i + 2];

    // 段内采样：t 从 1/SEG_POINTS 走到 1（起点已到位，从 t=1/SEG 开始）
    for (int k = 1; k <= SEG_POINTS; k++) {
      float t = (float)k / SEG_POINTS;        // 0 → 1 均匀分布
      float cx, cz;
      catmullPoint(P0x, P0z, P1x, P1z, P2x, P2z, P3x, P3z, t, cx, cz);
      moveToXY(cx, cz);                       // 末端走到曲线上的这个点
    }
  }

  Serial.println("平滑曲线完成");
}

/* ============================================================
 * parsePoints()：把 "x1,z1;x2,z2;..." 文本解析成坐标数组
 * ------------------------------------------------------------
 * 解析规则：
 *   用 ';' 切出一个个点 → "4,6"
 *   每个点再用 ',' 切成 x 和 z 两段 → "4" 和 "6"
 *   Arduino 字符串的 indexOf() 找分隔符，substring() 截取。
 * 返回：解析成功的点数（写进 ptsX / ptsZ 数组）
 * ============================================================ */
int parsePoints(String body, float ptsX[], float ptsZ[]) {
  int count = 0;
  int start = 0;

  while (count < MAX_POINTS) {
    // 找下一个分号（点与点的分隔符）
    int semi = body.indexOf(';', start);
    String seg;                                  // 当前这个点文本，如 "4,6"
    if (semi == -1) {
      seg = body.substring(start);               // 最后一个点（没有分号了）
    } else {
      seg = body.substring(start, semi);
    }
    seg.trim();
    start = semi + 1;

    if (seg.length() == 0) {                     // 空段（比如末尾多一个分号）跳过
      if (semi == -1) break;
      continue;
    }

    // 逗号切出 x 和 z
    int comma = seg.indexOf(',');
    if (comma == -1) {                           // 没有逗号 = 格式错误，跳过
      Serial.print("格式错误，忽略: "); Serial.println(seg);
      if (semi == -1) break;
      continue;
    }

    String xs = seg.substring(0, comma);
    String zs = seg.substring(comma + 1);
    xs.trim();
    zs.trim();

    ptsX[count] = xs.toFloat();                  // 字符串 → 数字
    ptsZ[count] = zs.toFloat();
    count++;
    Serial.print("  点 "); Serial.print(count);
    Serial.print(": ("); Serial.print(ptsX[count - 1]);
    Serial.print(", "); Serial.print(ptsZ[count - 1]);
    Serial.println(")");

    if (semi == -1) break;                       // 没有更多点了，结束
  }
  return count;
}

/* ============================================================
 * demoCurve()：内置示例曲线
 * ------------------------------------------------------------
 * 一条"波浪形"示例：5 个点，从左到右起伏。
 * 先看这个效果，再自己输入点列。
 * ============================================================ */
void demoCurve() {
  Serial.println("== 演示：波浪形示例曲线 ==");
  drawSmoothCurve("4,6;6,8;8,5;10,9;12,7;14,8");
}

/* ============================================================
 * goHome()：回到初始姿态
 * ============================================================ */
void goHome() {
  moveToXY(ARM_L1 + ARM_L2, DRAW_Z);    // 末端走到两臂总长处（可达最远点）
  Serial.println("已回初始位");
}
