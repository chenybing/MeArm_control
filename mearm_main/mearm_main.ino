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
 *                          RUN            依次跑 A、B、C 三组示教点位（一条命令走完三个）
 *   【任务二】示教找位      B30/S60/E100/G20   把某轴直接设到该位置
 *                          （B 底座  S 大臂  E 小臂  G 夹爪）
 *                          （B 数字小 = 往左、G 用 0~100 张开程度（大=张开），
 *                            这两条跟舵机原始值反向，见 task2_teach.ino 的换算）
 *                          JB/JS/JE/JG±5     在该轴基础上再转 5°（微调用）
 *                          W<0~16> / R<n> / LIST / WCLR
 *                                             记下当前姿态 / 回到该姿态 / 列出 / 清空
 *                          PP 0 1 2 3 4       按示教跑一遍取放（次序即走路次序）
 *   【任务三】按键遥控板    1              循环夹取：每按一次换下一个物体
 *                          2              录制开始 / 结束（摇杆操控，>10 秒）
 *                          3              回放上一次录制的动作
 *                          4              回中（回到初始姿态）
 *   【任务五】绘图          LINE           直线
 *                          N/T/Z/V        字母 N / 三角形 / 字母 Z / 字母 V
 *                                         （三角形也可发 TRI）
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
 *   ① 量尺寸：ARM_L1(肩轴→肘轴)、ARM_L2(肘轴→**腕轴**，不是笔尖)、L3A、L3B、
 *             PEN_DROP、H_SHOULDER(肩关节轴离**纸面**高度)
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
#include <EEPROM.h>     // 示教点位存这儿，重新烧程序不会丢（见 task2_teach.ino）

/* ── 按键遥控板接入方式（任务三）──────────────────────────────────────────
 *  0 = 用硬件串口 D0/D1（推荐，最稳）
 *      接线：遥控板 TX → 机械臂 D0(RX)，遥控板 GND → 机械臂 GND
 *      ⚠ 演示任务三时必须【拔掉机械臂的 USB 线】，改由外接 5V 供电。
 *        否则板载 CH340 的 TX 和遥控板的 TX 会一起驱动 D0，互相打架。
 *  1 = 用软串口 D12(RX)/D13(TX)
 *      接线：遥控板 TX → D12，遥控板 RX → D13（可选，回状态给 OLED）
 *      好处：USB 可以一直连着电脑，任务一/二/五 的串口指令随时能发。
 */
/* ★★ 2026-10-16 试过改成 1，**又改回来了**：
 *   开软串口要拉进整个 SoftwareSerial 库 ⇒ **+1.1KB flash**：
 *        =0 → 31460 B (96.0%)      =1 → 32618 B (99.5%)
 *   而 Uno 的草图上限是 **32256 B**（32768 − 512 bootloader）⇒
 *   **32618 直接超了 362 字节，IDE 会报 "Sketch too big"，烧不进去。**
 *
 *   ⇒ 任务三就走硬件串口（=0）：遥控板 TX 接机械臂 **D0**，
 *     演示时【拔掉机械臂的 USB 线】，用外接 5V 供电
 *     （舵机本来就必须外接电源，所以不算额外代价）。
 *     想看机械臂的输出？遥控板会把状态行转发到它自己的串口监视器上。
 *   ⚠ 真要用软串口，得先腾出 ≥400 字节 flash（现成的死代码：pickAndPlace 那一套）。 */
#define REMOTE_SOFT_SERIAL 0

#if REMOTE_SOFT_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial remoteSerial(12, 13);   // RX, TX
#endif

/* ---- 引脚定义 ---- */
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
const uint8_t BTN_CYCLE  = 12;     // 任务三：循环夹取
const uint8_t BTN_PLAY   = 13;     // 任务三：回放

/* ── 本地按键总开关 ────────────────────────────────────────────────────────
 *   0 = 这些引脚上【没接】按键（或还没接好）
 *   1 = 已按接线表把 D2 / D3 / D10 / D11 和两个摇杆自带的按键都接好
 *
 * ⚠ 这组按键是【可选】的，不接也能完成所有任务：
 *     任务五 暂停/继续/取消  → 也可以用串口 PAUSE / RESUME / CANCEL 触发
 *     任务五 记录示教点      → 也可以用串口 SET 触发
 *   但引脚上没接按键时，一定要设成 0！
 *   悬空的输入脚会被舵机大电流引起的地弹和干扰反复拉低，固件会以为
 *   "有人不停按暂停"，表现就是：什么都没按，串口却一直刷 STAT:PAUSED，
 *   机械臂卡在暂停里再也不动。上面那次"S 合不上"就是这么来的。
 *
 * ⚠ 任务五子任务3 考核要求"分别使用按键触发"暂停/继续/取消，所以等按键
 *   接好以后要把它改回 1。 */
/* ★ 2026-10-16：改成 1 —— 按键已经接好，任务三和任务五都靠按键触发，
 *   不再用电脑串口发指令。
 *
 *   按键分配（每个键一端接引脚、另一端接 GND，用内部上拉）：
 *     D2  暂停        D11 记录示教点
 *     D3  继续        D12 循环夹取
 *     D10 取消        D13 回放
 *     D4  录制        D5  回中      （这两个是摇杆自带的按键）
 *   ⚠ 没接按键的引脚必须保持悬空以外的处理 —— 见上面那段说明：
 *     引脚悬空会被舵机电流干扰拉低，固件会以为"有人在一直按"。
 *     所以【按键没接全的时候，这个开关要改回 0】。 */
#define LOCAL_BUTTONS 1


/* ---- 机械参数（2026-10-05 上机实测；模型见下方注释）-----------------------
 *  这台 meArm 是【平行四边形连杆】：那根从小臂上方牵到底座的长杆，让末端的
 *  "手"（装夹爪那块）始终【水平】，因此夹在爪上的【笔始终竖直向下】。
 *  实测确认：只动大臂、只动小臂，笔都保持竖直。
 *
 *  由此笔尖位置（相对底座转轴）为：
 *     半径 r = L3A + L1·cos(a1) + L2·cos(a2) + L3B
 *     高度 h = H_SHOULDER + L1·sin(a1) + L2·sin(a2) − PEN_DROP
 *  其中 a1 = 大臂绝对角、a2 = 小臂绝对角（都从水平算，向上为正）。
 *
 *  ⚠ 旧版本把「小臂 + 夹爪 + 笔」当成一根刚性连杆，直接量到笔尖 —— 那是错的，
 *    会让逆解算出的笔尖位置整体偏掉（缺 L3A/L3B/PEN_DROP 三项，而 L2 也量多了）。 */
/* ⚠ 2026-10-09 换平行夹爪后【六个全部重新实测】。量法见 docs/标定表.md 第 2~4 组。
 *   旧值（L3B=1.5 / PEN_DROP=7.0 / H_SHOULDER=6.0）是按旧爪定的，作废。
 *   ⚠ 变化最大的是 L3B：从 1.5 跳到 8.5 —— 新爪很长，笔尖比腕轴伸出 8.5cm。
 *     这一条直接决定笔尖离底座的最小半径（= L3A + L3B = 10cm），
 *     所以绘图区必须整体往外挪，详见 R_MIN/R_MAX 那一段。 */
const float ARM_L1   = 8.0;        // 大臂：肩关节轴 → 肘关节轴（和官方 80mm 一致）
const float ARM_L2   = 8.0;        // 小臂：肘关节轴 → 腕关节轴（⚠ 只到"腕"，不到笔尖！）
const float L3A      = 1.5;        // 底座转轴 → 肩关节轴 的水平距离（俯视量）
const float L3B      = 8.5;        // 腕关节轴 → 笔尖 的水平距离（俯视量）⚠ 可能为负，照实填
const float PEN_DROP = 7.5;        // 腕关节轴 → 笔尖 的竖直距离
                                   /* ★★ 2026-10-16 重新实测：9.0 → 7.5。
                                    *
                                    *   量法（现场端到端扫，最可靠）：
                                    *     用诊断指令 D<毫米> 在同一次烧录里逐毫米试，
                                    *     找"线连续、颜色正常、笔尖又没刮擦感"的那一档。
                                    *     扫描结果：D−13 已经淡得发虚、D−19 压得太死，
                                    *     **D−15 最好**。
                                    *   ⚠ D 和 PEN_DROP 在逆解里是【同一项】
                                    *     （z = (h + penHTrim + PEN_DROP) − H_SHOULDER），所以
                                    *       PEN_DROP 新值 = 旧值 + D值/10 = 9.0 + (−1.5) = 7.5。
                                    *   ⚠ 改完要把 D 归零（发 D0）—— 不能再靠它顶着。
                                    *
                                    *   为什么和 10-11 记的 9.0 差这么多：
                                    *     中间【换过爪、重装过笔】，10-16 当天又把笔【掰正】过，
                                    *     笔尖伸出的长度整个变了。
                                    *   ⚠⚠ 所以：**在笔杆和爪子上划一道对齐记号**，以后每次装笔
                                    *     都对齐它，否则每装一次都要重定一遍这个数。
                                    *
                                    * ⚠ 下面 2026-10-11 那一段【保留只作对照，别再引用】——
                                    *   那是上一支笔 + 上一个夹持姿态下的数。 */
                                   /* ★ 2026-10-11 重新实测：6.9 → 9.2 →(端到端复核)9.0。
                                    *   ⚠ 最后那 0.2 是【画图实测】调出来的，比尺子可信：
                                    *     烧 9.2 跑 LINE，笔尖浮着约 2mm、只"断断续续蹭一点"
                                    *     ⇒ 真值 = 9.2 − 0.2 = 9.0。（尺子那组反推是 9.13/8.94/9.47，
                                    *     均值 9.2 带 ±0.3，落在噪声里，这一改把误差收掉了。）
                                    *   为什么要重来：笔换了、机械组又把夹爪部件的【粗细】改了。
                                    *   机械组说"没改长度"——长度确实没改，但【笔尖挂在哪】整个变了，
                                    *   所以必须重量（换一支笔 PEN_DROP 就一定得重定）。
                                    *
                                    *   量法（钢直尺立纸面、眼睛与笔尖齐高、平视读【笔尖】高度）：
                                    *     S110/E100 → CAL 报 h=6.23，实测 4.0  ⇒ Δ=−2.23 ⇒ 9.13
                                    *     S130/E100 → CAL 报 h=4.44，实测 2.4  ⇒ Δ=−2.04 ⇒ 8.94
                                    *     S90 /E110 → CAL 报 h=8.57，实测 6.0  ⇒ Δ=−2.57 ⇒ 9.47
                                    *   三个 Δ 都是负的、都落在 −2.0~−2.6 ⇒ 是【固定偏移】不是乱数，
                                    *   取 9.2。三个姿态的几何差别很大还这么一致，可信。
                                    *
                                    * ⚠⚠ 符号别搞反：实测【低于】模型 ⇒ 笔尖挂得比模型以为的【低】
                                    *     ⇒ PEN_DROP 要【变大】。即 新值 = 旧值 − Δ（Δ 为负时就是加）。
                                    *     （10-10 晚那份记录写的 "6.9 + g" 只对当时的【旧笔】成立，别照抄。）
                                    *
                                    * ⚠ 还剩 ±0.3cm（尺子读数 + 可能轻微滑动、H_SHOULDER 自身 ±0.3）。
                                    *   最后要发一个 h=0 的目标，看笔尖到底触没触到纸，再微调一次。
                                    * ⚠ 这个数只吸收了【竖直】方向的误差；水平那个 L3B 还没验过。
                                    *   历史：换平行爪前 7.0；10-09 换爪后 6.7；10-10 晚 6.9。 */
const float H_SHOULDER = 6.2;      // 肩关节轴离【纸面】的高度 cm
                                   // ⚠ 必须从【纸面】量（不是桌面、更不是底座轴）。
                                   //   底座转轴自身的高度不进公式：水平零点在底座转轴那条
                                   //   竖线上、竖直零点在纸面上，两个零点不同，底座轴的高度
                                   //   被夹掉了。见本段上面的公式。
                                   //   这个值直接进 h = H_SHOULDER + … − PEN_DROP，
                                   //   差 0.5cm 就会让逆解算出的笔尖高度整体差 0.5cm。

/* ---- 舵机标定开关（2026-10-05 上机实测）------------------------------------
 * 这三个 INVERT 决定「连杆角 → 舵机角」的符号，翻错了会让逆解整体镜像、
 * 任务五画出来的图上下颠倒。下面每一条都有上机观察做依据：
 *
 *   底座 | 实测：x30（角度小于中位 90）时底座转向【右】。
 *        | u 轴右为正，θ = atan2(u,v) > 0 表示目标在右侧。
 *        | ⇒ 舵机角 = BASE_ANG − θ  ⇒  BASE_INVERT = true  ✅已确认
 *
 *   大臂 | 实测：y150 时大臂【压向桌面】、y30 时接近【竖直偏后】。
 *        | ⚠ 由这两条推出的舵机角方向，原文写反了。y150 经 servo = TRIM − s
 *        |   已落到下限 33（最低），y30 得到 88（较高），
 *        |   所以是【舵机角越小手臂越低】—— 不是原文说的"越大越低"。
 *        |   2026-10-05 上机直接敲 S30 看到大臂压桌面，与此一致，可以定论。
 *        | 逆解里 s > 0 表示抬起（s = 0 为水平前伸）。
 *        | ⇒ 舵机角 = TRIM − s  ⇒  SHOULDER_INVERT = true
 *        | ⚠ 但结论句既已写反，"当初为什么把 INVERT 翻过来"的理由也就不成立了。
 *        |   INVERT 暂仍为 true（改它要动逆解，现在的标定支撑不起验证），
 *        |   等实测到「大臂水平前伸」那一点时再复核。
 *
 *   小臂 | 实测：z30 时小臂【向下】，即舵机角越小越下垂。逆解里 e < 0 表示下垂。
 *        | ⇒ 舵机角 = TRIM + e  ⇒  ELBOW_INVERT = false  ✅已确认
 *        | ✅ TRIM = 100：2026-10-10 晚改成【绝对角模型】后重新定的，见下面 ELBOW_TRIM 处。
 *        |   ⚠ 值的含义变了：不再是"共线时的舵机角"，而是"小臂水平时的舵机角"。
 *        | ⚠ 旧值 113 作废 —— 那是"把大小臂摆成一条线"时读出来的，
 *        |   可这条机械臂【根本摆不成一条线】：小臂永远在大臂下面，
 *        |   所以 e = 0 这个姿态它压根到不了，那个 113 是读错的。
 */
const int  SHOULDER_TRIM   = 170;  // 大臂水平前伸（连杆角 s = 0）时的舵机角
                                   /* ✅ 2026-10-10 实测定死。
                                    * 做法：发 S<n>，逐点量【肘轴轴心】离纸面的高度。
                                    *   实测  S=90 → 14.0   S=110 → 13.0
                                    *        S=120 → 12.5   S=170 → 6.1  (cm)
                                    * 用 h = H_SHOULDER + ARM_L1·sin(TRIM−S) 拟合：
                                    *   最大残差 0.17cm；反解出的杆长 7.85~8.22，
                                    *   都落在 ARM_L1 = 8.0 上 ⇒ 量准了、模型也对。
                                    * ⚠ 旧值 118 作废。另外 task2_teach.ino 里
                                    *  "S 数字小 = 压得低" 的注释是【反的】，已随手改掉。
                                    * ⚠ 顺带结论：S 的机械止点 ≈ 175 ⇒ 大臂最低只到
                                    *   a1 ≈ −5°，**永远压不到水平以下**。画图规划时注意。 */
const bool SHOULDER_INVERT = true; // true → 舵机角 = TRIM − 连杆角
const int  ELBOW_TRIM      = 100;  // ★ 小臂【绝对角】a2 = 0（小臂水平）时的舵机角
                                   /* ★★ 2026-10-10 晚【改为绝对角模型】，值也从 138 改成 100。
                                    * 详见 kinematics.ino 文件头那段 ★★。
                                    *
                                    * 【为什么改】小臂舵机装在底座上、经【平行四边形】驱动小臂，
                                    *   平行四边形的性质 ⇒ 小臂的【绝对】方向由该舵机直接定死，
                                    *   与大臂转角无关。实测（S 130→110，E 都 100）：
                                    *     腕轴−肘轴  从 +0.15 只变到 −0.10 cm（几乎不动）
                                    *   而旧的相对角模型预测它要变 +2.72cm。⇒ 绝对角成立。
                                    * ⚠ 旧的相对角公式 a2 = a1 + (E − ELBOW_TRIM) 已作废。
                                    * ⚠ 为什么之前没发现：那次五点标定【全场固定 S130】（a1 = 40），
                                    *   两种模型在 a1 = 40 时是同一个式子，单点分不出来。
                                    *
                                    * 【100 是怎么来的】腕轴−肘轴 = 8·sin(a2)，a2 = E − TRIM：
                                    *   S130：E=110→+1.25  E=100→+0.15  E=90→−1.10 (cm)
                                    *   S110：E=100→−0.10（★这一条是判定绝对角的决定性数据）
                                    *   反算 TRIM = 101.0 / 98.9 / 97.9 / 100.7 ⇒ 平均 ≈ 99.6
                                    *   ⇒ 取整 【100】。（E=100 → a2 = 0 → 小臂水平，好记）
                                    * ⚠ 该组数据本身有 ~±1° 的散，100 与 99 都在误差内。
                                    *
                                    * 【同一次测量里的历史数据，方向对得上】
                                    *   上午五点（S130）：E=120→腕轴14.0/肘11.5、E=110→12.5、
                                    *   E=100→11.3、E=90→10.0、E=80→8.6；反算 TRIM ≈ 101.6。
                                    *   ⇒ 换舵机前 ≈101.6、换舵机后 ≈99.6，**偏了 2°**，
                                    *     和"换过小臂舵机、零点会动"完全吻合 ✅
                                    * ⚠ 注意：这一切都是在 S130 单点上做的，两模型等价，
                                    *   所以上面这些历史数【只定出 TRIM，定不出绝对/相对】。 */
/* ★ 机械止点：小臂【永远】在大臂下面、且折不直（结构事实，用户 10-10 亲口确认）。
 *   ⇒ 小臂绝对角 a2 必须 ≤ 大臂绝对角 a1 − 这个余量。
 *   ⚠ 改成【绝对角模型】之后，这条【不再自动成立】了 —— 相对角模型里 a2 = a1 + e
 *     且 e ≤ 0，所以 a2 ≤ a1 是白送的；绝对角模型里 a1 和 a2 各自独立，
 *     Cartesian / JOG 完全可能命令出 a2 > a1（= 把肘顶到止点上 = 堵转 = 发热 = 烧舵机）。
 *   ⚠ 余量【是估的，没实测】——实测要拿舵机去顶止点，不值当。真撞上工况再调。
 *     10° 的依据：S=130(a1=40) 时舵机量程上限 E=125 → a2=25（差 15° 还没到止点），
 *     所以真实余量 ≤ 15°；取 10° 是"够用又留一手"。 */
const float ELBOW_OVER_ARM_MAX = 10.0f; // 小臂至少要比大臂低这么多度

const bool ELBOW_INVERT    = false;// false → 舵机角 = ELBOW_TRIM + 肘角 e
const bool BASE_INVERT     = true; // true → 角度越小越靠右（实测 x30 = 往右）
                                   // ⚠ 这是【舵机角/逆解】的方向。示教指令 B30 是
                                   //   面向用户的，已镜像成"数字小 = 往左"，
                                   //   见 task2_teach.ino 的 baseUserToServo()。
const int  ELBOW_SIGN      = -1;   // 肘部解支：-1 = 小臂下垂解（桌面画图推荐）

const int BASE_ANG = 90;           // 底座中位（正前方）
const int BASE_SWEEP = 60;         // 底座左右最大摆幅（±度）

/* 舵机安全行程 ────────────────────────────────────────────────────────
 * 数据来源：mearm_range_test.ino 逐度试出来的【顶住那一下】的位置，各留 3° 余量。
 *
 * ★★ 2026-10-16 重扫（机械臂 + 夹爪都换过、装过，旧的一组全部作废）：
 *   底座 D9   实测 10 ~ 175  → BASE_MIN/MAX = 13 / 172   （原 8 超止点 2°）
 *   大臂 D7   实测 30 ~ 160  → SH_MIN/MAX   = 33 / 157   （原 172 超止点 12° ⚠）
 *   小臂 D8   实测 30 ~ 130  → EL_MIN/MAX   = 33 / 122   （原 23 超止点 7° ⚠）
 *   夹爪 D6   实测 70 ~ 175  → 换算到逻辑角，见下面夹爪那一段
 *
 * ⚠ 作废的那一组（只留个对照）：底座 5~175 / 大臂 30~175 / 小臂 20~125 / 夹爪 50~170。
 *   那是 2026-10-09 定的，之后机械臂拆装过，已经对不上 —— 三处都在【顶着机械止点】跑。
 * ⚠ 小臂那根是【反复烧过舵机】的那根（见 ELBOW_OVER_ARM_MAX 那段），它的下限尤其别放宽。
 *
 * ⚠ 小臂上限从 145 收到 122 —— 旧值会让小臂顶到机械止点。
 * ⚠ 夹爪两头全出了旧范围（旧 10~145，新原始角 50~170）。
 *
 * 这四个限位统一在 writeServos() 里生效 —— 它是所有舵机输出的唯一出口。
 * 因此无论是指令、逆解还是摇杆，都不可能把某个关节顶到机械止点上憋劲。
 * ⚠ 换机械臂 / 拆装舵机臂后必须重新实测并更新这里。
 *
 * ⚠ 待复核（并入零点标定一起做，见 docs/标定表.md 第 4 步）：
 *   实测底座 raw「5 = 向左」，但历史上本注释和 range test 工具都写「小 = 往右」。
 *   若现场 x90 时底座没指向 v 轴，说明底座舵机重装时翻了向，要动 BASE_INVERT。
 */
const int BASE_MIN = 13,  BASE_MAX = 172;   // ★ 2026-10-16：MIN 8→13（实测止点 10）
const int SH_MIN   = 33,  SH_MAX   = 157;   // ★ 2026-10-16：MAX 172→157（实测止点 160）
const int EL_MIN   = 33,  EL_MAX   = 122;   // ★ 2026-10-16：MIN 23→33（实测止点 30）
/* ⚠ 夹爪行程 —— 2026-10-09 第五次改。历史留着，别搞混：
 *     2026-10-05  实测 90~175      → 收成 93 / 172      （旧爪，作废）
 *     2026-10-06  排查时临时放开    → 0 / 180            （作废）
 *     2026-10-06  实测后的新值      → 20 / 90            （旧爪，作废）
 *     2026-10-07  换平行夹爪后实测  → 10 / 145           （作废）
 *     2026-10-09  重测 + 做镜像后   → 【53 / 167】       （作废）
 *     2026-10-09  上机试出「夹上」点 → 【66 / 167】       ← 现在用这个
 *
 * ⚠⚠ 这里是【逻辑角】，不是舵机原始角。
 *   夹爪硬件本身是「小角=张开、大角=闭合」（实测原始角 50=开 / 170=闭）。
 *   为了让全程序统一成「大=张开」，在 writeServos() 出口做了一次镜像：
 *       逻辑角 = GR_RAW_SUM − 原始角 = 220 − 原始角
 *   于是：逻辑 167(大) = 张开、逻辑 66(小) = 闭合。
 *   【和 range test 工具对照时记得换算：逻辑 = 220 − 工具读数】
 *
 * ★★ 2026-10-11 下限 66 → 90（机械组改了夹爪部件【粗细】之后重新确认）：
 *   空爪时两片爪尖的"刚好碰上"点，从原来的【逻辑 66】挪到了【约 90】。
 *   ⚠ 不改的后果（已实测确认）：66 现在远在"碰上"点之外 ⇒ 两片爪尖硬碰硬
 *     顶着还接着顶 20 多度 ⇒ 舵机【持续堵转】⇒ 一会儿就"特别特别烫"。
 *     严重时烧舵机 —— 这个爪子已经烧过一次。
 *   ⚠ 这次和【笔】无关：那是爪子自己的止点变了。（"笔太粗"是另一个独立问题。）
 * ⚠ 夹爪没有弹簧也没有软限位，夹过"刚好碰上"这个点就是【硬碰硬】，
 *   全程堵转会把电源拉下来（另外三个舵机跟着抖）。
 *   所以 GR_MIN 卡在"夹上"点上，任何路径（指令 / 逆解 / 摇杆 / 示教点）都别想夹更紧。
 * ⚠ 90 是从"空爪发 F90 就差不多合上了"反推的【保守值】——真正的止点应该比 90 还小几度。
 *   要精确的话：空爪从 F100 往下逐度扫，听/看【刚碰上】那一下。 */
/* ★★ 2026-10-16 用 range test 扫出来的【这个爪】的真实行程（原始角 70 开 ~ 175 夹紧）：
 *     空爪止点   原始 175  →  逻辑 45
 *     留 5° 余量 原始 170  →  逻辑 50   ← 本值取这个
 *   ⚠ 下面 2026-10-11 那条"约 90"以及更早的 66【全部作废，只留作对照】——
 *     那把爪已经不在了、刻度对不上。同一个量换一次爪就变一次：
 *       10-11 记「约 90」（拍脑袋反推的） / 10-12 实测 75 / 10-13 实测 60 /
 *       现在这个爪实测 45。**别再照抄历史值。**
 * ⚠ 夹爪没有弹簧也没有软限位，夹过"刚好碰上"这个点就是【硬碰硬】，全程堵转
 *   会把电源拉下来（另外三个舵机跟着抖，线会画不直）。
 *   所以 GR_MIN 卡在止点【之前】5°，不是卡在止点上。 */
const int GR_MIN   = 50,  GR_MAX   = 167;

/* 桌面可达范围（防止指令把机械臂甩到够不着的地方）。
 * 笔尖半径 = (L3A + L3B = 10cm 的爪子偏移) + 二连杆(0 ~ 16cm)，所以：
 *     理论最小半径 = 1.5 + |8−8| + 8.5 = 10.0cm
 *     理论最大半径 = 1.5 + (8+8)  + 8.5 = 26.0cm
 * ⚠ 2026-10-09 换新爪后 L3B 从 1.5 涨到 8.5，整个可达环带【整体外移了 7cm】——
 *   旧值 4/18 现在全错（下界 4 太小、上界 18 太小，绘图区会被白白卡住）。
 * ⚠ 下界 10 是【几何】极限：再近的话二连杆得折到底，爪子后端很可能先撞底座/
 *   小臂 —— 真正的可用下界要靠手动把笔尖往底座推、看先碰到什么来定（标定第 5 步）。
 *   真正够不够得着由 tableToJoints() 里的 d 检查兜底，这里只做粗筛。 */
const float R_MIN = 10.0;          // 笔尖离底座轴最小半径 cm（几何极限）
const float R_MAX = 26.0;          // 最大半径 cm（几何极限）


/* ---- 运行参数 ---- */
/* 摇杆中位（2026-10-04 用 JOY 指令实测，松手时连打 10 次取均值）。
 * ⚠ 四个轴各不相同，不能共用 512 —— 否则死区一边宽一边窄，
 *   表现是"往一个方向轻轻一推就动、往另一个方向要推很多才动"。
 * 顺序与 PIN_J1X / J1Y / J2X / J2Y 一一对应。
 *   实测：J1X≈529±5  J1Y≈512±4  J2X≈529±4  J2Y≈509±6 */
const int  JOY_CENTER[4] = { 529, 512, 529, 509 };
const int  JOY_DEADZONE  = 40;     // 摇杆死区（远大于 ±6 的抖动，不会自己漂）

/* ---- 摇杆方向开关（推杆方向与机械臂实际方向相反时翻这里）------------------
 * 生效范围：任务一摇杆操控、任务三录制、任务五示教 —— 三处共用这四个开关。
 *   true  = 该轴取反（推左 → 机械臂往右）
 *   false = 原方向
 *
 * 同一根轴在两种模式里的语义（只是叫法不同，物理上是同一根杆）：
 *   J1X  摇杆1 左右 → [手动] 底座   / [示教] 笔尖左右 u
 *   J1Y  摇杆1 前后 → [手动] 小臂   / [示教] 笔尖前后 v
 *   J2X  摇杆2 左右 → [手动] 夹爪
 *   J2Y  摇杆2 前后 → [手动] 大臂
 *
 * ⚠ 这里【只调摇杆手感】。与下面 SHOULDER_INVERT / ELBOW_INVERT / BASE_INVERT
 *   那组是两码事：那组是「几何正方向」的标定开关，由舵机实测行程反推得到，
 *   决定逆解算出的角朝哪边转。千万不要因为摇杆手感不顺去翻那一组 —— 翻了会
 *   让逆解和任务五绘图整体镜像，子任务 1~5 全部画歪。
 *   手动模式是直接写舵机角的，压根不经过逆解，所以「推左往右」只能由这里修。 */
const bool JOY_INV_J1X = true;     // 摇杆1 左右（底座 / 示教 u）
const bool JOY_INV_J1Y = true;     // 摇杆1 前后（小臂 / 示教 v）
const bool JOY_INV_J2X = true;     // 摇杆2 左右（夹爪）
const bool JOY_INV_J2Y = true;     // 摇杆2 前后（大臂）
const float TEACH_SPEED = 0.05;    // 示教模式：摇杆满偏时每步移动多少 cm（≈5cm/s）
/* ★★ 2026-10-16 定版（range test 实测 + 现场验证）：
 *   夹笔点 原始角 170  →  逻辑 50   ← 本值。用户现场确认「G170 夹得住笔」。
 *   （空爪止点 175 → 逻辑 45，所以夹笔是【离止点 5°】，和 GR_MIN 同一个数。）
 *   ⚠ 这正是 10-15 用户说的那个 "F50" —— 换算没错，逻辑 = 220 − 工具读数。
 *   ⚠ 代价：夹笔 = 夹到 GR_MIN 那道下限，全程顶着笔杆憋劲。
 *     【画完一条线就摸一下夹爪舵机烫不烫】；一旦发烫 / 有滋滋声就往上调几度
 *     （唯一的退路，别往下调 —— 往下就越过止点了）。
 *   ⚠ 下面 2026-10-11 那段「93 是暂填起点、好在 91~94 这一小段」的推导
 *     【已作废，保留只作对照】—— 那是按另一把爪的刻度推的。 */
const float PEN_GRIP = 50;         // 夹持铅笔的夹爪角度（【逻辑角】，大=张开）
                                   /* ⚠⚠ 2026-10-11 【待定值，还没定下来】—— 93 是暂填的起点，
                                    *   要用下面那个逐度扫描试出来再敲死。
                                    *
                                    *   背景（两件事同时发生，别混）：
                                    *   ① 【爪子自己】的止点从逻辑 66 挪到了约 90（机械组改粗细）
                                    *      ⇒ 发 66 是"爪尖压着爪尖还再顶 20 多度"，纯堵转发热。
                                    *        所以 GR_MIN / GRIP_CLOSE 也一起改成了 90。
                                    *   ② 【新笔】换了 ⇒ "刚好咬住笔杆"的那个角也跟着变了。
                                    *   合起来：90 会顶（烫），95 又夹不紧 ⇒ 好在 91~94 这一小段里。
                                    *
                                    *   ⚠ 夹爪是【位置舵机】：只要指令角比"咬住笔"的那个角更紧，
                                    *     它就一直顶着笔杆憋劲 —— 不管只紧 1° 还是 20°，都会发热。
                                    *     所以这个数【宁松勿紧】：松了顶多笔打滑（看得见、能补救），
                                    *     紧了是默默发热、烧舵机（这爪子已经烧过一次）。
                                    *     要调就往【大】调，别往小调。
                                    *
                                    *   ⚠ 定完之后必须验两条（任务五夹着笔一画就是好几分钟，热是累积的）：
                                    *     ① 用力扭笔杆、往外抽笔尖 → 笔不动；
                                    *     ② 夹持状态下【静止 2~3 分钟】→ 摸舵机不烫。
                                    *   历史：2026-10-09 实测定 66（旧爪 + 10mm 旧笔，笔卡在 V 槽里）。 */
const float PEN_LIFT = 2.0;        // 绘图时抬笔高度 cm
const float TEACH_H  = 0.3;        // 示教时笔尖离纸面的高度 cm（= 纸上 3mm）
                                   /* ★★ 2026-10-16 定档 0.3 —— 用户对比之后选的。
                                    *
                                    *   ⚠⚠ **这是个两难，不是"越小越好"**：
                                    *     · 示教【低】 ⇒ 示教姿态更接近落笔姿态(0mm) ⇒ **录点准**（画出来好）
                                    *     · 示教【高】 ⇒ 笔尖不蹭纸
                                    *   原理：程序记的是【模型算的笔尖位置】（jointsToTable）。
                                    *     示教姿态(h) 和落笔姿态(0) 差多少，"模型 vs 实机"的误差
                                    *     就差多少 ⇒ 那部分差【原样进到录下来的点里】，画出来就是系统偏移。
                                    *     所以示教越低、录得越准。
                                    *   ⚠ 代价：3mm 时【离底座最远的那两个区域会蹭纸】
                                    *     （臂在那儿下垂最多）。录点时尽量避开那两块；
                                    *     蹭到了就抬一下再回去重摆。
                                    *
                                    *   ⚠ 历史（别再重走）：0.3 → 0.15 → 0.3 → 0.6("不蹭但录不准")
                                    *     → 0.7 → 0.4 → 0.6 → **0.3（本值）**
                                    *     用户说"8:20 那版线画得好" —— 查了时间戳，当时就是 0.3。
                                    *   起因：用户实测——**在离底座最远的那两个区域示教会蹭到纸面**
                                    *   （纸上蓝笔圈出来的两处）。纸是平的，不是纸的问题。
                                    *
                                    *   根因：**臂伸得越长，下垂越多** ⇒ 笔尖比模型以为的低。
                                    *   这就是那个"姿态相关高度误差（实测 ~4~5mm）"——
                                    *   绘图时用 D/PEN_DROP 补偿掉的是【平均】位置，
                                    *   但**两端（半径最大处）比平均还低好几毫米**。
                                    *
                                    *   余量账（示教高度 − 最坏处下垂 4~5mm = 净余量）：
                                    *     3mm  → 净 ~−1mm   ⇒ 会蹭，但**录点准**  ← 本值
                                    *     4mm  → 净 ~0mm    ⇒ 还是蹭
                                    *     6mm  → 净 ~1.5mm  ⇒ 不蹭，但**录点带偏**（试过，画出来不如 3mm）
                                    *     7mm  → 净 ~2.5mm  ⇒ 余量够但"不好瞄了"（用户反馈）
                                    *    16mm  → 净 ~11mm   ⇒ 余量过剩，代价全在瞄准上（不建议）
                                    *
                                    *   ⚠ 下垂是【机械】的（连杆刚度 / 铰点间隙），
                                    *     代码治不了 —— 舵机开环、没有反馈，测不到实际低了多少。
                                    *     抬高度只能"绕开"它，代价就是上面说的录点变偏。
                                    *   ⚠ 真想把两边都解决，只有一条路：**紧平行四连杆的铰点螺丝**
                                    *     （档案里写着"还没紧过螺丝"）。紧完下垂变小，
                                    *     这个数就能降下来、同时不蹭纸。
                                    *     ⚠ 但紧螺丝会改回差，`PEN_DROP` 可能要重标一次。
                                    *
                                    *   ⚠ 这个数是【相对"笔尖刚好碰到纸"】的，只有在
                                    *     D / PEN_DROP 标定准了之后才有意义。
                                    *   ⚠ 必须和 penHTrim 一起用（见 teachStep），否则示教
                                    *     会和绘图差一个 D，笔尖悬空。
                                    *   ⚠ 历史：最早是 0.3 但 teachStep 没叠加 penHTrim，实机 22mm；
                                    *     修好叠加之后 0.3 = 3mm。**别看到数字就以为一样。** */

/* ---- ⚠ 诊断：笔尖高度【现场微调】 D<毫米> -------------------------------
 * 为什么要有它：换笔 / 换爪之后 PEN_DROP 得重定，而每试一档就要重烧一次 —— 太慢。
 *   有了它，在【同一次烧录】里就能一点点试到"笔尖刚好划出连续的线"，
 *   再把试出来的数填进 PEN_DROP（填完发 D0 清零）。
 *
 *   penHTrim 会【加】到绘图 / 示教的所有 h 目标上（唯一入口是 task5 的 penTo）。
 *     负值 = 往下压   例：D-2 = 整条轨迹往下压 2mm
 *     正值 = 抬高
 *     只发 D = 回显当前值（不清零）；D0 = 清零。
 *
 * ⚠ 最终定版后这里必须是 0 —— 数应该写进 PEN_DROP，不能靠这个偏移顶着，
 *   否则每次开机都得手动敲一遍 D。 */
float penHTrim = 0.0;              // cm。⚠ D 命令是【绝对设置】不是累加；
                                   //   最终定版后必须归 0（数要写进 PEN_DROP）
const float HOME_U = 0.0, HOME_V = 12.0, HOME_H = 4.0;  // 待机位
                                   /* ⚠ 2026-10-10 晚 v 从 8.0 改成 12.0：
                                    *   换平行夹爪后 L3A+L3B = 10cm，v=8 已经落在 R_MIN=10 外，
                                    *   每次回待机位都会被钳一下（"A" 那次折起来也有它一份）。 */
/* 夹爪张开 / 闭合角度（【逻辑角】，大 = 张开）。⚠ 必须落在 GR_MIN~GR_MAX(50~167) 内。
 *
 * 新爪是【四连杆】，舵机角 ↔ 两爪开口宽度 是【非线性】的，
 *   所以下面这两个数只能当"两端"用，中间每个物体该夹到几度必须实测。
 *   （旧爪的 20/90、12/142 都是按旧行程定的，换爪后语义整个变了，已作废。）
 *
 * 方向约定（2026-10-09 起）：数字大 = 张开，数字小 = 闭合。
 *   硬件其实正好相反（原始角小=开），靠下面这组 GR_RAW_* 在 writeServos() 出口
 *   做一次镜像扳过来。这样全程序口径统一：O、G100、摇杆推"开"、大数字 —— 全是张开。
 *
 * ⚠ GRIP_CLOSE 是【两爪刚好夹上、不顶】的位置，不是机械止点 —— 再往紧一点舵机就堵转。
 *   所以它和 GR_MIN 取同一个值。G0 就落在这里。 */
const int   GRIP_OPEN  = 145;      // ★★ 2026-10-16：167 → 145。
                                   //   实测【张开止点】= 原始 70 → 逻辑 150，留 5° ⇒ 145。
                                   //   ⚠⚠ 原值 167 = 原始 53，比张开止点【还多张 17°】——
                                   //     而 INIT_GRIPPER = GRIP_OPEN ⇒ 【每次上电】都在
                                   //     把夹爪往张开止点上顶（堵转）。这是个一直在犯的错。
const int   GRIP_CLOSE = 50;       // ★ 2026-10-15：90 → 50。必须跟着 GR_MIN 一起改
                                   //   （换回上一个爪后刻度挪位，见 GR_MIN 处；不改还是夹不紧）
                                   //   历史：2026-10-09 66 → 2026-10-11 90 → 现在 50

/* 夹爪镜像常数：逻辑角 = GR_RAW_SUM − 舵机原始角。
 * 50 = 硬件张开那一头，170 = 硬件闭合那一头（2026-10-09 实测）。
 * 只在 writeServos() 出口用一次；全程序其余地方一律用「逻辑角」。
 * 【和 range test 工具对照时：逻辑 = 220 − 工具读数】 */
const int   GR_RAW_OPEN = 50, GR_RAW_CLOSE = 170;
const int   GR_RAW_SUM  = GR_RAW_OPEN + GR_RAW_CLOSE;   // 220

/* 上电初始姿态 = 任务三按键4「回中」的目标姿态。
 * setup() 与 goHome() 共用这一份定义，避免两处各写一遍而对不上
 * （旧代码 goHome() 里夹爪写的是 90°，落在 122~168 的行程之外）。 */
const int   INIT_BASE = 90, INIT_SHOULDER = 90, INIT_ELBOW = 90, INIT_GRIPPER = GRIP_OPEN;

/* 速度档（任务一 H/L）：档位越高，每步延时越短，整体动作越快。
 * 表里是"每走 1° 停多久(ms)"，档间比值约 1.8 倍，快慢一眼可辨。
 *
 * 2026-10-05 整体调快了一档左右（旧表 100/50/28/16/9）：
 * 配合 stepDelayFor() 的梯形加减速，现在中段全速、两头才放慢，
 * 所以就算表里的数变小了，启停也不会有顿挫感。
 * 档 3（默认）14ms ≈ 70°/秒，比原来快一倍。 */
const int STEP_DELAY_TAB[5] = { 60, 30, 14, 9, 6 };  // 档 1~5
/* ⚠ 档 5（6ms）已经短过舵机库的刷新周期（20ms），再快也没用 ——
 * 舵机每个刷新周期才收到一次新角度，写得更密只是白写。
 * 想要更快要动的是"每步走几度"（现在是 1°），不是继续压这个延时。 */
/* 默认档位。单独列个常量，是因为任务三的回放要拿它当"1.0 倍速"的基准 ——
 * 原来那儿写死了 STEP_DELAY_TAB[2]（档 3），默认档一改成 4，
 * 回放就变成 60×9/14 ≈ 39ms/帧，比录制的 60ms 快 1.55 倍，"完整复现"就失真了。 */
const int SPEED_DEFAULT = 3;
int speedLevel = SPEED_DEFAULT;                        // 当前档位
/* 2026-10-05 调回档 3（约 63°/秒）—— 用户头一次试任务三，求稳。
 * 想快按 H（档 4、5），想更慢按 L（档 2、1）。
 * ⚠ 这个值只影响【机械臂自己走的动作】(PP、按键1、回中)；
 *   回放不受它影响 —— 回放按 REC_MS 原速重现，跟默认档是不是 3 没关系。 */
inline int stepDelay() { return STEP_DELAY_TAB[speedLevel - 1]; }


/* ---- 全局状态 ---- */
Servo svBase, svShoulder, svElbow, svGripper;
int curBase = 90, curShoulder = 90, curElbow = 90, curGripper = 90;

enum Mode { MODE_IDLE, MODE_MANUAL, MODE_TEACH, MODE_REC };
Mode mode = MODE_IDLE;

bool stPause = false;              // 暂停标志
bool stAbort = false;              // 取消标志

/* ---- 串口行缓冲（不使用 String，避免内存碎片）----
 * 两个通道各一份独立缓冲，互不干扰：CH_PC = 上位机，CH_RC = 按键遥控板。
 *
 * 这里用「数组 + 通道下标」而不是「结构体数组」，是为了让 pollOne() 的函数
 * 原型只引用内建类型。Arduino 会把自动生成的原型插到合并文件的开头，一旦
 * 原型里出现自定义结构体，就会落在结构体定义之前 —— 编译直接失败。
 * （本工程拆成多个 .ino，这个坑必须绕开。） */
/* 一次最多收这么多字符算一条指令。
 * ⚠ 这个值直接吃 RAM，Uno 上一字节都要省着用。曾经为了"一次贴 5 行 P 指令"
 *   把它拉到 128，结果 RAM 只剩 94 字节，板子一读到长指令就复位。
 *   现在定回 64：最长的单条指令是 "PP 10 11 12 13 14"（17 字符），绰绰有余。
 *   ⚠ 串口输入框是单行的，贴多行会被合成一行 —— 一次贴一行，分几次贴。 */
const uint8_t  LINE_MAX = 64;
const uint16_t LINE_GAP_MS = 40;   // 超过这么久没收到新字符就当作一条完整指令
const uint8_t  CH_PC = 0;          // 通道 0：上位机（硬件串口 Serial）
const uint8_t  CH_RC = 1;          // 通道 1：按键遥控板（REMOTE_SOFT_SERIAL=1 时才用）

/* ⚠ 缓冲按【实际启用的通道数】开，不要写死 2。
 * 遥控板通道关着的时候（REMOTE_SOFT_SERIAL=0），第二块缓冲纯粹是白占内存 ——
 * 在 Uno 上这是实打实的几十到上百字节。 */
#define LINE_CHANS (REMOTE_SOFT_SERIAL ? 2 : 1)
char     lineBuf[LINE_CHANS][LINE_MAX];
uint8_t  lineLen[LINE_CHANS]    = { 0 };
uint32_t lineLastMs[LINE_CHANS] = { 0 };

/* ---- 任务三：录制进行中标志 ----
 * 任务三的状态只有这一个留在公共区：modeName() 要用它区分 MANUAL / REC，
 * 而 modeName() 被主文件的状态上报共用。录制缓冲、帧计数、循环下标等其余
 * 任务三状态都定义在 task3_keyboard.ino 里。 */
bool recOn = false;

/* ---- 按键扫描 ---- */
const uint8_t BTN_COUNT = 8;
const uint8_t BTN_PINS[BTN_COUNT] = {
  BTN_PAUSE, BTN_GO, BTN_CANCEL, BTN_TEACH,
  BTN_CYCLE, BTN_PLAY, PIN_BTN2, PIN_BTN1
};
enum { B_PAUSE = 0, B_GO, B_CANCEL, B_TEACH, B_CYCLE, B_PLAY, B_REC, B_HOME };
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


/* ---- setup / loop ---- */
/* 剩余 RAM 字节数（AVR 上的经典算法：取一个栈上变量的地址，减去堆顶）。
 * 2026-10-05 加：出现过"打印到一半板子自己重启"，RAM 吃紧也是嫌疑之一，
 * 所以每次开机把余量打出来，一眼就能排除或坐实。
 * 正常应该剩几百字节；掉到 100 以下就要当心了。 */
extern int __heap_start, *__brkval;
int freeRam() {
  int v;
  /* 走 char* 相减、最后再收成 int：AVR 上 int 就是 16 位、指针也是 16 位，
   * 但对不上 64 位主机的语法检查（那儿指针到 int 直接转会报丢精度）。 */
  char *heapTop = (__brkval == 0) ? (char *)&__heap_start : (char *)__brkval;
  return (int)((char *)&v - heapTop);
}

void setup() {
  svBase.attach(PIN_BASE);
  svShoulder.attach(PIN_SHOULDER);
  svElbow.attach(PIN_ELBOW);
  svGripper.attach(PIN_GRIPPER);

  // 先摆到初始安全姿态，避免上电瞬间四个舵机乱甩
  // ⚠ 夹爪不能给 90°：它的实测行程是 122~168，90 在止点之外，一上电就憋劲。
  curBase     = INIT_BASE;
  curShoulder = INIT_SHOULDER;
  curElbow    = INIT_ELBOW;
  curGripper  = INIT_GRIPPER;
  writeServos();

  for (uint8_t i = 0; i < BTN_COUNT; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("READY | meArm 主控 | 输入 HELP 查看指令表"));
#if LOCAL_BUTTONS
  Serial.println(F("本地按键: 已启用 (D2暂停 D3继续 D10取消 D11示教记录)"));
#else
  Serial.println(F("本地按键: 已禁用 (LOCAL_BUTTONS=0)"));
  Serial.println(F("          暂停/继续/取消请发 PAUSE / RESUME / CANCEL"));
#endif
  Serial.print(F("RAM 余量 = ")); Serial.print(freeRam()); Serial.println(F(" 字节"));
  tpLoadAll();          // 把上次示教的点位从 EEPROM 读回来（没存过就什么都没发生）
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


/* ---- 串口接收 ---- */
/* 逐字符攒成一行；收到换行、或间隔超时，即认为一条指令结束。
 * 这样既能接标准上位机发的 "A\r\n"，也能接遥控板发的、不带换行的 "1"。 */
void pollOne(Stream &s, uint8_t ch, bool inMotion) {
  while (s.available() > 0) {
    char c = s.read();
    lineLastMs[ch] = millis();
    if (c == '\n' || c == '\r') {
      if (lineLen[ch] > 0) {
        lineBuf[ch][lineLen[ch]] = 0; lineLen[ch] = 0;
        deliver(lineBuf[ch], inMotion);
      }
    } else if (lineLen[ch] < LINE_MAX - 1) {
      lineBuf[ch][lineLen[ch]++] = c;
    } else {
      lineLen[ch] = 0;
      Serial.println(F("ERR: 指令过长"));
    }
  }
  if (lineLen[ch] > 0 && millis() - lineLastMs[ch] > LINE_GAP_MS) {
    lineBuf[ch][lineLen[ch]] = 0; lineLen[ch] = 0;
    deliver(lineBuf[ch], inMotion);
  }
}

void deliver(char *cmd, bool inMotion) {
  if (inMotion) dispatchInMotion(cmd);
  else          dispatch(cmd);
}

void pollSerial(bool inMotion) {
  pollOne(Serial, CH_PC, inMotion);
#if REMOTE_SOFT_SERIAL
  pollOne(remoteSerial, CH_RC, inMotion);
#endif
}

/* 运动过程中只受理「暂停 / 继续 / 取消 / 状态查询 / 加减速」，其余指令回 BUSY，
 * 避免在动作执行中途递归触发新动作。
 * ⚠ 低档很慢（档 1 一步 100ms），一条 O/S 要跑好几秒 —— 这期间发的指令都会被
 *   丢掉。所以 BUSY 提示必须把被丢掉的指令原样打出来，否则用户会以为"指令没生效"。 */
void dispatchInMotion(const char *cmd) {
  if      (eq(cmd, "PAUSE"))                    { stPause = true;  printStat("PAUSED"); }
  else if (eq(cmd, "RESUME") || eq(cmd, "GO"))  { stPause = false; printStat("DRAWING"); }
  else if (eq(cmd, "CANCEL"))                   { stAbort = true;  stPause = false; Serial.println(F("CANCEL...")); }
  else if (eq(cmd, "STATUS"))                   { printStat(modeName()); }
  /* H / L 只改速度档、不动任何目标角度，运动途中改是安全的 —— 而且是好事：
   * 档 1 慢得受不了时，直接敲 H 就能给正在走的这个动作加速，不用等它走完。 */
  else if (eq(cmd, "H")) { if (speedLevel < 5) speedLevel++; Serial.print(F("速度档 = ")); Serial.println(speedLevel); }
  else if (eq(cmd, "L")) { if (speedLevel > 1) speedLevel--; Serial.print(F("速度档 = ")); Serial.println(speedLevel); }
  else if (cmd[0]) {
    Serial.print(F("BUSY: 动作还没走完，忽略了「"));
    Serial.print(cmd);
    Serial.println(F("」。等它停稳再发；嫌慢可敲 H 加速，或敲 CANCEL 中断"));
  }
}


/* ---- 指令派发 ---- */
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
      /* 任务二：A/B/C = 跑该物体的【示教点位】（A 0~5 / B 6~11 / C 12~15）。
       * ⚠ 2026-10-10 晚【改过】：原来这里调 pickAndPlace()，走的是桌面坐标逆解
       *   + 一组按旧模型手调的常量。那天运动学改成"小臂绝对角"后落点全偏 ——
       *   实测敲 A 会解出 S≈8、E≈117（大臂压平 + 小臂折到顶），手臂直接杵住。
       *   现在和 RUN / 按键1 / PP 完全同源，全是示教点位。 */
      case 'A': runTeach5(0, "A"); return;         // 任务二
      case 'B': runTeach5(1, "B"); return;
      case 'C': runTeach5(2, "C"); return;
      case 'N': drawFigure('N'); return;           // 任务五：字母 N
      case 'T': drawFigure('T'); return;           // 任务五：三角形（等价于 TRI）
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
  /* RUN：把 A/B/C 三个物体的示教点位【连着跑一遍】，
   * 等价于 PP 0 1 2 3 4 → PP 5 6 7 8 9 → PP 10 11 12 13 14。
   * ⚠ 2026-10-10 晚改过：旧版调的是桌面坐标那套 pickAndPlace()，
   *   那天运动学模型改成"小臂绝对角"之后，那组手调常量的落点会变，已不可信。 */
  else if (eq(p, "RUN"))        runAllThree();
  else if (eq(p, "LINE"))  drawLineDemo();
  else if (eq(p, "TRI"))   drawFigure('T');
  else if (eq(p, "TEACH")) enterTeach();
  else if (eq(p, "SET"))   recordTeachPoint();
  else if (eq(p, "CLR"))   clearTeachPoints();
  else if (eq(p, "DRAW"))  drawByTeach(false);
  else if (eq(p, "CURVE")) drawByTeach(true);
  else if (eq(p, "DEMO"))  { loadDemoPoints(); drawByTeach(false); }
  else if (eq(p, "PAUSE")) { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  else if (eq(p, "RESUME")){ stPause = false; Serial.println(F("STAT:IDLE")); }
  else if (eq(p, "CANCEL")){ stAbort = true;  stPause = false; }
  else if (eq(p, "GO"))    goHome();
  else if (eq(p, "CAL"))   printCalib();
  else if (p[0] == 'J' && p[1] == 'O' && p[2] == 'G') cmdJog(p);
  /* ---- 绝对角度：B30 / S60 / E100 / G20（实现见 task2_teach.ino）----
   * ⚠ 必须带数字。单敲一个字母会先落进上面的「单字符指令」里跑掉：
   *   B = 任务二夹取物体B、S = 合爪（任务一的老指令），跟这里不是一回事。
   * 放在 "GO" 之后判也无妨 —— "GO" 的 p[1]='O' 不是数字，不会误入这里。 */
  else if (strchr("BSEG", p[0]) && isdigit((unsigned char)p[1])) cmdAxisAbs(p);
  /* ---- ⚠ 诊断指令：F<逻辑角> 把夹爪【直接】设到该角度（66~167，大 = 张开）----
   * 为什么要有它：G<0~100> 只在 GRIP_OPEN~GRIP_CLOSE 之间插值，要标定某个具体开口
   *   （硬币 / 物体 C / 笔）就得能直接下角度。
   * 入参是【逻辑角】（大=张开）；命令回显里会同时给出对应的舵机原始角，方便和
   *   range test 工具的读数对照（逻辑 = 220 − 原始角）。
   * ⚠ 正式提交前要么删掉这条，要么在说明文档里注明它是标定用的诊断指令。 */
  else if (p[0] == 'F' && isdigit((unsigned char)p[1])) cmdGripperRaw(p);
  /* ---- ⚠ 诊断：D<毫米> 笔尖高度现场微调（见 penHTrim 处）----
   * 判 p[1] 是不是数字/正负号，所以不会和 DRAW（p[1]='R'）撞。 */
  else if (p[0] == 'D' && (isdigit((unsigned char)p[1]) || p[1] == '-' || p[1] == '+'))
    cmdPenTrim(p);
  /* ---- 任务二现场示教工具（实现见 task2_teach.ino）----
   * 逆运动学没标定之前的替代方案：直接记忆关节角，绕开桌面坐标。
   * 判 p[1] 是否存在，否则光敲一个 "J" 时 strchr 会匹配到字符串末尾的 '\0'。 */
  else if (p[0] == 'J' && p[1] && strchr("BSEG", p[1])) cmdJog1(p);   // JB±n 单轴微调
  /* ⚠ W 必须跟数字：W0~W14=记录，光敲一个 "W" 不认。
   *   清空是 WCLR 而不是 W0 —— 0 号位现在是正经点位了。 */
  else if (p[0] == 'W' && isdigit((unsigned char)p[1])) cmdTeachWrite(p);
  else if (p[0] == 'R' && isdigit((unsigned char)p[1]))  cmdTeachGo(p);    // R<n> 去点位
  else if (eq(p, "WCLR"))                               cmdTeachClear();
  else if (eq(p, "LIST"))                               cmdTeachList();
  /* 随时查 RAM 余量。每加一个号位要吃掉 5 字节，加之前先看一眼还有多少。 */
  else if (eq(p, "RAM"))   { Serial.print(F("RAM 余量 = ")); Serial.print(freeRam()); Serial.println(F(" 字节")); }
  else if (p[0] == 'P' && p[1] == 'P')                  cmdPickPlace(p);  // PP 五个号位
  else if (p[0] == 'P' && isdigit((unsigned char)p[1])) cmdTeachPut(p);   // P<n> B S E G 直接设点位
  else if (eq(p, "JOY"))   printJoystick();
  else if (eq(p, "HELP"))  printHelp();
  else if (eq(p, "STATUS"))printStat(modeName());
  else { Serial.print(F("未知指令: ")); Serial.println(p); Serial.println(F("输入 HELP 查看指令表")); }
}


void printHelp() {
  Serial.println(F("---- meArm 指令表 ----"));
  Serial.println(F("任务一: M=摇杆操控  O/S=开爪/合爪  H/L=加速/减速  x10,y30,z20=多舵机"));
  Serial.println(F("任务二: RUN=依次跑三个物体的示教点位（一条走完 A→B→C）"));
  Serial.println(F("        PP 0 1 2 3 4=按五号位取放一遍  W<n>=记姿态  R<n>=回姿态  LIST=列出"));
  Serial.println(F("        A/B/C 单敲=跑该物体的示教点位（和 RUN 是同一套）"));
  Serial.println(F("        [示教] B30/S60/E100/G20=把某轴直接设到该位置(底座/大臂/小臂/夹爪)"));
  Serial.println(F("               S/E: 数字大 = 抬得高      B: 数字小 = 往左"));
  Serial.println(F("               G 用夹紧程度：0=合死 100=全开（不是 0~180 的舵机角）"));
  Serial.println(F("               JB/JS/JE/JG±5=在该轴基础上再转 5°(只剩一点没对准时用)"));
  Serial.println(F("               W<0~16>=记下当前姿态  R<n>=回到该姿态  LIST=列出  WCLR=清空"));
  Serial.println(F("               P<号> B S E G=直接敲数字设点位(机械臂不动，照 LIST 的数抄)"));
  Serial.println(F("                 └ 点位存在 EEPROM 里，重新烧程序/断电都不会丢"));
  Serial.println(F("               PP 0 1 2 3 4=按示教跑一遍取放(次序即走路次序)"));
  Serial.println(F("                 0物体上方 1物体位 2上升 3同高度平移 4放置位"));
  Serial.println(F("任务三: 1=循环夹取  2=录制  3=回放  4=回中"));
  Serial.println(F("        （这四个就是按键1~4，电路组遥控板还没到位，先用串口敲）"));
  Serial.println(F("          A=号位0~5  B=6~11  C=12~15；按键1 走示教点位，不是逆解）"));
  Serial.println(F("任务五: LINE=直线  N/T/Z/V(或TRI)=图形  TEACH=示教  SET=记录点  CLR=清空"));
  Serial.println(F("        DRAW=折线  CURVE=平滑曲线  DEMO=内置五点  GO=待机位"));
  Serial.println(F("        PAUSE/RESUME/CANCEL=暂停/继续/取消"));
  Serial.println(F("调试  : CAL=打印笔尖坐标  JOG s e=连杆角驱动  JOY=打摇杆原始读数"));
  Serial.println(F("        STATUS=状态  RAM=剩余内存字节数"));
  Serial.println(F("        F<角度>=夹爪直接打到该逻辑角(大=张开)，标定用；回显含原始角"));
  Serial.println(F("        D<毫米>=笔尖高度微调(负=往下压，如 D-2)，画图时现场调 PEN_DROP 用"));
}
