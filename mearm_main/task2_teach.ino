/* ============================================================================
 *  task2_teach.ino · 任务二：现场示教工具
 *  单轴微调 / 点位记忆 / 列出点位 / 用记下的点位跑完整取放流程
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 *
 *  ── 为什么要有这个文件 ───────────────────────────────────────────────────
 *  任务二原本是「用桌面坐标 (u,v) 定义取物位/放置位，交给逆解算舵机角」。
 *  那套代码还留着（task2_pick_place.ino），只要标定做完就能用。
 *
 *  但标定卡在硬件上：现在这个夹爪夹不住笔，笔在爪子里会转、会滑，
 *  「笔尖到转轴的距离」这类量测全都不可信，逆运动学的参数定不下来。
 *
 *  于是这里提供第二条路：**绕开运动学，直接记忆关节角**。
 *  用摇杆或 x/y/z 指令把机械臂摆到想要的位置，敲 W<号> 记下来；
 *  六个位置（3 个取物位 + 3 个放置位）都记好之后，敲 PP 就能跑完整的取放。
 *
 *  记的是【四轴舵机角】而不是桌面坐标 —— 所以完全不需要标定，
 *  也不需要知道大臂多长、零点偏多少。代价是换一台机械臂就要重新示教。
 * ============================================================================*/

/* ---- 点位记忆 ---- */
/* 号位编号从 0 开始：0~16 共 17 个。
 * ⚠ 0 号位是正经点位 —— 所以"清空全部"不能再叫 W0 了，改叫 WCLR。
 *   （其实这样更安全：WCLR 得多打三个字母，手滑不出来。） */
const uint8_t TP_N = 17;                    // 0~16 号位
/* ⚠ 用 uint8_t 不用 int：四个轴的角度最大 177（底座上限），一个字节装得下，
 *   17 个号位 × 4 个轴 = 68 字节。Uno 上这点内存很金贵。 */
uint8_t tpB[TP_N], tpS[TP_N], tpE[TP_N], tpG[TP_N];
bool    tpSet[TP_N] = { false };            // 该号位是否已记录

/* ---- ★ 号位分工（2026-10-16 按【用户实录音位】定稿）----------------------------
 *
 *   A = 0 ~ 5      B = 6 ~ 11      C = 12 ~ 15      （16 空着，没用）
 *   A / B 各 6 步（第 ⑥ 步是"放完抬起"），C 只有 4 步（没有 ⑥）。
 *
 *   步序  姿势                        该动哪几根轴        A     B     C
 *   ----------------------------------------------------------------------
 *   ①    物体上方（够高、张着爪）    B/S/E 随便调         0     6    12
 *   ②    降到物体位、夹住            ★ 只动 S/E          1     7    13
 *   ③    抬起来 / 往里收            ★ 只动 S/E          2     8    —
 *   ④    ★ 只转底座（同高度平移）    ★ 只动 B            3     9    14
 *   ⑤    降到放置位、松开            ★ 只动 S/E          4    10    15
 *   ⑥    放完抬起（回到④的高度）    ★ 只动 S/E          5    11    —
 *
 *   ⚠ ④（平移）记的时候【只转底座、别动大臂小臂】，这样 ③→④ 才是纯水平移动。
 *   ⚠ ⑥（放完抬起）摆的时候 S/E 要和④一模一样、只把底座转到下一个物体的方向。
 *     A 的 5 号位没记会退回 3、B 的 11 号位没记会退回 9 —— 两者几何上是同一个姿势
 *     （见 tpResolve）。
 *   ⚠ C 没有⑥：runChain() 末尾会自动补一步"抬回④的高度"（因为 C 的末步是开爪）。
 *
 *   号位和"到位后干什么"的那张表在文件后半段：TP_CHAIN_SLOT / TP_CHAIN_ACT。 */

/* ---- 点位存进 EEPROM：重新烧程序不会丢 -------------------------------------
 * 点位原来只在 RAM 里，一上传新程序就全没了，每个物体都得重摆一遍。
 * 写进 EEPROM 之后，重新烧录、断电重启都还在。
 *
 * 布局：  地址 0        = 魔术字节（从没写过就是 0xFF，靠它判断"到底存过没有"）
 *        地址 2 起，每个号位占 5 字节：B, S, E, G, 是否已记录
 *        17 个号位共 85 字节，EEPROM 有 1024 字节，够用。
 * 用 update() 而不是 write()：值没变就不动，省 EEPROM 的擦写寿命。
 *
 * ⚠ 四轴角度都塞得进一个字节（B 8~172、S 33~172、E 23~122、G 90~167）。
 *   以后要是把行程放宽到 255 以上，这里就得改成两字节。
 *
 * ⚠ 2026-10-11 magic 从 0xA6 改成 0xA7：号位从"三个物体各占 5 个"改成
 *   "A 0~5 / B 6~11 / C 12~16"（每组多了一个"放完抬起"位）。
 *   旧数据里 5~14 号位在新方案下【指的是别的姿势】—— 留着不删的话，
 *   没重记的号位会带着机械臂跑到一个完全不相干的位置上去，桌上还放着物体。
 *   所以整片作废，重新示教。（A 组 0~4 的值没变，可以敲 P0~P4 直接打回来） */
#define EE_MAGIC 0xA7
#define EE_BASE  2
const uint8_t EE_STEP = 5;

void tpSaveSlot(uint8_t i) {
  int a = EE_BASE + (int)i * EE_STEP;
  EEPROM.update(a + 0, (uint8_t)tpB[i]);
  EEPROM.update(a + 1, (uint8_t)tpS[i]);
  EEPROM.update(a + 2, (uint8_t)tpE[i]);
  EEPROM.update(a + 3, (uint8_t)tpG[i]);
  EEPROM.update(a + 4, 1);            // 1 = 这个号位有内容
  EEPROM.update(0, EE_MAGIC);         // 存过至少一次，打上标记
}

/* 只清"已记录"标记，不用逐字节擦 —— 下次覆盖写就行 */
void tpEraseAll() {
  for (uint8_t i = 0; i < TP_N; i++) {
    tpSet[i] = false;
    EEPROM.update(EE_BASE + (int)i * EE_STEP + 4, 0);
  }
  EEPROM.update(0, EE_MAGIC);
}

void tpLoadAll() {
  if (EEPROM.read(0) != EE_MAGIC) return;   // 从没存过：不读，免得把垃圾当点位
  for (uint8_t i = 0; i < TP_N; i++) {
    int a = EE_BASE + (int)i * EE_STEP;
    if (EEPROM.read(a + 4) != 1) { tpSet[i] = false; continue; }
    /* 存进去之后行程上限可能被改过，读回来一律按【当前】限位再钳一遍 */
    tpB[i] = constrain((int)EEPROM.read(a + 0), BASE_MIN, BASE_MAX);
    tpS[i] = constrain((int)EEPROM.read(a + 1), SH_MIN,   SH_MAX);
    tpE[i] = constrain((int)EEPROM.read(a + 2), EL_MIN,   EL_MAX);
    tpG[i] = constrain((int)EEPROM.read(a + 3), GR_MIN,   GR_MAX);
    tpSet[i] = true;
  }
  Serial.println(F("已从 EEPROM 读回上次的示教点位（敲 LIST 看）"));
}

/* 「抬起偏移」（原来叫 liftDS / liftDE）2026-10-05 已删除，配套的 LIFT 指令也没了。
 *
 * 原因：它是拿固定增量硬凑一个"取物位正上方"，但机械臂没有标定过的运动学，
 * 这个凑出来的位置根本不是正上方 —— 大臂一旦顶到上限，整条手臂往下扫，
 * 爪子贴着桌面横着走，把小球推飞。
 *
 * 现在"上方"由用户自己摆好、用 W<号> 记下来，跑 PP 时照着走。
 * 详见 cmdPickPlace()。 */

/* 只打印关节角，不打印笔尖坐标。
 * ⚠ 不用 printCalib()：那个还会打一行「笔尖 u= v= h=」，那是逆解算出来的，
 *   现在的参数没标定过，坐标是错的 —— 示教时看着那行数会把人带偏。 */
/* 形参必须是 __FlashStringHelper*：两个调用点传的是 F("微调") / F("到位")，
 * 而 F() 展开后不是 const char*，装在 const char* 形参上真板上编译不过。 */
void printPose(const __FlashStringHelper *tag) {
  Serial.print(tag);
  Serial.print(F(" 姿态 B=")); Serial.print(curBase);
  Serial.print(F(" S="));      Serial.print(curShoulder);
  Serial.print(F(" E="));      Serial.print(curElbow);
  Serial.print(F(" G="));      Serial.println(curGripper);
}

/* ---- J<轴>±<度数>：只动一个关节，精细找位 ----
 *   JB 底座   JS 大臂   JE 小臂   JG 夹爪
 *   例：JB-3（底座往回转 3°）  JS2（大臂抬 2°）  JG-5（夹爪松 5°）
 * 为什么不用 x/y/z：那个得把三个角都重打一遍，调一格要改一整串，
 * 找位阶段一次只动一个关节顺手得多。 */
void cmdJog1(const char *p) {
  char ax = toupper(p[1]);
  float d = atof(p + 2);
  if (d == 0) {
    Serial.println(F("格式: JB±n / JS±n / JE±n / JG±n   例 JB-3"));
    return;
  }
  int tB = curBase, tS = curShoulder, tE = curElbow, tG = curGripper;
  switch (ax) {
    case 'B': tB = curBase     + (int)round(d); break;
    case 'S': tS = curShoulder + (int)round(d); break;
    case 'E': tE = curElbow    + (int)round(d); break;
    case 'G': tG = curGripper  + (int)round(d); break;
    default:
      Serial.println(F("轴只能是 B(底座) / S(大臂) / E(小臂) / G(夹爪)"));
      return;
  }
  /* 越界的部分由 moveJoints() 自己钳进实测行程，不会顶到机械止点 */
  moveJoints(tB, tS, tE, tG);
  printPose(F("微调"));
}

/* ---- B / S / E / G <数值>：把某个关节直接设到指定位置（找位就用这个）----
 *   B30   底座往左          —— 数字小 = 往左，数字大 = 往右
 *   S30   大臂压向桌面      —— ⚠ 数字【大】= 压得低，数字【小】= 抬得高
 *                              （2026-10-10 实测修正：S=90 时肘轴 14.0cm，
 *                                S=170 时 6.1cm。原来这里写反了，
 *                                和 mearm_main.ino 的 SHOULDER_INVERT 打架。）
 *   E30   小臂贴向桌面      —— 数字小 = 贴得低，数字大 = 抬得高
 *   G0    夹爪完全合死      —— 0 = 合死，100 = 完全张开
 *
 * 方向和单位 2026-10-05 上机确认过（⚠ 其中 S 的方向是错的，10-10 已更正）。
 * S/E 的数字就是舵机角本身、原样下传（敲 S30，回显里就是 S=30）。
 * B 和 G 做了换算，因为这两根舵机本身的方向/单位跟直觉反着来：
 *   B：舵机实测「30 = 往右、150 = 往左」，用户要的是「30 = 往左」，镜像一下。
 *   G：舵机原始角是「小 = 开、大 = 闭」，数字越大反而越紧，换成 0~100 张开程度。
 * B 这条换算过的轴，动作走完会额外打一行「指令值 = …」—— 因为 printPose 里
 * 那一栏仍是舵机原始角，跟刚敲的数字对不上，不打这行用户会以为指令没生效。
 * G 从 2026-10-09 起不用补偿了：夹爪在出口做镜像，curGripper 本身已经是逻辑角，
 * printPose 打出的 G= 就是刚敲/刚设的那个逻辑角（大 = 张开）。
 *
 * ── 和 J 系列的分工 ────────────────────────────────────────────────────
 *   B30     「设成 30」      —— 大范围找位用这个，摆动幅度心里有数
 *   JB-5    「往回转 5 度」   —— 只剩一点点没对准时用这个
 * 四条轴字母一一对应：B=底座  S=大臂  E=小臂  G=夹爪，只是 J 系列多带一个 J。
 * 每敲一条只有那一个关节动，另外三个原地不动。
 * ⚠ J 系列里只有 B 是舵机原始角；S/E 本来就是舵机角；G 是【逻辑角】
 *   （大 = 张开，和上面 G 那条命令同一口径 —— 夹爪的镜像统一在出口做）。 */
int gripPctToServo(int pct) {                 // 张开程度 0~100 → 夹爪逻辑角（大=张开）
  /* 公式和 2026-10-05 那版一字未改：换约定时 GRIP_OPEN / GRIP_CLOSE 的大小
   * 关系跟着翻了，所以结果自然仍是「100 = 全开、0 = 合死」。 */
  return GRIP_CLOSE - (GRIP_CLOSE - GRIP_OPEN) * pct / 100;
}

/* 底座左右镜像：以 B90（正前方）为轴翻过来，即 180 - v。
 * ⚠ 不要图省事写成 (BASE_MIN + BASE_MAX) - v（= 185 - v）：那样范围是好看了，
 *   但"正前方"会从中位 90 挪到 95，以后凡是拿 90 当正前方的代码都对不上。
 * 镜像后可用范围跟着翻：舵机 8~177 对应指令值 172~3。 */
const int B_USER_MIN = 180 - BASE_MAX;        // 3
const int B_USER_MAX = 180 - BASE_MIN;        // 172
int baseUserToServo(int v) { return 180 - v; }

void cmdAxisAbs(const char *p) {
  char ax = toupper(p[0]);
  const char *q = p + 1;                       // 字母后面就是数值
  while (*q == ' ' || *q == '\t') q++;          // 容忍 "B 30" 这种带空格的写法
  if (*q == '+') q++;
  /* 光敲一个字母后面没数时，q 指向 '\0'，atof 会给 0 —— 那就成了"底座转到 0°"，
   * 手臂会一头栽过去。必须拦住。（何况光敲字母本来就走不到这里，另有老指令。） */
  if (!(isdigit((unsigned char)*q) || (*q == '-' && isdigit((unsigned char)q[1])))) {
    Serial.println(F("格式: B30 / S60 / E100 / G20   (字母后面要跟数字)"));
    return;
  }

  int v = (int)round(atof(q));
  int lo, hi;
  switch (ax) {
    case 'B': lo = B_USER_MIN; hi = B_USER_MAX; break;   // 已镜像，不是舵机角
    case 'S': lo = SH_MIN;   hi = SH_MAX;   break;
    case 'E': lo = EL_MIN;   hi = EL_MAX;   break;
    case 'G': lo = 0;        hi = 100;      break;   // 是夹紧程度，不是舵机角
    default:
      Serial.println(F("轴只能是 B(底座) / S(大臂) / E(小臂) / G(夹爪)"));
      return;
  }

  /* 超出范围要说出来。moveJoints() 会默默钳进限位，用户打 S200 却看到停在 147
   * 会以为是自己看错了 —— 其实是被行程挡住了。 */
  int cv = constrain(v, lo, hi);
  if (cv != v) {
    Serial.print(F("注意: ")); Serial.print(v);
    Serial.print(F(" 超出该轴范围 ")); Serial.print(lo); Serial.print('~'); Serial.print(hi);
    Serial.print(F("，实际按 ")); Serial.print(cv); Serial.print(F(" 走"));
    if (ax == 'G') { Serial.print(F("（舵机角 ")); Serial.print(gripPctToServo(cv)); Serial.print(F("）")); }
    Serial.println();
  }

  switch (ax) {
    case 'B': moveJoints(baseUserToServo(cv), curShoulder, curElbow, curGripper); break;
    case 'S': moveJoints(curBase, cv,          curElbow, curGripper);        break;
    case 'E': moveJoints(curBase, curShoulder, cv,       curGripper);        break;
    case 'G': moveJoints(curBase, curShoulder, curElbow, gripPctToServo(cv)); break;
  }
  printPose(F("设定"));
  /* B 和 G 是换算过的轴，printPose 打的是舵机原始角、跟刚敲的数字对不上，
   * 各补一行用户视角的值，免得以为指令没生效。S/E 原样下传，不用补。 */
  if (ax == 'B') { Serial.print(F("底座指令值 = ")); Serial.println(cv); }
  if (ax == 'G') { Serial.print(F("夹爪夹紧程度 = ")); Serial.println(cv); }
}

/* 号位编号合法性检查：0 <= n < TP_N。
 * 老版本用 1~9、而且 W0 被"清空全部"占着；0 号位现在要当正经点位用，
 * 清空挪到了 WCLR（见 cmdTeachClear）。 */
bool tpBadIndex(int n) {
  if (n < 0 || n >= (int)TP_N) {
    Serial.print(F("ERR: 编号只能是 0~")); Serial.println(TP_N - 1);
    return true;
  }
  return false;
}

/* ---- W<n>：把当前四轴角度记到第 n 号位（n = 0~16）---- */
void cmdTeachWrite(const char *p) {
  int n = atoi(p + 1);
  if (tpBadIndex(n)) return;
  tpB[n] = curBase; tpS[n] = curShoulder; tpE[n] = curElbow; tpG[n] = curGripper;
  tpSet[n] = true;
  tpSaveSlot((uint8_t)n);              // 立刻写进 EEPROM，重烧程序也不丢
  Serial.print(F("已记录 ")); Serial.print(n); Serial.print(F(" 号位:"));
  Serial.print(F(" B=")); Serial.print(tpB[n]);
  Serial.print(F(" S=")); Serial.print(tpS[n]);
  Serial.print(F(" E=")); Serial.print(tpE[n]);
  Serial.print(F(" G=")); Serial.println(tpG[n]);
}

/* ---- WCLR：清空全部点位 ----
 * 为什么不再叫 W0：0 号位现在是正经点位了。多打三个字母也更安全 ——
 * 手滑打不出 WCLR，老版本敲个 W0 就把全部点位清光了。 */
void cmdTeachClear() {
  tpEraseAll();                        // 连 EEPROM 里的标记一起清
  Serial.println(F("已清空全部点位（EEPROM 也清了）"));
}

/* ---- P<号> <B> <S> <E> <G>：直接敲数字设号位（机械臂不动）----
 *   例：P0 120 90 90 103
 * 用途：不想再摆一遍姿势时，把之前 LIST 抄下来的数直接敲回去。
 * 数就是 LIST 打印的那种舵机原始角，原样对得上。超过行程的会被钳住。
 *
 * ── 支持一次贴多行 ──────────────────────────────────────────────────────
 * 串口输入框是【单行】的：贴多行进去，换行会被合并成空格，整段长成
 *   P0 120 90 90 103  P1 120 150 20 155  P2 120 90 40 155 ...
 * 所以这里不是"一条 P 指令设一个号位"，而是【见到一个 P<数字> 就开一组】，
 * 一路读下去把每一组都设好。五行一次贴完也行，只要塞得进 LINE_MAX 个字符。
 * 老版本按"一条指令 = 一个号位"解析，贴多行只会设上第一组、剩下的悄悄丢掉。 */
void cmdTeachPut(const char *p) {
  const char *q = p;                 // 指向 "P0 ..." 里的那个 P
  uint8_t done = 0;

  while (*q) {
    while (*q == ' ') q++;
    if (!*q) break;

    if (toupper((unsigned char)*q) != 'P') {
      Serial.println(F("ERR: 格式是 P<号> <B> <S> <E> <G>，例如 P0 120 90 90 103"));
      return;
    }

    int n = atoi(q + 1);             // "P" 后面跟的号位
    if (tpBadIndex(n)) return;

    q++;                             // 跳过 P
    while (*q && *q != ' ') q++;     // 再跳过号位那串数字

    int v[4];
    if (parseNums(q, v, 4) < 4) {
      Serial.print(F("ERR: ")); Serial.print(n);
      Serial.println(F(" 号位后面要跟 B S E G 四个数"));
      return;
    }

    tpB[n] = constrain(v[0], BASE_MIN, BASE_MAX);
    tpS[n] = constrain(v[1], SH_MIN,   SH_MAX);
    tpE[n] = constrain(v[2], EL_MIN,   EL_MAX);
    tpG[n] = constrain(v[3], GR_MIN,   GR_MAX);
    tpSet[n] = true;
    tpSaveSlot((uint8_t)n);

    Serial.print(F("已设 ")); Serial.print(n); Serial.print(F(" 号位:"));
    Serial.print(F(" B=")); Serial.print(tpB[n]);
    Serial.print(F(" S=")); Serial.print(tpS[n]);
    Serial.print(F(" E=")); Serial.print(tpE[n]);
    Serial.print(F(" G=")); Serial.println(tpG[n]);
    done++;

    /* parseNums 是值传递、不会推进指针，得自己把这 4 个词跳过去，
     * q 才会停在下一组的 P 上（或者字符串末尾）。 */
    for (uint8_t k = 0; k < 4; k++) {
      while (*q == ' ') q++;
      while (*q && *q != ' ') q++;
    }
  }

  if (done > 1) {
    Serial.print(F("（这一条一共设了 ")); Serial.print(done); Serial.println(F(" 个号位）"));
  }
}

/* ---- R<n>：走到第 n 号位 ---- */
void cmdTeachGo(const char *p) {
  int n = atoi(p + 1);
  if (tpBadIndex(n)) return;
  if (!tpSet[n]) {
    Serial.print(F("第 ")); Serial.print(n); Serial.println(F(" 号位还没记录"));
    return;
  }
  Serial.print(F("→ ")); Serial.print(n); Serial.println(F(" 号位"));
  moveJoints(tpB[n], tpS[n], tpE[n], tpG[n]);
  printPose(F("到位"));
}

/* ---- LIST：把所有已记录的点位打出来（方便一次抄给上位机）---- */
void cmdTeachList() {
  Serial.println(F("---- 已记录点位 ----"));
  bool any = false;
  for (uint8_t i = 0; i < TP_N; i++) {
    if (!tpSet[i]) continue;
    any = true;
    Serial.print(F("  ")); Serial.print(i); Serial.print(F(" 号位: B="));
    Serial.print(tpB[i]); Serial.print(F(" S=")); Serial.print(tpS[i]);
    Serial.print(F(" E=")); Serial.print(tpE[i]);
    Serial.print(F(" G=")); Serial.println(tpG[i]);
  }
  if (!any) Serial.println(F("  （还没有记录任何点位）"));
  Serial.println(F("抬起偏移: 已取消（改成自己记“正上方”点位，见 HELP 里的 PP）"));
  Serial.println(F("--------------------"));
}

/* ---- 分段移动：升降/伸缩 与 转底座 分开走 ----------------------------------
 * moveJoints() 是让四根轴【同时】朝目标走的，爪子于是斜着扫过去 ——
 * 一边转一边降，半路就从侧面扎到桌面上的物体上，把小物块碰飞。
 *
 * 底座是绕竖直轴转的：单独转它时爪子走一个水平圆周，高度一丝不变。
 * 这是唯一一根【不需要运动学就能保证"纯水平"】的轴，所以把它单独抽出来走，
 * 横向移动那一段就真的是水平的了。
 *
 * 顺序是【先升降 → 后横移】，不是反过来：
 * 目标多半是"某点正上方"这种高位。先在原地把手臂升上去、再横着兜过去，
 * 横移全程都是高的；反过来先横移的话，抬起之前那一段是贴着桌面在低位扫，
 * 照样会刮到东西。
 *
 * 整套取放于是变成「升起来 → 横移到目标上方 → 降下去」，
 * 每一步只动一类轴。夹爪不动（保持 curGripper），由调用者单独控制。 */
bool moveStaged(int tB, int tS, int tE) {
  if (!moveJoints(curBase, tS, tE, curGripper)) return false;   // ① 原地升降 / 伸缩
  return moveJoints(tB, tS, tE, curGripper);                    // ② 高度不变地转过去
}

/* 从字符串里依次读出最多 maxN 个整数（空格分隔），返回真正读出几个。
 * 不能用 strchr 找空格再 atoi 的老写法：那个对 "PP 1 2 3 4" 只能读前两个。 */
uint8_t parseNums(const char *p, int *out, uint8_t maxN) {
  uint8_t n = 0;
  while (*p && n < maxN) {
    while (*p == ' ') p++;
    if (!*p) break;
    out[n++] = atoi(p);
    while (*p && *p != ' ') p++;        // 跳过这个数字，接着找下一个
  }
  return n;
}


/* ---- PP <0> <1> <2> <3> <4>：跑一遍完整的取放 ----
 *
 * ── 为什么要五个号位 ────────────────────────────────────────────────────
 * 机械臂只有关节角，没有标定过的运动学模型，所以代码【算不出】"正上方"在
 * 哪 —— 动一根轴，爪子走的一定是圆弧。老版本按固定偏移量硬凑一个"上方"，
 * 大臂一顶到上限那个"上方"就完全不在上方了：整条手臂往下扫，爪子贴着桌面
 * 横着走，把小球推飞。
 *
 * 既然算不出来，就【让你摆出来】：把每一步该在的姿势记成一个号位，
 * 跑的时候照着走。你摆的时候是直的，走出来的就接近直的。
 *
 * ── 六个号位的含义（次序就是机械臂走路的次序）────────────────────────────
 *   0  物体上方   下降前的起始位：悬在物体正上方、够高，能安全横过桌面
 *   1  物体位     爪子夹住物体
 *   2  物体位上升 夹住后升起来（机械臂头一回上到"运输高度"）
 *   3  同高度平移 跟 2 一样高，只是底座转到放置位那个方向
 *   4  放置位     物体该放下的位置
 *   5  放完抬起   放下后把爪子抬回运输高度（和 3 是同一个姿势）
 *
 * ⚠ 3 号位记的时候【只转底座、别动大臂小臂】，这样 2→3 才是纯水平移动。
 *
 * ── ★ 整个任务二 =【一条 16 步的链】──────────────────────────────────────
 *   A 的 6 步 → B 的 6 步 → C 的 4 步（C 是最后一个，不需要"放完抬起"）。
 *   `RUN` 就是从第 0 步一路走到第 15 步：**中间不停、不回中、不重新张爪**，
 *   是一整个连贯的过程。路径和「到这一步要夹还是要放」写在同一张表里，
 *   走路和夹放就不会走岔。
 *
 *   每一步：① 用 moveStaged() 走过去（升降/伸缩和转底座分开走，
 *             横移那一段高度才不会变）
 *           ② 到位后按表里写的做动作：合爪 / 开爪 / 什么都不做
 *   只有开局的那一次张爪是统一的。 */
#define ACT_NONE  0        // 到位就完了
#define ACT_CLOSE 1        // 到位后合爪（夹住物体）
#define ACT_OPEN  2        // 到位后开爪（放下物体）

/* 每一步走哪个号位 + 到位后做什么 —— 这就是任务二的全部流程
 *
 * ★ 2026-10-16 按【用户实录音位】定稿：
 *   A = 0~5（6 步，末步"放完抬起"）  B = 6~11（6 步，同构）  C = 12~15（4 步，没有收尾）
 *   ⚠ 16 号位没用到。
 *   ⚠ C 的收尾由 runChain() 末尾那段【自动补一步】完成
 *     （走 C 的④"平移"位 = 同一个高度），所以不需要再占一个号位。 */
const uint8_t TP_CHAIN_SLOT[16] = {
  /* A */  0,  1,  2,  3,  4,  5,
  /* B */  6,  7,  8,  9, 10, 11,
  /* C */ 12, 13, 14, 15
};
const uint8_t TP_CHAIN_ACT[16] = {
  /* A */ ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_NONE, ACT_OPEN, ACT_NONE,
  /* B */ ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_NONE, ACT_OPEN, ACT_NONE,
  /* C */ ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_OPEN
};
const uint8_t TP_CHAIN_N     = 16;
/* 三组各自的起始步和步数（A / B / C）—— A/B 各 6 步，C 4 步 */
const uint8_t TP_GRP_START[3] = { 0, 6, 12 };
const uint8_t TP_GRP_LEN[3]   = { 6, 6,  4 };

/* 「放完抬起」位（A 的 5 / B 的 11）没记就退回同组的「平移」位 ——
 * 两者几何上是同一个姿势（同一个底座方向、同样的运输高度），
 * 少记一个点也少一个出错机会。 */
uint8_t tpResolve(uint8_t s) {
  if (tpSet[s]) return s;
  if (s ==  5) return 3;
  if (s == 11) return 9;
  return s;
}

/* ---- runWalk：按给定的号位 + 到位动作，从头走到尾 ----
 * ⚠ 用两个平行数组而不是结构体：Arduino 会把函数原型提到文件最前面，
 *   形参里要是出现自定义类型，原型会插在类型定义之前、直接编译不过。
 * 返回 true = 完整跑完；false = 号位没记全，或者中途被 CANCEL 中断。 */
bool runWalk(const uint8_t *slot, const uint8_t *act, uint8_t n) {
  for (uint8_t k = 0; k < n; k++) {
    if (!tpSet[slot[k]]) {
      Serial.print(F("ERR: ")); Serial.print(slot[k]);
      Serial.println(F(" 号位还没记录。先摆好姿势敲 W<号>"));
      return false;
    }
  }

  stAbort = false; stPause = false;
  if (!moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN)) return false;   // 开局张爪

  for (uint8_t k = 0; k < n; k++) {
    uint8_t s = slot[k];
    if (!moveStaged(tpB[s], tpS[s], tpE[s])) return false;
    if (act[k] == ACT_CLOSE) {
      if (!moveJoints(tpB[s], tpS[s], tpE[s], tpG[s])) return false;          // 合爪
      delay(300);                                                             // 等夹稳
    } else if (act[k] == ACT_OPEN) {
      if (!moveJoints(tpB[s], tpS[s], tpE[s], GRIP_OPEN)) return false;       // 开爪
      delay(300);                                                             // 等落稳
    }
  }
  return true;
}

/* ---- runChain：从第 from 步开始，顺着 TP_CHAIN 连着走 n 步 ----
 * RUN 是 runChain(0, 17)，单字母 A/B/C 是各自那一段，任务三按键1 也是。 */
bool runChain(uint8_t from, uint8_t n) {
  if (from >= TP_CHAIN_N || n == 0) return true;
  if ((int)from + (int)n > (int)TP_CHAIN_N) n = (uint8_t)(TP_CHAIN_N - from);

  uint8_t slot[17], act[17];
  for (uint8_t k = 0; k < n; k++) {
    slot[k] = tpResolve(TP_CHAIN_SLOT[from + k]);
    act[k]  = TP_CHAIN_ACT[from + k];
  }

  Serial.print(F("== 第 ")); Serial.print(from);
  Serial.print(F(" ~ "));    Serial.print((int)from + n - 1);
  Serial.println(F(" 步 =="));

  if (!runWalk(slot, act, n)) return false;

  /* 末步是"开爪"的组，放完东西会停在放置位 ——
   * 这里补一个"抬回上一步"（同组的④"平移"位，同一个高度），
   * 别让爪子压在刚放好的东西上。★ 2026-10-16 之后 A/B/C 三组都靠它收尾。 */
  if (n >= 2 && TP_CHAIN_ACT[from + n - 1] == ACT_OPEN) {
    uint8_t s = slot[n - 2];
    Serial.print(F("（补一步：抬回 ")); Serial.print(s); Serial.println(F(" 号位）"));
    if (!moveStaged(tpB[s], tpS[s], tpE[s])) return false;
  }
  return true;
}

/* ---- 跑某一个物体（A / B / C）----
 * 单字母 A/B/C、任务三按键1 都走它。某一段没记全就返回 false 停下来。 */
bool runTeach5(uint8_t idx, const char *name) {
  Serial.print(F("== 物体 ")); Serial.print(name); Serial.println(F(" =="));
  if (!runChain(TP_GRP_START[idx], TP_GRP_LEN[idx])) {
    Serial.print(F("（物体 ")); Serial.print(name);
    Serial.println(F(" 的号位还没记全，先摆好姿势敲 W<号>）"));
    return false;
  }
  return true;
}

/* ---- RUN：★ 从 0 号位一路走到最后一个（16 步不停、不回中）----
 *
 * ⚠ 2026-10-10 晚【整条改过】：旧版这里（以及单字母 A/B/C）调的是 pickAndPlace()，
 *   也就是 task2_pick_place.ino 里那套【桌面坐标逆解】，靠 PICK_U/V、PLACE_U/V 一组手调常量。
 *   那天晚上运动学模型改成了"小臂是绝对角"，那组常量的落点全偏了 ——
 *   实测敲 A：目标 (0,8,4) 被 R_MIN=10 钳住，解出 S≈8（大臂压到底）、E≈117（小臂折到顶），
 *   手臂直接折起来杵住。所以现在整条改成和 PP / 按键1 共用同一套【示教点位】。
 *   （pickAndPlace() 那段代码留着当"备用逆解方案"的说明，但界面上已经没有入口了。）
 *
 * 某一步的号位没记就【停下来不往下跑】（返回 false），免得机械臂按空点位乱走。
 * ⚠ 和按键1 用的 cycleIdx 循环指针【互不影响】：RUN 永远从 A 开始。 */
bool runAllThree() {
  Serial.println(F("== RUN：A → B → C 一路走完 =="));
  if (!runChain(0, TP_CHAIN_N)) {
    Serial.println(F("（RUN 就停在这里）"));
    return false;
  }
  Serial.println(F("== RUN 完成：A / B / C 全都走完了 =="));
  return true;
}

void cmdPickPlace(const char *p) {
  int n[6];
  uint8_t cnt = parseNums(p + 2, n, 6);       // p+2 跳过 "PP"
  if (cnt < 5) {
    Serial.print(F("用法: PP <0> <1> <2> <3> <4> [<5>]   编号 0~"));
    Serial.println(TP_N - 1);
    Serial.println(F("  至少五个，次序就是机械臂走路的次序："));
    Serial.println(F("    0 物体上方 → 1 物体位 → 2 上升 → 3 同高度平移 → 4 放置位"));
    Serial.println(F("  第六个（可省）是「放完抬起」：给了就走它，不给就退回 3 的高度。"));
    Serial.println(F("  A = PP 0 1 2 3 4 5   B = PP 6 7 8 9 10 11   C = PP 12 13 14 15"));
    return;
  }
  uint8_t slot[6], act[6];
  for (uint8_t k = 0; k < cnt; k++) {
    if (n[k] < 0 || n[k] >= (int)TP_N) {
      Serial.print(F("ERR: 号位只能是 0~")); Serial.println(TP_N - 1);
      return;
    }
    slot[k] = (uint8_t)n[k];                // 号位号就是数组下标，不用再 -1
    /* 到位动作按位置定，和 TP_CHAIN 的约定一致：第 2 个合爪、第 5 个开爪 */
    act[k] = (k == 1) ? ACT_CLOSE : (k == 4) ? ACT_OPEN : ACT_NONE;
  }
  runWalk(slot, act, cnt);
}
