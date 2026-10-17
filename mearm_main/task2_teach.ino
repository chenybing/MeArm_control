// 示教并保存 17 个姿态点位，按记录顺序完成夹取放置
// 点位写入 EEPROM，掉电后仍可读取

const uint8_t TP_N = 17;

uint8_t tpB[TP_N], tpS[TP_N], tpE[TP_N], tpG[TP_N];
bool    tpSet[TP_N] = { false };

#define EE_MAGIC 0xA7
#define EE_BASE  2
const uint8_t EE_STEP = 5;

// EEPROM 布局：地址 0 为魔术字节，每个点位占 5 字节
void tpSaveSlot(uint8_t i) {
  int a = EE_BASE + (int)i * EE_STEP;
  EEPROM.update(a + 0, (uint8_t)tpB[i]);
  EEPROM.update(a + 1, (uint8_t)tpS[i]);
  EEPROM.update(a + 2, (uint8_t)tpE[i]);
  EEPROM.update(a + 3, (uint8_t)tpG[i]);
  EEPROM.update(a + 4, 1);
  EEPROM.update(0, EE_MAGIC);
}

void tpEraseAll() {
  for (uint8_t i = 0; i < TP_N; i++) {
    tpSet[i] = false;
    EEPROM.update(EE_BASE + (int)i * EE_STEP + 4, 0);
  }
  EEPROM.update(0, EE_MAGIC);
}

void tpLoadAll() {
  if (EEPROM.read(0) != EE_MAGIC) return;
  for (uint8_t i = 0; i < TP_N; i++) {
    int a = EE_BASE + (int)i * EE_STEP;
    if (EEPROM.read(a + 4) != 1) { tpSet[i] = false; continue; }

    tpB[i] = constrain((int)EEPROM.read(a + 0), BASE_MIN, BASE_MAX);
    tpS[i] = constrain((int)EEPROM.read(a + 1), SH_MIN,   SH_MAX);
    tpE[i] = constrain((int)EEPROM.read(a + 2), EL_MIN,   EL_MAX);
    tpG[i] = constrain((int)EEPROM.read(a + 3), GR_MIN,   GR_MAX);
    tpSet[i] = true;
  }
  Serial.println(F("读回了上次存的点位，LIST 可以看"));
}

void printPose(const __FlashStringHelper *tag) {
  Serial.print(tag);
  Serial.print(F(" 姿态 B=")); Serial.print(curBase);
  Serial.print(F(" S="));      Serial.print(curShoulder);
  Serial.print(F(" E="));      Serial.print(curElbow);
  Serial.print(F(" G="));      Serial.println(curGripper);
}

void cmdJog1(const char *p) {
  char ax = toupper(p[1]);
  float d = atof(p + 2);
  if (d == 0) {
    Serial.println(F("用法像 JB-3、JS5 这样"));
    return;
  }
  int tB = curBase, tS = curShoulder, tE = curElbow, tG = curGripper;
  switch (ax) {
    case 'B': tB = curBase     + (int)round(d); break;
    case 'S': tS = curShoulder + (int)round(d); break;
    case 'E': tE = curElbow    + (int)round(d); break;
    case 'G': tG = curGripper  + (int)round(d); break;
    default:
      Serial.println(F("轴只认 B S E G"));
      return;
  }

  moveJoints(tB, tS, tE, tG);
  printPose(F("微调"));
}

int gripPctToServo(int pct) {

  return GRIP_CLOSE - (GRIP_CLOSE - GRIP_OPEN) * pct / 100;
}

const int B_USER_MIN = 180 - BASE_MAX;
const int B_USER_MAX = 180 - BASE_MIN;
int baseUserToServo(int v) { return 180 - v; }

void cmdAxisAbs(const char *p) {
  char ax = toupper(p[0]);
  const char *q = p + 1;
  while (*q == ' ' || *q == '\t') q++;
  if (*q == '+') q++;

  if (!(isdigit((unsigned char)*q) || (*q == '-' && isdigit((unsigned char)q[1])))) {
    Serial.println(F("写法像 B30、S60、E100、G20"));
    return;
  }

  int v = (int)round(atof(q));
  int lo, hi;
  switch (ax) {
    case 'B': lo = B_USER_MIN; hi = B_USER_MAX; break;
    case 'S': lo = SH_MIN;   hi = SH_MAX;   break;
    case 'E': lo = EL_MIN;   hi = EL_MAX;   break;
    case 'G': lo = 0;        hi = 100;      break;
    default:
      Serial.println(F("轴只认 B S E G"));
      return;
  }

  int cv = constrain(v, lo, hi);
  if (cv != v) {
    Serial.print(F("超范围了：")); Serial.print(v);
    Serial.print(F("，这个轴是 ")); Serial.print(lo); Serial.print('~'); Serial.print(hi);
    Serial.print(F("，按 ")); Serial.print(cv); Serial.print(F(" 走"));
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

  if (ax == 'B') { Serial.print(F("底座指令值 = ")); Serial.println(cv); }
  if (ax == 'G') { Serial.print(F("夹爪夹紧程度 = ")); Serial.println(cv); }
}

bool tpBadIndex(int n) {
  if (n < 0 || n >= (int)TP_N) {
    Serial.print(F("ERR: 编号只能是 0~")); Serial.println(TP_N - 1);
    return true;
  }
  return false;
}

void cmdTeachWrite(const char *p) {
  int n = atoi(p + 1);
  if (tpBadIndex(n)) return;
  tpB[n] = curBase; tpS[n] = curShoulder; tpE[n] = curElbow; tpG[n] = curGripper;
  tpSet[n] = true;
  tpSaveSlot((uint8_t)n);
  Serial.print(F("已记录 ")); Serial.print(n); Serial.print(F(" 号位:"));
  Serial.print(F(" B=")); Serial.print(tpB[n]);
  Serial.print(F(" S=")); Serial.print(tpS[n]);
  Serial.print(F(" E=")); Serial.print(tpE[n]);
  Serial.print(F(" G=")); Serial.println(tpG[n]);
}

void cmdTeachClear() {
  tpEraseAll();
  Serial.println(F("点位全清了，EEPROM 也清了"));
}

void cmdTeachPut(const char *p) {
  const char *q = p;
  uint8_t done = 0;

  while (*q) {
    while (*q == ' ') q++;
    if (!*q) break;

    if (toupper((unsigned char)*q) != 'P') {
      Serial.println(F("写法像 P0 120 90 90 103"));
      return;
    }

    int n = atoi(q + 1);
    if (tpBadIndex(n)) return;

    q++;
    while (*q && *q != ' ') q++;

    int v[4];
    if (parseNums(q, v, 4) < 4) {
      Serial.print(F("ERR: ")); Serial.print(n);
      Serial.println(F(" 号位后面少了数字，要 B S E G 四个"));
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

    for (uint8_t k = 0; k < 4; k++) {
      while (*q == ' ') q++;
      while (*q && *q != ' ') q++;
    }
  }

  if (done > 1) {
    Serial.print(F("（这一条一共设了 ")); Serial.print(done); Serial.println(F(" 个号位）"));
  }
}

void cmdTeachGo(const char *p) {
  int n = atoi(p + 1);
  if (tpBadIndex(n)) return;
  if (!tpSet[n]) {
    Serial.print(F("第 ")); Serial.print(n); Serial.println(F(" 号位还没记录"));
    return;
  }
  Serial.print(F("去 ")); Serial.print(n); Serial.println(F(" 号位"));
  moveJoints(tpB[n], tpS[n], tpE[n], tpG[n]);
  printPose(F("到位"));
}

void cmdTeachList() {
  Serial.println(F("已记录的点位："));
  bool any = false;
  for (uint8_t i = 0; i < TP_N; i++) {
    if (!tpSet[i]) continue;
    any = true;
    Serial.print(F("  ")); Serial.print(i); Serial.print(F(" 号位: B="));
    Serial.print(tpB[i]); Serial.print(F(" S=")); Serial.print(tpS[i]);
    Serial.print(F(" E=")); Serial.print(tpE[i]);
    Serial.print(F(" G=")); Serial.println(tpG[i]);
  }
  if (!any) Serial.println(F("（空）"));
}

bool moveStaged(int tB, int tS, int tE) {
  if (!moveJoints(curBase, tS, tE, curGripper)) return false;
  return moveJoints(tB, tS, tE, curGripper);
}

uint8_t parseNums(const char *p, int *out, uint8_t maxN) {
  uint8_t n = 0;
  while (*p && n < maxN) {
    while (*p == ' ') p++;
    if (!*p) break;
    out[n++] = atoi(p);
    while (*p && *p != ' ') p++;
  }
  return n;
}

#define ACT_NONE  0
#define ACT_CLOSE 1
#define ACT_OPEN  2

const uint8_t TP_CHAIN_SLOT[16] = {
    0,  1,  2,  3,  4,  5,
    6,  7,  8,  9, 10, 11,
   12, 13, 14, 15
};
const uint8_t TP_CHAIN_ACT[16] = {
   ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_NONE, ACT_OPEN, ACT_NONE,
   ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_NONE, ACT_OPEN, ACT_NONE,
   ACT_NONE, ACT_CLOSE, ACT_NONE, ACT_OPEN
};
const uint8_t TP_CHAIN_N     = 16;

const uint8_t TP_GRP_START[3] = { 0, 6, 12 };
const uint8_t TP_GRP_LEN[3]   = { 6, 6,  4 };

uint8_t tpResolve(uint8_t s) {
  if (tpSet[s]) return s;
  if (s ==  5) return 3;
  if (s == 11) return 9;
  return s;
}

// 依次移动每个号位，按 act 在对应步骤合爪或开爪
bool runWalk(const uint8_t *slot, const uint8_t *act, uint8_t n) {
  for (uint8_t k = 0; k < n; k++) {
    if (!tpSet[slot[k]]) {
      Serial.print(F("ERR: ")); Serial.print(slot[k]);
      Serial.println(F(" 号位还没记，先摆好再 Wn"));
      return false;
    }
  }

  stAbort = false; stPause = false;
  if (!moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN)) return false;

  for (uint8_t k = 0; k < n; k++) {
    uint8_t s = slot[k];
    if (!moveStaged(tpB[s], tpS[s], tpE[s])) return false;
    if (act[k] == ACT_CLOSE) {
      if (!moveJoints(tpB[s], tpS[s], tpE[s], tpG[s])) return false;
      delay(300);
    } else if (act[k] == ACT_OPEN) {
      if (!moveJoints(tpB[s], tpS[s], tpE[s], GRIP_OPEN)) return false;
      delay(300);
    }
  }
  return true;
}

bool runChain(uint8_t from, uint8_t n) {
  if (from >= TP_CHAIN_N || n == 0) return true;
  if ((int)from + (int)n > (int)TP_CHAIN_N) n = (uint8_t)(TP_CHAIN_N - from);

  uint8_t slot[17], act[17];
  for (uint8_t k = 0; k < n; k++) {
    slot[k] = tpResolve(TP_CHAIN_SLOT[from + k]);
    act[k]  = TP_CHAIN_ACT[from + k];
  }

  Serial.print(F("第 ")); Serial.print(from);
  Serial.print(F(" 到 "));    Serial.print((int)from + n - 1);
  Serial.println(F(" 步"));

  if (!runWalk(slot, act, n)) return false;

  if (n >= 2 && TP_CHAIN_ACT[from + n - 1] == ACT_OPEN) {
    uint8_t s = slot[n - 2];
    Serial.print(F("补一下，抬回 ")); Serial.print(s); Serial.println();
    if (!moveStaged(tpB[s], tpS[s], tpE[s])) return false;
  }
  return true;
}

bool runTeach5(uint8_t idx, const char *name) {
  Serial.print(F("物体 ")); Serial.print(name); Serial.println();
  if (!runChain(TP_GRP_START[idx], TP_GRP_LEN[idx])) {
    Serial.print(F("物体 ")); Serial.print(name);
    Serial.println(F(" 的号位没记全，先摆好再 Wn"));
    return false;
  }
  return true;
}

bool runAllThree() {
  Serial.println(F("RUN：A B C 一次走完"));
  if (!runChain(0, TP_CHAIN_N)) {
    Serial.println(F("停在这了"));
    return false;
  }
  Serial.println(F("全走完了"));
  return true;
}

void cmdPickPlace(const char *p) {
  int n[6];
  uint8_t cnt = parseNums(p + 2, n, 6);
  if (cnt < 5) {
    Serial.print(F("用法 PP 0 1 2 3 4，编号 0~"));
    Serial.println(TP_N - 1);
    Serial.println(F("至少 5 个，顺序就是走位顺序：0 物体上方，1 物体位，2 上升，3 平移，4 放置位"));
    Serial.println(F("第 6 个可有可无，是放完抬起；不给就退回 3 的高度"));
    Serial.println(F("A 是 0 1 2 3 4 5，B 是 6 7 8 9 10 11，C 是 12 13 14 15"));
    return;
  }
  uint8_t slot[6], act[6];
  for (uint8_t k = 0; k < cnt; k++) {
    if (n[k] < 0 || n[k] >= (int)TP_N) {
      Serial.print(F("号位只能是 0~")); Serial.println(TP_N - 1);
      return;
    }
    slot[k] = (uint8_t)n[k];

    act[k] = (k == 1) ? ACT_CLOSE : (k == 4) ? ACT_OPEN : ACT_NONE;
  }
  runWalk(slot, act, cnt);
}
