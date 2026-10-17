// 按键扫描、舵机输出、关节插值以及桌面坐标正逆解

void scanButtons() {
#if LOCAL_BUTTONS
  for (uint8_t i = 0; i < BTN_COUNT; i++) {
    bool now = (digitalRead(BTN_PINS[i]) == LOW);
    btnEdge[i] = false;
    if (now && !btnLast[i] && (millis() - btnLock[i] > 180)) {
      btnEdge[i] = true;
      btnLock[i] = millis();
    }
    btnLast[i] = now;
  }
#else

  for (uint8_t i = 0; i < BTN_COUNT; i++) btnEdge[i] = false;
#endif
}

void handleLocalButtons() {
  if (btnEdge[B_CYCLE])  cyclePick();
  if (btnEdge[B_REC])    toggleRecord();
  if (btnEdge[B_PLAY])   playRecord();
  if (btnEdge[B_HOME])   goHome();
  if (btnEdge[B_TEACH])  recordTeachPoint();
}

void handleMotionButtons() {
  if (btnEdge[B_PAUSE])  { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  if (btnEdge[B_GO])     { stPause = false; }
  if (btnEdge[B_CANCEL]) { stAbort = true;  stPause = false; }
}

float trimS = 0, trimE = 0;

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

  svGripper.writeMicroseconds((int)(usOf(GR_RAW_SUM - curGripper) + 0.5f));
}

bool stepGate() {
  scanButtons();
  handleMotionButtons();
  pollSerial(true);
  while (stPause && !stAbort) {
    scanButtons();
    handleMotionButtons();
    pollSerial(true);
    delay(5);
  }
  return stAbort;
}

inline int iabs(int v) { return v < 0 ? -v : v; }
inline int imax4(int a, int b, int c, int d) {
  int m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  if (d > m) m = d;
  return m;
}

bool gDrawSteady = false;

int stepDelayFor(int remain, int total) {
  const int base = stepDelay();

  if (gDrawSteady) return base;
  int ramp = total * 2 / 5;
  if (ramp < 1) ramp = 1;
  int d = remain;
  if (total - d < d) d = total - d;
  if (d >= ramp) return base;
  int x = (ramp - d) * 100 / ramp;
  return base + base * x * x / 10000;
}

// 四轴同时按最大差值逐步逼近目标角，差值大的轴先走
bool moveJoints(int tB, int tS, int tE, int tG) {
  tB = constrain(tB, BASE_MIN, BASE_MAX); tS = constrain(tS, SH_MIN, SH_MAX);
  tE = constrain(tE, EL_MIN,   EL_MAX);   tG = constrain(tG, GR_MIN, GR_MAX);

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

void applyFine() {
  trimS = fineS;
  trimE = fineE;
  writeServos();
}

bool moveToTable(float u, float v, float h) {
  int tB, tS, tE;
  if (!tableToJoints(u, v, h, tB, tS, tE)) return true;
  if (!moveJoints(tB, tS, tE, curGripper)) return false;
  applyFine();
  return true;
}

// 在桌面坐标上等分插值，保证末端走直线
bool moveTipLinear(float u, float v, float h) {
  float cu, cv, ch;
  jointsToTable(curBase, curShoulder, curElbow, cu, cv, ch);

  const int N = 20;
  bool moved = false;
  for (int i = 1; i <= N; i++) {
    float t = (float)i / N;
    int tB, tS, tE;
    if (!tableToJoints(cu + (u - cu) * t, cv + (v - cv) * t,
                       ch + (h - ch) * t, tB, tS, tE)) continue;
    if (!moveJoints(tB, tS, tE, curGripper)) return false;
    applyFine();
    moved = true;
  }

  if (!moved) {
    int tB, tS, tE;
    if (!tableToJoints(u, v, h, tB, tS, tE)) return true;
    if (!moveJoints(tB, tS, tE, curGripper)) return false;
    applyFine();
    return true;
  }
  return true;
}

// 逆解：(u, v, h) -> 底座、大臂、小臂角度，超出范围时就近截断
bool tableToJoints(float u, float v, float h, int &tB, int &tS, int &tE) {
  bool clipped = false;

  float theta = degrees(atan2(u, v));
  int base = BASE_ANG + (int)round(BASE_INVERT ? -theta : theta);
  if (base < BASE_ANG - BASE_SWEEP) { base = BASE_ANG - BASE_SWEEP; clipped = true; }
  if (base > BASE_ANG + BASE_SWEEP) { base = BASE_ANG + BASE_SWEEP; clipped = true; }
  tB = constrain(base, BASE_MIN, BASE_MAX);

  float r0 = sqrt(u * u + v * v);
  float rt = constrain(r0, R_MIN, R_MAX);
  if (rt != r0) clipped = true;

  float r = rt - L3A - L3B;
  float z = (h + PEN_DROP) - H_SHOULDER;

  float d = sqrt(r * r + z * z);
  if (d > ARM_L1 + ARM_L2 || d < fabs(ARM_L1 - ARM_L2)) {

    float limit = (d > ARM_L1 + ARM_L2) ? (ARM_L1 + ARM_L2) * 0.995
                                        : fabs(ARM_L1 - ARM_L2) * 1.005;
    float k = limit / d;
    r *= k; z *= k; d = limit;
    clipped = true;
  }
  float cosE = (d * d - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) / (2.0 * ARM_L1 * ARM_L2);
  cosE = constrain(cosE, -1.0f, 1.0f);
  float e = ELBOW_SIGN * degrees(acos(cosE));
  float s = degrees(atan2(z, r)) - degrees(atan2(ARM_L2 * sin(radians(e)),
                                                ARM_L1 + ARM_L2 * cos(radians(e))));
  float a2 = s + e;

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

  {
    float a2Max = s - ELBOW_OVER_ARM_MAX;
    int teLim = ELBOW_TRIM + (int)floor(ELBOW_INVERT ? -a2Max : a2Max);
    if (ELBOW_INVERT ? (tE < teLim) : (tE > teLim)) {
      tE = constrain(teLim, EL_MIN, EL_MAX);
      fineE = 0.0f;
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

// 正解：关节角 -> 笔尖坐标
void jointsToTable(int b, int s, int e, float &u, float &v, float &h) {
  float cs = SHOULDER_INVERT ? -(s - SHOULDER_TRIM) : (s - SHOULDER_TRIM);
  float ce = ELBOW_INVERT    ? -(e - ELBOW_TRIM)    : (e - ELBOW_TRIM);
  float a2 = ce;

  float r = L3A + ARM_L1 * cos(radians(cs))
                + ARM_L2 * cos(radians(a2)) + L3B;
  float z = ARM_L1 * sin(radians(cs)) + ARM_L2 * sin(radians(a2))
            - PEN_DROP + H_SHOULDER;
  float th = radians(BASE_INVERT ? -(b - BASE_ANG) : (b - BASE_ANG));
  h = z;
  u = r * sin(th);
  v = r * cos(th);
}

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

void printJoystick() {
  Serial.println(F("== 摇杆实时读数：一直打，敲任意字符停（最多 60 秒）=="));
  Serial.println(F("   括号里是【相对中位的偏差】，推杆应该让它跳到 ±几百"));
  for (uint16_t i = 0; i < 300; i++) {
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
    if (Serial.available()) break;
  }
  while (Serial.available()) (void)Serial.read();
  Serial.println(F("== 结束 =="));
}

void cmdJog(const char *p) {
  const char *q = p + 3;
  while (*q == ' ') q++;
  float s = atof(q);
  const char *r = strchr(q, ' ');
  float e = r ? atof(r + 1) : 0;

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

bool eq(const char *a, const char *b) {
  while (*a && *b) {
    if (toupper(*a) != toupper(*b)) return false;
    a++; b++;
  }
  return *a == 0 && *b == 0;
}

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
