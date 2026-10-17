// 任务五：直线、基础图形、五点示教折线与平滑曲线

const uint8_t TEACH_N = 5;
float   teachU[TEACH_N], teachV[TEACH_N];
uint8_t teachCount = 0;
float   tgU = 0, tgV = 0;

void clearTeachPoints() {
  teachCount = 0;
  Serial.println(F("示教点已清空"));
}

bool penTo(float u, float v, float h) { return moveToTable(u, v, h + penHTrim); }

void cmdPenTrim(const char *p) {
  penHTrim = atof(p + 1) / 10.0;
  Serial.print(F("[诊断] 笔尖高度偏移 = "));
  Serial.print(penHTrim * 10.0, 1);
  Serial.println(F(" mm  （负 = 往下压）"));
}

bool penTravel(float u, float v, float h) { return moveTipLinear(u, v, h + penHTrim); }
bool penUp(float u, float v)   { return penTravel(u, v, PEN_LIFT); }
bool penDown(float u, float v) { return penTravel(u, v, 0.0); }

bool goStandby() { return penTravel(HOME_U, HOME_V, HOME_H); }

void beginDrawTask(const char *title) {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.print(F("== ")); Serial.print(title); Serial.println(F(" =="));
  printStat("DRAWING");
  moveJoints(curBase, curShoulder, curElbow, (int)PEN_GRIP);
  gDrawSteady = true;
}

void endDrawTask(const char *title, bool cancelled) {
  if (cancelled) {
    Serial.println(F(">> 已取消当前绘图，返回待机位"));
    stAbort = false;
  }
  goStandby();
  Serial.print(F("== ")); Serial.print(title);
  Serial.println(cancelled ? F(" 已取消 ==") : F(" 完成 =="));
  printStat("IDLE");
  gDrawSteady = false;
}

const float LINE_U1 = -6.0, LINE_V1 = 13.0;
const float LINE_U2 =  6.0, LINE_V2 = 13.0;

void drawLineDemo() {
  beginDrawTask("子任务1 直线");
  bool cc = false;
  if (!penUp(LINE_U1, LINE_V1)) cc = true;
  if (!cc && !penDown(LINE_U1, LINE_V1)) cc = true;
  if (!cc) {
    const int N = 40;
    for (int i = 1; i <= N && !cc; i++) {
      float t = (float)i / N;
      if (!penTo(LINE_U1 + (LINE_U2 - LINE_U1) * t,
                 LINE_V1 + (LINE_V2 - LINE_V1) * t, 0.0)) cc = true;
    }
  }
  endDrawTask("直线", cc);
}

const float FIG_CU = 0.0, FIG_CV = 14.0, FIG_S = 4.0;

void drawFigure(char kind) {
  const char *name = (kind == 'N') ? "字母 N" : (kind == 'T') ? "三角形"
                   : (kind == 'Z') ? "字母 Z" : "字母 V";
  beginDrawTask(name);

  const float c = FIG_CU, s = FIG_S / 2.0;
  const float vB = FIG_CV - s, vT = FIG_CV + s;
  const float uL = c - s, uR = c + s;
  float px[6], py[6]; int n = 0;

  if (kind == 'N') {
    px[0] = uL; py[0] = vB; px[1] = uL; py[1] = vT;
    px[2] = uR; py[2] = vB; px[3] = uR; py[3] = vT; n = 4;
  } else if (kind == 'T') {
    float hgt = FIG_S * 0.866;
    px[0] = c;  py[0] = vB + hgt; px[1] = uL; py[1] = vB;
    px[2] = uR; py[2] = vB;       px[3] = c;  py[3] = vB + hgt; n = 4;
  } else if (kind == 'Z') {
    px[0] = uL; py[0] = vT; px[1] = uR; py[1] = vT;
    px[2] = uL; py[2] = vB; px[3] = uR; py[3] = vB; n = 4;
  } else {
    px[0] = uL; py[0] = vT; px[1] = c;  py[1] = vB;
    px[2] = uR; py[2] = vT; n = 3;
  }

  bool cc = false;
  if (!penUp(px[0], py[0])) cc = true;
  if (!cc && !penDown(px[0], py[0])) cc = true;
  for (int i = 1; i < n && !cc; i++) {
    const int M = 25;
    for (int k = 1; k <= M && !cc; k++) {
      float t = (float)k / M;
      if (!penTo(px[i - 1] + (px[i] - px[i - 1]) * t,
                 py[i - 1] + (py[i] - py[i - 1]) * t, 0.0)) cc = true;
    }
  }
  endDrawTask(name, cc);
}

void enterTeach() {
  if (mode == MODE_TEACH) { mode = MODE_IDLE; Serial.println(F("退出示教")); printStat("IDLE"); return; }
  stAbort = false; stPause = false;
  mode = MODE_TEACH;
  moveJoints(curBase, curShoulder, curElbow, (int)PEN_GRIP);

  float h;
  jointsToTable(curBase, curShoulder, curElbow, tgU, tgV, h);
  Serial.println(F("== 示教模式：摇杆1 移动笔尖(X=左右 Y=前后)，按 D11 或发送 SET 记录 =="));
  Serial.print(F("   当前笔尖 u=")); Serial.print(tgU, 1);
  Serial.print(F(" v="));           Serial.println(tgV, 1);
  printStat("TEACH");
}

void teachStep() {

  int lr = analogRead(PIN_J1X) - JOY_CENTER[0];
  int ff = analogRead(PIN_J1Y) - JOY_CENTER[1];
  if (JOY_INV_J1X) lr = -lr;
  if (JOY_INV_J1Y) ff = -ff;
  int du = -lr;
  int dv =  ff;
  bool moved = false;
  if (abs(du) > JOY_DEADZONE) { tgU += (du / 512.0f) * TEACH_SPEED; moved = true; }
  if (abs(dv) > JOY_DEADZONE) { tgV += (dv / 512.0f) * TEACH_SPEED; moved = true; }

  if (moved) {

    float rr = sqrt(tgU * tgU + tgV * tgV);
    if (rr > R_MAX) { tgU *= R_MAX / rr; tgV *= R_MAX / rr; }
    if (rr < R_MIN && rr > 0.01f) { tgU *= R_MIN / rr; tgV *= R_MIN / rr; }

    int tB, tS, tE;

    tableToJoints(tgU, tgV, TEACH_H + penHTrim, tB, tS, tE);
    curBase = tB; curShoulder = tS; curElbow = tE;
    writeServos();
  }

  if (btnEdge[B_TEACH])  recordTeachPoint();
  if (btnEdge[B_HOME])   { mode = MODE_IDLE; goHome(); }
  if (btnEdge[B_CANCEL]) { mode = MODE_IDLE; Serial.println(F("退出示教")); printStat("IDLE"); }
  delay(10);
}

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

// Catmull-Rom 样条，曲线经过 p1、p2 两个控制点
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

void drawByTeach(bool smooth) {
  if (teachCount < 2) { Serial.println(F("ERR: 示教点不足 2 个，请先 TEACH + SET")); return; }
  beginDrawTask(smooth ? "子任务5 五点示教 + 平滑曲线" : "子任务4 五点示教 + 折线");

  bool cc = false;
  if (!goStandby()) cc = true;
  if (!cc && !penUp(teachU[0], teachV[0])) cc = true;
  if (!cc && !penDown(teachU[0], teachV[0])) cc = true;

  int segN = teachCount - 1;
  for (int i = 0; i < segN && !cc; i++) {
    const int M = smooth ? 30 : 25;
    for (int k = 1; k <= M && !cc; k++) {
      float t = (float)k / M, u, v;
      if (smooth) {
        // 端点复制到两侧，保证首尾两段也有完整的四个控制点
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
