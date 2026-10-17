// 任务一：摇杆实时控制、夹爪开合、x/y/z 多舵机指令

#define JOY_M_SCALE 0.65f

const float JOY_RATE_TAB[5] = { 0.020f, 0.035f, 0.055f, 0.080f, 0.110f };
inline float joyFollowRate() { return JOY_RATE_TAB[speedLevel - 1] * JOY_M_SCALE; }

const float JOY_STEP_BASE     = 1.3f * JOY_M_SCALE;
const float JOY_STEP_SHOULDER = 1.3f * JOY_M_SCALE;
const float JOY_STEP_ELBOW    = 3.0f * JOY_M_SCALE;
const float JOY_STEP_GRIPPER  = 1.3f * JOY_M_SCALE;

float joyPosBase = 90, joyPosShoulder = 90, joyPosElbow = 90, joyPosGrip = 90;

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

// 摇杆偏差映射到 0~180 的目标角，死区内保持当前值
int joyTarget(int joy, int cur, int center, bool invert) {
  int off = joy - center;
  if (off > -JOY_DEADZONE && off < JOY_DEADZONE) return cur;
  if (invert) off = -off;
  int half = (off > 0) ? (1023 - center) : center;
  return constrain(90 + (int)((long)off * 90L / half), 0, 180);
}

void smoothJoint(int &cur, float &pos, int target, int lo, int hi,
                 float maxStep) {

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

void cmdGripperRaw(const char *p) {
  int a = atoi(p + 1);
  a = constrain(a, GR_MIN, GR_MAX);
  Serial.print(F("[诊断] 夹爪 逻辑角 → ")); Serial.print(a);
  Serial.print(F("（舵机原始角 ")); Serial.print(GR_RAW_SUM - a); Serial.println(F("）"));
  moveJoints(curBase, curShoulder, curElbow, a);
  Serial.println(F("[诊断] 已下发。舵机是开环的，实际停在哪得用眼睛看"));
}

void cmdGripper(int target, const char *name) {
  Serial.print(F("CMD ")); Serial.println(name);
  moveJoints(curBase, curShoulder, curElbow, target);
}

// 解析 "x10,y30,z20"，一次更新三个舵机
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
