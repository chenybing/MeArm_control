/* ============================================================================
 *  task3_keyboard.ino · 任务三：按键遥控板联动
 *  按键1 循环夹取 / 按键2 录制 / 按键3 回放 / 按键4 回中
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 * ============================================================================*/

/* ---- 任务三状态 ---- */
const uint16_t REC_MAX = 200;      // 最多 200 帧（SRAM 受限，勿盲目调大）
const uint16_t REC_MS  = 60;       // 每帧 60ms → 最长 12 秒（考核要求 >10 秒）
uint8_t  recBuf[REC_MAX][4];       // 每帧存 4 个舵机角度（各 1 字节，共 800 字节）
uint16_t recCount  = 0;            // 已录帧数
uint32_t recNextMs = 0;            // 下一帧的采样时刻
uint8_t  cycleIdx  = 0;            // 按键1 的循环下标：0=A 1=B 2=C

/* 任务三 · 按键1：每次按下循环切换 A → B → C → A …
 *
 * ⚠ 这里原来调的是 pickAndPlace()，也就是 task2_pick_place.ino 里那套【逆解】。
 *   逆解要标定完才准，而标定因为夹爪会滑已经放弃 —— 所以那条路是跑不起来的。
 *   现在改成调 runTeach5()，跟串口的 A/B/C 指令共用同一套【示教点位】。
 *
 * 号位分配（2026-10-11 重排）：A = 0~5、B = 6~11、C = 12~16
 * （见 task2_teach.ino 的 TP_CHAIN_SLOT / TP_GRP_START）。
 * 这里直接调 runTeach5()，和串口的 A/B/C、RUN 完全同一条路 ——
 * 免得「按键1」和「串口敲 A」哪天又走岔了。 */
void cyclePick() {
  const char *names[3] = { "A", "B", "C" };
  Serial.print(F("== 按键1 循环夹取，本次: ")); Serial.print(names[cycleIdx]);
  Serial.println(F(" =="));
  if (!runTeach5(cycleIdx, names[cycleIdx])) {
    return;                            // 没跑成 → 不推进循环，下次还是它
  }
  cycleIdx = (cycleIdx + 1) % 3;       // 成功了才换下一个物体
}

/* ---- 任务三：录制 / 回放 / 回中 ---- */
void toggleRecord() {
  if (mode == MODE_REC) {                       // 第二次按下：结束并保存
    mode = MODE_IDLE; recOn = false;
    Serial.print(F("录制结束，共 ")); Serial.print(recCount);
    Serial.print(F(" 帧 ≈ ")); Serial.print(recCount * REC_MS / 1000.0, 1);
    Serial.println(F(" 秒"));
    if (recCount * REC_MS < 10000) {
      Serial.println(F("WARN 录制时长不足 10 秒，建议重新录制"));
    }
    printStat("IDLE");
  } else {                                      // 首次按下：开始录制
    if (mode != MODE_IDLE) { Serial.println(F("BUSY: 请先结束当前模式")); return; }
    recCount = 0; recOn = true; mode = MODE_REC;
    recNextMs = millis();
    Serial.println(F("== 开始录制：请用摇杆操控机械臂（>10 秒），再按一次结束 =="));
    printStat("REC");
  }
}

/* 录制态每个循环：摇杆驱动四关节 + 定时采样 */
void recordStep() {
  joystickAll();
  if (millis() >= recNextMs) {
    recNextMs += REC_MS;
    if (recCount < REC_MAX) {
      recBuf[recCount][0] = (uint8_t)curBase;
      recBuf[recCount][1] = (uint8_t)curShoulder;
      recBuf[recCount][2] = (uint8_t)curElbow;
      recBuf[recCount][3] = (uint8_t)curGripper;
      recCount++;
    } else {                                    // 缓冲满：自动结束，保住已录数据
      Serial.println(F("WARN 录制缓冲已满（约 12 秒），自动结束"));
      toggleRecord();
    }
  }
}

/* 回放帧间隔：随速度档缩放，让任务一的 H/L「整体运行速度」对回放同样生效。
 * 以【默认档】(SPEED_DEFAULT) 为 1.0 倍 → 不动 H/L 时严格按录制时的 REC_MS(60ms)
 * 逐帧重现，保证考核要求的「完整复现」；按 H/L 调过速之后按同一比例缩放。
 * ⚠ 基准必须跟着 SPEED_DEFAULT 走。老代码写死了 STEP_DELAY_TAB[2]（档 3），
 *   默认档一改成 4，回放就快了 1.55 倍，不再是原速复现。 */
inline uint16_t playbackDelay() {
  return (uint16_t)(REC_MS * stepDelay() / (float)STEP_DELAY_TAB[SPEED_DEFAULT - 1]);
}

void playRecord() {
  if (recCount < 2) { Serial.println(F("ERR: 还没有录制数据，请先按 2 录制")); return; }
  if (mode != MODE_IDLE) { Serial.println(F("BUSY: 请先结束当前模式")); return; }

  stAbort = false; stPause = false;
  Serial.print(F("== 回放录制动作，")); Serial.print(recCount);
  Serial.print(F(" 帧 ≈ ")); Serial.print(recCount * REC_MS / 1000.0, 1);
  Serial.println(F(" 秒 =="));
  printStat("PLAY");

  for (uint16_t i = 0; i < recCount; i++) {
    curBase     = recBuf[i][0];
    curShoulder = recBuf[i][1];
    curElbow    = recBuf[i][2];
    curGripper  = recBuf[i][3];
    writeServos();
    if (stepGate()) { Serial.println(F("回放被取消")); printStat("IDLE"); return; }
    delay(playbackDelay());
  }
  Serial.println(F("回放完成"));
  printStat("IDLE");
}

/* 按键4 / 回中：回到初始姿态 */
void goHome() {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.println(F("== 回中 =="));
  // ⚠ 必须用 INIT_* 常量（与 setup() 共用同一份定义）。
  //    旧代码这里是 moveJoints(90,90,90,90)，夹爪 90° 落在 122~168 行程之外。
  moveJoints(INIT_BASE, INIT_SHOULDER, INIT_ELBOW, INIT_GRIPPER);
  printStat("IDLE");
}
