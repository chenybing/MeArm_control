// 按键 1 循环夹取，按键 2/3 录制回放，按键 4 回中

const uint16_t REC_MAX = 200;
const uint16_t REC_MS  = 60;
uint8_t  recBuf[REC_MAX][4];
uint16_t recCount  = 0;
uint32_t recNextMs = 0;
uint8_t  cycleIdx  = 0;

void cyclePick() {
  const char *names[3] = { "A", "B", "C" };
  Serial.print(F("循环夹取，这次 ")); Serial.print(names[cycleIdx]);
  Serial.println();
  if (!runTeach5(cycleIdx, names[cycleIdx])) {
    return;
  }
  cycleIdx = (cycleIdx + 1) % 3;
}

void toggleRecord() {
  if (mode == MODE_REC) {
    mode = MODE_IDLE; recOn = false;
    Serial.print(F("录制完了，")); Serial.print(recCount);
    Serial.print(F(" 帧，约 ")); Serial.print(recCount * REC_MS / 1000.0, 1);
    Serial.println(F(" 秒"));
    if (recCount * REC_MS < 10000) {
      Serial.println(F("不到 10 秒，建议重录"));
    }
    printStat("IDLE");
  } else {
    if (mode != MODE_IDLE) { Serial.println(F("先把当前模式退出来")); return; }
    recCount = 0; recOn = true; mode = MODE_REC;
    recNextMs = millis();
    Serial.println(F("开始录制，用摇杆动，再按一次结束（最好超过 10 秒）"));
    printStat("REC");
  }
}

// 每 60 ms 记录一帧关节角，约可存 12 秒
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
    } else {
      Serial.println(F("缓冲满了（约 12 秒），自动停"));
      toggleRecord();
    }
  }
}

inline uint16_t playbackDelay() {
  return (uint16_t)(REC_MS * stepDelay() / (float)STEP_DELAY_TAB[SPEED_DEFAULT - 1]);
}

void playRecord() {
  if (recCount < 2) { Serial.println(F("还没录过，先按 2")); return; }
  if (mode != MODE_IDLE) { Serial.println(F("先把当前模式退出来")); return; }

  stAbort = false; stPause = false;
  Serial.print(F("回放，")); Serial.print(recCount);
  Serial.print(F(" 帧 ≈ ")); Serial.print(recCount * REC_MS / 1000.0, 1);
  Serial.println(F(" 秒"));
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

void goHome() {
  stAbort = false; stPause = false;
  mode = MODE_IDLE;
  Serial.println(F("回中"));
  moveJoints(INIT_BASE, INIT_SHOULDER, INIT_ELBOW, INIT_GRIPPER);
  printStat("IDLE");
}
