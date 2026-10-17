// 任务二：三个物体的取物位、放置位及对应夹爪高度

const float PICK_U[3]  = { -5.0,  0.0,  5.0 };
const float PICK_V[3]  = {  8.5,  9.0,  8.5 };
const float PLACE_U[3] = { -2.5,  2.5, -5.0 };
const float PLACE_V[3] = { 12.5, 13.0, 12.0 };

const int   GRIP_CLOSE_OBJ[3] = { 140, 145, 150 };

const float GRIP_H_OBJ[3]     = { 0.6, 0.9, 0.8 };

const float CARRY_H    = 3.5;

void pickAndPlace(uint8_t idx, const char *name) {
  stAbort = false; stPause = false;
  const float gh = GRIP_H_OBJ[idx];
  const int   gc = GRIP_CLOSE_OBJ[idx];
  Serial.print(F("-- 夹取物体 ")); Serial.println(name);

  moveToTable(HOME_U, HOME_V, HOME_H);
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);
  moveToTable(PICK_U[idx], PICK_V[idx], gh);
  moveJoints(curBase, curShoulder, curElbow, gc);
  delay(300);
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);
  moveToTable(PLACE_U[idx], PLACE_V[idx], gh);
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);
  delay(300);
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);

  Serial.print(F("-- 物体 ")); Serial.print(name); Serial.println(F(" 放置完成"));
}
