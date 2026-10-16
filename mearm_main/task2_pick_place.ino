/* ============================================================================
 *  task2_pick_place.ino · 任务二：A / B / C 夹取与放置
 *  三个物体各自的取物位与放置位，六步安全取放路径
 *  （与 mearm_main.ino 属于同一个 Arduino 工程，编译时自动合并为一个编译单元）
 * ============================================================================*/

/* ---- 任务二：夹取与放置 ---- */
/* 三个物体的取物位 / 放置位（桌面坐标 cm）。
 * 放置位与取物位在 u、v 两个方向上都有明显位移，且三个放置位互不相同。 */
const float PICK_U[3]  = { -5.0,  0.0,  5.0 };
const float PICK_V[3]  = {  8.5,  9.0,  8.5 };
const float PLACE_U[3] = { -2.5,  2.5, -5.0 };
const float PLACE_V[3] = { 12.5, 13.0, 12.0 };

/* 三种物体各自的夹取参数（考核指定：A = 三枚叠放硬币，B = 笔，C = 自定义物）。
 * 三者厚度差很多，共用一个夹取角必然出问题：
 *   偏张开的一侧（数字大）→ 夹爪压根没碰到物体，一抬就掉；
 *   偏闭合的一侧（数字小）→ 夹爪咬到底还差一截，舵机顶着机械止点"卡卡"响。
 *
 * ⚠ 口径：和全程序一致，是【逻辑角】（大 = 张开），不是舵机原始角。
 * ⚠ 下面三个数是【偏开一侧的保守占位值】，夹不紧但绝不会顶死，必须现场逐个标定：
 *   手动把夹爪合到刚好夹住该物体，看 CAL 打印的 G=（打的就是逻辑角）填进来。
 *   松了往【小】调（更闭合），顶死了往【大】调（更张开）。
 *   夹取角必须落在 GR_MIN~GR_MAX（50~167）之内。 */
const int   GRIP_CLOSE_OBJ[3] = { 140, 145, 150 };   // A 硬币 / B 笔 / C 自定义
/* ⚠ 2026-10-09 换平行夹爪 + 换方向约定后填的【保守占位值】：
 *   三个都取在明显偏开的一侧（对应舵机原始角 80 / 75 / 70），
 *   保证任何情况下都不会堵转烧舵机。保持「A 最紧、C 最松」的相对次序。
 *   第 1 批实测后替换。（旧值 {85,76,70} 是换了口径前的，已作废） */
const float GRIP_H_OBJ[3]     = { 0.6, 0.9, 0.8 };   // 下降夹取的高度 cm（取物体中段）

const float CARRY_H    = 3.5;     // 搬运时的抬升高度

void pickAndPlace(uint8_t idx, const char *name) {
  stAbort = false; stPause = false;
  const float gh = GRIP_H_OBJ[idx];        // 该物体对应的夹取高度
  const int   gc = GRIP_CLOSE_OBJ[idx];    // 该物体对应的夹紧角
  Serial.print(F("-- 夹取物体 ")); Serial.println(name);

  moveToTable(HOME_U, HOME_V, HOME_H);                              // ① 抬到安全高度
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);            // ② 张开夹爪
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);                   // ③ 到取物位上方
  moveToTable(PICK_U[idx], PICK_V[idx], gh);                        // ④ 下降对准物体
  moveJoints(curBase, curShoulder, curElbow, gc);                   // ⑤ 合爪抓取
  delay(300);                                                       //    等夹稳
  moveToTable(PICK_U[idx], PICK_V[idx], CARRY_H);                   // ⑥ 抬起
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);                 // ⑦ 转到放置位上方
  moveToTable(PLACE_U[idx], PLACE_V[idx], gh);                      // ⑧ 下降
  moveJoints(curBase, curShoulder, curElbow, GRIP_OPEN);            // ⑨ 开爪放下
  delay(300);                                                       //    等落稳
  moveToTable(PLACE_U[idx], PLACE_V[idx], CARRY_H);                 // ⑩ 抬起离开

  Serial.print(F("-- 物体 ")); Serial.print(name); Serial.println(F(" 放置完成"));
}
