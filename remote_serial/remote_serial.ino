/* ============================================================================
 *  Arduino 按键遥控板 · 发送端固件（任务三 · 串口通信侧）
 * ----------------------------------------------------------------------------
 *  这是「电路组自制的按键遥控板」上烧的程序。它负责：
 *    ① 捕获 4 个按键（带消抖），把对应指令通过串口发给机械臂主控；
 *    ② （可选）接收机械臂回传的 STAT:xxx 状态行，显示在 OLED 上。
 *
 *  ── 指令协议（与 mearm_main.ino 一致，如需改动两边都要改）──
 *    按键1 → "1"   循环夹取 A / B / C（每按一次切换下一个物体）
 *    按键2 → "2"   开始 / 结束录制（摇杆操控，需 >10 秒）
 *    按键3 → "3"   回放上一次录制的动作
 *    按键4 → "4"   回中（回到初始姿态）
 *
 *  ── 接线（与机械臂主控对接）────────────────────────────────────────────────
 *    遥控板 TX  →  机械臂 D0(RX)        ← 必须交叉接
 *    遥控板 RX  →  机械臂 D1(TX)        ← 可选，只用来收状态显示到 OLED
 *    遥控板 GND →  机械臂 GND           ← 必须共地！否则通信一定失败
 *
 *    ⚠ 两个坑（现场演示最容易翻车的地方）：
 *    1) 机械臂那一端用的是硬件串口 D0/D1，与板载 CH340 并联。
 *       如果机械臂还插着 USB 线，CH340 的 TX 会和遥控板的 TX 一起驱动 D0，
 *       双方互相打架 → 指令乱码或收不到。
 *       解决办法：下载完程序后【拔掉机械臂的 USB 线】，改用外接 5V 供电。
 *       如果必须同时插 USB，请把 mearm_main.ino 顶部的
 *       REMOTE_SOFT_SERIAL 改成 1，用软串口 D12/D13 接遥控板。
 *    2) 两块板子必须共地，否则 5V 电平没有共同参考，数据全是乱码。
 *
 *  ── 按键引脚 ───────────────────────────────────────────────────────────────
 *    ⚠ 这里给的是【默认值】，实际引脚请以电路组画板子时的连接为准！
 *      拿不准就直接问电路方向的同学要引脚表，改下面 4 个常量即可。
 * ==========================================================================*/

#include <SoftwareSerial.h>   // 用于把指令转发给机械臂（如果不用板载 D0/D1）

/* ======================= 1. 按键引脚（按实际板子修改）===================== */
const uint8_t KEY1_PIN = 2;    // 按键1：循环夹取
const uint8_t KEY2_PIN = 3;    // 按键2：录制
const uint8_t KEY3_PIN = 4;    // 按键3：回放
const uint8_t KEY4_PIN = 5;    // 按键4：回中

/* 按键接法：一端接引脚、一端接 GND → 用内部上拉，按下读到 LOW。
 * 如果电路组画的是「按下读到 HIGH」（下拉 + 接 VCC），把这里改成 false。 */
const bool KEY_ACTIVE_LOW = true;

/* ======================= 2. 与机械臂通信的口 ============================= */
/* 用哪一路把指令发给机械臂：
 *   0 = 板载硬件串口 D0/D1（TXD/RXD 排针直接引到机械臂，最常用）
 *   1 = 软串口（默认 D6/D7，可在下面改）——当板载 D0/D1 被 CH340 占用、
 *       需要同时插着 USB 调试时用。                                  */
#define ARM_SOFT_SERIAL 0
#if ARM_SOFT_SERIAL
  SoftwareSerial armSerial(6, 7);        // RX(可不用), TX → 机械臂 RX
  Stream &ARM = armSerial;
#else
  Stream &ARM = Serial;
#endif

/* 指令之间加一点间隔，避免机械臂还在处理上一条时把两条粘在一起 */
const uint16_t SEND_GAP_MS = 30;

/* ======================= 3. 全局变量 ===================================== */
const uint8_t KEY_COUNT = 4;
const uint8_t KEY_PINS[KEY_COUNT] = { KEY1_PIN, KEY2_PIN, KEY3_PIN, KEY4_PIN };
const char *KEY_CMDS[KEY_COUNT] = { "1", "2", "3", "4" };   // 对应发出的指令

bool     keyLast[KEY_COUNT];
uint32_t keyLock[KEY_COUNT];      // 消抖时间戳
uint32_t lastSendMs = 0;

/* ======================= 4. setup / loop ================================= */
void setup() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    pinMode(KEY_PINS[i], KEY_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    keyLast[i] = false;
    keyLock[i] = 0;
  }

  Serial.begin(9600);              // 与电脑通信（烧录 / 调试）
#if ARM_SOFT_SERIAL
  armSerial.begin(9600);           // 与机械臂通信
#endif

  Serial.println(F("READY | 按键遥控板 | 1=循环夹取 2=录制 3=回放 4=回中"));
  Serial.println(F("提示：可直接在串口监视器里打字，指令会转发给机械臂"));
}

void loop() {
  /* ---- ① 扫描按键，有按下就发指令 ---- */
  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    bool now = (digitalRead(KEY_PINS[i]) == LOW);   // 上拉接法：按下 = LOW
    if (!KEY_ACTIVE_LOW) now = !now;                // 下拉接法：按下 = HIGH

    if (now && !keyLast[i] && (millis() - keyLock[i] > 200)) {  // 200ms 消抖
      keyLock[i] = millis();
      sendCommand(KEY_CMDS[i]);
    }
    keyLast[i] = now;
  }

  /* ---- ② 把电脑串口监视器里输入的内容转发给机械臂（方便现场救急）---- */
  if (Serial.available() > 0) {
    char c = Serial.read();
    ARM.write(c);
  }

  /* ---- ③ 把机械臂回传的状态行打印出来（有 OLED 时改成显示到屏上）---- */
  while (ARM.available() > 0) {
    char c = ARM.read();
    Serial.write(c);
    // 若电路组接了 OLED：把状态行攒起来，遇到 '\n' 时 OLED 显示即可
  }
}

/* ======================= 5. 发送指令 ===================================== */
void sendCommand(const char *cmd) {
  if (millis() - lastSendMs < SEND_GAP_MS) delay(SEND_GAP_MS);  // 限流
  lastSendMs = millis();

  ARM.print(cmd);
  ARM.print('\n');                 // 机械臂按「行」解析，必须带换行

  Serial.print(F("发送指令 → "));
  Serial.println(cmd);
}
