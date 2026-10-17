// 按键遥控板：按键 1~4 发动作指令，5~8 控制绘图
// 通过串口把指令发送给机械臂主控

#include <SoftwareSerial.h>

const uint8_t KEY1_PIN = A0;
const uint8_t KEY2_PIN = A1;
const uint8_t KEY3_PIN = A2;
const uint8_t KEY4_PIN = A3;

const uint8_t KEY5_PIN = 5;
const uint8_t KEY6_PIN = 6;
const uint8_t KEY7_PIN = 7;
const uint8_t KEY8_PIN = 8;

const bool KEY_ACTIVE_LOW = true;

#define ARM_SOFT_SERIAL 0
#if ARM_SOFT_SERIAL
  SoftwareSerial armSerial(6, 7);
  Stream &ARM = armSerial;
#else
  Stream &ARM = Serial;
#endif

const uint16_t SEND_GAP_MS = 30;

const uint8_t KEY_COUNT = 8;
const uint8_t KEY_PINS[KEY_COUNT] = {
  KEY1_PIN, KEY2_PIN, KEY3_PIN, KEY4_PIN,
  KEY5_PIN, KEY6_PIN, KEY7_PIN, KEY8_PIN
};

const char *KEY_CMDS[KEY_COUNT] = {
  "1", "2", "3", "4",
  "PAUSE", "RESUME", "CANCEL", "SET"
};

bool     keyLast[KEY_COUNT];
uint32_t keyLock[KEY_COUNT];
uint32_t lastSendMs = 0;

void setup() {
  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    pinMode(KEY_PINS[i], KEY_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    keyLast[i] = false;
    keyLock[i] = 0;
  }

  Serial.begin(9600);
#if ARM_SOFT_SERIAL
  armSerial.begin(9600);
#endif

  Serial.println(F("遥控板就绪：1 循环 2 录制 3 回放 4 回中，5 暂停 6 继续 7 取消 8 记点"));
  Serial.println(F("也能直接在串口里打字，会转给机械臂"));
}

void loop() {

  for (uint8_t i = 0; i < KEY_COUNT; i++) {
    bool now = (digitalRead(KEY_PINS[i]) == LOW);
    if (!KEY_ACTIVE_LOW) now = !now;

    if (now && !keyLast[i] && (millis() - keyLock[i] > 200)) {
      keyLock[i] = millis();
      sendCommand(KEY_CMDS[i]);
    }
    keyLast[i] = now;
  }

  if (Serial.available() > 0) {
    char c = Serial.read();
    ARM.write(c);
  }

  while (ARM.available() > 0) {
    char c = ARM.read();
    Serial.write(c);

  }
}

void sendCommand(const char *cmd) {
  if (millis() - lastSendMs < SEND_GAP_MS) delay(SEND_GAP_MS);
  lastSendMs = millis();

  ARM.print(cmd);
  ARM.print('\n');

  Serial.print(F("发 "));
  Serial.println(cmd);
}
