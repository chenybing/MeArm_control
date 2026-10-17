// meArm 主控：模式调度与串口指令解析
// 其余功能在同目录的 kinematics / task1 / task2 / task3 / task5 文件中

#include <Servo.h>
#include <math.h>
#include <ctype.h>
#include <EEPROM.h>

#define REMOTE_SOFT_SERIAL 0

#if REMOTE_SOFT_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial remoteSerial(12, 13);
#endif

// 舵机与摇杆引脚
const uint8_t PIN_BASE     = 9;
const uint8_t PIN_SHOULDER = 7;
const uint8_t PIN_ELBOW    = 8;
const uint8_t PIN_GRIPPER  = 6;

const uint8_t PIN_J1X = A0;
const uint8_t PIN_J1Y = A1;
const uint8_t PIN_J2X = A2;
const uint8_t PIN_J2Y = A3;
const uint8_t PIN_BTN1 = 5;
const uint8_t PIN_BTN2 = 4;

const uint8_t BTN_PAUSE  = 2;
const uint8_t BTN_GO     = 3;
const uint8_t BTN_CANCEL = 10;
const uint8_t BTN_TEACH  = 11;
const uint8_t BTN_CYCLE  = 12;
const uint8_t BTN_PLAY   = 13;

#define LOCAL_BUTTONS 1

// 连杆尺寸与零点标定（单位 cm / 度）
const float ARM_L1   = 8.0;
const float ARM_L2   = 8.0;
const float L3A      = 1.5;
const float L3B      = 8.5;
const float PEN_DROP = 7.5;

const float H_SHOULDER = 6.2;

const int  SHOULDER_TRIM   = 170;

const bool SHOULDER_INVERT = true;
const int  ELBOW_TRIM      = 100;

const float ELBOW_OVER_ARM_MAX = 10.0f;

const bool ELBOW_INVERT    = false;
const bool BASE_INVERT     = true;

const int  ELBOW_SIGN      = -1;

const int BASE_ANG = 90;
const int BASE_SWEEP = 60;

const int BASE_MIN = 13,  BASE_MAX = 172;
const int SH_MIN   = 33,  SH_MAX   = 157;
const int EL_MIN   = 33,  EL_MAX   = 122;

const int GR_MIN   = 50,  GR_MAX   = 167;

const float R_MIN = 10.0;
const float R_MAX = 26.0;

const int  JOY_CENTER[4] = { 529, 512, 529, 509 };
const int  JOY_DEADZONE  = 40;

const bool JOY_INV_J1X = true;
const bool JOY_INV_J1Y = true;
const bool JOY_INV_J2X = true;
const bool JOY_INV_J2Y = true;
const float TEACH_SPEED = 0.05;

const float PEN_GRIP = 50;

const float PEN_LIFT = 2.0;
const float TEACH_H  = 0.3;

float penHTrim = 0.0;

const float HOME_U = 0.0, HOME_V = 12.0, HOME_H = 4.0;

const int   GRIP_OPEN  = 145;

const int   GRIP_CLOSE = 50;

const int   GR_RAW_OPEN = 50, GR_RAW_CLOSE = 170;
const int   GR_RAW_SUM  = GR_RAW_OPEN + GR_RAW_CLOSE;

const int   INIT_BASE = 90, INIT_SHOULDER = 90, INIT_ELBOW = 90, INIT_GRIPPER = GRIP_OPEN;

const int STEP_DELAY_TAB[5] = { 60, 30, 14, 9, 6 };

const int SPEED_DEFAULT = 3;
int speedLevel = SPEED_DEFAULT;

inline int stepDelay() { return STEP_DELAY_TAB[speedLevel - 1]; }

Servo svBase, svShoulder, svElbow, svGripper;
int curBase = 90, curShoulder = 90, curElbow = 90, curGripper = 90;

enum Mode { MODE_IDLE, MODE_MANUAL, MODE_TEACH, MODE_REC };
Mode mode = MODE_IDLE;

bool stPause = false;
bool stAbort = false;

const uint8_t  LINE_MAX = 64;
const uint16_t LINE_GAP_MS = 40;
const uint8_t  CH_PC = 0;
const uint8_t  CH_RC = 1;

#define LINE_CHANS (REMOTE_SOFT_SERIAL ? 2 : 1)
char     lineBuf[LINE_CHANS][LINE_MAX];
uint8_t  lineLen[LINE_CHANS]    = { 0 };
uint32_t lineLastMs[LINE_CHANS] = { 0 };

bool recOn = false;

const uint8_t BTN_COUNT = 8;
const uint8_t BTN_PINS[BTN_COUNT] = {
  BTN_PAUSE, BTN_GO, BTN_CANCEL, BTN_TEACH,
  BTN_CYCLE, BTN_PLAY, PIN_BTN2, PIN_BTN1
};
enum { B_PAUSE = 0, B_GO, B_CANCEL, B_TEACH, B_CYCLE, B_PLAY, B_REC, B_HOME };
bool     btnEdge[BTN_COUNT];
bool     btnLast[BTN_COUNT];
uint32_t btnLock[BTN_COUNT];

const char* modeName() {
  switch (mode) {
    case MODE_MANUAL: return recOn ? "REC" : "MANUAL";
    case MODE_TEACH:  return "TEACH";
    case MODE_REC:    return "REC";
    default:          return "IDLE";
  }
}
void printStat(const char *s) { Serial.print(F("STAT:")); Serial.println(s); }

extern int __heap_start, *__brkval;
int freeRam() {
  int v;

  char *heapTop = (__brkval == 0) ? (char *)&__heap_start : (char *)__brkval;
  return (int)((char *)&v - heapTop);
}

void setup() {
  svBase.attach(PIN_BASE);
  svShoulder.attach(PIN_SHOULDER);
  svElbow.attach(PIN_ELBOW);
  svGripper.attach(PIN_GRIPPER);

  curBase     = INIT_BASE;
  curShoulder = INIT_SHOULDER;
  curElbow    = INIT_ELBOW;
  curGripper  = INIT_GRIPPER;
  writeServos();

  for (uint8_t i = 0; i < BTN_COUNT; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("meArm 就绪，发 HELP 看指令"));
#if LOCAL_BUTTONS
  Serial.println(F("本地按键开着：D2 暂停，D3 继续，D10 取消，D11 记点"));
#else
  Serial.println(F("本地按键关着（LOCAL_BUTTONS=0）"));
  Serial.println(F("暂停、继续、取消用 PAUSE / RESUME / CANCEL"));
#endif
  Serial.print(F("内存还剩 ")); Serial.print(freeRam()); Serial.println(F(" 字节"));
  tpLoadAll();
  printStat("IDLE");
}

void loop() {
  pollSerial(false);
  scanButtons();

  switch (mode) {
    case MODE_MANUAL: joystickAll();       break;
    case MODE_TEACH:  teachStep();         break;
    case MODE_REC:    recordStep();        break;
    default:          handleLocalButtons(); break;
  }
}

void pollOne(Stream &s, uint8_t ch, bool inMotion) {
  while (s.available() > 0) {
    char c = s.read();
    lineLastMs[ch] = millis();
    if (c == '\n' || c == '\r') {
      if (lineLen[ch] > 0) {
        lineBuf[ch][lineLen[ch]] = 0; lineLen[ch] = 0;
        deliver(lineBuf[ch], inMotion);
      }
    } else if (lineLen[ch] < LINE_MAX - 1) {
      lineBuf[ch][lineLen[ch]++] = c;
    } else {
      lineLen[ch] = 0;
      Serial.println(F("指令太长，丢掉了"));
    }
  }
  if (lineLen[ch] > 0 && millis() - lineLastMs[ch] > LINE_GAP_MS) {
    lineBuf[ch][lineLen[ch]] = 0; lineLen[ch] = 0;
    deliver(lineBuf[ch], inMotion);
  }
}

void deliver(char *cmd, bool inMotion) {
  if (inMotion) dispatchInMotion(cmd);
  else          dispatch(cmd);
}

void pollSerial(bool inMotion) {
  pollOne(Serial, CH_PC, inMotion);
#if REMOTE_SOFT_SERIAL
  pollOne(remoteSerial, CH_RC, inMotion);
#endif
}

// 动作执行期间只受理暂停、继续、取消和调速
void dispatchInMotion(const char *cmd) {
  if      (eq(cmd, "PAUSE"))                    { stPause = true;  printStat("PAUSED"); }
  else if (eq(cmd, "RESUME") || eq(cmd, "GO"))  { stPause = false; printStat("DRAWING"); }
  else if (eq(cmd, "CANCEL"))                   { stAbort = true;  stPause = false; Serial.println(F("CANCEL...")); }
  else if (eq(cmd, "STATUS"))                   { printStat(modeName()); }

  else if (eq(cmd, "H")) { if (speedLevel < 5) speedLevel++; Serial.print(F("速度档 ")); Serial.println(speedLevel); }
  else if (eq(cmd, "L")) { if (speedLevel > 1) speedLevel--; Serial.print(F("速度档 ")); Serial.println(speedLevel); }
  else if (cmd[0]) {
    Serial.print(F("还在动，先不发「"));
    Serial.print(cmd);
    Serial.println(F("」。H 可以加速，CANCEL 可以停"));
  }
}

// 单字符指令按字母分派，多字符指令按关键字匹配
void dispatch(char *cmd) {
  char *p = cmd;
  while (*p == ' ' || *p == '\t') p++;
  char *e = p + strlen(p);
  while (e > p && (e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
  if (!*p) return;
  for (char *q = p; *q; q++) *q = toupper(*q);

  if (p[0] == 'X' && strchr(p, ',')) { cmdMultiServo(p); return; }

  if (strlen(p) == 1) {
    switch (p[0]) {
      case 'M': mode = (mode == MODE_MANUAL) ? MODE_IDLE : MODE_MANUAL;
                stPause = stAbort = false;
                Serial.println(mode == MODE_MANUAL ? F("STAT:MANUAL 摇杆操控中")
                                                   : F("STAT:IDLE"));
                return;
      case 'O': cmdGripper(GRIP_OPEN,  "开爪"); return;
      case 'S': cmdGripper(GRIP_CLOSE, "合爪"); return;
      case 'H': if (speedLevel < 5) speedLevel++;
                Serial.print(F("速度档 ")); Serial.println(speedLevel); return;
      case 'L': if (speedLevel > 1) speedLevel--;
                Serial.print(F("速度档 ")); Serial.println(speedLevel); return;

      case 'A': runTeach5(0, "A"); return;
      case 'B': runTeach5(1, "B"); return;
      case 'C': runTeach5(2, "C"); return;
      case 'N': drawFigure('N'); return;
      case 'T': drawFigure('T'); return;
      case 'Z': drawFigure('Z'); return;
      case 'V': drawFigure('V'); return;
      case '1': cyclePick(); return;
      case '2': toggleRecord(); return;
      case '3': playRecord(); return;
      case '4': goHome(); return;
      default: break;
    }
  }

  else if (eq(p, "RUN"))        runAllThree();
  else if (eq(p, "LINE"))  drawLineDemo();
  else if (eq(p, "TRI"))   drawFigure('T');
  else if (eq(p, "TEACH")) enterTeach();
  else if (eq(p, "SET"))   recordTeachPoint();
  else if (eq(p, "CLR"))   clearTeachPoints();
  else if (eq(p, "DRAW"))  drawByTeach(false);
  else if (eq(p, "CURVE")) drawByTeach(true);
  else if (eq(p, "DEMO"))  { loadDemoPoints(); drawByTeach(false); }
  else if (eq(p, "PAUSE")) { stPause = true;  Serial.println(F("STAT:PAUSED")); }
  else if (eq(p, "RESUME")){ stPause = false; Serial.println(F("STAT:IDLE")); }
  else if (eq(p, "CANCEL")){ stAbort = true;  stPause = false; }
  else if (eq(p, "GO"))    goHome();
  else if (eq(p, "CAL"))   printCalib();
  else if (p[0] == 'J' && p[1] == 'O' && p[2] == 'G') cmdJog(p);

  else if (strchr("BSEG", p[0]) && isdigit((unsigned char)p[1])) cmdAxisAbs(p);

  else if (p[0] == 'F' && isdigit((unsigned char)p[1])) cmdGripperRaw(p);

  else if (p[0] == 'D' && (isdigit((unsigned char)p[1]) || p[1] == '-' || p[1] == '+'))
    cmdPenTrim(p);

  else if (p[0] == 'J' && p[1] && strchr("BSEG", p[1])) cmdJog1(p);

  else if (p[0] == 'W' && isdigit((unsigned char)p[1])) cmdTeachWrite(p);
  else if (p[0] == 'R' && isdigit((unsigned char)p[1]))  cmdTeachGo(p);
  else if (eq(p, "WCLR"))                               cmdTeachClear();
  else if (eq(p, "LIST"))                               cmdTeachList();

  else if (eq(p, "RAM"))   { Serial.print(F("内存还剩 ")); Serial.print(freeRam()); Serial.println(F(" 字节")); }
  else if (p[0] == 'P' && p[1] == 'P')                  cmdPickPlace(p);
  else if (p[0] == 'P' && isdigit((unsigned char)p[1])) cmdTeachPut(p);
  else if (eq(p, "JOY"))   printJoystick();
  else if (eq(p, "HELP"))  printHelp();
  else if (eq(p, "STATUS"))printStat(modeName());
  else { Serial.print(F("没这个指令： ")); Serial.println(p); Serial.println(F("发 HELP 看看")); }
}

void printHelp() {
  Serial.println(F("---- 指令 ----"));
  Serial.println(F("摇杆：M 开/关"));
  Serial.println(F("夹爪：O 张开，S 合上"));
  Serial.println(F("速度：H 快一档，L 慢一档"));
  Serial.println(F("三个舵机一起转：x10,y30,z20"));
  Serial.println(F(""));
  Serial.println(F("取放：A B C 各走一个，RUN 从 A 一路走到 C"));
  Serial.println(F("点位用 W<号> 记，R<号> 回去，LIST 看全部，WCLR 清空"));
  Serial.println(F("PP 0 1 2 3 4 按号位顺序走一遍"));
  Serial.println(F("0 物体上方，1 物体位，2 上升，3 平移，4 放置位"));
  Serial.println(F("点位存在 EEPROM 里，断电不丢"));
  Serial.println(F(""));
  Serial.println(F("摆姿势：B30 S60 E100 G20 把某个轴设到指定位置"));
  Serial.println(F("S/E 数字大是抬高，B 数字小是往左"));
  Serial.println(F("G 是夹紧程度，0 合死，100 全开"));
  Serial.println(F("JB-3 这种是在当前角度上加减小角度"));
  Serial.println(F("P0 后面的四个数是直接抄 LIST，机械臂不动"));
  Serial.println(F(""));
  Serial.println(F("按键/遥控：1 循环夹取，2 录制，3 回放，4 回中"));
  Serial.println(F("A 是号位 0~5，B 是 6~11，C 是 12~15"));
  Serial.println(F(""));
  Serial.println(F("画图：LINE 直线，N T Z V 几个图形，三角形也可以发 TRI"));
  Serial.println(F("TEACH 进示教，摇杆移笔尖，SET 记点，CLR 清空"));
  Serial.println(F("DRAW 折线，CURVE 平滑曲线，DEMO 用内置五点"));
  Serial.println(F("画的时候 PAUSE 暂停，RESUME 继续，CANCEL 取消，GO 回待机"));
  Serial.println(F(""));
  Serial.println(F("调试：CAL 看笔尖坐标，JOG s e 直接给连杆角，JOY 看摇杆读数"));
  Serial.println(F("STATUS 看状态，RAM 看剩多少内存"));
  Serial.println(F("F<角> 夹爪直接转，D<毫米> 笔尖高低微调，负数往下压"));
}
