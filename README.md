# MeArm_control

机器人学院十二期二面考核 · 算法（控制、视觉）方向

meArm 机械臂的 Arduino 控制程序。

## 目录

- `mearm_main/` —— 主程序，任务一 / 二 / 三 / 五都在这里，按任务分成七个 `.ino`，打开 `mearm_main.ino` 一起编译。
- `remote_serial/` —— 任务三按键遥控板的发送端。

## 使用

1. Arduino IDE 打开 `mearm_main/mearm_main.ino`，选 Arduino Uno，编译上传。
2. 串口监视器波特率 9600，行尾选“换行符”。
3. 发 `HELP` 看指令，发 `CAL` 看笔尖坐标。

## 接线

舵机：底座 D9，大臂 D7，小臂 D8，夹爪 D6

摇杆：X1=A0 Y1=A1 X2=A2 Y2=A3，摇杆按键 D5 / D4

本地按键：暂停 D2，继续 D3，取消 D10，示教记录 D11（另一端接 GND）
