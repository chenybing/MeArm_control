# mearm_main —— meArm 控制主程序（提交用）

任务一 / 二 / 三 / 五 全部功能的**整合工程**，一份可直接编译、烧录、复现视频效果的代码。

> Arduino 只会编译**与本文件同名的 .ino**，所以整个程序都写在 `mearm_main.ino` 一个文件里。
> 烧录：Arduino IDE 打开 `mearm_main.ino` → 选板子 Arduino Uno → 上传。

## 串口指令表（波特率 9600）

| 任务 | 指令 | 功能 |
|---|---|---|
| 一 | `M` | 进入 / 退出摇杆实时操控 |
| 一 | `O` / `S` | 夹爪张开 / 闭合 |
| 一 | `H` / `L` | 速度档 +1 / −1（1~5 档，全局生效） |
| 一 | `x10,y30,z20` | x=底座 y=大臂 z=小臂，三舵机同步转动 |
| 二 | `A` / `B` / `C` | 夹取对应物体并放到各自的放置位 |
| 二 | `RUN` | 依次夹取 A、B、C |
| 三 | `1` | 循环夹取（每按一次换下一个物体） |
| 三 | `2` | 录制开始 / 结束（摇杆操控，>10 秒） |
| 三 | `3` | 回放上次录制的动作 |
| 三 | `4` | 回中 |
| 五 | `LINE` | 直线 |
| 五 | `N` / `TRI` / `Z` / `V` | 字母 N / 三角形 / 字母 Z / 字母 V |
| 五 | `TEACH` | 进入五点示教模式 |
| 五 | `SET` / `CLR` | 记录示教点 / 清空示教点 |
| 五 | `DRAW` / `CURVE` | 按示教点画折线 / 平滑曲线 |
| 五 | `DEMO` | 内置五点演示 |
| 五 | `PAUSE` / `RESUME` / `CANCEL` | 暂停 / 继续 / 取消 |
| — | `GO` | 回待机位 |
| 调试 | `CAL` / `JOG s e` / `HELP` / `STATUS` | 打印笔尖坐标 / 连杆角驱动 / 指令表 / 状态 |

## 本地按键

| 按键 | 引脚 | 功能 |
|---|---|---|
| 暂停 | D2 | 任务五：暂停绘制 |
| 继续 | D3 | 任务五：从暂停处接续 |
| 取消 | D10 | 任务五：终止并返回待机位 |
| 示教记录 | D11 | 任务五：记录当前笔尖位置为一个示教点 |
| 摇杆2 按键 | D4 | 任务三：录制开始 / 结束 |
| 摇杆1 按键 | D5 | 任务三：回中 |

## 现场演示任务五（考官临时指定五点）

```
TEACH          → 摇杆1 把笔尖移到目标点（X=左右，Y=前后）
SET            → 记录第 1 个点（或按 D11）
... 重复到第 5 个点
DRAW           → 画折线
CURVE          → 画平滑曲线
PAUSE / RESUME / CANCEL  → 暂停 / 继续 / 取消（也可按 D2 / D3 / D10）
```

**全过程不需要重新编译或烧录程序**，符合考核要求。

## 画图前必须标定

`mearm_main.ino` 顶部：

```cpp
const float ARM_L1 = 8.0;        // 大臂长度 cm
const float ARM_L2 = 8.0;        // 小臂长度 cm（量到笔尖）
const float H_SHOULDER = 6.0;    // 肩关节轴离桌面高度 cm
const int  SHOULDER_TRIM = 90;   // 大臂水平前伸时对应的舵机角
const bool SHOULDER_INVERT = true;
const int  ELBOW_TRIM = 0;
const bool ELBOW_INVERT = true;
const bool BASE_INVERT = false;
const int  ELBOW_SIGN = -1;      // 肘部解支，桌面画图用 -1
```

标定流程见 `docs/说明文档.md` 第七节。用 `JOG 0 0` 和 `CAL` 两个指令可以快速校准。

## 资源占用（实测）

Flash 14.1 KB / 32 KB，静态 SRAM 1.0 KB / 2 KB —— Uno 上余量充足。
录制缓冲已按 SRAM 限制压缩为 `uint8_t[200][4]`（12 秒），不要盲目调大 `REC_MAX`。
