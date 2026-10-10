# 屠龙勇士智能车

王晨旭所在屠龙勇士竞赛团队的智能车工程源码快照。本人担任队长，团队获第十一届屠龙勇士智能车大赛校级二等奖。

## 程序与迭代

- `xunji*.ino`、`xunji_bt_pid*/`：循迹、PID 与蓝牙控制版本。
- `bizhang*/`、`chaoshengfinal.cpp`：避障与超声波相关程序。
- `huandao*/`：环岛及直角场景处理。
- `multimode_rover/`：多模式整车程序。
- `yaokong/`、`duojitest/`、`mixed*/`：遥控、舵机及模块联调版本。

这是工程迭代代码集，各程序独立选择使用，不应把所有 ino 合并编译。基础依赖为 Arduino API，部分版本使用 SoftwareSerial 和 Servo；接线与目标开发板以各文件的引脚定义及注释为准。未发布同学的参考代码目录。

本人主要贡献是硬件装配、传感器和电机接入、固件调试、整车联调及赛道测试。此代码集用于展示工程实现，不作为当前任意硬件平台的即插即用固件。

个人主页：https://lucianwangchn.github.io/
