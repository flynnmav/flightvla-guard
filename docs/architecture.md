# FlightVLA Guard 架构与安全边界

## 分层

```
┌────────────────────────────────────────────────────────────┐
│  Agent 层(VLA / VLM / LLM)         1 次推理 / 1.6 s        │
│  输入:Observation(状态 + 感知摘要)                          │
│  输出:ActionBlock(短时 6-DoF 动作块)+ 模拟推理延迟          │
├────────────────────────────────────────────────────────────┤
│  SafetyGuard(本项目的核心)          每块 11 级检查           │
│  schema → frame → 感知时效 → stop 信号 → 围栏 → 禁区绕行      │
│  → 最小距离球 → 运动学(v/a/jerk)→ 机型可行性(时间拉伸/拒绝) │
│  → 分配饱和度 → 相机可行性标注                                │
│  失败降级:Hold(悬停不停相机)/ guard 规划的 RTL              │
├────────────────────────────────────────────────────────────┤
│  Offboard 流(唯一写入者 = Guard)    setpoint 流,>2 Hz       │
├────────────────────────────────────────────────────────────┤
│  PX4(现场)/ 仿真级联(本项目内置)                           │
│  EKF、位置、速度、姿态、角速度闭环;offboard 心跳超时即 failsafe │
└────────────────────────────────────────────────────────────┘
```

## 为什么 VLA 不能进入姿态环 / 电机环(刻意的设计决策)

1. **频率不匹配。** VLA 推理是 0.1–1 Hz 量级、且延迟抖动大;姿态环跑在数百 Hz 到 kHz。
   让慢、抖的决策直接驱动快环,等于放弃整个控制系统的可证性。
2. **责任不可分。** 一旦大模型可以直接写出 PWM 或 actuator command,"这次碰撞是谁的锅"
   就再也说不清。ActionBlock 让 AI 的输出停留在**几何意图**层,guard 把它翻译成可验证的
   setpoints——AI 的每一分影响都可审计、可重放。
3. **边界可以被机制而非约定强制。** schema 表达不了电机量;guard 是 offboard 流唯一写入者;
   PX4 在流中断时自带 failsafe。三层机制互相独立,任何一层失守还有两层。

## PX4 Offboard 作为执行边界

PX4 官方要求 Offboard 模式必须持续收到 setpoint(>2 Hz),流中断后 PX4 退出 offboard 并执行
配置好的 failsafe(Hold / Land / RTL)。FlightVLA Guard 把这条规则当作自己的**执行边界**:

- guard 活着 → 由它以稳定节奏推送经过验证的 setpoints;
- agent 超时 / 视觉丢失 → guard 用 Hold 条目继续喂流(悬停,但相机继续朝目标慢转);
- offboard 流本身失联(故障 `offboard-loss`)→ guard 立刻自己规划一条**仍受全部约束检查**的
  RTL 轨迹接上;若 guard 进程也死,PX4 的 failsafe 兜底。

仿真中的简化:`SimBackend` + 简化的位置/速度级联(Kp=2.4, Kv=3.6,倾斜动力学、一阶姿态
滞后、阵风扰动)。它不替代 PX4 的权威性,只负责让评测指标(跟踪误差、能耗、最小距离)有
物理意义的载体。

## quad vs omni:被如实建模的物理差异

两机共用同一个 ActionBlock 与同一个 guard,差异只来自机型模型:

- **quad(欠驱动)**:水平加速度需要机体倾斜(倾角 = atan(a/g),上限 35°);相机固定
  12° 下俯安装座 → 任何俯仰/横滚都甩动视轴。相机 pitch 指令被物理否决(guard 显式标注)。
- **omni(全驱动)**:平移与姿态解耦,可以悬停状态俯仰机体以锁定相机;代价是水平加速度
  预算更小(4.0 vs 5.0 m/s²)。

在 valve-inspection 上的结果:转移段相机偏差 RMS quad ≈ 13°,omni ≈ 5°;阵风时 quad 倾斜
入风、视轴摆动,omni 以侧力抗风、姿态不动。

## 扩展点

| 想接什么 | 实现什么 | 不需要动什么 |
| --- | --- | --- |
| 真实 VLA(SmolVLA / OpenVLA / LLM) | `FlightAgent.propose()` + registry 注册 | guard、仿真、报告 |
| PX4 SITL / 真机 | `OffboardBackend`(MAVSDK / MAVROS / uXRCE-DDS) | agent、guard |
| 新机型(tilt / bi-rotor / gimbal) | `Vehicle` 子类(动力学 + 姿态语义 + 相机) | guard 检查逻辑 |
| 新任务场景 | `Scene` 定义(几何 + 指令 + 约束) | 一切 |
| 新评测指标 | `metrics.compute_metrics`(纯函数,吃 run.json) | 仿真 |

## 模块图

```
cli.py ──▶ sim.Runner ──▶ agents.ScriptedVLA   (可替换)
                │            ▲ Observation
                │            └ ActionBlock + latency
                ├──▶ safety.SafetyGuard ──▶ GuardResult(checks 逐条留痕)
                ├──▶ faults.FaultSchedule (阵风/视觉/延迟/offboard 失联)
                ├──▶ vehicles.Quad / Omni (动力学 + 相机安装座)
                └──▶ metrics / report ──▶ 单文件 HTML 报告
backends.SimBackend   (真实部署替换为 MAVSDK/MAVROS)
```
