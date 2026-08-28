# FlightVLA Guard

**让任意 VLA / VLM / LLM 飞行 Agent 安全接入 PX4 的开源运行时、数据格式与评测平台。**
*An open-source runtime, data format and evaluation platform for safely connecting any
VLA / VLM / LLM flight agent to PX4 — "LeRobot for drones" plus a VLA safety gateway.*

```
自然语言 + 相机画面 + 无人机状态
              ↓
      VLA / VLM / LLM Agent          ← 只输出 ActionBlock(短时 6-DoF 动作块)
              ↓
       FlightVLA Guard               ← 验证 / 修正 / 拒绝危险动作
              ↓
     PX4 执行安全轨迹                 ← EKF 与位置/速度/姿态/角速度闭环仍属 PX4
              ↓
  记录、回放并生成评测报告
```

## 60 秒上手

```bash
# 两侧对比 Demo:普通四旋翼 vs 全向六旋翼,同一个任务、同一个 VLA、同一组故障
python -m flightvla demo                      # 生成 reports/demo-valve-inspection.html

# 单次运行(与 README 顶部命令一致)
python -m flightvla run \
  --agent smolvla \
  --vehicle omni-hex \
  --task valve-inspection \
  --fault latency-300ms

# 把一个 ActionBlock JSON 直接送进安全层做静态检查
python -m flightvla validate chunk.json --vehicle quad
```

零依赖(纯 Python 标准库),Python ≥ 3.9,`pip install -e .` 后也可用 `flightvla` 命令。
生成的 HTML 报告**单文件、离线可用**,直接拖进浏览器;回放页支持 `report.html#t=17.5` 跳转播放头,录屏即可得到 20–30 s 的传播 GIF。

![demo](docs/img/report-demo.png)

## 演示任务:valve-inspection

> 找到红色压力表,从左侧靠近并检查;保持相机始终正对仪表;距离不得小于 1.5 米;不要进入黄色区域;看不清时立即悬停。

页面左右分屏:左边普通四旋翼,右边全向六旋翼。VLA 生成一条**红色原始轨迹**,其中一部分穿过黄色禁区;
FlightVLA Guard 将它修正成**绿色安全轨迹**。注入阵风、视觉丢失和 300 ms 推理延迟后,系统自动触发 Hold。
报告同时呈现:

| 报告内容 | 报告区块 |
| --- | --- |
| VLA 看到了什么 | 合成相机帧关键帧画廊(quad 机体倾斜时仪表移出画面中心,omni 全程锁定) |
| 用户给了什么自然语言指令 | 报告头部指令卡 + 逐条约束的执行方式 |
| VLA 原本想怎么飞 | 红色虚线原始动作路径 + AI 动作检查器(原始 ActionBlock JSON) |
| 哪些动作被安全层修改或拒绝 | 干预日志(逐条 check:rescale / projection / reject / hold) |
| PX4 最终执行了什么 | 白色实际执行轨迹 + 修复后 setpoints |
| 推理延迟和超时次数 | 延迟散点图(带预算线)+ 超时计数 |
| 碰撞、越界和最小安全距离 | 拦截的禁区动作点数、执行越界数(恒为 0)、最小距离 |
| 轨迹跟踪误差与能耗 | 跟踪 RMSE / 能耗代理时序图 |
| 普通四旋翼 vs 全向无人机 | 对比表 + 相机偏离 RMS 曲线(quad ≈ 13°,omni ≈ 5°) |

## 关键技术边界(刻意设计)

**VLA 只输出短时动作块,永远不碰电机:**

```json
{
  "frame": "body",
  "horizon": 8,
  "dt": 0.2,
  "delta_position":    [[0.36, 0.15, 0.01], ...],
  "delta_orientation": [[0.05, -0.02, 0.0], ...],
  "stop_probability":  [0.03, 0.03, ...],
  "confidence": 0.82
}
```

电机 PWM、电机转速、单旋翼推力、PX4 actuator command、绕过安全层的 MAVLink 指令——
在 **schema 层就无法表达**(见 [`flightvla/schema.py`](flightvla/schema.py) 与
[docs/action-format.md](docs/action-format.md))。"AI 不能越权"不是约定,是数据格式保证。

**安全层检查**(每次 chunk 全部跑一遍,逐条留痕):
速度 / 加速度 / jerk 上限 · 禁飞区与围栏 · 最小安全距离球 · 机型可行性(跟踪该轨迹需要的姿态
与推力)· 控制分配饱和度 · 相机帧是否过期 · VLA 推理是否超时 · 该机型能否完成这个 6-DoF 动作
(quad 的相机俯仰不由指令决定)· 失败后 Hold / RTL / 人工接管的降级策略。

**PX4 仍是闭环的主人。** Guard 输出的只是 offboard setpoint 流;PX4 继续负责 EKF、位置、速度、
姿态和角速度闭环。PX4 官方要求 Offboard 控制持续发送健康信号,失联后退出 Offboard 并执行配置的
failsafe——这正是 FlightVLA Guard 的执行边界(见 [docs/architecture.md](docs/architecture.md))。
不能为了"端到端 AI"让大模型进入高速姿态环或电机环。

## 全向 vs 四旋翼:物理差异被如实建模

| | quad(欠驱动) | omni-hex / omni-octo(全驱动) |
| --- | --- | --- |
| 平移 | 必须倾斜机体(倾角 = atan(a/g)) | 平移与姿态解耦 |
| 相机(固定 12° 下俯安装座) | 机体倾斜 → 视轴甩动,横移/加速时仪表离开画面中心 | 可在悬停状态俯仰机体,全程锁定目标 |
| 转移段相机偏差 RMS(演示任务) | ≈ 13° | ≈ 5° |
| 侧向力权限 | 更大(倾角预算高) | 略小(侧力效率低)——权衡如实呈现 |

## 接入真实 VLA / 真机

- **真实 VLA(SmolVLA / OpenVLA / 任意 LLM 规划器)**:实现
  [`FlightAgent.propose()`](flightvla/agents/base.py),注册进 registry,其余一切(安全层、仿真、报告)不变。
- **PX4 真机 / SITL**:实现 [`OffboardBackend`](flightvla/backends/base.py)
  (MAVSDK / MAVROS / uXRCE-DDS),按 PX4 Offboard 要求以 >2 Hz 推送 guard 通过的 setpoints。
- **评测**:`run.json` 记录了完整时间线与逐条判定,可离线重新打分;详见 action-format 文档的
  "Run logs" 一节。

## 目录

```
flightvla/
├── schema.py          # ActionBlock v0.1 数据格式(schema 即边界)
├── agents/            # FlightAgent 接口 + ScriptedVLA(smolvla 占位,可替换为真实模型)
├── vehicles/          # quad(欠驱动)与 omni(全驱动)模型,相机安装座物理
├── safety/guard.py    # SafetyGuard:11 级验证 / 修形 / 拒绝 / Hold-RTL 降级
├── faults.py          # 阵风 / 视觉丢失 / 推理延迟 / offboard 失联(按运行时长分段注入)
├── sim.py             # 50 Hz 闭环仿真:agent → guard → offboard 流 → 车辆
├── metrics.py         # 评测指标(可离线重打分)
├── report.py + report_template.html   # 单文件 HTML 评测报告(离线可用)
└── backends/          # OffboardBackend 契约(PX4 后端接入点)
docs/                  # 架构与数据格式规范
tests/smoke_test.py    # 端到端不变量测试(python tests/smoke_test.py)
```

## Roadmap

- [ ] MAVSDK / MAVROS offboard 后端 + PX4 SITL(Gazebo)闭环
- [ ] 真实 SmolVLA / OpenVLA 适配器(GPU 推理延迟如实进入评测)
- [ ] 导出 LeRobotDataset 格式,把飞行 episode 变成可训练数据
- [ ] 场景矩阵评测 CLI(`flightvla eval --grid`)与排行榜
- [ ] 更多机型:fully-actuated octo、bi-rotor tilt、gimbal 相机模型

## License

Apache-2.0
