# ActionBlock v0.1 — FlightVLA Guard 数据格式规范

*The wire format between a flight agent (VLA / VLM / LLM) and FlightVLA Guard.*

## 设计原则

1. **Schema 即边界。** 电机 PWM、电机转速、单旋翼推力、PX4 actuator command、原始 MAVLink
   指令在 ActionBlock 中**无法表达**——不是"不允许",是"表示不出来"。Agent 输出再离谱,
   最坏也只是几何上不可行的运动提议。
2. **短时、增量、带自我评估。** 每个块只覆盖一个短视界(默认 8 步 × 0.2 s = 1.6 s),
   由 agent 的 `confidence` 与 `stop_probability` 自陈把握程度。
3. **载体无关。** `delta_position` 是几何量,不含机型参数;同一个块可以发给 quad 也可以发给
   omni,由 guard 按机型回答"飞不飞得动"。

## 字段

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `frame` | `"body"` \| `"world"` | — | 位置增量的参考系。`body`:x 前 y 左 z 上(仅绕 z 对齐 yaw);`world`:ENU |
| `horizon` | int ≥ 1, ≤ 64 | 步 | 块内步数 H |
| `dt` | float ∈ [0.001, 1.0] | s | 每步时长 |
| `delta_position` | float[H][3] | m | 每步位移(schema 上限 2.5 m/步;guard 再按机型收紧) |
| `delta_orientation` | float[H][3] | rad | 每步 `[d_yaw, d_pitch, d_roll]`;语义为**相机**朝向指令 |
| `stop_probability` | float[H] ∈ [0,1] | — | 第 k 步"我不确定 / 建议停"的置信。超过阈值(默认 0.6)处截断并悬停 |
| `confidence` | float ∈ [0,1] | — | 整块置信度(进入评测指标) |
| `agent` | str | — | 产出者标识(运行时填写 `seq` / `t_created`) |

### 约定

- **姿态增量的语义是相机朝向**,不是机体姿态。欠驱动机体(quad)的相机俯仰由平移加速度决定,
  guard 会显式标注 "pitch commands will not be honoured";全驱动机体(omni)可以悬停改变俯仰,
  精确跟踪相机指令。
- `frame: "body"` 由 **agent 负责**转换;guard 用当前 yaw 做逆变换。`d_yaw`/`d_pitch` 视为
  世界系角度(小步长下近似成立)。
- 块在 `t_created + inference_latency + guard_overhead` 时刻生效;超迟到的块仍会被执行
  (比没有好),但**计一次 timeout** 进入评测。

## Guard 契约(对块做什么)

依次通过 11 级检查,全部逐条留痕进入报告:

```
schema → frame 变换 → 感知时效 → stop 信号截断(悬停不停相机)→ 围栏
      → 禁区绕行(wall-follow,细步长)→ 最小距离球 → 运动学(v/a/jerk,带动量)
      → 机型可行性(不可行则时间拉伸,超限则拒绝)→ 分配饱和度告警 → 相机可行性标注
```

输出 = 世界系 setpoint 流(`position_refs` / `orientation_refs` / 每步 `dt`),交给 offboard
backend 以 >2 Hz 推给 PX4。**Guard 是 offboard 流的唯一写入者。**

## Run logs(评测数据格式)

`flightvla run` / `demo` 生成的 `run.json` 结构:

```
meta        agent / vehicle / faults / seed / duration / flightvla_version
scene       任务几何与指令、逐条约束
timeline    50 Hz 状态:t, p, v, ref, yaw/pitch/roll, bore(±), dist, speed,
            track_err, energy, wind, hold, on_target
chunks      每个 agent 调用:t_call / t_arrival / latency / timeout / confidence /
            status / 逐条 checks / raw_path / repaired_path / raw_block / stats
events      启停、故障窗口、hold、timeout、success、RTL
keyframes   报告相机帧采样点(bore 偏角、距离、清晰度)
outcome     success / success_t / rtl / holds / timeouts
metrics     见 flightvla/metrics.py(可离线对 run.json 重新计算)
```

同一 `run.json` 可以被第三方工具重新打分;导出 LeRobotDataset 在 Roadmap 中。

## 版本化

`schema_version: "0.1"`。破坏性变更将升 minor 版本并在 guard 中保留旧版解析器至少一个大版本。
