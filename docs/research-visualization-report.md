# 可视化调研报告:评测报告如何做出专业级质感

*调研范围:同领域 PX4 官方报告平台、机器人可视化基础设施(Rerun / Foxglove / LeRobot)、
期刊官方制图规范(Nature)、编辑级网页排版(Distill)、高传播度机器人项目页(OpenVLA)。
结论用于指导 `flightvla/report_template.html` 的第三版重设计。*

---

## 0. 结论摘要

方向没错(白底、Helvetica、克制配色是期刊的正统),**差距在执行密度与交互深度**。
六类顶级参照物交叉验证出六个"专业感"来源:

1. **一条全局时间轴驱动所有面板**(Rerun/Foxglove 的签名):播放头同时驱动 3D、全部曲线
   游标、Inspector——而不是各图各放。
2. **高数据密度的 KPI 头部**(PX4 Flight Review):打开报告 3 秒内读到任务结论。
3. **共享 x 轴的堆叠多面板时序流**(Flight Review 把 36 组图垂直排布 + 侧边导航,而非网格铺开):
   同一时间轴上下的曲线天然可对齐读图。
4. **背景色带语言**(Flight Review:所有时序图带"飞行模式"着色带;振动指标带绿/橙/红阈值域):
   状态是画在背景里的,不是图例框里的一行字。
5. **期刊排版的精确参数**(Nature 官方):面板字母 8pt 粗体正体小写;其余文字 ≤7pt;
   Arial/Helvetica;轴必须有刻度与括号单位 "Data (unit)";Wong 色盲安全色;
   不用网格线阴影和装饰图标;色块内必须是黑字。
6. **诚实的失败呈现**(OpenVLA 项目页专门放失败案例):安全层"拒绝/悬停"的记录本身就是
   报告的卖点,要当成 first-class 内容排。

---

## 1. 逐对象调研

### 1.1 PX4 Flight Review(同领域权威,review.px4.io)

技术形态:Python + Bokeh + Tornado,ULog 上传后生成浏览器报告;3D 回放用 Cesium。

报告结构(从 `configured_plots.py` / `templates/header.html` 实测):

- 顶部导航条:logo + Upload/Browse/Statistics + **Navigation 下拉(每个图一个锚点 fragment)**
  + **Plot Legend 下拉(飞行模式色标)** + Download(日志/参数/KML)。
- 所有时序图的背景按**飞行模式着色带**分段,图例解释色带——状态直接画在背景层。
- 36 组图垂直流:位置地图 → 高度 → 姿态角/角速率 → 位置/速度 → 手动输入 → 执行器 →
  FFT/PSD → 振动指标 → GPS → 电源 → 温度 → 估计器/失败保护标志 → CPU → 采样规整度。
- **振动指标用绿/橙/红阈值背景框**(边界 4.905 与 9.81 m/s²)——阈值是色域不是虚线。
- 参数变化:全局 "Hide/Show Parameter Changes" 开关,图上标注点联动。
- 次要数据(控制台输出、性能计数器)折叠在 "Show additional Data" 里。

**采纳**:KPI 摘要头;共享 x 轴堆叠图流;导航条;背景色带语言;阈值色域;折叠次要信息;
"事件条可点击跳转"。

### 1.2 Rerun / Foxglove(机器人可视化基础设施)

- 两者的共同签名:**多面板由同一条时间轴驱动**,回放时 3D、曲线、图像全部同步;
  events/annotation 是一等数据类型;支持 side-by-side 多 run 对比(Foxglove 明确以
  "visually identify differences across multiple runs" 为卖点)。
- Rerun 提出 "multi-rate, multimodal data" 的时间对齐是数据层核心问题。

**采纳**:播放头即全局状态;事件标注可点击 seek;对比模式强调"同任务双机"的差值阅读。

### 1.3 LeRobot(数据格式与呈现)

- LeRobotDataset = Parquet(状态/动作)+ MP4(视觉)**同步成对**,episode 是一等概念,
  Hub 在线浏览数据集。
- 品牌感来自:动态演示横幅、数据集即产品、community 语气。

**采纳**:run 元数据头部规范化(run id / seed / 任务 / 机型),让每份报告像一条
"episode 记录"而不是一张网页。

### 1.4 Nature 官方制图规范(research-figure-guide.nature.com)

- 字体:Arial / Helvetica;面板字母 **8pt 粗体、正体、小写 a, b, c**;
  其余文字 **≤7pt**,最小 5pt(按最终印刷尺寸计)。
- 轴:必须有轴线与刻度;标签格式 **"Name (unit)"**。
- 颜色:**Wong 色盲安全色**(Nature Methods 8, 441);避免红绿组合;色块内黑字;
  无网格线阴影、无装饰图标;实色不用花纹。
- 尺寸:单栏 89 mm / 双栏 183 mm(通识值);RGB;矢量优先。

**屏幕换算**:双栏 183 mm ≈ 692 px @96dpi,缩放系数 ≈1.5 → 面板字母 ≈12px,图内文字 9–11px,
轴题与刻度 10–11px。我们的模板按此校准。

### 1.5 Distill.pub(编辑级网页排版)

- 衬线正文 + 居中窄单列 + Tufte 式边注;逐图编号题注,题注含**出处级细节**
  ("(layer mixed4a, unit 11)");交互图是一等公民;审稿过程公开可见。
- 无多余装饰,meta 信息(作者、DOI、日期)是版面的一部分。

**采纳**:英文衬线副标题;约束条件做成边注式;题注里写元数据(seed、chunk 8×0.2 s、Kp/Kv)。

### 1.6 机器人项目页范式(OpenVLA 等)

- Hero:大标题 + 作者机构 + Paper/Code/Data 三按钮;结果**按主题分组**、逐条 caption;
  **明确放失败案例**(❌ 行)以换取可信度;播放速度等披露写进题注。

**采纳**:头部加 Highlights;把"被拒绝的 AI 动作"作为 first-class 叙事(这正是本项目卖点);
题注披露模拟参数。

---

## 2. 差距诊断

| 维度 | 现状(v2) | 参照物做法 | 判定 |
| --- | --- | --- | --- |
| 时间轴 | 3D 回放自转,图表静态 | 全局时间轴驱动一切 | **核心差距** |
| 图表布局 | 6 图两列网格,各自独立 | 共享 x 轴垂直堆叠流 | **核心差距** |
| 数据头部 | 一行文字结果条 | KPI 磁贴阵列 | 重点改 |
| 背景/阈值 | 虚线阈值 | 色带背景 + 阈值色域 | 重点改 |
| 3D 质感 | 线框 + 三角箭头 | 机体造型、阴影、FOV、遥测 HUD | 重点改 |
| 事件 | 独立小节条带 | 事件条内嵌回放、可点击 seek | 合并升级 |
| 交互 | 拖拽旋转 + 播放 | hover 读数、seek、键盘、游标 | 补齐 |
| 排版细节 | 已对齐 Nature(白底/字重/色板) | tabular 数字、衬线副题、边注 | 打磨 |

## 3. 修改方案(已全部落地,验收见下)

| # | 改动 | 对应调研来源 |
| --- | --- | --- |
| 1 | 吸顶导航条:logo + Fig.1–6/Table 1 锚点 + run 标识 | Flight Review Navigation 下拉 |
| 2 | KPI 磁贴阵列(6 项,双机并列,tabular-nums) | Flight Review 摘要表 |
| 3 | Fig.4 改为**共享 x 轴堆叠多面板**,只最底面板显示时间刻度 | Flight Review 垂直图流 |
| 4 | 全局播放头游标同步到每一条曲线;hover 出跨面板读数 | Rerun/Foxglove |
| 5 | 面板 a 加"on-target 18°"阈值色域(绿/红色域背景) | Flight Review 振动阈值框 |
| 6 | 3D:机体造型(quad X 形四桨 / omni 六边形六桨)、地面投影阴影、对地垂线、相机 FOV 楔面、右上角遥测 HUD、地面微渐变 | Flight Review 3D / Cesium |
| 7 | 事件条并入 Fig.1,点击 seek;空格播放/暂停,←/→ ±1 s | Rerun/Foxglove |
| 8 | 英文衬线副标题、题注写元数据(seed、8×0.2 s、Kp/Kv)、约束边注化 | Distill |
| 9 | Table 1 每行最优值加粗;执行越界恒 0 用绿字强调 | OpenVLA 诚实呈现 |
| 10 | 全部数字 tabular-nums;轴题 "Name (unit)" 逐项校对 | Nature 官方 |

## 4. 参考链接

- PX4 Flight Review: https://github.com/PX4/flight_review (configured_plots.py, templates/header.html, 3d.html)
- Rerun: https://rerun.io · Foxglove: https://foxglove.dev
- LeRobot: https://github.com/huggingface/lerobot
- Nature figure guide: https://research-figure-guide.nature.com/figures/preparing-figures-our-specifications/ ·
  https://www.nature.com/nature/for-authors/formatting-guide
- Distill: https://distill.pub · https://distill.pub/2017/feature-visualization/
- OpenVLA 项目页: https://openvla.github.io/
- Wong 色盲安全色: Nature Methods 8, 441 (2011)
