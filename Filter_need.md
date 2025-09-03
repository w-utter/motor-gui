# motor-gui 实时滤波功能 TODO 清单（V1）

## 0. 基线与依赖
- [ ] 新建分支：`feat/filter-v1`
- [ ] `Cargo.toml` 补依赖（若尚未补）：
  - [ ] `[dependencies]` 增加 `csv = "1"`, `chrono = "0.4"`
- [ ] 编译通过（无功能变更）：`cargo check`

---

## 1. 目录与核心接口
- [ ] 新建目录 `src/filter/`
  - [ ] `src/filter/mod.rs`
    - [ ] 定义 `pub trait Filter { fn push(&mut self, x: f64, dt: f32) -> f64; fn reset(&mut self); }`
    - [ ] 定义 `pub enum FilterType { MovingAverage { window: usize }, ExponentialMA { alpha: f32 }, LowpassButter { fc_hz: f32, order: u8 } }`
    - [ ] 定义 `pub struct FilterChain` 封装三类滤波器
  - [ ] `src/filter/ma.rs`：实现移动平均（环形缓冲 + 运行和）
  - [ ] `src/filter/ema.rs`：实现指数移动平均
  - [ ] `src/filter/butter.rs`：实现一阶/二阶 Butterworth 低通

---

## 2. 领域模型集成（`motor_ctx.rs`）
- [ ] 新建 `FilterConfig`（启用开关、三路选择、FilterType 参数）
- [ ] 在每电机上下文 `MotorCtx` 中增加：
  - [ ] `filter_cfg: FilterConfig`
  - [ ] `filters_velocity/position/current: Option<FilterChain>`
  - [ ] `series_filt_velocity/position/current: Vec<(t, y)>`
- [ ] 新增方法：
  - [ ] `apply_filters_on_new_sample(&mut self, ...)`
  - [ ] `reinit_filters(&mut self)`
  - [ ] `reset_filters(&mut self)`

---

## 3. 数据路径接入（后端→UI）
- [ ] 在事件循环中新样本进入时：
  - [ ] 计算 `dt`，异常夹持
  - [ ] 调用 `apply_filters_on_new_sample`
  - [ ] “reset start time” 时清空 raw/filt 并调用 `reset_filters()`

---

## 4. GUI（`main.rs`）
- [ ] 在电机面板增加折叠区 **Signal Filter**：
  - [ ] `[ ] Enable filtering`
  - [ ] `[ ] Velocity  [ ] Position  [ ] Current`
  - [ ] 单选：`Moving Average | Exponential MA | Lowpass (Butterworth)`
  - [ ] 参数输入（MA: window；EMA: alpha 或 tau；Lowpass: fc_hz + order）
  - [ ] 参数校验（非法值报错并禁用应用）
  - [ ] 参数变更时 `reinit_filters()`

---

## 5. 绘图叠加
- [ ] 在速度/位置/电流图中支持：
  - [ ] `[ ] overlay raw`（默认开，细线）
  - [ ] `[ ] overlay filtered`（默认开，主线）

---

## 6. CSV 导出扩展
- [ ] 增加导出选项：
  - [ ] `Raw only`
  - [ ] `Filtered only`
  - [ ] `Raw + Filtered`
- [ ] Raw+Filtered 时列名为 `t, v_raw, v_filt` 等

---

## 7. 生命周期钩子
- [ ] “reset start time”：清空 raw & filt 并 `reset_filters()`
- [ ] Add/Remove motor：初始化/释放滤波器
- [ ] “stop”：保留滤波状态，继续处理回传

---

## 8. 健壮性
- [ ] UI 禁止 `alpha<=0`、`window<1`、`fc_hz<=0`、`order∉{1,2}`
- [ ] 运行时：
  - [ ] `dt` 下限夹持
  - [ ] NaN/Inf 输入丢弃
  - [ ] IIR 系数异常回退为直通或禁用

---

## 9. 测试
- [ ] `tests/filters.rs`：
  - [ ] MA/EMA/Butter 对阶跃/脉冲的响应正确
  - [ ] IIR 稳定性（随机 dt 抖动）
  - [ ] `reset()` 后首样本行为
  - [ ] 热切换参数不中断

---

## 10. 文档
- [ ] README 增补 “Signal Filter” 小节：
  - [ ] 三类滤波说明与参数建议
  - [ ] 面板与曲线叠加截图
  - [ ] CSV 导出示例

---

## 11. 提交规划
- [ ] `chore: add deps & scaffold filter module`
- [ ] `feat(filter): MA/EMA/Butter implementations`
- [ ] `feat(ctx): per-signal FilterChain + filtered series`
- [ ] `feat(ui): filter panel & validation`
- [ ] `feat(plot): overlay raw/filtered`
- [ ] `feat(csv): export raw/filtered/both`
- [ ] `test: filters basic specs`
- [ ] `docs: README filter section`

---
