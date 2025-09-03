# motor-gui 实时滤波功能进度（V1）

本文档总结截至当前分支 `feat/filter-v1` 的完成情况与后续计划。

## 概览
- 分支：`feat/filter-v1`（已创建）
- 约束：尽量仅新增文件；在你授权后我已最小化修改了 `Cargo.toml` 与 README 中路径错误，并新增了 `FilterConfig`（用于后续集成）。
- 当前未执行 `cargo check`/`cargo test`（环境可能受网络/依赖获取限制）。

## 已完成（对应 TODO 清单）
- 0. 基线与依赖
  - [x] 新建分支：`feat/filter-v1`
  - [x] `Cargo.toml` 增加依赖：`csv = "1"`, `chrono = "0.4"`
  - [ ] 编译通过（未执行 `cargo check`）
- 1. 目录与核心接口（新增文件）
  - [x] `src/lib.rs`：暴露库入口（`pub mod filter;`）
  - [x] `src/filter/mod.rs`：`Filter` trait、`FilterType`、`FilterChain`
  - [x] `src/filter/ma.rs`：移动平均（环形缓冲 + 运行和，O(1) 更新）
  - [x] `src/filter/ema.rs`：指数移动平均（alpha∈(0,1]，首样本直通，支持 reset）
  - [x] `src/filter/butter.rs`：低通 Butterworth（一阶 RC、二阶 biquad，按 dt 计算系数，抖动稳定性保护）
- 9. 测试（新增集成测试）
  - [x] `tests/filters.rs`：
    - MA/EMA/Butter 阶跃响应基本正确
    - 二阶 IIR 在 dt 抖动下稳定（范围、非发散）
    - `reset()` 后首样本直通
    - 非法参数回退为直通（构建时）
- 10. 文档（新增，不改原 README 功能说明）
  - [x] `README_filters.md`：模块使用与后续集成计划
  - [x] 修正 README / README_CHN 中的路径描述：`src/motor_backend.rs`（原先写成 `src/motor_backend/mod.rs`）
- 2. 领域模型（部分）
  - [x] 新增 `FilterConfig` 结构体（默认禁用；参数占位）

## 未完成（后续待做）
- 0. 基线与依赖
  - [ ] 运行 `cargo check` 验证本地可编译
- 2. 领域模型集成（`motor_ctx.rs`）
  - [ ] 在“每电机上下文”增加：`filter_cfg`、`filters_velocity/position/current: Option<FilterChain>`、`series_filt_*`
  - [ ] 方法：`apply_filters_on_new_sample()`、`reinit_filters()`、`reset_filters()`
- 3. 数据路径接入（后端→UI）
  - [ ] 在事件循环计算 `dt` 并调用 `apply_filters_on_new_sample`
  - [ ] “reset start time” 清空 raw/filt 并 `reset_filters()`
- 4. GUI（`main.rs`）
  - [ ] 新增面板 “Signal Filter” 与启用/信号选择/参数输入/校验
  - [ ] 参数变更触发 `reinit_filters()`
- 5. 绘图叠加
  - [ ] 速度/位置/电流图支持 raw/filtered 叠加、默认开关
- 6. CSV 导出扩展
  - [ ] 导出 Raw / Filtered / Raw+Filtered，列名 `t, v_raw, v_filt` 等
- 7. 生命周期钩子
  - [ ] “reset start time” 清空并 `reset_filters()`
  - [ ] Add/Remove motor 初始化/释放滤波器
  - [ ] “stop” 保留滤波状态继续处理回传
- 8. 健壮性
  - [ ] UI 参数校验：`alpha<=0`、`window<1`、`fc_hz<=0`、`order∉{1,2}` 禁止
  - [ ] 运行时：更精细的 IIR 系数异常回退策略（目前构造期非法参数直通，运行期有 dt 夹持与有限性检查）
- 9. 测试（补充）
  - [ ] 热切换参数不中断（`reinit_filters()` 行为）
- 10. 文档（补充）
  - [ ] 在主 README 增加 “Signal Filter” 小节、参数建议
  - [ ] 面板/曲线叠加截图、CSV 导出示例
- 11. 提交规划
  - [ ] 分步骤提交（chore/feat/test/docs），目前尚未执行 commit（仅切分支与文件变更）

## 设计与实现要点（当前版本）
- 滤波接口：`Filter::push(x, dt) -> y`，`dt` 单位为秒
- 运行时健壮性：
  - 丢弃 NaN/Inf 输入
  - `dt` 夹持下限（避免二阶离散化发散）
  - 二阶 biquad 采用每次按 `dt` 计算系数，抗采样周期抖动
  - 参数非法时构建期回退为直通（Bypass）

## 下一步建议
1) 将 `FilterConfig` 挂到每个电机实例的上下文，构造对应 `FilterChain`
2) 在四ier后端事件循环中计算 `dt` 并接入滤波路径
3) UI 增加 “Signal Filter” 面板与参数校验；绘图叠加 raw/filtered
4) 扩展 CSV 导出选项
5) 补充热切换参数与集成级测试

## 验收/验证建议
- 运行：`cargo test -p motor-gui`（如依赖齐全）
- 本地编译：`cargo check -p motor-gui`
- 后续集成完成后，手动切换滤波类型/参数，观察叠加曲线与导出 CSV

