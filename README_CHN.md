# motor-gui

基于 Amber AIOS 协议与电机控制器通信的桌面 GUI。可配置输入波形（恒值/阶跃/脉冲/自定义）、实时可视化反馈（速度/位置/电流）、并导出 CSV 数据。

## 项目概览

- 名称：motor-gui（Rust + egui 桌面应用）
- 用途：与电机控制器交互，发送输入波形并绘制返回的 CVP（电流/速度/位置）。
- 架构：GUI 使用 `eframe/egui`；设备 I/O 在独立线程中使用 `io_uring`；GUI 与后端通过 `mpsc` 消息通信；设备协议由 `amber-aios` 提供（支持 JSON 或二进制编码）。

## 主要功能

- 多电机：顶部“add motor”可添加多个电机面板。
- 后端选择：当前实现 `Fourier` 后端，可设置 IPv4 地址与编码（JSON/Binary）。
- 控制模式：位置 / 速度 / 力矩（可选择显示关联量）。
- 输入波形：Idle、Constant、Step、Impulse、Custom（幅值 + 持续时间的序列）；切换类型保留先前参数（缓存）。
- 实时曲线：速度主图；依据模式可显示位置/电流子图；“reset start time” 清空数据从零开始。
- CSV 导出：各图窗内提供 “save as CSV” 导出当前曲线。
- 设备控制：“send to motor” 发送当前输入；“stop” 停止波形。后端线程回传超时与错误事件。

## 目录结构

- `src/main.rs`：应用入口、UI 状态、绘图、波形编辑与消息交互。
- `src/controller.rs`：控制器接口占位（TODO）。
- `src/motor_ctx.rs`：领域模型（控制模式、CVP、配置）。
- `src/motor_backend.rs`：后端模块入口。
- `src/motor_backend/fourier.rs`：Fourier 后端（amber-aios + io_uring 事件循环）。

## 技术栈

- GUI：`eframe = 0.31`、`egui = 0.31`、`egui_plot = 0.32`。
- I/O：`io-uring`（仅支持 Linux）。
- 协议：`amber-aios`（Git 依赖，JSON/二进制编码）。
- 工具：`duration-string`（解析持续时间）、`libc`（错误码）。

## 运行环境

- 操作系统：Linux（需内核支持 `io_uring`，推荐 5.10+）。
- Rust：Edition 2024（建议最新 stable）。
- 网络：与设备同一网络；开放 `amber-aios` 使用的 UDP 端口。

## 构建与运行

1）安装 Rust（例如使用 `rustup`）。
2）拉取依赖（需要网络访问 Git 依赖，如 `amber-aios`、`io-uring`）。
3）运行：

```bash
cargo run            # 调试
cargo run --release  # 发布
```

首次启动后，点击顶部 “add motor” 开始配置设备。

## 使用说明

1）添加电机：点击 “add motor”，选择后端 `Fourier`。
2）设置设备：输入 IPv4 地址；选择编码（JSON/Binary）。
3）创建连接：地址就绪后点击 “add”；已添加后可 “remove” 断开。
4）选择控制模式：Position / Velocity / Torque，并可勾选显示关联量。
5）编辑波形：
   - Constant：设置幅值。
   - Step：设置 `delay`、`magnitude`、`duration`（支持 `1s`、`500ms` 等）。
   - Impulse：设置 `delay` 与 `magnitude`。
   - Custom：配置多段 `(magnitude, duration)`。
6）发送/停止：“send to motor” 应用输入；“stop” 停止波形。
7）查看/导出：打开速度/位置/电流窗口查看曲线，点击 “save as CSV” 导出。

## 后端事件循环

- 通道：GUI 下发 `(Ipv4Addr, FourierCmd)`；后端回传 `(Ipv4Addr, FourierResponse)`。
- io_uring：链接 `Send/Recv/LinkTimeout`；按编码构造请求；解析响应为 `CVP`。
- 波形推进：按时间推进 Step/Impulse/Custom；结束时发出 `EndWaveform`。
- 超时/错误：在超时/错误时重新入队 I/O 并上报事件。

## 已知问题/注意事项

- 依赖声明缺失：代码使用了 `csv` 与 `chrono`，但 `Cargo.toml` 未声明。请在 `[dependencies]` 添加：

  ```toml
  csv = "1"
  chrono = "0.4"
  ```

- 路径与权限：速度 CSV 导出使用了已弃用的 `std::env::home_dir()`；其余导出在当前目录。请确保写权限。
- 控制器逻辑：`controller.rs` 的 `Controller::update` 尚未实现；当前不做闭环调整，仅显示设备输出。
- 平台限制：后端依赖 Linux（`io_uring`）；其他平台无法运行后端。
- 错误展示：详细错误当前仅打印到控制台，UI 暂未展示。

## 开发提示

- 绘图：使用 `egui_plot` 的 `PlotPoints` 绘制多曲线；每个图窗独立显示。
- 新增后端：在 `motor_backend` 下新增模块，参考 `fourier.rs`，实现 `Backend<T>` 钩子与 UI 绑定。
- 性能：I/O 队列深度为 16；序列化由 `amber-aios` 处理；可按需调整 `FourierBackend::<R, W>` 读写缓冲大小。
