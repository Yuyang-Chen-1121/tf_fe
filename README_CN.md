# CWT / OSSM 汇报版 RTL

单位：EPFL INL  
作者：Yuyang Chen  
最后修改：2026-09-29

## 1. 文件组织

```text
syn/
├── CWT/
│   ├── cwt_feature_engine.v   独立 CWT 顶层、控制器、地址与存储
│   ├── fe_cwt_mac.v          INT12 复数核点积
│   └── fe_sqrt.v             幅值开方
├── OSSM/
│   ├── ossm_feature_engine.v 独立 OSSM 顶层、控制器、地址与存储
│   └── fe_ossm_step.v        基线、预测、共同残差、修正和功率
├── common/
│   ├── fe_config_store.v    参数双 bank 加载与激活
│   ├── fe_stream_control.v  平均区间与运行状态计数
│   ├── fe_input_affine.v    signed12 输入标准化
│   ├── fe_output_quant.v    平均/量化的整数除法
│   ├── fe_token_output.v    双 token bank、布局与输出握手
│   ├── fe_round.v           RNE 舍入和饱和
│   └── fe_storage.v         标准 DFF 与同步 RAM
├── cwt.f                    仅 CWT 所需的源文件清单
├── ossm.f                   仅 OSSM 所需的源文件清单
├── all.f                    两种算法的联合源文件清单，common 只列一次
├── run_testbench.py         一键编译、仿真、校验与汇总（Python 标准库）
├── tb/
│   ├── tb_fe_stream.v       纯 Verilog-2005 外部驱动与输出校验
│   └── data_manifest.json   数据清单、格式、来源和 SHA-256
├── sim_data/                本地配置、输入、golden；数据文件不提交 Git
│   └── README_CN.md         需要复制的 5 个文件及格式说明
└── sim_build/               自动生成的编译结果、日志、CSV、可选波形；不提交 Git
```

`common` 表示共用模块定义；两个引擎分别实例化自己的公共模块。同时实例化两个引擎时，不会自动共用物理 RAM、除法器或输出队列。

## 2. 状态机的展示方式

两个外层引擎，以及 OSSM 内部的 `fe_ossm_step`，都采用传统三段式：

1. `always @(posedge clk or negedge rst_n)` 保存当前状态，处理 reset/clear。
2. 第二个 `always @*` 以 `case` 列出下一状态；默认保持，非法编码回到 IDLE。
3. 第三个 `always @*` 以 `case` 产生当前状态的读地址、操作使能、握手、错误采样和循环控制。


### CWT 主线

```text
IDLE → NORM_INV / NORM_BIAS（遍历通道）
     → KERNEL_LOAD / TAP_ACCUM（遍历 tap）
     → MAC_RESULT → SQRT_WAIT（遍历频率和通道）
     → PROCESS_START → POOL（遍历 320 个特征）
     → 若为平均末帧：DEN_LOW → DEN_HIGH → DIV_START → DIV_WAIT
     → 完整 token 才 COPY → FINISH → IDLE
```

### OSSM 主线

```text
IDLE → NORM_INV / NORM_BIAS（遍历通道）
     → ROTATE_LOAD → PROCESS_START → GAIN_LOAD
     → STEP_START → STEP_WAIT
     → POOL（当前通道的五个功率；末帧还要逐特征量化）
     → 下一个通道，或完整 token COPY → FINISH → IDLE
```
`fe_ossm_step` 中另一个三段式状态机，控制共享乘法器的 `LOAD → MULTIPLY → STORE`，保留每通道 178 个内部运算周期。

## 3. 综合与文件选择

在 Efinity 中仅导入本目录中对应 `.f` 列出的 Verilog 文件，并选择：

- CWT 顶层：`cwt_feature_engine`。
- OSSM 顶层：`ossm_feature_engine`。

`tb/` 和 `run_testbench.py` 仅用于仿真，不加入 Efinity 综合。原有 `cwt.f`、`ossm.f`、`all.f` 仍然只包含可综合 RTL。

## 4. 一键运行 testbench

### 4.1 运行环境

需要 **Python 3.9 或更新版本**，以及以下任意一套仿真工具，均应在 `PATH` 中：

- **Verilator 5.x**：支持 `--binary --timing`，另需 C++ 编译器及 make。适合完整 1000 帧回归。
- **Icarus Verilog**：需要 `iverilog` 和 `vvp`，可运行四态仿真。Windows 可直接选择这一后端；完整长流比 Verilator 慢。

无需 pip 安装包，不依赖 NumPy、PyTorch、Efinity 或原工程的 `hdl`、`Final`、模型目录。Python 命令在部分系统上叫 `python3`，可将下文的 `python` 替换为 `python3`。

### 4.2 准备数据

本机的 `sim_data/` 已经放好数据。克隆 GitHub 仓库后，需要另外复制以下文件：

| 放入 `syn/sim_data/` 的文件 | 原工程来源 |
| --- | --- |
| `cwt_payload.memh` | `hdl/coeff/cwt_payload.memh` |
| `ossm_payload.memh` | `hdl/coeff/ossm_payload.memh` |
| `stream_frames.memh` | `hdl/vectors/stream_frames.memh` |
| `cwt_stream_golden.memh` | `hdl/vectors/cwt_stream_golden.memh` |
| `ossm_stream_golden.memh` | `hdl/vectors/ossm_stream_golden.memh` |

**总计约 280 KiB，不需要复制完整原始数据集。** 数据格式和来源详见 [sim_data/README_CN.md](sim_data/README_CN.md)。脚本会先校验 SHA-256、字数和格式，缺文件或版本不匹配时返回非零退出码，并列出需要补齐的文件。

### 4.3 一条命令运行完整回归

在工程目录下执行：

```sh
python run_testbench.py
```

默认自动优先选择 Verilator，否则使用 Icarus；执行 **CWT / OSSM × channel-major / frequency-major，共 4 次仿真**。每次输入 1000 帧，`average_sample_count=50`，期待 20 个 token、6400 个输出字节；四次共核对 **25600 个字节**。

脚本以自身所在目录作为工程根目录。因此也可以从任意工作目录执行 `python /path/to/syn/run_testbench.py`，不需要修改路径。

常用选项：

```sh
# 快速验证：每次只送前 100 帧，输出 2 个 token。
python run_testbench.py --frames 100

# 只运行一个算法 / 一种布局，并明确选择 Icarus。
python run_testbench.py --algorithm ossm --layout channel --frames 50 --simulator iverilog

# 按 100 MHz 时钟下每 200000 周期，即每 2 ms，送一帧。
python run_testbench.py --frames 100 --frame-interval 200000

# 保存接口波形（建议用短流，波形也放在被忽略的 sim_build 下）。
python run_testbench.py --algorithm cwt --layout channel --frames 50 --vcd

# 只验证数据是否齐全、版本是否匹配，不要求安装仿真器。
python run_testbench.py --check-data-only
```

`--frames` 必须是 50 的整数倍，范围 50～1000。当前数据集只适用于平均长度 50，testbench 不提供任意修改平均长度的选项。`--layout` 可选 `channel`、`frequency`、`both`；`--algorithm` 可选 `cwt`、`ossm`、`both`。

默认 `--frame-interval 0` 按 `in_ready` 尽快送入下一帧，以缩短墙钟运行时间，**不是声称输入源按 2 ms 间隔驱动**。脚本仍检查每帧处理时间不超过 200000 个时钟；设置 `--frame-interval 200000` 才是按真实 2 ms 节奏输入。在无额外阻塞时，50 帧对应每 100 ms 一个 token。

## 5. 外部数据如何进入电路

入口文件是 [tb/tb_fe_stream.v](tb/tb_fe_stream.v)。它通过 `generate` 实例化对应引擎，所有驱动和校验都使用公开端口。`$readmemh` 只加载 **testbench 自己的数组**，没有直接初始化或修改 DUT 内部 RAM。

### 5.1 参数加载：`load_parameters`

1. 拉低 `rst_n`，保持若干时钟，再释放复位。
2. 将参数文件的第 `i` 个 24-bit 字放到 `param_word`，拉高 `param_valid`。
3. 第一个字同时拉高 `param_first`，最后一个字同时拉高 `param_last`；其他字这两个标志均为 0。
4. DUT 在上升沿采样有效参数字。每 17 个字主动插入一个空拍，验证加载过程允许间断。
5. 结束后拉低 `param_valid/first/last`，等待 `params_valid` 和 `in_ready` 有效。

该接口没有 `param_ready`。本测试完整加载一次配置，然后再开始送输入帧。CWT 加载 1117 字，OSSM 加载 6785 字。

### 5.2 输入帧：`send_frame`

每帧同时包含 64 个通道，各通道是 signed12 补码：

| 位段 | 内容 |
| --- | --- |
| `in_frame[11:0]` | channel 0 |
| `in_frame[23:12]` | channel 1 |
| `in_frame[ch*12 +: 12]` | channel ch |
| `in_frame[767:756]` | channel 63 |

`send_frame` 在下降沿设置全部 768 位并拉高 `in_valid`，一直保持到某个上升沿 `in_valid && in_ready` 成立；随后在下降沿撤销 valid。即使 DUT 暂时忙，源数据也保持不变。测试时钟为 100 MHz，所有驱动在下降沿、所有传输检查在上升沿，避免与 DUT 的时序逻辑竞争。

输入文件已经是送入 FE 的 signed12 编码；RTL 自己完成 normalization。源文件拼接和编码信息见数据目录说明。

### 5.3 输出接收与校验

testbench 模拟外部消费者：周期性拉低 `out_ready`，只在 `out_valid && out_ready` 时接收一个字节。检查内容包括：

- 每个输出字节与 Python 定点整数 golden 完全相同。
- `out_data[7:6] == 2'b00`，即值在 0～63；Icarus 下还检查有效信号无 X。
- `out_index` 每 token 从 0 到 319，`out_last` 只在最后一个字节有效。
- 输出暂停时，valid、data、index、last 必须保持稳定。
- 累计不足 50 帧不能提前发布 token；最终输入帧数和 token 数必须准确。
- `error` 不得置位；参数生效前不得接受帧；仿真有周期和墙钟超时限制。

两个布局使用同一份 channel-major golden。frequency-major 输出按 `f*64+c` 发送，校验时映射到 golden 的 `c*5+f`，避免因输出次序不同而误报。

这套独立 testbench 验证的是 **RTL 与 Python 定点参考的逐字节一致性及接口协议**；不重新运行浮点 FE 误差分析、神经网络模型或原版 RTL 双实例等价验证。默认 1000 帧覆盖 CWT 环形历史回绕，以及 OSSM gain_age 达到 749 后保持最后一组的阶段。

## 6. 结果在哪里看

运行时终端会显示 `PARAM_BEGIN`、`PARAM_ACTIVE`、`INPUT_ACCEPT`、`TOKEN_PASS`，最后每个配置应出现 `TB_PASS`。总脚本只有在所有选中配置成功后才输出最终 `PASS`；失败时退出码非零，不会将仅有 `$finish` 的退出当作成功。

`sim_build/summary.json` 汇总本次结果、工具版本、数据与源文件哈希。每种测试配置有独立目录，例如 `sim_build/verilator_cwt_channel_n1000_gap0/`：

| 文件 | 用途 |
| --- | --- |
| `build.log` | 实际编译命令及编译器输出 |
| `simulation.log` | 实际运行命令、参数加载/帧输入/token 完成日志 |
| `parameters.csv` | 每个参数字写入的 cycle、地址顺序、24-bit 值、first/last |
| `inputs.csv` | 每次输入握手的 cycle、帧号、channel 0 有符号值和完整 768-bit 数据 |
| `output.csv` | 每次输出握手的 cycle、token、index、实际值、golden 值和 last |
| `result.json` | 帧数、token 数、字节数、最大帧处理周期和耗时 |
| `wave.vcd` | 指定 `--vcd` 时产生，供波形查看器打开 |

推荐先打开 `parameters.csv`、`inputs.csv`、`output.csv`，沿同一个 cycle 体系查看数据如何从外部进入并最终输出。

